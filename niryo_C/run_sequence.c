// run_sequence.c
// C runner for Niryo Ned2 using uint16-LE length-prefixed JSON TCP protocol.
// - Loads poses & sequences from sequences.json
// - Supports: "joints" (deg -> rad), {"wait": s}, {"gripper":"open|close"}
// - Optional: calibrate before sequence.
//
// Build: gcc -O2 -Wall -o run_sequence run_sequence.c
// Usage: ./run_sequence [IP] [json_path] [sequence_name]
//
// NOTE: Only "joints" poses are executed. "xyz_pos" entries are currently skipped with a warning.
//       Gripper commands assume OPEN_GRIPPER/CLOSE_GRIPPER (adjust if your server uses other names).

#define _POSIX_C_SOURCE 200112L
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/tcp.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <ctype.h>

// ----------- JSON parsing: tiny built-in helpers (no external libs) -----------
// We implement just enough JSON parsing for your file structure:
// - top-level { "poses": {...}, "sequences": {...} }
// - poses[name]: { "type":"joints"/"xyz_pos", "unit":"deg", "values":[6 numbers] or object }
// - sequences[name]: array of strings or objects like {"wait": seconds} or {"gripper":"open|close"}
// This is not a full JSON parser; it’s strict and expects well-formed input like your file.

typedef struct {
  char *name;
  char type[16];   // "joints" or "xyz_pos"
  char unit[8];    // "deg"
  double joints[6];
  bool has_joints;
} Pose;

typedef struct {
  char **steps;        // step strings OR marker tokens like "__WAIT__:<seconds>" or "__GRIP__:<open/close>"
  size_t nsteps;
} Sequence;

typedef struct {
  Pose   *poses;
  size_t nposes;
  Sequence *seqs;
  char  **seq_names;
  size_t nseqs;
} Repo;

static char* slurp_file(const char* path, size_t* out_len){
  FILE* f = fopen(path,"rb");
  if(!f){ perror("fopen"); return NULL; }
  fseek(f,0,SEEK_END);
  long sz = ftell(f);
  if(sz < 0){ fclose(f); return NULL; }
  fseek(f,0,SEEK_SET);
  char* buf = (char*)malloc((size_t)sz+1);
  if(!buf){ fclose(f); return NULL; }
  size_t rd = fread(buf,1,(size_t)sz,f);
  fclose(f);
  buf[rd] = '\0';
  if(out_len) *out_len = rd;
  return buf;
}

// Simple helpers
static void skip_ws(const char **s){ while(isspace((unsigned char)**s)) (*s)++; }
static bool match(const char **s, const char* lit){
  size_t L=strlen(lit);
  if(strncmp(*s, lit, L)==0){ *s += L; return true; }
  return false;
}
static bool parse_string(const char **s, char *out, size_t outsz){
  skip_ws(s);
  if(**s!='"') return false;
  (*s)++;
  size_t i=0;
  while(**s && **s!='"'){
    char c = *(*s)++;
    if(c=='\\' && **s){ c=*(*s)++; } // very light unescape
    if(i+1<outsz) out[i++]=c;
  }
  if(**s!='"') return false;
  (*s)++;
  if(i<outsz) out[i]='\0';
  return true;
}
static bool parse_number(const char **s, double *out){
  skip_ws(s);
  char *end=NULL;
  double v = strtod(*s, &end);
  if(end==*s) return false;
  *out = v; *s = end; return true;
}

static bool parse_key(const char **s, const char* key){
  skip_ws(s);
  char tmp[64];
  if(!parse_string(s,tmp,sizeof(tmp))) return false;
  skip_ws(s);
  if(!match(s, ":")) return false;
  return strcmp(tmp,key)==0;
}

// Parse array of 6 numbers into arr[6]
static bool parse_six_numbers(const char **s, double arr[6]){
  skip_ws(s);
  if(!match(s,"[")) return false;
  for(int i=0;i<6;i++){
    if(i>0){ skip_ws(s); if(!match(s,",")) return false; }
    if(!parse_number(s,&arr[i])) return false;
  }
  skip_ws(s);
  if(!match(s,"]")) return false;
  return true;
}

// Extract a named pose object
static bool parse_pose_obj(const char **s, Pose* out){
  // expects: { "type": "...", "unit":"...", "values": [...] or {...} }
  skip_ws(s);
  if(!match(s,"{")) return false;
  bool got_type=false, got_unit=false, got_values=false;
  char tbuf[16]={0}, ubuf[8]={0};
  double joints[6]={0};
  bool has_joints=false;

  for(;;){
    skip_ws(s);
    if(match(s,"}")) break;
    if(got_type==false && parse_key(s,"type")){
      if(!parse_string(s,tbuf,sizeof(tbuf))) return false;
      got_type=true;
    } else if(got_unit==false && parse_key(s,"unit")){
      if(!parse_string(s,ubuf,sizeof(ubuf))) return false;
      got_unit=true;
    } else if(got_values==false && parse_key(s,"values")){
      skip_ws(s);
      if(match(s,"[")){ // joints array
        (*s)--; // step back to let parse_six_numbers see '['
        if(!parse_six_numbers(s,joints)) return false;
        has_joints=true;
        got_values=true;
      } else if(match(s,"{")){
        // xyz_pos object: skip until matching '}'
        int depth=1;
        while(**s && depth>0){
          if(**s=='{') depth++;
          else if(**s=='}') depth--;
          (*s)++;
        }
        got_values=true;
      } else {
        return false;
      }
    } else {
      // skip unknown key: value
      char dummy_key[64];
      if(!parse_string(s,dummy_key,sizeof(dummy_key))) return false;
      skip_ws(s);
      if(!match(s,":")) return false;
      // skip a JSON value roughly:
      int braces=0, brackets=0;
      skip_ws(s);
      if(**s=='{'){ braces=1; (*s)++; while(**s && braces){ if(**s=='{') braces++; else if(**s=='}') braces--; (*s)++; } }
      else if(**s=='['){ brackets=1; (*s)++; while(**s && brackets){ if(**s=='[') brackets++; else if(**s==']') brackets--; (*s)++; } }
      else if(**s=='"'){ (*s)++; while(**s && **s!='"'){ if(**s=='\\' && *(*s+1)) (*s)++; (*s)++; } if(**s=='"') (*s)++; }
      else { double dv; if(!parse_number(s,&dv)) return false; }
    }
    skip_ws(s);
    if(match(s,",")) continue;
    if(match(s,"}")) break;
  }

  if(!got_type || !got_unit || !got_values) return false;
  strncpy(out->type, tbuf, sizeof(out->type)-1);
  strncpy(out->unit, ubuf, sizeof(out->unit)-1);
  out->has_joints = has_joints;
  if(has_joints) memcpy(out->joints, joints, sizeof(out->joints));
  return true;
}

// Parse {"poses": {...}, "sequences": {...}}
static bool parse_repo(const char *json, Repo *repo){
  memset(repo,0,sizeof(*repo));
  const char *s=json;
  skip_ws(&s);
  if(!match(&s,"{")) return false;

  // We’ll scan for "poses" and "sequences"
  bool got_poses=false, got_seqs=false;
  while(*s){
    skip_ws(&s);
    if(match(&s,"}")) break;

    if(parse_key(&s,"poses")){
      skip_ws(&s);
      if(!match(&s,"{")) return false;

      // count entries first
      const char *p=s; size_t count=0;
      while(*p){
        skip_ws(&p);
        if(match(&p,"}")) break;
        // key:
        char name[128];
        if(!parse_string(&p,name,sizeof(name))) return false;
        skip_ws(&p); if(!match(&p,":")) return false;
        // value is an object; we’ll skip to matching '}'
        skip_ws(&p);
        const char *q=p;
        if(!match(&q,"{")) return false;
        int depth=1; while(*q && depth){ if(*q=='{') depth++; else if(*q=='}') depth--; q++; }
        if(depth) return false;
        p=q;
        count++;
        skip_ws(&p);
        if(match(&p,",")) continue;
        if(match(&p,"}")) break;
      }

      // allocate and parse for real
      repo->poses = (Pose*)calloc(count, sizeof(Pose));
      repo->nposes = count;

      for(size_t i=0;i<count;i++){
        skip_ws(&s);
        char *nm = NULL;
        // name
        char name[128]; if(!parse_string(&s,name,sizeof(name))) return false;
        nm = strdup(name);
        skip_ws(&s); if(!match(&s,":")) return false;
        Pose pz; memset(&pz,0,sizeof(pz));
        pz.name = nm;
        if(!parse_pose_obj(&s,&pz)){ free(nm); return false; }
        repo->poses[i]=pz;
        skip_ws(&s);
        if(match(&s,",")) continue;
        if(match(&s,"}")) break;
      }
      got_poses=true;

    } else if(parse_key(&s,"sequences")){
      skip_ws(&s);
      if(!match(&s,"{")) return false;

      // count entries
      const char *p=s; size_t count=0;
      while(*p){
        skip_ws(&p);
        if(match(&p,"}")) break;
        // key
        char nm[128]; if(!parse_string(&p,nm,sizeof(nm))) return false;
        skip_ws(&p); if(!match(&p,":")) return false;
        // value is array [...]
        skip_ws(&p);
        if(!match(&p,"[")) return false;
        int depth=1; while(*p && depth){ if(*p=='[') depth++; else if(*p==']') depth--; p++; }
        if(depth) return false;
        count++;
        skip_ws(&p);
        if(match(&p,",")) continue;
        if(match(&p,"}")) break;
      }

      repo->seqs = (Sequence*)calloc(count, sizeof(Sequence));
      repo->seq_names = (char**)calloc(count, sizeof(char*));
      repo->nseqs = count;

      // parse each sequence
      for(size_t i=0;i<count;i++){
        skip_ws(&s);
        char nm[128]; if(!parse_string(&s,nm,sizeof(nm))) return false;
        repo->seq_names[i] = strdup(nm);
        skip_ws(&s); if(!match(&s,":")) return false;

        // parse array
        skip_ws(&s); if(!match(&s,"[")) return false;
        // first pass: count items
        const char *p2=s; size_t cnt=0;
        int depth=1;
        while(*p2 && depth){
          skip_ws(&p2);
          if(*p2==']'){ depth--; p2++; break; }
          if(*p2=='"'){ // string step
            p2++; while(*p2 && *p2!='"'){ if(*p2=='\\' && *(p2+1)) p2+=2; else p2++; } if(*p2=='"') p2++;
            cnt++;
          } else if(*p2=='{'){
            // object: {"wait": number} or {"gripper":"open"}
            int d=1; p2++;
            while(*p2 && d){ if(*p2=='{') d++; else if(*p2=='}') d--; p2++; }
            cnt++;
          }
          skip_ws(&p2);
          if(*p2==',') p2++;
        }

        // second pass: actually parse items into strings
        Sequence *seq = &repo->seqs[i];
        seq->steps = (char**)calloc(cnt, sizeof(char*));
        seq->nsteps = cnt;

        size_t idx=0;
        while(*s){
          skip_ws(&s);
          if(match(&s,"]")) break;

          if(*s=='"'){
            char val[128]; parse_string(&s,val,sizeof(val));
            seq->steps[idx++] = strdup(val);
          } else if(*s=='{'){
            s++; skip_ws(&s);
            // key
            char key[32]; if(!parse_string(&s,key,sizeof(key))) return false;
            skip_ws(&s); if(!match(&s,":")) return false;
            if(strcmp(key,"wait")==0){
              double secs=0; if(!parse_number(&s,&secs)) return false;
              char tmp[64]; snprintf(tmp,sizeof(tmp),"__WAIT__:%g", secs);
              seq->steps[idx++] = strdup(tmp);
            } else if(strcmp(key,"gripper")==0){
              char val[32]; if(!parse_string(&s,val,sizeof(val))) return false;
              for(char *p=val; *p; ++p) *p=(char)tolower((unsigned char)*p);
              char tmp[64]; snprintf(tmp,sizeof(tmp),"__GRIP__:%s", val);
              seq->steps[idx++] = strdup(tmp);
            } else {
              // skip unknown object: consume until '}'
              int d=1; while(*s && d){ if(*s=='{') d++; else if(*s=='}') d--; s++; }
              // but record a stub
              seq->steps[idx++] = strdup("__UNKNOWN__");
              continue;
            }
            skip_ws(&s);
            if(!match(&s,"}")) return false;
          }
          skip_ws(&s);
          if(match(&s,",")) continue;
        }

        skip_ws(&s);
        if(match(&s,",")) continue;
        if(match(&s,"}")) break;
      }
      got_seqs=true;

    } else {
      // skip unknown key
      char dummy[64];
      if(!parse_string(&s,dummy,sizeof(dummy))) return false;
      skip_ws(&s); if(!match(&s,":")) return false;
      // skip a value
      skip_ws(&s);
      if(*s=='{'){ int d=1; s++; while(*s && d){ if(*s=='{') d++; else if(*s=='}') d--; s++; } }
      else if(*s=='['){ int d=1; s++; while(*s && d){ if(*s=='[') d++; else if(*s==']') d--; s++; } }
      else if(*s=='"'){ s++; while(*s && *s!='"'){ if(*s=='\\' && *(s+1)) s+=2; else s++; } if(*s=='"') s++; }
      else { double dv; if(!parse_number(&s,&dv)) return false; }
    }
    skip_ws(&s);
    if(match(&s,",")) continue;
    if(match(&s,"}")) break;
  }

  return got_poses && got_seqs;
}

static const Pose* find_pose(const Repo* repo, const char* name){
  for(size_t i=0;i<repo->nposes;i++){
    if(repo->poses[i].name && strcmp(repo->poses[i].name,name)==0) return &repo->poses[i];
  }
  return NULL;
}
static const Sequence* find_seq(const Repo* repo, const char* name){
  for(size_t i=0;i<repo->nseqs;i++){
    if(repo->seq_names[i] && strcmp(repo->seq_names[i],name)==0) return &repo->seqs[i];
  }
  return NULL;
}

// ----------- TCP client (uint16-LE length prefix + JSON) -----------
static int set_rcv_timeout(int s,int sec){ struct timeval tv={.tv_sec=sec,.tv_usec=0}; return setsockopt(s,SOL_SOCKET,SO_RCVTIMEO,&tv,sizeof(tv)); }
static int set_snd_timeout(int s,int sec){ struct timeval tv={.tv_sec=sec,.tv_usec=0}; return setsockopt(s,SOL_SOCKET,SO_SNDTIMEO,&tv,sizeof(tv)); }

static int connect_tcp(const char *ip, int port){
  int s = socket(AF_INET,SOCK_STREAM,0);
  if(s<0){ perror("socket"); return -1; }
  int one=1; setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
  set_rcv_timeout(s, 20);
  set_snd_timeout(s, 10);
  struct sockaddr_in a; memset(&a,0,sizeof(a));
  a.sin_family=AF_INET; a.sin_port=htons(port);
  if(inet_pton(AF_INET, ip, &a.sin_addr)!=1){ perror("inet_pton"); close(s); return -1; }
  if(connect(s,(struct sockaddr*)&a,sizeof(a))<0){ perror("connect"); close(s); return -1; }
  return s;
}
static int read_full(int s, void *buf, size_t n){
  unsigned char *p=(unsigned char*)buf;
  while(n){
    ssize_t r=recv(s,p,n,0);
    if(r==0){ errno=ECONNRESET; return -1; }
    if(r<0){ if(errno==EINTR) continue; perror("recv"); return -1; }
    p+=r; n-= (size_t)r;
  }
  return 0;
}
static int send_frame(int s, const char* json){
  size_t len=strlen(json);
  if(len>0xFFFFu){ fprintf(stderr,"JSON too long\n"); return -1; }
  unsigned char hdr[2]; unsigned n=(unsigned)len;
  hdr[0]=(unsigned char)(n & 0xFF);
  hdr[1]=(unsigned char)((n>>8)&0xFF);
  struct iovec iov[2]={{.iov_base=hdr,.iov_len=2},{.iov_base=(void*)json,.iov_len=len}};
  struct msghdr msg={.msg_iov=iov,.msg_iovlen=2};
  ssize_t w=sendmsg(s,&msg,0);
  if(w<0){ perror("sendmsg"); return -1; }
  if((size_t)w != len+2){ fprintf(stderr,"short send\n"); return -1; }
  return 0;
}
static int recv_frame(int s, char **out_json){
  unsigned char hdr[2];
  if(read_full(s,hdr,2)<0) return -1;
  unsigned n = (unsigned)hdr[0] | ((unsigned)hdr[1]<<8);
  if(n==0){ fprintf(stderr,"empty frame\n"); return -1; }
  char *buf=(char*)malloc(n+1); if(!buf) return -1;
  if(read_full(s,buf,n)<0){ free(buf); return -1; }
  buf[n]='\0'; *out_json=buf; return 0;
}
static int cmd_with_timeout(int s, const char* cmd, const char* params_json, int timeout_s, const char* tag){
  if(timeout_s>0) set_rcv_timeout(s, timeout_s);
  char req[1024];
  int n = snprintf(req,sizeof(req), "{\"command\": \"%s\", \"param_list\": %s}", cmd, params_json?params_json:"[]");
  if(n<=0 || (size_t)n>=sizeof(req)){ fprintf(stderr,"req too long\n"); return -1; }
  if(send_frame(s,req)<0) return -1;
  char *rep=NULL;
  if(recv_frame(s,&rep)<0) return -1;
  bool ok = strstr(rep,"\"status\": \"OK\"")!=NULL;
  printf("%s -> %s\n", tag?tag:cmd, rep);
  free(rep);
  set_rcv_timeout(s, 20);
  return ok?0:-1;
}
static int handshake(int s){ return cmd_with_timeout(s,"HANDSHAKE","[\"1.2.1\"]",10,"HANDSHAKE"); }
static int calibrate_auto(int s){ return cmd_with_timeout(s,"CALIBRATE","[\"AUTO\"]",120,"CALIBRATE"); }
static int set_learning(int s, bool en){ return cmd_with_timeout(s,"SET_LEARNING_MODE", en?"[\"TRUE\"]":"[\"FALSE\"]", 10, "SET_LEARNING_MODE"); }
static int open_gripper(int s){ return cmd_with_timeout(s,"OPEN_GRIPPER","[]",10,"OPEN_GRIPPER"); }
static int close_gripper(int s){ return cmd_with_timeout(s,"CLOSE_GRIPPER","[]",10,"CLOSE_GRIPPER"); }
static int move_joints_rad(int s, const double j[6]){
  // MOVE_JOINTS [j0..j5]
  char p[256];
  int n=snprintf(p,sizeof(p),"[%.15g,%.15g,%.15g,%.15g,%.15g,%.15g]", j[0],j[1],j[2],j[3],j[4],j[5]);
  if(n<=0 || (size_t)n>=sizeof(p)) return -1;
  return cmd_with_timeout(s,"MOVE_JOINTS",p,60,"MOVE_JOINTS");
}

// deg -> rad
static double d2r(double d){ return d * 3.14159265358979323846 / 180.0; }

static int run_sequence_on_robot(int s, const Repo* repo, const Sequence* seq){
  // Ensure ready
  if(handshake(s)<0) return -1;
  if(calibrate_auto(s)<0) return -1;
  if(set_learning(s,false)<0) return -1; // disable learning mode to allow motion

  for(size_t i=0;i<seq->nsteps;i++){
    const char* step = seq->steps[i];
    if(!step) continue;

    if(strncmp(step,"__WAIT__:",9)==0){
      double secs = atof(step+9);
      printf("WAIT %.3f s\n", secs);
      struct timespec ts; ts.tv_sec=(time_t)secs; ts.tv_nsec=(long)((secs-ts.tv_sec)*1e9);
      nanosleep(&ts,NULL);
      continue;
    }
    if(strncmp(step,"__GRIP__:",9)==0){
      const char* what = step+9;
      if(strcmp(what,"open")==0){ if(open_gripper(s)<0) return -1; }
      else if(strcmp(what,"close")==0){ if(close_gripper(s)<0) return -1; }
      else { printf("WARN: unknown gripper cmd '%s' (skipped)\n", what); }
      continue;
    }
    if(strcmp(step,"__UNKNOWN__")==0){
      printf("WARN: unknown step object skipped\n");
      continue;
    }

    // pose name
    const Pose* p = find_pose(repo, step);
    if(!p){ printf("WARN: pose '%s' not found (skipped)\n", step); continue; }
    if(strcmp(p->type,"joints")==0 && p->has_joints){
      double r[6];
      // assume unit "deg"; convert to radians
      for(int k=0;k<6;k++) r[k]=d2r(p->joints[k]);
      if(move_joints_rad(s,r)<0) return -1;
    } else {
      printf("NOTE: pose '%s' has type '%s' — not supported yet, skipping.\n", p->name, p->type);
    }
  }

  // Re-enable learning mode at end (optional)
  set_learning(s,true);
  return 0;
}

static void free_repo(Repo *repo){
  if(!repo) return;
  for(size_t i=0;i<repo->nposes;i++) free(repo->poses[i].name);
  free(repo->poses);
  for(size_t i=0;i<repo->nseqs;i++){
    for(size_t j=0;j<repo->seqs[i].nsteps;j++) free(repo->seqs[i].steps[j]);
    free(repo->seqs[i].steps);
    free(repo->seq_names[i]);
  }
  free(repo->seqs);
  free(repo->seq_names);
}

int main(int argc, char**argv){
  const char* ip   = (argc>1)? argv[1] : "192.168.100.205";
  const char* jpth = (argc>2)? argv[2] : "sequences.json";
  const char* sname= (argc>3)? argv[3] : "say-hi";

  size_t jsz=0;
  char* jtxt = slurp_file(jpth,&jsz);
  if(!jtxt){ fprintf(stderr,"Failed to read %s\n", jpth); return 1; }

  Repo repo;
  if(!parse_repo(jtxt,&repo)){ fprintf(stderr,"Failed to parse %s\n", jpth); free(jtxt); return 1; }
  free(jtxt);

  const Sequence* seq = find_seq(&repo,sname);
  if(!seq){ fprintf(stderr,"Sequence '%s' not found in %s\n", sname, jpth); free_repo(&repo); return 1; }

  printf("Connecting to Niryo at %s:%d ...\n", ip, 40001);
  int s = connect_tcp(ip, 40001);
  if(s<0){ free_repo(&repo); return 1; }

  int rc = run_sequence_on_robot(s,&repo,seq);
  if(rc==0) printf("Sequence '%s' completed.\n", sname);
  else      fprintf(stderr,"Sequence '%s' failed.\n", sname);

  close(s);
  free_repo(&repo);
  return (rc==0)?0:2;
}

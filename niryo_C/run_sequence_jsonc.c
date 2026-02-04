// run_sequence_jsonc.c
// Niryo Ned2 TCP (uint16-LE length-prefixed JSON), sequencing from sequences.json via json-c.
// Build: gcc -O2 -Wall -o run_sequence_jsonc run_sequence_jsonc.c -ljson-c
// Usage: ./run_sequence_jsonc [IP] [json_path] [sequence_name]

#define _POSIX_C_SOURCE 200112L
#include <arpa/inet.h>
#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <netinet/tcp.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <json-c/json.h>

#define NIRYO_PORT 40001

// ---------- tiny utils ----------
static char* slurp(const char* path, size_t* len_out){
  FILE* f = fopen(path, "rb"); if(!f){ perror("fopen"); return NULL; }
  fseek(f, 0, SEEK_END); long n = ftell(f); if(n<0){ fclose(f); return NULL; }
  fseek(f, 0, SEEK_SET);
  char* buf = (char*)malloc((size_t)n+1); if(!buf){ fclose(f); return NULL; }
  size_t r = fread(buf,1,(size_t)n,f); fclose(f); buf[r]='\0'; if(len_out) *len_out=r; return buf;
}
static double d2r(double d){ return d * 3.14159265358979323846 / 180.0; }
static void sleep_seconds(double secs){
  if (secs <= 0) return;
  struct timespec ts; ts.tv_sec = (time_t)secs; ts.tv_nsec = (long)((secs - ts.tv_sec) * 1e9);
  if (ts.tv_nsec < 0) ts.tv_nsec = 0;
  nanosleep(&ts, NULL);
}

// ---------- TCP wire (uint16-LE len prefix + JSON) ----------
static int set_timeout(int s, int rcv_sec, int snd_sec){
  struct timeval r = { .tv_sec = rcv_sec, .tv_usec = 0 };
  struct timeval w = { .tv_sec = snd_sec, .tv_usec = 0 };
  setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &r, sizeof(r));
  setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &w, sizeof(w));
  int one=1; setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
  return 0;
}
static int dial(const char* ip){
  int s = socket(AF_INET, SOCK_STREAM, 0);
  if(s<0){ perror("socket"); return -1; }
  set_timeout(s, 20, 10);
  struct sockaddr_in a; memset(&a,0,sizeof(a));
  a.sin_family = AF_INET; a.sin_port = htons(NIRYO_PORT);
  if(inet_pton(AF_INET, ip, &a.sin_addr) != 1){ perror("inet_pton"); close(s); return -1; }
  if(connect(s, (struct sockaddr*)&a, sizeof(a)) < 0){ perror("connect"); close(s); return -1; }
  return s;
}
static int read_full(int s, void* buf, size_t need){
  unsigned char* p = (unsigned char*)buf;
  while(need){
    ssize_t n = recv(s, p, need, 0);
    if(n == 0){ errno = ECONNRESET; return -1; }
    if(n < 0){ if(errno==EINTR) continue; perror("recv"); return -1; }
    p += n; need -= (size_t)n;
  }
  return 0;
}
static int send_frame(int s, const char* json){
  size_t len = strlen(json);
  if(len > 0xFFFFu){ fprintf(stderr,"JSON too long\n"); return -1; }
  unsigned char hdr[2] = { (unsigned char)(len & 0xFF), (unsigned char)((len>>8)&0xFF) };
  struct iovec iov[2] = { { .iov_base=hdr, .iov_len=2 }, { .iov_base=(void*)json, .iov_len=len } };
  struct msghdr msg = { .msg_iov=iov, .msg_iovlen=2 };
  ssize_t w = sendmsg(s, &msg, 0);
  if(w < 0){ perror("sendmsg"); return -1; }
  if((size_t)w != len+2){ fprintf(stderr,"short send\n"); return -1; }
  return 0;
}
static int recv_frame(int s, char** out_json){
  unsigned char hdr[2];
  if(read_full(s, hdr, 2) < 0) return -1;
  unsigned n = (unsigned)hdr[0] | ((unsigned)hdr[1] << 8);
  if(n == 0){ fprintf(stderr,"empty frame\n"); return -1; }
  char* buf = (char*)malloc(n+1); if(!buf) return -1;
  if(read_full(s, buf, n) < 0){ free(buf); return -1; }
  buf[n] = '\0'; *out_json = buf; return 0;
}
static void set_rcv_timeout(int s, int sec){
  struct timeval tv = { .tv_sec = sec, .tv_usec = 0 };
  setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

static int cmd_with_timeout(int s, const char* cmd, const char* params_json, int timeout_s, const char* tag){
  if(timeout_s>0) set_rcv_timeout(s, timeout_s);
  char req[1024];
  int n = snprintf(req, sizeof(req), "{\"command\":\"%s\",\"param_list\":%s}", cmd, params_json?params_json:"[]");
  if(n<=0 || (size_t)n>=sizeof(req)){ fprintf(stderr,"req too long\n"); return -1; }
  if(send_frame(s, req) < 0) return -1;
  char* reply = NULL;
  if(recv_frame(s, &reply) < 0) return -1;
  bool ok = strstr(reply, "\"status\": \"OK\"") != NULL;
  printf("%s -> %s\n", tag?tag:cmd, reply);
  free(reply);
  set_rcv_timeout(s, 20);
  return ok ? 0 : -1;
}
static int HANDSHAKE(int s){ return cmd_with_timeout(s,"HANDSHAKE","[\"1.2.1\"]",10,"HANDSHAKE"); }
static int CALIBRATE(int s){ return cmd_with_timeout(s,"CALIBRATE","[\"AUTO\"]",120,"CALIBRATE"); }
static int SET_LEARNING(int s, bool en){ return cmd_with_timeout(s,"SET_LEARNING_MODE", en?"[\"TRUE\"]":"[\"FALSE\"]", 10, "SET_LEARNING_MODE"); }
static int OPEN_GRIPPER(int s){ return cmd_with_timeout(s,"OPEN_GRIPPER","[]",10,"OPEN_GRIPPER"); }
static int CLOSE_GRIPPER(int s){ return cmd_with_timeout(s,"CLOSE_GRIPPER","[]",10,"CLOSE_GRIPPER"); }
static int MOVE_JOINTS_RAD(int s, const double j[6]){
  char p[256];
  int n = snprintf(p, sizeof(p), "[%.15g,%.15g,%.15g,%.15g,%.15g,%.15g]", j[0],j[1],j[2],j[3],j[4],j[5]);
  if(n<=0 || (size_t)n>=sizeof(p)) return -1;
  return cmd_with_timeout(s,"MOVE_JOINTS",p,60,"MOVE_JOINTS");
}

// ---------- JSON-c helpers ----------
static json_object* jget(json_object* obj, const char* key, enum json_type expect){
  json_object* v=NULL;
  if(!json_object_object_get_ex(obj, key, &v)) return NULL;
  if(expect != json_type_null && json_object_get_type(v) != expect) return NULL;
  return v;
}
static const char* jstr(json_object* obj){
  return obj ? json_object_get_string(obj) : NULL;
}
static double jnum(json_object* obj){
  return obj ? json_object_get_double(obj) : 0.0;
}

static bool pose_to_radians(json_object* pose_obj, double out[6]){
  // Expect: {"type":"joints","unit":"deg","values":[6 numbers]}
  json_object* jtype = jget(pose_obj, "type", json_type_string);
  json_object* junit = jget(pose_obj, "unit", json_type_string);
  json_object* jvals = jget(pose_obj, "values", json_type_array);
  if(!jtype || !junit || !jvals) return false;
  const char* type = jstr(jtype);
  const char* unit = jstr(junit);
  if(strcmp(type,"joints")!=0) return false;
  if(json_object_array_length(jvals) != 6) return false;

  for(int i=0;i<6;i++){
    json_object* x = json_object_array_get_idx(jvals, i);
    out[i] = d2r(json_object_get_double(x));  // unit "deg" expected
  }
  (void)unit; // if in the future we add "rad", adjust here
  return true;
}

static int run_sequence(int s, json_object* root, const char* seq_name){
  json_object* poses = jget(root, "poses", json_type_object);
  json_object* seqs  = jget(root, "sequences", json_type_object);
  if(!poses || !seqs){ fprintf(stderr,"JSON missing 'poses' or 'sequences'\n"); return -1; }

  json_object* seq = jget(seqs, seq_name, json_type_array);
  if(!seq){ fprintf(stderr,"Sequence '%s' not found\n", seq_name); return -1; }

  // Prepare robot
  if(HANDSHAKE(s)<0) return -1;
  if(CALIBRATE(s)<0) return -1;
  if(SET_LEARNING(s,false)<0) return -1;

  size_t n = json_object_array_length(seq);
  for(size_t i=0;i<n;i++){
    json_object* step = json_object_array_get_idx(seq, i);
    enum json_type t = json_object_get_type(step);

    if(t == json_type_string){
      // Pose by name
      const char* name = jstr(step);
      json_object* pose_obj=NULL;
      if(!json_object_object_get_ex(poses, name, &pose_obj) || !pose_obj){
        printf("WARN: pose '%s' not found, skipping\n", name);
        continue;
      }
      double rad[6];
      if(pose_to_radians(pose_obj, rad)){
        if(MOVE_JOINTS_RAD(s, rad) < 0) return -1;
      }else{
        printf("NOTE: pose '%s' not 'joints/deg'; skipped\n", name);
      }

    } else if(t == json_type_object){
      // Wait / gripper object
      json_object* jwait = jget(step, "wait", json_type_double);
      if(jwait){
        double secs = jnum(jwait);
        printf("WAIT %.3f s\n", secs);
        sleep_seconds(secs);
        continue;
      }
      json_object* jgrip = jget(step, "gripper", json_type_string);
      if(jgrip){
        const char* g = jstr(jgrip);
        if(!g){ /* noop */ }
        else if(strcasecmp(g,"open")==0){ if(OPEN_GRIPPER(s)<0) return -1; }
        else if(strcasecmp(g,"close")==0){ if(CLOSE_GRIPPER(s)<0) return -1; }
        else printf("WARN: unknown gripper '%s'\n", g);
        continue;
      }
      printf("NOTE: unsupported step object, skipped\n");
    } else {
      printf("NOTE: unsupported step type, skipped\n");
    }
  }

  // Optional: re-enable learning
  //SET_LEARNING(s,true);
  return 0;
}

int main(int argc, char** argv){
  const char* ip   = (argc>1)? argv[1] : "192.168.100.205";
  const char* jpth = (argc>2)? argv[2] : "sequences.json";
  const char* sname= (argc>3)? argv[3] : "say-hi";

  size_t jsz=0; char* txt = slurp(jpth, &jsz);
  if(!txt){ fprintf(stderr,"Failed to read %s\n", jpth); return 1; }

  json_object* root = json_tokener_parse(txt);
  free(txt);
  if(!root){ fprintf(stderr,"Failed to parse %s as JSON\n", jpth); return 1; }

  printf("Connecting to Niryo at %s:%d ...\n", ip, NIRYO_PORT);
  int s = dial(ip);
  if(s<0){ json_object_put(root); return 2; }

  int rc = run_sequence(s, root, sname);
  close(s);
  json_object_put(root);

  if(rc==0) printf("Sequence '%s' completed.\n", sname);
  else fprintf(stderr,"Sequence '%s' failed.\n", sname);
  return rc==0 ? 0 : 3;
}

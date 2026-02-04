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
#include <sys/uio.h>
#include <time.h>
#include <unistd.h>
#include <json-c/json.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define NIRYO_PORT 40001

// ---------- tiny utils ----------
static char* slurp(const char* path, size_t* len_out){
  FILE* f = fopen(path, "rb"); if(!f){ perror("fopen"); return NULL; }
  if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return NULL; }
  long n = ftell(f); if(n<0){ fclose(f); return NULL; }
  if (fseek(f, 0, SEEK_SET) != 0) { fclose(f); return NULL; }
  char* buf = (char*)malloc((size_t)n+1); if(!buf){ fclose(f); return NULL; }
  size_t r = fread(buf,1,(size_t)n,f); fclose(f); buf[r]='\0'; if(len_out) *len_out=r; return buf;
}
static double d2r(double d){ return d * M_PI / 180.0; }
static void sleep_seconds(double secs){
  if (secs <= 0) return;
  struct timespec ts;
  ts.tv_sec = (time_t)secs;
  ts.tv_nsec = (long)((secs - (double)ts.tv_sec) * 1e9);
  if (ts.tv_nsec < 0) ts.tv_nsec = 0;
  nanosleep(&ts, NULL);
}

// ---------- TCP wire (uint16-LE len prefix + JSON) ----------
static int set_timeout(int s, int rcv_sec, int snd_sec){
  struct timeval r = { .tv_sec = rcv_sec, .tv_usec = 0 };
  struct timeval w = { .tv_sec = snd_sec, .tv_usec = 0 };
  (void)setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &r, sizeof(r));
  (void)setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &w, sizeof(w));
  int one=1; (void)setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
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
    if(n < 0){
      if(errno==EINTR) continue;
      // timeout is expected sometimes; still report
      perror("recv"); return -1;
    }
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
  (void)setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
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
static int PULL_AIR(int s){ return cmd_with_timeout(s,"PULL_AIR_VACUUM_PUMP","[]",10,"PULL_AIR_VACUUM_PUMP"); }
static int PUSH_AIR(int s){ return cmd_with_timeout(s,"PUSH_AIR_VACUUM_PUMP","[]",10,"PUSH_AIR_VACUUM_PUMP"); }
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
static double jnum_any(json_object* obj, bool* ok){
  if(!obj){ if(ok) *ok=false; return 0.0; }
  enum json_type t = json_object_get_type(obj);
  if(t == json_type_double || t == json_type_int){
    if(ok) *ok=true;
    return json_object_get_double(obj);
  }
  if(ok) *ok=false;
  return 0.0;
}

// Accept: {"type":"joints","values":[...]} with optional "unit":"rad"/"deg"
// Also accept legacy "joints_rad"/"joints_deg".
static bool pose_to_radians(json_object* pose_obj, double out[6]){
  if(!pose_obj) return false;

  const char* type = NULL;
  json_object* jtype = jget(pose_obj, "type", json_type_string);
  if (jtype) type = jstr(jtype);

  const char* unit = "rad"; // default to radians
  json_object* junit = jget(pose_obj, "unit", json_type_string);
  if (junit) unit = jstr(junit);

  json_object* jvals = jget(pose_obj, "values", json_type_array);
  if(!jvals || json_object_array_length(jvals) != 6) return false;

  bool is_joints = false;
  bool vals_are_deg = false;

  if (type) {
    if (strcmp(type, "joints") == 0) is_joints = true;
    else if (strcmp(type, "joints_deg") == 0) { is_joints = true; vals_are_deg = true; }
    else if (strcmp(type, "joints_rad") == 0) { is_joints = true; vals_are_deg = false; }
    else return false;
  } else {
    // Be permissive if shape is correct
    is_joints = true;
  }

  if (strcmp(unit, "deg") == 0) vals_are_deg = true;
  else if (strcmp(unit, "rad") == 0) /* keep */ ;
  // anything else: keep default (rad)

  for(int i=0;i<6;i++){
    json_object* x = json_object_array_get_idx(jvals, i);
    if(!x) return false;
    bool ok=false;
    double v = jnum_any(x, &ok);
    if(!ok) return false;
    out[i] = vals_are_deg ? d2r(v) : v;
  }
  return is_joints;
}

// ---------- Execution ----------
static int step_pump(int s, const char *action){
  if(!action) return 0;
  // Map both vacuum semantics and gripper semantics onto gripper as a safe fallback.
  if(strcasecmp(action,"grasp")==0 || strcasecmp(action,"pull")==0){
    //return CLOSE_GRIPPER(s);
    return PULL_AIR(s);
  } else if(strcasecmp(action,"release")==0 || strcasecmp(action,"push")==0){
    //return OPEN_GRIPPER(s);
    return PUSH_AIR(s);
  } else {
    printf("NOTE: unsupported pump action '%s', skipped\n", action);
    return 0;
  }
}

static int run_sequence(int s, json_object* root, const char* seq_name){
  //printf("Running sequence '%s' ...\n", seq_name);
  json_object* poses = jget(root, "poses", json_type_object);
  json_object* seqs  = jget(root, "sequences", json_type_object);
  if(!poses || !seqs){ fprintf(stderr,"JSON missing 'poses' or 'sequences'\n"); return -1; }

  json_object* seq = jget(seqs, seq_name, json_type_array);
  if(!seq){ fprintf(stderr,"Sequence '%s' not found\n", seq_name); return -1; }

  // Prepare robot
  if(HANDSHAKE(s)<0) return -1;
  if(CALIBRATE(s)<0) return -1;
  if(SET_LEARNING(s,false)<0) return -1;
  //printf("Robot calibrated and ready.\n");
  size_t n = json_object_array_length(seq);
  for(size_t i=0;i<n;i++){
    //printf("Step %zu/%zu ...\n", i+1, n);
    json_object* step = json_object_array_get_idx(seq, i);
    if(!step){ printf("NOTE: null step, skipped\n"); continue; }
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
        //printf("Moving to pose '%s' ...\n", name);
        if(MOVE_JOINTS_RAD(s, rad) < 0) return -1;
        //printf("Reached pose '%s'.\n", name);
      }else{
        printf("NOTE: pose '%s' invalid; skipped\n", name);
      }

    } else if(t == json_type_object){
      // wait / gripper / pump
      //printf("Object step ...\n");
      json_object *jwait=NULL, *jgrip=NULL, *jpump=NULL;

      if(json_object_object_get_ex(step, "wait", &jwait)){
        bool ok=false; double secs = jnum_any(jwait, &ok);
        if(ok){
          printf("WAIT %.3f s\n", secs);
          sleep_seconds(secs);
          continue;
        }
      }
      if(json_object_object_get_ex(step, "gripper", &jgrip) && json_object_is_type(jgrip, json_type_string)){
        const char* g = jstr(jgrip);
        if(g){
          if(strcasecmp(g,"open")==0){ if(OPEN_GRIPPER(s)<0) return -1; }
          else if(strcasecmp(g,"close")==0){ if(CLOSE_GRIPPER(s)<0) return -1; }
          else printf("WARN: unknown gripper '%s'\n", g);
        }
        continue;
      }
      if(json_object_object_get_ex(step, "pump", &jpump) && json_object_is_type(jpump, json_type_string)){
        //printf("PUMP action ...\n");
        const char* a = jstr(jpump);
        if(step_pump(s, a) < 0) return -1;
        continue;
      }

      printf("NOTE: unsupported step object, skipped\n");
    } else {
      printf("NOTE: unsupported step type, skipped\n");
    }
  }

  // Optional: re-enable learning
  // (void)SET_LEARNING(s,true);
  return 0;
}

/* int main(int argc, char** argv){
  const char* ip   = (argc>1)? argv[1] : "192.168.100.190";
  const char* jpth = (argc>2)? argv[2] : "loop.json";
  const char* sname= (argc>3)? argv[3] : "loop-pp";

  size_t jsz=0; char* txt = slurp(jpth, &jsz);
  if(!txt){ fprintf(stderr,"Failed to read %s\n", jpth); return 1; }

  json_object* root = json_tokener_parse(txt);
  free(txt);
  if(!root){ fprintf(stderr,"Failed to parse %s as JSON\n", jpth); return 1; }

  printf("Connecting to Niryo at %s:%d ...\n", ip, NIRYO_PORT);
  int s = dial(ip);
  if(s<0){ json_object_put(root); return 2; }
  //printf("Running sequence '%s' ...\n", sname);
  int i;
  int rc;
  for (i=0; i<500; i++) {
    rc = run_sequence(s, root, sname);
    if (rc == -1) break;
    printf("Sequence '%s' completed %d times.\n", sname, i+1);
  }
  //int rc = run_sequence(s, root, sname);
  close(s);
  json_object_put(root);

  if(rc==0) printf("Sequence '%s' completed.\n", sname);
  else fprintf(stderr,"Sequence '%s' failed.\n", sname);
  return rc==0 ? 0 : 3;
}
 */

 int main(int argc, char** argv){
  const char* ip   = (argc>1)? argv[1] : "192.168.100.192";
  const char* jpth = (argc>2)? argv[2] : "loop.json";
  const char* sname= (argc>3)? argv[3] : "loop-pp";

  size_t jsz=0; char* txt = slurp(jpth, &jsz);
  if(!txt){ fprintf(stderr,"Failed to read %s\n", jpth); return 1; }

  json_object* root = json_tokener_parse(txt);
  free(txt);
  if(!root){ fprintf(stderr,"Failed to parse %s as JSON\n", jpth); return 1; }

  printf("Connecting to Niryo at %s:%d ...\n", ip, NIRYO_PORT);
  int s = dial(ip);
  if(s<0){ json_object_put(root); return 2; }

  // ---- run for up to 1 hour from now ----
  const double LIMIT_SEC = 3600.0; // 1 hour
  struct timespec t0, t1;
  clock_gettime(CLOCK_MONOTONIC, &t0);

  int count = 0;
  int rc = 0; // assume success unless run_sequence signals failure

  for(;;){
    // Stop if we've already reached/passed the time limit before starting a new cycle
    clock_gettime(CLOCK_MONOTONIC, &t1);
    double elapsed = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec)/1e9;
    if (elapsed >= LIMIT_SEC) {
      break;
    }

    rc = run_sequence(s, root, sname);
    if (rc == -1) {
      // Stop on failure
      break;
    }

    // One more successful cycle completed
    count++;

    // Recompute elapsed and print progress
    clock_gettime(CLOCK_MONOTONIC, &t1);
    elapsed = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec)/1e9;

    int eh = (int)(elapsed / 3600.0);
    int em = (int)((elapsed - eh*3600.0) / 60.0);
    double es = elapsed - eh*3600.0 - em*60.0;

    // Elapsed printed as HH:MM:SS.sss
    printf("Sequence '%s' completed %d time%s. Elapsed: %02d:%02d:%06.3f\n",
           sname, count, (count==1?"":"s"), eh, em, es);
    fflush(stdout);
  }

  close(s);
  json_object_put(root);

  if(rc==0) {
    printf("Sequence '%s' finished. Total cycles: %d\n", sname, count);
    return 0;
  } else {
    fprintf(stderr,"Sequence '%s' failed after %d cycle%s.\n",
            sname, count, (count==1?"":"s"));
    return 3;
  }
}

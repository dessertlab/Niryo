// niryo_tcp_demo.c — uint16 LE length prefix + per-command timeouts
// Build: gcc -O2 -Wall niryo_tcp_demo.c -o niryo_tcp_demo
// Run:   ./niryo_tcp_demo 192.168.100.205

#define _POSIX_C_SOURCE 200112L
#include <arpa/inet.h>
#include <errno.h>
#include <inttypes.h>
#include <netinet/tcp.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#define NIRYO_PORT 40001

static int set_rcv_timeout(int s, int sec){
  struct timeval tv = { .tv_sec = sec, .tv_usec = 0 };
  return setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}
static int set_snd_timeout(int s, int sec){
  struct timeval tv = { .tv_sec = sec, .tv_usec = 0 };
  return setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
}
static int set_sockopts_base(int s){
  int one = 1; setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
  set_rcv_timeout(s, 20);
  set_snd_timeout(s, 10);
  return 0;
}

static int connect_tcp(const char *ip){
  int s = socket(AF_INET, SOCK_STREAM, 0);
  if (s < 0) { perror("socket"); return -1; }
  set_sockopts_base(s);
  struct sockaddr_in a; memset(&a, 0, sizeof(a));
  a.sin_family = AF_INET; a.sin_port = htons(NIRYO_PORT);
  if (inet_pton(AF_INET, ip, &a.sin_addr) != 1) { perror("inet_pton"); close(s); return -1; }
  if (connect(s, (struct sockaddr *)&a, sizeof(a)) < 0) { perror("connect"); close(s); return -1; }
  return s;
}

static int read_full(int s, void *buf, size_t need){
  unsigned char *p = (unsigned char*)buf;
  while (need){
    ssize_t r = recv(s, p, need, 0);
    if (r == 0){ errno = ECONNRESET; return -1; }
    if (r < 0){
      if (errno == EINTR) continue;
      perror("recv");
      return -1;
    }
    p += r; need -= (size_t)r;
  }
  return 0;
}

static int send_frame(int s, const char *json){
  size_t len = strlen(json);
  if (len > 0xFFFFu){ fprintf(stderr, "JSON too long\n"); return -1; }
  unsigned char hdr[2];
  uint16_t n = (uint16_t)len;                 // uint16 little-endian
  hdr[0] = (unsigned char)(n & 0xFF);
  hdr[1] = (unsigned char)((n >> 8) & 0xFF);

  struct iovec iov[2] = {
    { .iov_base = hdr,        .iov_len = 2   },
    { .iov_base = (void*)json,.iov_len = len }
  };
  struct msghdr msg = { .msg_iov = iov, .msg_iovlen = 2 };
  ssize_t w = sendmsg(s, &msg, 0);
  if (w < 0){ perror("sendmsg"); return -1; }
  if ((size_t)w != len + 2){ fprintf(stderr, "short send (%zd/%zu)\n", w, len+2); return -1; }
  return 0;
}

static int recv_frame(int s, char **out_json){
  unsigned char hdr[2];
  if (read_full(s, hdr, 2) < 0) return -1;
  uint16_t n = (uint16_t)hdr[0] | ((uint16_t)hdr[1] << 8);  // LE
  if (n == 0){ fprintf(stderr, "empty frame\n"); return -1; }
  char *buf = (char*)malloc((size_t)n + 1);
  if (!buf) return -1;
  if (read_full(s, buf, n) < 0){ free(buf); return -1; }
  buf[n] = '\0';
  *out_json = buf;
  return 0;
}

static int niryo_cmd_with_timeout(int s, const char *cmd, const char *params_json, int timeout_s){
  // per-command receive timeout
  if (timeout_s > 0) set_rcv_timeout(s, timeout_s);

  char req[1024];
  int n = snprintf(req, sizeof(req),
    "{\"command\": \"%s\", \"param_list\": %s}",
    cmd, params_json ? params_json : "[]");
  if (n <= 0 || (size_t)n >= sizeof(req)){ fprintf(stderr,"request too long\n"); return -1; }

  if (send_frame(s, req) < 0) return -1;

  char *reply = NULL;
  if (recv_frame(s, &reply) < 0) return -1;

  bool ok = strstr(reply, "\"status\": \"OK\"") != NULL;
  printf("%s -> %s\n", cmd, reply);
  free(reply);

  // restore default (optional)
  set_rcv_timeout(s, 20);
  return ok ? 0 : -1;
}

static int cmd_handshake(int s)                 { return niryo_cmd_with_timeout(s, "HANDSHAKE", "[\"1.2.1\"]", 10); }
static int cmd_calibrate_auto(int s)            { return niryo_cmd_with_timeout(s, "CALIBRATE", "[\"AUTO\"]", 120); }
static int cmd_set_learning_mode(int s, bool e) { return niryo_cmd_with_timeout(s, "SET_LEARNING_MODE", e ? "[\"TRUE\"]":"[\"FALSE\"]", 10); }
static int cmd_move_joints(int s, const double j[6]){
  char p[256];
  int n = snprintf(p, sizeof(p), "[%.10g,%.10g,%.10g,%.10g,%.10g,%.10g]", j[0],j[1],j[2],j[3],j[4],j[5]);
  if (n <= 0 || (size_t)n >= sizeof(p)) return -1;
  return niryo_cmd_with_timeout(s, "MOVE_JOINTS", p, 60);
}

int main(int argc, char **argv){
  const char *ip = (argc > 1) ? argv[1] : "192.168.100.205";
  printf("Connecting to Niryo at %s:%d ...\n", ip, NIRYO_PORT);
  int s = connect_tcp(ip);
  if (s < 0) return 1;

  if (cmd_handshake(s)       < 0) { fprintf(stderr,"Handshake failed.\n"); close(s); return 2; }
  if (cmd_calibrate_auto(s)  < 0) { fprintf(stderr,"Calibrate failed.\n"); close(s); return 3; }

  // Optional sanity motion after calibration
  const double zeros[6] = {0,0,0,0,0,0};
  if (cmd_move_joints(s, zeros) < 0) { fprintf(stderr,"Move joints failed.\n"); close(s); return 4; }
  if (cmd_set_learning_mode(s, true) < 0) { fprintf(stderr,"Set learning failed.\n"); close(s); return 5; }

  printf("Done.\n");
  close(s);
  return 0;
}

// handshake_crlf_open.c — send JSON+"\r\n" in a single send, keep socket open, read one JSON reply
// Build: gcc -O2 -Wall handshake_crlf_open.c -o handshake_crlf_open
// Run:   ./handshake_crlf_open 192.168.100.205

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
#include <unistd.h>

static int set_sockopts(int s, int rcv_sec){
  struct timeval tv = { .tv_sec = rcv_sec, .tv_usec = 0 };
  setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  int one = 1; setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
  return 0;
}

static int recv_one_json(int s, char **out){
  size_t cap = 8192, used = 0;
  char *buf = malloc(cap);
  if(!buf) return -1;
  int depth = 0; bool in_str=false, esc=false, started=false;

  for(;;){
    if(used == cap){ cap *= 2; if(cap > (1<<20)){ free(buf); return -1; } buf = realloc(buf, cap); if(!buf) return -1; }
    ssize_t n = recv(s, buf+used, cap-used, 0);
    if(n == 0){ free(buf); errno = EPIPE; return -1; }
    if(n < 0){ if(errno == EINTR) continue; perror("recv"); free(buf); return -1; }
    used += (size_t)n;
    for(size_t i=0;i<used;i++){
      char c = buf[i];
      if(in_str){ if(esc) esc=false; else if(c=='\\') esc=true; else if(c=='"') in_str=false; continue; }
      if(c=='"'){ in_str=true; continue; }
      if(c=='{') { depth++; started=true; }
      else if(c=='}'){ if(depth>0) depth--; }
      if(started && depth==0){
        size_t json_len = i+1;
        char *outjson = malloc(json_len+1);
        if(!outjson){ free(buf); return -1; }
        memcpy(outjson, buf, json_len);
        outjson[json_len] = '\0';
        free(buf);
        *out = outjson;
        return 0;
      }
    }
  }
}

int main(int argc, char**argv){
  const char* ip = (argc>1)?argv[1]:"192.168.100.205";
  int s = socket(AF_INET, SOCK_STREAM, 0);
  if(s<0){perror("socket"); return 1;}
  set_sockopts(s, 20);   // give it up to ~20s for the first reply

  struct sockaddr_in a; memset(&a,0,sizeof(a));
  a.sin_family = AF_INET; a.sin_port = htons(40001);
  if(inet_pton(AF_INET, ip, &a.sin_addr)!=1){perror("inet_pton"); return 1;}
  if(connect(s,(struct sockaddr*)&a,sizeof(a))<0){perror("connect"); return 1;}

  const char *req = "{\"command\": \"HANDSHAKE\", \"param_list\": [\"1.2.1\"]}\r\n";
  size_t len = strlen(req);

  // Single send (no extra bytes, no prefix, no FIN)
  ssize_t n = send(s, req, len, 0);
  if(n < 0 || (size_t)n != len){ perror("send"); close(s); return 1; }

  char *reply = NULL;
  if(recv_one_json(s, &reply) < 0){ /* perror already printed */ close(s); return 1; }

  printf("%s\n", reply);
  free(reply);

  // Now close fully
  shutdown(s, SHUT_RDWR);
  close(s);
  return 0;
}

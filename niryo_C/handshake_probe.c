// handshake_probe.c — find the server's expected terminator/behavior for HANDSHAKE
// Build: gcc -O2 -Wall handshake_probe.c -o handshake_probe
// Run:   ./handshake_probe 192.168.100.205

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

static const char *IP_DEFAULT = "192.168.100.205";
static const char *BASE = "{\"command\": \"HANDSHAKE\", \"param_list\": [\"1.2.1\"]}";

struct variant {
    const char *label;
    const char *suffix;
    size_t      slen;
};
static const struct variant VARS[] = {
    {"(no terminator)", "", 0},
    {"\\n", "\n", 1},
    {"\\r\\n", "\r\n", 2},
    {"\\0", "\0", 1},
    {"\\r\\n\\0", "\r\n\0", 3},
    {"\\n\\n", "\n\n", 2},
};

static int set_sockopts(int s, int sec){
    struct timeval tv = { .tv_sec = sec, .tv_usec = 0 };
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    int one=1; setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    return 0;
}

static int connect_to(const char *ip){
    int s = socket(AF_INET, SOCK_STREAM, 0);
    if(s<0){ perror("socket"); return -1; }
    set_sockopts(s, 12);
    struct sockaddr_in a; memset(&a,0,sizeof(a));
    a.sin_family = AF_INET; a.sin_port = htons(40001);
    if(inet_pton(AF_INET, ip, &a.sin_addr)!=1){ perror("inet_pton"); close(s); return -1; }
    if(connect(s,(struct sockaddr*)&a,sizeof(a))<0){ perror("connect"); close(s); return -1; }
    return s;
}

static int recv_one_json(int s, char **out){
    size_t cap=8192, used=0; char *buf=malloc(cap); if(!buf) return -1;
    int depth=0; bool in_str=false, esc=false, started=false;
    for(;;){
        if(used==cap){ cap*=2; if(cap>1<<20){ free(buf); return -1; } char *nb=realloc(buf,cap); if(!nb){ free(buf); return -1; } buf=nb; }
        ssize_t n = recv(s, buf+used, cap-used, 0);
        if(n==0){ free(buf); errno=EPIPE; return -1; }
        if(n<0){ if(errno==EINTR) continue; perror("recv"); free(buf); return -1; }
        used += (size_t)n;
        for(size_t i=0;i<used;i++){
            char c=buf[i];
            if(in_str){ if(esc) esc=false; else if(c=='\\') esc=true; else if(c=='"') in_str=false; continue; }
            if(c=='"'){ in_str=true; continue; }
            if(c=='{'){ depth++; started=true; }
            else if(c=='}'){ if(depth>0) depth--; }
            if(started && depth==0){
                size_t len=i+1; char *outjson=malloc(len+1); if(!outjson){ free(buf); return -1; }
                memcpy(outjson, buf, len); outjson[len]='\0'; free(buf); *out=outjson; return 0;
            }
        }
    }
}

int main(int argc, char **argv){
    const char *ip = (argc>1)? argv[1] : IP_DEFAULT;

    for(size_t v=0; v<sizeof(VARS)/sizeof(VARS[0]); ++v){
        for(int halfclose=0; halfclose<=1; ++halfclose){
            int s = connect_to(ip);
            if(s<0){ fprintf(stderr, "connect failed\n"); return 2; }

            // build frame
            size_t blen=strlen(BASE), tlen=VARS[v].slen;
            size_t len = blen + tlen;
            char *frame = malloc(len);
            memcpy(frame, BASE, blen);
            memcpy(frame+blen, VARS[v].suffix, tlen);

            // single send
            ssize_t n = send(s, frame, len, 0);
            free(frame);
            if(n < 0 || (size_t)n != len){ perror("send"); close(s); continue; }
            if(halfclose) shutdown(s, SHUT_WR);

            char *reply=NULL;
            int ok = recv_one_json(s, &reply);
            if(ok==0){
                printf("== SUCCESS with terminator %s and %s ==\n", VARS[v].label,
                       halfclose ? "HALF-CLOSE" : "KEEP-OPEN");
                printf("%s\n", reply);
                free(reply);
                shutdown(s, SHUT_RDWR); close(s);
                return 0;
            }else{
                // clean up and try next variant
                shutdown(s, SHUT_RDWR); close(s);
            }
            // small pause between attempts (server can be sensitive)
            sleep(2);
        }
    }
    fprintf(stderr, "All handshake variants failed.\n");
    return 1;
}

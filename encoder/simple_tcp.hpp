#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>
#include <string>

class simple_tcp {
  int sockfd;
  int client_sockfd;
  sockaddr_in addr;
  sockaddr_in from_addr;
  uint8_t buf[5000000];

 public:
  simple_tcp(std::string address, int port) : sockfd(-1), client_sockfd(-1) {
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = inet_addr(address.c_str());
    addr.sin_port        = htons(port);
  }

  ~simple_tcp() { this->destroy(); }

  void destroy() {
    close(client_sockfd);
    close(sockfd);
  }

  int create_server() {
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    // set option
    int opt = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));
    // bind
    bind(sockfd, (struct sockaddr *)&addr, sizeof(addr));
    // listen
    listen(sockfd, SOMAXCONN);
    socklen_t len = sizeof(sockaddr_in);
    client_sockfd = accept(sockfd, (struct sockaddr *)&from_addr, &len);
    return sockfd;
  }

  int create_client() {
    sockfd  = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
      return -1;
    }
    if (connect(sockfd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
      return -1;
    }
    return 0;
  }

  int Rx(uint8_t *dst) {
    int len = 0;
    memset(buf, 0, sizeof(buf));
    int rsize = recv(client_sockfd, buf, sizeof(buf), 0);
    len += rsize;
    while (rsize > 0) {
      for (int i = 0; i < rsize; ++i) {
        dst[i] = buf[i];
      }
      dst += rsize;
      rsize = recv(client_sockfd, buf, sizeof(buf), 0);
      len += rsize;
    }
    return len;
  }

  void Tx(uint8_t *src, size_t len) { send(sockfd, src, len, 0); }
};

#include <iostream>
#include <sys/types.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>

// 服务器端
int main(){

    // 创建socket
    // 使用ipv4协议，流式套接字
    int listenfd = socket(AF_INET, SOCK_STREAM, 0);
    if (listenfd == -1){
        std::cout << "socket error" << std::endl;
        return -1;
    }


    // 连接服务器

    // 存储要绑定的地址和端口信息
    struct sockaddr_in bindaddr;

    // 设置协议族
    bindaddr.sin_family = AF_INET;
    // 设置服务器地址为INADDR_ANY，绑定到所有可用的网络接口上
    bindaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    // 设置端口号为3000，端口号以网络字节顺序存储
    bindaddr.sin_port = htons(8080);
    // 绑定地址和端口
    if (bind(listenfd, (struct sockaddr *)& bindaddr, sizeof(bindaddr)) == -1) {
        std::cout << "bind listen socket error" << std::endl;
        return -1;
    }

    // 启动监听
    if (listen(listenfd, SOMAXCONN) == -1) {
        std::cout << "listen error" << std::endl;
        return -1;
    }

    while (true) {
        // 创建客户端socket
        struct sockaddr_in clientaddr;
        // 客户端地址长度
        socklen_t clientaddrlen = sizeof(clientaddr);
        // 接受客户端连接

        //accept函数会阻塞直到有客户端连接进来，成功返回一个新的套接字文件描述符clientfd，用于与客户端通信，失败返回-1
        // listenfd是服务器的监听套接字描述符，clientaddr是客户端的地址信息，clientaddrlen是客户端地址信息的长度
        int clientfd = accept(listenfd, (struct sockaddr *)& clientaddr, &clientaddrlen);
        if (clientfd != -1) {
            // recvBuf用于存储接收到的数据，作为缓冲区
            char recvBuf[32] = {0};
            // 从客户端接受数据

            // recv函数会阻塞直到接收到数据，成功返回接收到的字节数，失败返回-1
            // clientfd是客户端的套接字描述符，recvBuf是接收数据的缓冲区，32是缓冲区的大小，0是标志位
            int ret = recv(clientfd, recvBuf, 32, 0);

            // 如果接收到数据
            if (ret > 0) {
                std::cout << "recv data from cilent , data:" << recvBuf << std::endl;
                // 将接收到的数据原封不动地发给客户端
                ret = send(clientfd, recvBuf, strlen(recvBuf), 0);
                if (ret != strlen(recvBuf)) {
                    std::cout << "send data error" << std::endl;
                } else {
                    std::cout << "send data to client successfully, data " << recvBuf <<std::endl;
                }
            } else {
                std::cout << "recv data error" <<std::endl;
            }
            close(clientfd);
        }
    }
}
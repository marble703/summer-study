#include <iostream>
#include <sys/types.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>

// 客户端
//10.2.20.112
// 客户端地址
#define SERVER_ADDRESS "127.0.0.1"
// 服务器端口
#define SERVER_PORT 8080
// 发送数据
#define SEND_DATA "helloworld"

int main(){
    // 创建一个socket
    int clientfd = socket(AF_INET, SOCK_STREAM, 0);
    if (clientfd == -1) {
        std::cout << " create client socket error " << std::endl;
        return -1;
    }

    // 存储要绑定的地址和端口信息
    struct sockaddr_in serveraddr;
    // 设置协议族
    serveraddr.sin_family = AF_INET;
    // 设置服务器地址
    serveraddr.sin_addr.s_addr = inet_addr(SERVER_ADDRESS);
    // 设置端口号
    serveraddr.sin_port = htons(SERVER_PORT);
    
    if (connect(clientfd, (struct sockaddr *)& serveraddr, sizeof(serveraddr)) == -1) {
        std::cout << "connect socket error" << std::endl;
        return -1;
    }
    // 向服务器发送数据
    // send函数会阻塞直到发送完数据，成功返回发送的字节数，失败返回-1
    // clientfd是客户端的套接字描述符，SEND_DATA是要发送的数据，strlen(SEND_DATA)是要发送的数据的长度，0是标志位
    int ret = send(clientfd, SEND_DATA, strlen(SEND_DATA), 0);
    if (ret != strlen(SEND_DATA)) {
        std::cout << "send data error" << std::endl;
        return -1;
    } else {
        std::cout << "send data to client successfully, data " << SEND_DATA << std::endl;
    }

    // 从服务器拉取数据
    // recv函数会阻塞直到接收到数据，成功返回接收到的字节数，失败返回-1
    char recvBuf[32] = {0};
    // ret是接收到的字节数
    ret = recv(clientfd, recvBuf, 32, 0);
    if (ret > 0) {
        std::cout << "recv data to client successfully, data " << recvBuf << std::endl;
    } else {
        std::cout << "recv data to client error" << std::endl;
    }
    // 关闭socket
    close(clientfd);
    return 0;

}
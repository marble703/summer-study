#include <opencv2/opencv.hpp>
#include <iostream>
#include <sys/types.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>

#define SERVER_ADDRESS "127.0.0.1"
#define SERVER_PORT 8080

int main() {
    int clientfd = socket(AF_INET, SOCK_STREAM, 0);
    if (clientfd == -1) {
        std::cout << "create client socket error" << std::endl;
        return -1;
    }

    struct sockaddr_in serveraddr;
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_addr.s_addr = inet_addr(SERVER_ADDRESS);
    serveraddr.sin_port = htons(SERVER_PORT);

    if (connect(clientfd, (struct sockaddr *)&serveraddr, sizeof(serveraddr)) == -1) {
        std::cout << "connect socket error" << std::endl;
        return -1;
    }

    cv::Mat image = cv::imread("/media/chen/Data/programme/Visual/SummerStudy/pnp/detect/1.jpg");
    cv::imshow("image", image);
    std::vector<uchar> buf;
    cv::imencode(".jpg", image, buf);
    auto bufSize = buf.size();
    send(clientfd, reinterpret_cast<char*>(&bufSize), sizeof(bufSize), 0);
    send(clientfd, reinterpret_cast<char*>(buf.data()), buf.size(), 0);

    close(clientfd);
    return 0;
}
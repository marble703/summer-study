#include <opencv2/opencv.hpp>
#include <vector>
#include<string>

std::vector<cv::Mat> ReadImage(cv::String pattern)
{
    std::vector<cv::String> fn;
        std::cout<<"1"<<std::endl;

    cv::glob(pattern, fn, false);
        std::cout<<"1"<<std::endl;

    std::vector<cv::Mat> images;
    int count = fn.size(); //number of png files in images folder
    for (int i = 0; i < count; i++)
    {
        images.emplace_back(cv::imread(fn[i]));
    }
    return images;
}

int main()
{

    cv::String pattern="./armor_imgs/*.jpg";
    std::cout<<"1"<<std::endl;
    std::vector<cv::Mat> img_list = ReadImage(pattern);

    auto it_img_list = img_list.begin();


    int num = -1;
    while(it_img_list != img_list.end()){
        num +=1;
        std::cout << *it_img_list << std::endl;

        cv::Mat image = *it_img_list;
        if (image.empty()) {
            std::cerr << "Failed to read image." << std::endl;
            return -1;
        }
        cv::imshow("origin_image", image);

        cv::Mat gray, binary;
        //灰度化
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    //    cv::imshow("gray", gray);
        //二值化
        cv::threshold(gray, binary, 128, 255, cv::THRESH_BINARY);
    //    cv::imshow("binary", binary);
        //定义核
        int x_erode_kernal_size = 5;
        int y_erode_kernal_size = 5;
        cv::Mat erode_kernal = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(x_erode_kernal_size, y_erode_kernal_size));

        int x_dilate_kernal_size = 7;
        int y_dilate_kernal_size = 7;
        cv::Mat dilate_kernal = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(x_dilate_kernal_size, y_dilate_kernal_size));

        //腐蚀
        cv::Mat eroded_image;
        cv::erode(binary, eroded_image, erode_kernal);

    //    cv::imshow("eroded_image", eroded_image); 

        //膨胀
        cv::Mat dilated_image;
        cv::dilate(eroded_image, dilated_image, dilate_kernal);
    //    cv::imshow("dilated_image", dilated_image);
        
        // 查找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(dilated_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        for (size_t i = 0; i < contours.size(); i++) {
            // 对每个轮廓计算最小外接旋转矩形
            cv::RotatedRect minRect = cv::minAreaRect(contours[i]);
            
            // 获取矩形的宽度和高度
            float width = minRect.size.width;
            float height = minRect.size.height;
            
            // 判断是否为长条形状
            if (width > 3*height || height > 3*width) {
                // 绘制旋转矩形
                cv::Point2f rectPoints[4];
                minRect.points(rectPoints);
                for (int j = 0; j < 4; j++) {
                    cv::line(image, rectPoints[j], rectPoints[(j+1)%4], cv::Scalar(0,255,0), 2);
                }
            }
        }

        // 使用两个外接矩形的上下边中点作为边界点
        std::vector<cv::Point> points;
        for (size_t i = 0; i < contours.size(); i++) {
            cv::RotatedRect minRect = cv::minAreaRect(contours[i]);
            cv::Point2f rectPoints[4];
            minRect.points(rectPoints);
            cv::Point2f top = (rectPoints[0] + rectPoints[1]) / 2;
            cv::Point2f bottom = (rectPoints[2] + rectPoints[3]) / 2;
            points.push_back(top);
            points.push_back(bottom);
        }

        // 绘制边界点
        for (size_t i = 0; i < points.size(); i++) {
            cv::circle(image, points[i], 5, cv::Scalar(0,0,255), -1);
        }

        //连接对角线
        
        cv::line(image, points[1], points[2], cv::Scalar(255,0,0), 2);
        cv::line(image, points[0], points[3], cv::Scalar(255,0,0), 2);

        // 显示结果
    //    cv::imshow("Result", image);




        // 保存处理后的图片到result文件夹
        cv::imwrite("./result/result_" + std::to_string(num) + ".jpg", image);
    }
//    cv::waitKey(0);
    return 0;
}
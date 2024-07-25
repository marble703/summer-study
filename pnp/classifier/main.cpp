#include "Armor.hpp"
#include "number_classifier.hpp"

int main(){
    cv::Mat image = cv::imread("/media/chen/Data/programme/Visual/SummerStudy/pnp/detect/0.jpg");
    
    //将图片写入vector
    std::vector<cv::Mat> images;

    images.push_back(image);

    Armor armor(image);
    //std::cout<<"1"<<std::endl;
    std::vector<Armor> armors;
    armors.push_back(armor);

    std::vector<std::string> ignore = {"negative"};
    armor::NumberClassifier number_classifier("mlp.onnx", "label.txt", 0.5);

    number_classifier.ExtractNumbers(image, armors);

    number_classifier.Classify(armors);

    std::cout << armors[0].classification_result << std::endl;

    std::cout << armors[0].classification_confidence << std::endl;

    return 0;
}
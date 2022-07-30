#include<iostream>
#include<opencv2/opencv.hpp>
#include"orb_detect.h"
using namespace std;

//实现功能1 ：输入两张图片，完成fast图像匹配并且计算出E，恢复R T， 计算三角化恢复的地图点。

// orb_detect类，输入图像
int main(int argc, char **argv)
{
     cv::Mat image1 = cv::imread(argv[1], 1);
     cv::Mat image2 = cv::imread(argv[2], 1);
     assert(image1.data != nullptr && image2.data != nullptr);
     jiabin::orb_tect T1(image1);
     jiabin::orb_tect T2(image2);


    //还要写一个类计算RT矩阵。


    //输出匹配的图像
     cv::imshow("image1", image1);
     cv::imshow("image2", image2);
     cv::waitKey(0);
}

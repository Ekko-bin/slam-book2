
#include<iostream>
#include<opencv2/opencv.hpp>
using namespace std;

namespace jiabin
{

class orb_tect
{
public:
    orb_tect(cv::Mat inputimage);
    ~orb_tect();
private:
    /* data */
    cv::Mat m_image;

};

}

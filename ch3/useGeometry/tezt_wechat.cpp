#include<iostream>
#include<Eigen/Dense>
#include<Eigen/Core>
#include<opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace Eigen;
using namespace cv;

cv::Mat impl__aruco_getRTMatrix(const cv::Mat& _rvec, const cv::Mat& _tvec);

int main(int argc, char **argv)
{
     //Z Y Z 
    AngleAxisd w(M_PI / 2, Vector3d(0, 0, 1));  
   // cout << "旋转向量 = " << w.matrix() << endl;
   cout << w.matrix() <<endl;
     Vector3d v(0, 0 ,1);
    // cout << "v =\n" << v.matrix() << endl;

     Vector3d Angle_trans_v = w *v;
   //  cout << "  Angle_trans_v = \n" <<  Angle_trans_v << endl;

     Matrix3d R = w.matrix();
  //   cout <<"R = \n" << R <<endl;
     Vector3d R_trans_v = R *v;
   //  cout << "R_trans_v  = " << R_trans_v  <<endl;


     Quaterniond Q;
     Q = Quaterniond(w);
     cout << Q.matrix() << endl;
   //  cout << "四元素 \n" << Q.coeffs() << endl;
     Vector3d Q_tran_v = Q *v;
    // cout << "Q_tran_v " << Q_tran_v << endl;

      cout << "gggg" << endl;
     // AngleAxisd gg(Q_tran_v);
      //cout << gg.matrix() << endl;
    
    
    
    //Mat vect = (Mat_<float>(3, 1) << 0.0, 0.0, 1.0);
    cv::Mat _rvec = (Mat_<float>(3,1) <<  0,0, 1);
    cv::Mat _tvec = (Mat_<float>(3,1) << 2,2,2);
      cv::Mat dst;
      cv::Rodrigues( _rvec,  dst);
       cout << dst << endl;
    Mat TT = impl__aruco_getRTMatrix(_rvec ,  _tvec);
    cout << TT << endl;

    return 0;
}



cv::Mat impl__aruco_getRTMatrix(const cv::Mat& _rvec, const cv::Mat& _tvec)
{
    assert(_rvec.type()==CV_32F && _rvec.total()==3);
    assert(_tvec.type()==CV_32F && _tvec.total()==3);

    cv::Mat Matrix(4,4,CV_32F);
    float *rt_44=Matrix.ptr<float>(0);
    //makes a fast conversion to the 4x4 array passed
         float rx=_rvec.ptr<float>(0)[0];
        float ry=_rvec.ptr<float>(0)[1];
        float rz=_rvec.ptr<float>(0)[2];

        float tx=_tvec.ptr<float>(0)[0];
        float ty=_tvec.ptr<float>(0)[1];
        float tz=_tvec.ptr<float>(0)[2];
        float nsqa=rx*rx + ry*ry + rz*rz;
        float a=std::sqrt(nsqa);
        float i_a=a?1./a:0;
        float rnx=rx*i_a;
        float rny=ry*i_a;
        float rnz=rz*i_a;
        float cos_a=cos(a);
        float sin_a=sin(a);
        float _1_cos_a=1.-cos_a;
        rt_44[0] =cos_a+rnx*rnx*_1_cos_a;
        rt_44[1]=rnx*rny*_1_cos_a- rnz*sin_a;
        rt_44[2]=rny*sin_a + rnx*rnz*_1_cos_a;
        rt_44[3]=tx;
        rt_44[4]=rnz*sin_a +rnx*rny*_1_cos_a;
        rt_44[5]=cos_a+rny*rny*_1_cos_a;
        rt_44[6]= -rnx*sin_a+ rny*rnz*_1_cos_a;
        rt_44[7]=ty;
        rt_44[8]= -rny*sin_a + rnx*rnz*_1_cos_a;
        rt_44[9]= rnx*sin_a + rny*rnz*_1_cos_a;
        rt_44[10]=cos_a+rnz*rnz*_1_cos_a;
        rt_44[11]=tz;
        rt_44[12]=rt_44[13]=rt_44[14]=0;
        rt_44[15]=1;
     return Matrix;
}

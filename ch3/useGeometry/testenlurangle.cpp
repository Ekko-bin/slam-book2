
#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <fstream>
#include <cmath>
 
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>


using namespace std;
using namespace cv;

//matlab 旋转顺序是Z-Y-X
//而下面的实现是X-Y-Z
//rotm2euler.m


bool isRotationMatrix(Mat &R)
{
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());
    return  norm(I, shouldBeIdentity) < 1e-6;
}
 
Vec3f rotationMatrixToEulerAngles(Mat &R)
{
     
    // cout << "R = " << R << endl;
     float sy = sqrt(R.at<float>(0,0) * R.at<float>(0,0) +  R.at<float>(1,0) * R.at<float>(1,0) );
    bool singular = sy < 1e-6; // If
    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<float>(2,1) , R.at<float>(2,2));
      //  cout << R.at<float>(2,1)  << " " <<  R.at<float>(2,2) << endl;
       // cout << "x = " << x << endl;
        y = atan2(-R.at<float>(2,0), sy);
        z = atan2(R.at<float>(1,0), R.at<float>(0,0));
    }
    else
    {
        x = atan2(-R.at<float>(1,2), R.at<float>(1,1));

        y = atan2(-R.at<float>(2,0), sy);
        z = 0;
    }
    return Vec3f(x, y, z);
     
}

int main(int argc, char **argv) 
{
   
   Mat vect = (Mat_<float>(3, 1) << 0.0, 0.0, 1.0);
   Mat matrix1 = (Mat_<float>(3, 3) << -0.92838174, -0.37056851, -0.028045049,     
                                                                              0.36943084, -0.92845523, 0.03863031,
                                                                               -0.040353741, 0.025502967, 0.99886);
   //  cout << "matrix1 = " << matrix1 <<endl;
    Mat matrix2 = (Mat_<float>(3, 3) <<  -0.93693584, 0.34723067, -0.039775982,
                                                                              -0.34649989, -0.93773764, -0.024211602,
                                                                              -0.04570644, -0.008902343, 0.99891531);
     
    // Mat matrix1 = (Mat_<float>(3, 3) << -0.90912992, -0.41232491, -0.058914252,
    //                                                                             0.41301581, -0.91072351, 0.00049322238,
    //                                                                             -0.053857971, -0.023884114, 0.99826294);
    //  Mat matrix2 = (Mat_<float>(3, 3) << -0.92893291, 0.36751434, -0.044909209,
    //                                                                             -0.36636338, -0.92992413, -0.031919129,
    //                                                                             -0.0534929, -0.013197635, 0.99848109);
                                                                                

     Vec3f v1 = rotationMatrixToEulerAngles(matrix1);
   //  cout << v1 << endl;
     cout << v1[0, 0] << " ";
     cout << v1[0,1] << " ";
     cout << v1[0,2] << endl;

     Vec3f v2 = rotationMatrixToEulerAngles(matrix2);
     cout << v2[0 ,0 ] << " " << v2[0, 1] << v2[0, 2] << endl;

    cout << "====================" << endl;
    Eigen::Matrix3d m1 , m2;
    m1 << -0.92838174, -0.37056851, -0.028045049,     
                0.36943084, -0.92845523, 0.03863031,
                 -0.040353741, 0.025502967, 0.99886;
    m2 <<  -0.93693584, 0.34723067, -0.039775982,
                 -0.34649989, -0.93773764, -0.024211602,
                 -0.04570644, -0.008902343, 0.99891531;
     Eigen::Vector3d ea1 = m1.normalized().eulerAngles(0, 1, 2);

     Eigen::Vector3d ea2 = m2.eulerAngles(0, 1, 2);
     cout << "ea1" << ea1.transpose() << endl;
     cout << "ea2 " << ea2.transpose() << endl;


    Eigen::Quaterniond q1(m1);
    cout << q1.normalized().toRotationMatrix() << endl;

        Eigen::Quaterniond q2(m2);
    cout << q2.normalized().toRotationMatrix() << endl;


  return 0;
}

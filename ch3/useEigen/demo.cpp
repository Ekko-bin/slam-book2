/*
 * @Author: your name
 * @Date: 2021-11-12 14:17:26
 * @LastEditTime: 2022-04-28 17:10:07
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /slambook2-master/ch3/useEigen/demo.cpp
 */
#include<iostream>
#include<Eigen/Dense>
#include<Eigen/Core>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
   
//    Matrix3d M31;
//    Matrix<double , 3, 1> mM31;
//    mM31 << 1, 0, 0 ;
//     cout << mM31  << endl;
//     MatrixXd mat(2, 2);
//     mat << 1, 2,
//                    3, 4 ;
//     cout << mat << endl;
   

   
//     Matrix3d X3d2;
//     X3d2 << 1 , 0, 0,
//                    0, 1, 0,
//                     0, 0, 1;
//     cout << X3d2 << endl;

//     MatrixX3d X3d;
//     X3d << 1 , 0, 0, 1, 1,  2;
//     cout << X3d << endl;

    Vector3d vec;
    vec(0,0)= 1;
    vec(1,0) = 2.0;
    vec(2,0) = 3.0;

    vec[1,0] = 123;
    cout << "vec " << vec << endl;  
    return 0;
}
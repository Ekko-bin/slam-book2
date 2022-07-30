/*
 * @Author: error: git config user.name && git config user.email & please set dead value or install git
 * @Date: 2022-03-11 21:20:09
 * @LastEditors: error: git config user.name && git config user.email & please set dead value or install git
 * @LastEditTime: 2022-06-10 11:12:37
 * @FilePath: /slambook2-master/ch3/useGeometry/demoG1.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include<iostream>
#include<Eigen/Dense>
#include<Eigen/Core>

using namespace std;
using namespace Eigen;


int main(int argc, char **argv)
{
    
    //1 旋转向量
    //旋转向量由角度和方向组成,一般输出的都是弧度
    //1.1旋转向量初始化方式一      （前面角度，后面方向）
    AngleAxisd rotation_vectorX(M_PI / 2, Vector3d(1, 0, 0));    // 按X轴旋转90度。
    AngleAxisd rotation_vectorY(M_PI / 4, Vector3d(0, 1, 0));    //按Y轴旋转45度。
    AngleAxisd rotation_vectorZ(M_PI / 6, Vector3d(0, 0, 1));   //按Z轴旋转30度。
     
     //先绕X 180，再绕Z -90
     AngleAxisd rotation_vectorX180(M_PI , Vector3d(1, 0, 0));    // 按X轴旋转180度。
     AngleAxisd rotation_vectorZ90_(M_PI / 2, Vector3d(0, 0, -1));   //按Z轴旋转-90度。
    //XZ
    Matrix3d roatation =rotation_vectorX180.toRotationMatrix() * rotation_vectorZ90_.toRotationMatrix() ;
    cout << " roatation =  \n " <<  roatation << endl;


    //验证绕XYZ
    Vector3d ea = roatation.eulerAngles(0,1,2);
    //X Y Z顺序
    cout << ea(0)/M_PI *180 << " " << ea(1)/M_PI *180 << " " << ea(2)/M_PI *180 << endl;

    /*
          roatation =  
                        6.12323e-17            1            0
                                1       -6.12323e-17     -1.22465e-16
                      -1.22465e-16   7.4988e-33           -1
         180   0   -90
    */


     //
     AngleAxisd rotation_vectorZ90(M_PI / 2, Vector3d(0, 0, 1));   //按Z轴旋转90度。
      Matrix3d roatationz90 = rotation_vectorZ90.toRotationMatrix();
      cout << "roatationz90 =  \n " <<  roatationz90 << endl;





    return 0;
}                                                                                        
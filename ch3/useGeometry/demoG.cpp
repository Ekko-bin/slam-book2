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

     //旋转向量初始化方式二
    AngleAxisd rotation_vectorX_(M_PI / 2, Eigen::Vector3d::UnitX());
    AngleAxisd rotation_vectorY_(M_PI / 4,  Eigen::Vector3d::UnitY());    
    AngleAxisd rotation_vectorZ_(M_PI / 6,  Eigen::Vector3d::UnitZ());   
                                                                                                                  //输出
    cout << rotation_vectorX.angle()/M_PI *180 << endl;    //90
    cout << rotation_vectorX.axis().transpose() << endl;    //1 0 0 

    cout << rotation_vectorY.angle()/M_PI *180 << endl;    //45
    cout << rotation_vectorY.axis().transpose() << endl;    //0 1 0 
    
   
    //1.2 旋转向量转旋转矩阵
    Eigen::Matrix3d rotation_matrix1 = Eigen::Matrix3d::Identity();
    rotation_matrix1 = rotation_vectorX.matrix();
    cout << "rotation_vectorX =\n" << rotation_matrix1 << endl;                
    //或者由罗德里格公式进行转换
    Eigen::Matrix3d rotation_matrix2 =  rotation_vectorX.toRotationMatrix();
    cout << "rotation_vectorX  =\n" << rotation_matrix2 << endl;
   
   /*
            rotation_vectorX =
                    1           0           0
                    0 6.12323e-17          -1
                    0           1 6.12323e-17
          rotation_vectorX  =
                    1           0           0
                    0 6.12323e-17          -1
                    0           1 6.12323e-17
   */

     //1.3 旋转向量转欧拉角  2,1,0代表旋转顺序是ZYX
     Eigen::Vector3d eulerAngle1 = rotation_vectorX.matrix().eulerAngles(2,1,0);
     cout << "eulerAngle1, z y x: " << eulerAngle1.transpose() << endl;                             //eulerAngle1, z y x:      0     -0   1.5708
     cout << "eulerAngle1, z y x: " << eulerAngle1.transpose()/M_PI *180 << endl;     //eulerAngle1, z y x:      0      -0   90
  
    //1.4 旋转向量转四元数
    Eigen::Quaterniond quaternion1(rotation_vectorX);
    //或者
    Eigen::Quaterniond quaternion1_1;
    quaternion1_1 = rotation_vectorX;
     
     //coeffs()的顺序是（x y z w）w是实部
    cout << "quaternion1 = " << quaternion1.coeffs().transpose() << endl;       //  quaternion1 = 0.707107        0        0    0.707107
     //分开输出
    cout << "quaternion1.x = " << quaternion1.x() << endl;         //quaternion1.x = 0.707107
    cout << "quaternion1.y = " << quaternion1.y() << endl;        //quaternion1.y = 0
    cout << "quaternion1.z = " << quaternion1.z() << endl;       //quaternion1.z = 0
    cout << "quaternion1.w = " << quaternion1.w() << endl;    //quaternion1.w = 0.707107
    quaternion1.normalized();
      //coeffs()的顺序是（x y z w）w是实部
    cout << "quaternion1 = " << quaternion1.coeffs().transpose() << endl;       //  quaternion1 = 0.707107        0        0    0.707107



    //2 旋转矩阵
   //性质
    Eigen::Matrix3d M1 ;
    M1.setZero();
    Eigen::Matrix3d M2 = Eigen::Matrix3d::Identity() ;


    //2.1 旋转矩阵转旋转向量
    Eigen::AngleAxisd rotation_vector2;
    rotation_vector2.fromRotationMatrix(rotation_matrix2);
    //或者
    Eigen::AngleAxisd rotation_vector2_1(rotation_matrix2);
    cout << "rotation_vector2 " << "angle is: " << rotation_vector2.angle() * (180 / M_PI) 
                                << " axis is: " << rotation_vector2.axis().transpose() << endl;            //rotation_vector2 angle is: 90 axis is: 1 0 0
 
    cout << "rotation_vector2_1 " << "angle is: " << rotation_vector2_1.angle() * (180 / M_PI) 
                                  << " axis is: " << rotation_vector2_1.axis().transpose() << endl;     //rotation_vector2_1 angle is: 90 axis is: 1 0 0

    
    //2.2 旋转矩阵转欧拉角
    /*
       构造一个Euler angle 按ZYX顺序的旋转矩阵
       Euler angle 指的是按旋转之后的轴旋转
       fixed angle 指的是按固定不动的坐标轴的轴旋转
   */
  
    //Euler angle ZYX，先乘放前面，后乘放后面，直接按顺序乘就对了。
    Matrix3d Euler_angleZYX = rotation_vectorZ.toRotationMatrix() *rotation_vectorY.toRotationMatrix() * rotation_vectorX.toRotationMatrix() ;
    cout <<" Euler_angleZYX = \n "<<  Euler_angleZYX << endl;
    Vector3d euler_angle = Euler_angleZYX.eulerAngles(2,1,0) ;
    cout <<"euler_angle.transpose() = " <<  euler_angle.transpose() << endl;
    //yaw pitch roll  Z Y X
    cout << euler_angle(0)/3.14*180 << " " <<  euler_angle(1)/3.14*180 << " " <<  euler_angle(2)/3.14*180 << endl;
    //   fixed angle顺序与Euler angle 相反，直接反着写就对了，比如旋转顺序是ZYX，那就X*Y*Z
   
    /*
                Euler_angleZYX =
                                0.612372    0.612372         0.5
                               0.353553    0.353553   -0.866025
                              -0.707107    0.707107 4.32978e-17
          euler_angle.transpose() = 0.523599 0.785398   1.5708
          30.0152 45.0228 90.0456

    */
    //2.3 旋转矩阵转换为四元数
    //直接赋值
    Quaterniond matrixtoquaternion(Euler_angleZYX);
    //间接赋值，感觉都是强制转化的，重载了=符号
    Quaterniond matrixtoquaternion2;
    matrixtoquaternion2 = Euler_angleZYX;
    cout << "四元数 " << matrixtoquaternion.coeffs().transpose() << endl;    //四元数  0.560986  0.430459 -0.092296  0.701057
     
  
   /**** 3. 欧拉角 ****/
    cout << endl << "********** EulerAngle **********" << endl;
    //3.0 初始化欧拉角(Z-Y-X，即RPY, 先绕z轴yaw,再绕y轴pitch,最后绕x轴roll)
    Eigen::Vector3d ea0(0.785398, -0, 0);  //这里是绕Z轴45度
    Eigen::Vector3d ea = Euler_angleZYX.eulerAngles(2,1,0) ;  //这里是绕Z轴30度  Y 45 X 90

    //3.1 欧拉角转换为旋转矩阵，Z-Y-X，直接按顺序乘就对了。
    Eigen::Matrix3d rotation_matrix3;
    rotation_matrix3 = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) * 
                                           Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) * 
                                           Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());
    cout << "rotation matrix3 =\n" << rotation_matrix3 << endl;   
 
    //3.2 欧拉角转换为四元数,
    Eigen::Quaterniond quaternion3;
     //旋转向量初始化（前面角度，后面方向）
    quaternion3 = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) * 
                                  Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) * 
                                  Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());
    cout << "quaternion3 x: " << quaternion3.x() << endl;            
    cout << "quaternion3 y: " << quaternion3.y() << endl;
    cout << "quaternion3 z: " << quaternion3.z() << endl;
    cout << "quaternion3 w: " << quaternion3.w() << endl;

    //3.3 欧拉角转换为旋转向量
    Eigen::AngleAxisd rotation_vector3;
    rotation_vector3 = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) * 
                       Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) * 
                       Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());  
    cout << "rotation_vector3 " << "angle is: " << rotation_vector3.angle() * (180 / M_PI) 
                                << " axis is: " << rotation_vector3.axis().transpose() << endl;

          /*
             rotation matrix3 =
                    0.612372  0.612372       0.5
                    0.353553  0.353553 -0.866025
                    -0.707107  0.707107         0

              quaternion3 x: 0.560986
              quaternion3 y: 0.430459
              quaternion3 z: -0.092296
              quaternion3 w: 0.701057
              rotation_vector3 angle is: 90.9762 axis is:   0.78668  0.603641 -0.129428
              qtoeuler_angle 0.523599 0.785398   1.5708
              30 45 90
    */                          
  

  /**** 4.四元数 ****/
    cout << endl << "********** Quaternion **********" << endl;
    //4.0 初始化四元素,注意eigen Quaterniond类四元数初始化参数顺序为w,x,y,z
    Eigen::Quaterniond quaternion4(0.701057, 0.560986,  0.430459, -0.092296 );
 
    //4.1 四元数转换为旋转向量
    Eigen::AngleAxisd rotation_vector4(quaternion4);
    //或者
    Eigen::AngleAxisd rotation_vector4_1;
    rotation_vector4_1 = quaternion4;
    cout << "rotation_vector4 " << "angle is: " << rotation_vector4.angle() * (180 / M_PI) 
                                << " axis is: " << rotation_vector4.axis().transpose() << endl;
 
    cout << "rotation_vector4_1 " << "angle is: " << rotation_vector4_1.angle() * (180 / M_PI) 
                                  << " axis is: " << rotation_vector4_1.axis().transpose() << endl;
 
    //4.2 四元数转换为旋转矩阵
    Eigen::Matrix3d rotation_matrix4;
    rotation_matrix4 = quaternion4.matrix();
    Eigen::Matrix3d rotation_matrix4_1;
    rotation_matrix4_1 = quaternion4.toRotationMatrix();
    cout << "rotation matrix4 =\n" << rotation_matrix4 << endl;
    cout << "rotation matrix4_1 =\n" << rotation_matrix4_1 << endl;      
 
 
    //4.4 四元数转欧拉角(Z-Y-X，即RPY)
    Eigen::Vector3d eulerAngle4 = quaternion4.matrix().eulerAngles(2,1,0);
    cout << "yaw(z) pitch(y) roll(x) = " << eulerAngle4.transpose() << endl;
    cout << eulerAngle4(0)/M_PI *180 << " " << eulerAngle4(1)/M_PI *180 << " " << eulerAngle4(2)/M_PI *180 << endl;
 
     /*
        ********** Quaternion **********
        rotation_vector4 angle is: 90.9762 axis is:  0.786681   0.60364 -0.129428
        rotation_vector4_1 angle is: 90.9762 axis is:  0.786681   0.60364 -0.129428
        rotation matrix4 =
            0.612373     0.612372     0.499999
            0.353553     0.353552    -0.866026
          -0.707106     0.707107 -4.85754e-07
        rotation matrix4_1 =
            0.612373     0.612372     0.499999
            0.353553     0.353552    -0.866026
          -0.707106     0.707107 -4.85754e-07
        yaw(z) pitch(y) roll(x) = 0.523598 0.785398   1.5708
        30 45 90

     */
      
        

    

    return 0;
}
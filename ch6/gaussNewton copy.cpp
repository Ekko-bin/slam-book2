/*
 * @Author: your name
 * @Date: 2022-03-03 11:19:27
 * @LastEditTime: 2022-03-03 11:29:26
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /slambook2-master/ch6/gaussNewton copy.cpp
 */
#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
using namespace std;

//高斯牛顿曲线拟合

int main(int argc, char **argv) 
{

  double ar = 1.0, br = 2.0, cr = 1.0;         // 真实参数值
  double ae = 2.0, be = -1.0, ce = 5.0;        // 估计参数值  这里给的是初始值，根据初始值来进行迭代

  cv::RNG rng;
  //50组数据
  vector<double>x_data, y_data;
  int N = 50;
  for(int i = 0; i < N; i++)
  {
      x_data.push_back
  }


  


  return 0;

}

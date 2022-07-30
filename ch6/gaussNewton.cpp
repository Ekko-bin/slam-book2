#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
using namespace std;

//

int main(int argc, char **argv) 
{

  double ar = 1.0, br = 2.0, cr = 1.0;         // 真实参数值
  double ae = 2.0, be = -1.0, ce = 5.0;        // 估计参数值  这里给的是初始值，根据初始值来进行迭代

  int N = 100;                                 // 数据点
  double w_sigma = 1.0;                        // 噪声Sigma值
  double inv_sigma = 1.0 / w_sigma;

  //RNG::gaussian( ) 产生一个高斯分布的随机数
 // RNG::gaussian( σ) 返回一个均值为0，标准差为σ的随机数

   cv::RNG rng;                                 // OpenCV随机数产生器

  vector<double> x_data, y_data, y_datayuanshi, gaussiandata;      // 数据
  for (int i = 0; i < N; i++)
   {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
        y_datayuanshi.push_back(exp(ar * x * x + br * x + cr) );
        gaussiandata.push_back( rng.gaussian(w_sigma * w_sigma));
  }

  sort(gaussiandata.begin(), gaussiandata.end());
  
  for (int i = 0; i < x_data.size(); i++)
  {
     cout << "x_data" << i << "= " <<  x_data[i] << " y_data" << i << " = " << y_data[i] << endl;
     cout << "y_datayuanshi  =  " << y_datayuanshi[i] << endl;
     cout << "gaussiandata = " << gaussiandata[i] << endl;
  }

  // 开始Gauss-Newton迭代
  int iterations = 100;    // 迭代次数
  double cost = 0, lastCost = 0;  // 本次迭代的cost和上一次迭代的cost

  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  for (int iter = 0; iter < iterations; iter++)
  {

    Matrix3d H = Matrix3d::Zero();             // Hessian = J^T W^{-1} J in Gauss-Newton
    Vector3d b = Vector3d::Zero();             // bias
    cost = 0;

      for (int i = 0; i < N; i++)//N是数据个数100
      {
          double xi = x_data[i], yi = y_data[i];  // 第i个数据点
          double error = yi - exp(ae * xi * xi + be * xi + ce);
          Vector3d J; // 雅可比矩阵
          J[0] = -xi * xi * exp(ae * xi * xi + be * xi + ce);  // de/da
          J[1] = -xi * exp(ae * xi * xi + be * xi + ce);  // de/db
          J[2] = -exp(ae * xi * xi + be * xi + ce);  // de/dc*

          H += inv_sigma * inv_sigma * J * J.transpose();
          b += -inv_sigma * inv_sigma * error * J;

          cost += error * error;
      }//

    // 求解线性方程 Hx=b
    cout << "迭代次数 = " << iter << endl;
    Vector3d dx = H.ldlt().solve(b);
    cout << "增量是"  <<dx[0] << " " << dx[1] << " " << dx[2] << endl; 
    if (isnan(dx[0]))
     {
      cout << "result is nan!" << endl;
      break;
    }

    if (iter > 0 && cost >= lastCost) 
    {
      cout << "cost: " << cost << ">= last cost: " << lastCost << ", break." << endl;
      break;
    }

    ae += dx[0];
    be += dx[1];
    ce += dx[2];

    lastCost = cost;

    cout << "total cost: " << cost << ", \t\tupdate: " << dx.transpose() <<
         "\t\testimated params: " << ae << "," << be << "," << ce << endl;
  }//for

  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

  cout << "estimated abc = " << ae << ", " << be << ", " << ce << endl;
  return 0;
}

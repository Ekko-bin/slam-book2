#include<iostream>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<vector>
#include<sophus/se3.hpp>
#include<fstream>

using namespace std;
using  namespace Eigen;

/*
   ICP轨迹求R T 
   
*/

typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;
TrajectoryType ReadTrajectory(const string &path);
void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti);

int main(int argc, char **argv)
{
     string groundtruth_file = "/home/ekko/code/SLAM/slambook2-master/ch4/example/groundtruth.txt";
      string estimated_file = "/home/ekko/code/SLAM/slambook2-master/ch4/example/estimated.txt";
     TrajectoryType estmin = ReadTrajectory(estimated_file) ;
     TrajectoryType groundtrue =  ReadTrajectory(groundtruth_file) ;
      
      //求质心
      Eigen::Vector3d est_mid;
      Eigen::Vector3d gd_mid;
       int size_est = estmin.size();
       int size_gd = groundtrue.size();
      for(int i = 0 ; i < estmin.size(); i++)
      {
            est_mid += estmin[i].translation();
             gd_mid += groundtrue[i].translation();
      }
       est_mid = est_mid / size_est;
       gd_mid = gd_mid/ size_gd;
      
      //去质心
      vector<Eigen::Vector3d,  Eigen::aligned_allocator<Eigen::Vector3d>> est_remove;
      vector<Eigen::Vector3d,  Eigen::aligned_allocator<Eigen::Vector3d>> gd_remove;
      for(int i = 0 ; i < est_mid.size(); i++)
      {
           est_remove.push_back(estmin[i].translation() - est_mid );
           gd_remove.push_back( groundtrue[i].translation() - gd_mid);
      }
      
     Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
     for(int i = 0 ; i < est_mid.size(); i++)
      {
           W += gd_remove[i] * est_remove[i].transpose();
      }

      //compute R and T
    cout << "W=" << W << endl;
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
   
    Eigen::Matrix3d R_ = U * (V.transpose());
    cout << "U=" << U << endl;
    cout << "V=" << V << endl;
    
    //p' = Rp + t
    Eigen::Vector3d t = gd_mid - R_ *est_mid;
    
     //整合est曲线和gd曲线
     

    return 0;
}

TrajectoryType ReadTrajectory(const string &path) 
{
  ifstream fin(path);
  TrajectoryType trajectory;
  if (!fin)
   {
    cerr << "trajectory " << path << " not found." << endl;
    return trajectory;
  }

  while (!fin.eof()) 
  {
    double time, tx, ty, tz, qx, qy, qz, qw;
    fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

    Eigen::Vector3d t(tx , ty , tz);
    Eigen::Quaterniond q(qx, qy, qz, qw);
    q.normalized(); //归一化
    Eigen::Matrix3d Rotation_matriz(q);
    Sophus::SE3d p1(Rotation_matriz , t);  //等同于下面的,只是下面的没有归一化

    //Sophus::SE3d p1(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
    trajectory.push_back(p1);
  }
  return trajectory;
}
#include<iostream>
#include<Eigen/Core>
#include<Eigen/Dense>
#include <pangolin/pangolin.h>
using namespace std;
using namespace Eigen;

string trajectory_file = "../../examples/trajectory.txt";

int main(int arhc, char **argv)
{
     Eigen::Quaterniond q1(0.35, 0.2, 0.3, 0.1) , q2(-0.5, 0.4, -0.1, 0.2);
      q1.normalized();
      q2.normalized();
      Eigen::Vector3d t1(0.3, 0.1, 0.1), t2(-0.1, 0.5, 0.3);
      Eigen::Isometry3d T1w(q1) , T2w(q2);
       T1w.pretranslate(t1);
       T2w.pretranslate(t2);
        
        
   
         Matrix<double , 3 ,1> M31;

      
    return 0;
}
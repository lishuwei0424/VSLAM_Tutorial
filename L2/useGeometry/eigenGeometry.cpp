#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
// Eigen 几何模块
#include <Eigen/Geometry>

/****************************
* 本程序演示了 Eigen 几何模块的使用方法
****************************/

int main ( int argc, char** argv )
{
    //小罗卜1的位姿T1
    Eigen::Quaterniond q1(0.55,0.3,0.2,0.2);
    //cout<<q1.coeffs()<<endl;   // 请注意coeffs的顺序是(x,y,z,w),w为实部，前三者为虚部
    Eigen::Isometry3d T1=Eigen::Isometry3d::Identity();
    T1.rotate(q1.normalized().toRotationMatrix());
    T1.pretranslate(Eigen::Vector3d(0.7,1.1,0.2));

    //小罗卜2的位姿T2
    Eigen::Quaterniond q2(-0.1,0.3,-0.7,0.2);
    // cout<<q2.coeffs()<<endl;
    Eigen::Isometry3d T2=Eigen::Isometry3d::Identity();
    T2.rotate(q2.normalized().toRotationMatrix());
    T2.pretranslate(Eigen::Vector3d(-0.1,0.4,0.8));

    //空间某点在T1的位置
    Eigen::Vector3d  p1(0.5,-0.1,0.2);
    //空间某点在T2的位置
    Eigen::Vector3d  p2(0,0,0);
    
    p2=T2*T1.inverse()*p1;
    cout<<"空间某点在T2的位置:"<<p1.transpose()<<endl;
    cout<<"空间某点对应的在T2的位置:"<<p2.transpose()<<endl;

    return 0;
}

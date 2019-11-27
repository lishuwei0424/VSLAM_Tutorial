#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;


// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>,vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);

int main(int argc, char **argv) {

   
    /// implement pose reading code
    // start your code here (5~10 lines)

    Eigen::Matrix< double ,6,1 > error ;
    double   all_error,RMSE;


    ifstream infile;
    infile.open("../data/estimated.txt");
    if(!infile) cout<<"error"<<endl;

    ifstream truth;
    truth.open("../data/groundtruth.txt");
    if(!truth) cout<<"error"<<endl;
    
    double data;
    double a[612][8];
    double *p=&a[0][0];


    double data1;
    double b[612][8];
    double *p1=&b[0][0];

    //读数据
    while(infile>>data&&truth>>data1)             //遇到空白符结束
    {
        *p=data;
         p++;
        *p1=data1;
         p1++;
    }
    infile.close();
    truth.close();

    //处理数据
    for(int i=0;i<612;i++)   //分别对每一行数据生成一个变换矩阵，然后存入动态数组poses中
    {	
	Eigen::Quaterniond q1 = Eigen::Quaterniond(a[i][7],a[i][4],a[i][5],a[i][6]);
    	Eigen::Vector3d t1;
	t1<<a[i][1],a[i][2],a[i][3];
        Sophus::SE3 SE3_e(q1,t1);


        Eigen::Quaterniond q2 = Eigen::Quaterniond(b[i][7],b[i][4],b[i][5],b[i][6]);
    	Eigen::Vector3d t2;
	t2<<b[i][1],b[i][2],b[i][3];
        Sophus::SE3 SE3_g(q2,t2);

        error=(SE3_g.inverse()*SE3_e).log();
        all_error += error.transpose()*error;
    }
   //输出
    RMSE=sqrt(all_error/612); 
    cout<<"RMSE"<<RMSE<<endl;

    return 0;
}



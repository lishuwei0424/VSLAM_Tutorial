//
// Created by 高翔 on 2017/12/19.
// 本程序演示如何从Essential矩阵计算R,t
//

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

#include <sophus/so3.h>
using namespace Sophus;

#include <iostream>

using namespace std;

int main(int argc, char **argv) {
    cout<<"E2Rt main..."<<endl;
    // 给定Essential矩阵
    Matrix3d E;
    E << -0.0203618550523477, -0.4007110038118445, -0.03324074249824097,
            0.3939270778216369, -0.03506401846698079, 0.5857110303721015,
            -0.006788487241438284, -0.5815434272915686, -0.01438258684486258;

    // 待计算的R,t
    Matrix3d R;
    Vector3d t;

    // SVD and fix sigular values
    // START YOUR CODE HERE
    JacobiSVD<MatrixXd> svd(E, ComputeThinU | ComputeThinV);
    Matrix3d U = svd.matrixU();
    Matrix3d V = svd.matrixV();
    Matrix3d Sigma = U.inverse() * E * V.transpose().inverse();
    cout<<"U:\n"<<U<<"\nV:\n"<<V<<"\nSigma:\n"<<Sigma<<endl;
    vector<double> tao = {Sigma(0,0), Sigma(1,1), Sigma(2,2)};
    sort(tao.begin(),tao.end());
    Matrix3d SigmaFix = Matrix3d::Zero();
    double tao_mean = (tao[1]+tao[2])*0.5;
    SigmaFix(0,0) = tao_mean;
    SigmaFix(1,1) = tao_mean;
    cout<<"Sigma after fix: \n"<<SigmaFix<<endl;
    // END YOUR CODE HERE

    // set t1, t2, R1, R2 
    // START YOUR CODE HERE
    Matrix3d R_Z1 = AngleAxisd(M_PI/2,Vector3d(0,0,1)).matrix();
    Matrix3d R_Z2 = AngleAxisd(-M_PI/2,Vector3d(0,0,1)).matrix();

    Matrix3d t_wedge1 = U * R_Z1 * SigmaFix * U.transpose();
    Matrix3d t_wedge2 = U * R_Z2 * SigmaFix * U.transpose();

    Matrix3d R1 = U * R_Z1.transpose() * V.transpose();
    Matrix3d R2 = U * R_Z2.transpose() * V.transpose();
    
    // END YOUR CODE HERE

    cout << "R1 = " << R1 << endl;
    cout << "R2 = " << R2 << endl;
    cout << "t1 = " << SO3::vee(t_wedge1).transpose() << endl;
    cout << "t2 = " << SO3::vee(t_wedge2).transpose() << endl;

    // check t^R=E up to scale
    Matrix3d tR1 = t_wedge1 * R1;
    cout << "1 t^R = " << tR1 << endl;

    Matrix3d tR2 = t_wedge2 * R2;
    cout << "2 t^R = " << tR2 << endl;
    Matrix3d tR3 = t_wedge1 * R2;
    cout << "3 t^R = " << tR3 << endl;
    Matrix3d tR4 = t_wedge2 * R1;
    cout << "4 t^R = " << tR4 << endl;
    return 0;
}

#include <iostream>
using namespace std;
#include <ctime>
// Eigen 部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>
#include <Eigen/Cholesky>

#define MATRIX_SIZE 100


int main( int argc, char** argv )
{
    cout<<"useEigen..."<<endl;
  
    // 解方程
    // 我们求解 matrix_NN * x = v_Nd 这个方程
    // N的大小在前边的宏里定义，它由随机数生成
    // 直接求逆自然是最直接的，但是求逆运算量

    // 如果不确定矩阵大小，可以使用动态大小的矩阵
    Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic > matrix_dynamic_100_100;
    matrix_dynamic_100_100=Eigen::MatrixXd::Random(MATRIX_SIZE,MATRIX_SIZE);

    Eigen::Matrix< double,Eigen::Dynamic,1 >  vnd;
    vnd=Eigen::MatrixXd::Random(MATRIX_SIZE,1);
    
    clock_t time_stt = clock(); // 计时
    // 直接求逆
    Eigen::Matrix<double,MATRIX_SIZE,1> x1 = matrix_dynamic_100_100.inverse()*vnd;
    cout <<"time use in normal inverse is " << 1000* (clock() - time_stt)/(double)CLOCKS_PER_SEC << "ms"<< endl;
    cout<<"直接求逆的结果"<<endl<<x1.transpose()<<endl;
    
    //QR分解
    time_stt = clock();
    Eigen::Matrix<double,MATRIX_SIZE,1> x2=matrix_dynamic_100_100.colPivHouseholderQr().solve(vnd);
    cout <<"time use in Qr decomposition is " <<1000*  (clock() - time_stt)/(double)CLOCKS_PER_SEC <<"ms" << endl;
    cout<<"QR分解"<<endl<<x2.transpose()<<endl;
    
    //Cholesky分解
    time_stt = clock(); // 计时
    Eigen::Matrix<double,MATRIX_SIZE,1> x3=matrix_dynamic_100_100.llt().solve(vnd);
    cout <<"time use in Cholesky decomposition is " <<1000*  (clock() - time_stt)/(double)CLOCKS_PER_SEC <<"ms" << endl;
    cout<<"Cholesky分解"<<endl<<x3.transpose()<<endl;

    return 0;
}

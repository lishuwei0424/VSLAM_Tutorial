//
// Created by 高翔 on 2017/12/15.
//

#include <opencv2/opencv.hpp>
#include <string>

using namespace std;

string image_file = "../test.png";   // 请确保路径正确

int main(int argc, char **argv) {

    // 本程序需要你自己实现去畸变部分的代码。尽管我们可以调用OpenCV的去畸变，但自己实现一遍有助于理解。
    // 畸变参数
    double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;
    // 内参
    double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

    cv::Mat image = cv::imread(image_file,0);   // 图像是灰度图，CV_8UC1
    int rows = image.rows, cols = image.cols;
    cv::Mat image_undistort = cv::Mat(rows, cols, CV_8UC1);   // 去畸变以后的图

    // 计算去畸变后图像的内容
    for (int v = 0; v < rows; v++)
        for (int u = 0; u < cols; u++) {

            double u_distorted = 0, v_distorted = 0;
            // TODO 按照公式，计算点(u,v)对应到畸变图像中的坐标(u_distorted, v_distorted) (~6 lines)
            // start your code here

            /* 去畸变步骤：
            1. image_undistort中含有非畸变的图像坐标,将image_undistort的坐标通过内参转换到归一化坐标系下，此时得到的归一化坐标是对的
            2. 将得到的归一化坐标系进行畸变处理
            3. 将畸变处理后的坐标通过内参转换为图像坐标系下的坐标
            4. 这样就相当于是在非畸变图像的图像坐标和畸变图像的图像坐标之间建立了一个对应关系,相当于是非畸变图像坐标在畸变图像中找到了映射
             对畸变图像进行遍历之后，然后赋值（一般需要线性插值，因为畸变后图像的坐标不一定是整数的），即可得到矫正之后的图像
            */

            //x_undistort,y_undistort为未发生畸变时归一化坐标，x_distort,y_distort为发生畸变时归一化坐标
            double x_undistort,y_undistort,x_distort,y_distort;

            x_undistort=(u-cx)/fx;
            y_undistort=(v-cy)/fy;

            //像素到光心轴之间的距离
            double r=sqrt(pow(x_undistort,2)+pow(y_undistort,2));
            x_distort=x_undistort*(1+k1*pow(r,2)+k2*pow(r,4)) +2*p1*x_undistort*y_undistort +p2*(pow(r,2)+2*x_undistort*x_undistort);
            y_distort=y_undistort*(1+k1*pow(r,2)+k2*pow(r,4)) +2*p2*x_undistort*y_undistort +p1*(pow(r,2)+2*y_undistort*y_undistort);

            //畸变后的像素坐标（u_distorted，v_distorted）
            u_distorted=x_distort*fx+cx;
            v_distorted=y_distort*fy+cy;

            // end your code here

            // 赋值 (最近邻插值)
            if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols && v_distorted < rows) {
                image_undistort.at<uchar>(v, u) = image.at<uchar>((int) v_distorted, (int) u_distorted);
            } else {
                image_undistort.at<uchar>(v, u) = 0;
            }
        }

    // 画图去畸变后图像
    cv::imshow("image undistorted", image_undistort);
    cv::waitKey();

    return 0;
}

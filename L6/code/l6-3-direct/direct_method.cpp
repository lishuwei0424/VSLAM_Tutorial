#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include <Eigen/Core>
#include <vector>
#include <string>
#include <boost/format.hpp>
#include <pangolin/pangolin.h>
#include <chrono>

bool DEBUG = false;

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace Sophus;
using namespace boost;
using namespace pangolin;
using namespace chrono;

typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;

// Camera intrinsics
// 内参
double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
// 基线
double baseline = 0.573;
// paths
string left_file = "../left.png";
string disparity_file = "../disparity.png";
boost::format fmt_others("../%06d.png");    // other files
// useful typedefs
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 2, 6> Matrix26d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

// TODO implement this function
/**
 * pose estimation using direct method
 * @param img1
 * @param img2
 * @param px_ref
 * @param depth_ref
 * @param T21
 */
void DirectPoseEstimationMultiLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3 &T21
);

// TODO implement this function
/**
 * pose estimation using direct method
 * @param img1
 * @param img2
 * @param px_ref
 * @param depth_ref
 * @param T21
 */
void DirectPoseEstimationSingleLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3 &T21
);

// bilinear interpolation
inline float GetPixelValue(const cv::Mat &img, float x, float y) {
    uchar *data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
            (1 - xx) * (1 - yy) * data[0] +
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[img.step] +
            xx * yy * data[img.step + 1]
    );
}

int main(int argc, char **argv) {

    cv::Mat left_img = cv::imread(left_file, 0);
    cv::Mat disparity_img = cv::imread(disparity_file, 0);

    // let's randomly pick pixels in the first image and generate some 3d points in the first image's frame
    cv::RNG rng;
    int nPoints = 1000;
    int boarder = 40;
    VecVector2d pixels_ref;
    vector<double> depth_ref;

    // generate pixels in ref and load depth data
    for (int i = 0; i < nPoints; i++) {
        int x = rng.uniform(boarder, left_img.cols - boarder);  // don't pick pixels close to boarder
        int y = rng.uniform(boarder, left_img.rows - boarder);  // don't pick pixels close to boarder
        int disparity = disparity_img.at<uchar>(y, x);
        double depth = fx * baseline / disparity; // you know this is disparity to depth
        depth_ref.push_back(depth);
        pixels_ref.push_back(Eigen::Vector2d(x, y));
    }

    // estimates 01~05.png's pose using this information
    Sophus::SE3 T_cur_ref;//(Matrix3d::Identity(),Vector3d::Zero());

    for (int i = 1; i < 6; i++) {  // 1~10
    //	if(5 == i) DEBUG = true;
	cv::Mat img = cv::imread((fmt_others % i).str(), 0);
	steady_clock::time_point t1 = steady_clock::now();
        //DirectPoseEstimationSingleLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref);    // first you need to test single layer
        DirectPoseEstimationMultiLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref);
	steady_clock::time_point t2 = steady_clock::now();
	duration<double> time_used = duration_cast<duration<double>>(t2 - t1);
	cout<<"Image "<<i<<" time cost= "<< (time_used.count() * 1000) <<" ms. T_cur_ref is : \n"<<T_cur_ref.matrix()<<endl;
//	waitKey();
    }
}
//Caculate the position in Frame 2's camera frame and pixe frame
inline void GetCurrFramePostionFromRefFrame(
	/*IN*/ float u1, float v1, double depth, SE3 T21,
	/*OUT*/ double &xc2, double &yc2, double &zc2, float &u2, float &v2)
{
    double zc1 = depth;
    double xc1 = double(zc1 / fx * (u1 - cx)); 
    double yc1 = double(zc1 / fy * (v1 - cy));
    Vector3d Pc1(xc1, yc1, zc1);
    Vector3d Pc2 = T21 * Pc1;
    xc2 = Pc2[0];  yc2 = Pc2[1];  zc2 = Pc2[2];
    u2 = float(fx * xc2 / zc2 + cx);
    v2 = float(fy * yc2 / zc2 + cy); 
                    
}

bool bInImage(float u, float v, int w, int h) {
    if(u >= 0 && u < w && v >= 0 && v < h) {
	return true;	
    } else {
	return false;	
    }
}

void DirectPoseEstimationSingleLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3 &T21
) {
    if(DEBUG) cout<<"Single Layer T21 Init: "<<T21.matrix()<<endl;

    // parameters
    int half_patch_size = 4;
    int iterations = 100;

    double cost = 0, lastCost = 0;
    int nGood = 0;  // good projections
    VecVector2d goodProjection;

    for (int iter = 0; iter < iterations; iter++) {
        nGood = 0;
        goodProjection.clear();

        // Define Hessian and bias
        Matrix6d H = Matrix6d::Zero();  // 6x6 Hessian
        Vector6d b = Vector6d::Zero();  // 6x1 bias

        for (size_t i = 0; i < px_ref.size(); i++) {

            // compute the projection in the second image
            // TODO START YOUR CODE HERE
            float u = px_ref[i][0];
	    float v = px_ref[i][1];
	    double depth = depth_ref[i];
            double xc2_opt, yc2_opt, zc2_opt;
	    float u2_opt, v2_opt;
	    GetCurrFramePostionFromRefFrame(u, v, depth, T21, xc2_opt, yc2_opt, zc2_opt, u2_opt, v2_opt);
	    if(DEBUG) cout<<"img1 point num is "<<i<<" Puv1: "<<u<<","<<v<<" Puv2: "<<u2_opt<<","<<v2_opt<<endl;
	    if(!bInImage(u - half_patch_size, v - half_patch_size, img1.cols, img1.rows) ||
		!bInImage(u + half_patch_size - 1, v + half_patch_size - 1, img1.cols, img1.rows) ||
		!bInImage(u2_opt - half_patch_size, v2_opt - half_patch_size, img2.cols, img2.rows) || 
		!bInImage(u2_opt + half_patch_size - 1, v2_opt + half_patch_size - 1, img2.cols, img2.rows)) {
		if(DEBUG) cout<<"ERROR: Puv1 or Puv2 is not in the Image!"<<endl;
		continue;
	    }
            nGood++;
            goodProjection.push_back(Eigen::Vector2d(u2_opt, v2_opt));

	    double xc2 = xc2_opt;
	    double yc2 = yc2_opt;
	    double zc2 = zc2_opt;
	
            Matrix26d J_pixel_xi = Matrix26d::Zero();   // pixel to \xi in Lie algebra
	    double xc2_2 = xc2 * xc2;  double yc2_2 = yc2 * yc2;   double zc2_2 = zc2 * zc2;
	    J_pixel_xi(0,0) = fx / zc2;
	    J_pixel_xi(0,2) = - fx * xc2 / zc2_2;
	    J_pixel_xi(0,3) = - fx * xc2 * yc2 / zc2_2;
	    J_pixel_xi(0,4) = fx + fx * xc2_2 / zc2_2;
	    J_pixel_xi(0,5) = - fx * yc2 / zc2;
	    
	    J_pixel_xi(1,1) = fy / zc2;
	    J_pixel_xi(1,2) = -fy * yc2 / zc2_2;
	    J_pixel_xi(1,3) = -fy - fy * yc2_2 / zc2_2;
	    J_pixel_xi(1,4) = fy * xc2 * yc2 / zc2_2;
	    J_pixel_xi(1,5) = fy * xc2 / zc2;
     
            // and compute error and jacobian
            for (int x = -half_patch_size; x < half_patch_size; x++)
                for (int y = -half_patch_size; y < half_patch_size; y++) 
		//int x = 0; int y = 0;
		{
		    float u1 = float(u + x);
		    float v1 = float(v + y);
		    //double xc2, yc2, zc2;
/*		    double xc2 = xc2_opt;
		    double yc2 = yc2_opt;
		    double zc2 = zc2_opt;*/
		    float u2 = float(u2_opt + x);
		    float v2 = float(v2_opt + y);
		    //GetCurrFramePostionFromRefFrame(u1, v1, depth, T21, xc2, yc2, zc2, u2, v2);
			
                    double error = GetPixelValue(img1, u1, v1) - GetPixelValue(img2, u2, v2);

/*                    Matrix26d J_pixel_xi = Matrix26d::Zero();   // pixel to \xi in Lie algebra
		    double xc2_2 = xc2 * xc2;  double yc2_2 = yc2 * yc2;   double zc2_2 = zc2 * zc2;
		    J_pixel_xi(0,0) = fx / zc2;
		    J_pixel_xi(0,2) = - fx * xc2 / zc2_2;
		    J_pixel_xi(0,3) = - fx * xc2 * yc2 / zc2_2;
		    J_pixel_xi(0,4) = fx + fx * xc2_2 / zc2_2;
		    J_pixel_xi(0,5) = - fx * yc2 / zc2;
		    
		    J_pixel_xi(1,1) = fy / zc2;
		    J_pixel_xi(1,2) = -fy * yc2 / zc2_2;
		    J_pixel_xi(1,3) = -fy - fy * yc2_2 / zc2_2;
		    J_pixel_xi(1,4) = fy * xc2 * yc2 / zc2_2;
		    J_pixel_xi(1,5) = fy * xc2 / zc2;*/
                    
		    Eigen::Vector2d J_img_pixel = Vector2d::Zero();    // image gradients
		    if(!bInImage(u2 - 1, v2 - 1, img2.cols, img2.rows) && 
			!bInImage(u2 + 1, v2 + 1, img2.cols, img2.rows)) {
			continue;
		    }
		    J_img_pixel[0] = (GetPixelValue(img2, u2 + 1, v2) - GetPixelValue(img2, u2 - 1, v2)) / 2;
		    J_img_pixel[1] = (GetPixelValue(img2, u2, v2 + 1) - GetPixelValue(img2, u2, v2 - 1)) / 2 ;	
                    // total jacobian
                    Vector6d J = -J_pixel_xi.transpose() * J_img_pixel;

                    H += J * J.transpose();
                    b += -error * J;
                    cost += error * error;
                }
            // END YOUR CODE HERE
        }

        // solve update and put it into estimation
        // TODO START YOUR CODE HERE
        Vector6d update;
	update = H.ldlt().solve(b);
	if(DEBUG) cout<<"iter: "<<iter<<" update: "<<update.transpose()<<endl;
        T21 = Sophus::SE3::exp(update) * T21;
        // END YOUR CODE HERE

        cost /= nGood;

        if (std::isnan(update[0])) {
            // sometimes occurred when we have a black or white patch and H is irreversible
            cout << "update is nan" << endl;
            break;
        }
        if (iter > 0 && cost > lastCost) {
            if(DEBUG) cout << "iter: "<<iter<<" cost increased: " << cost << ", " << lastCost << endl;
            break;
        }
        lastCost = cost;
	if(DEBUG) cout << "iter: " << iter << " cost = " << cost << ", good = " << nGood << endl;
    }
    if(DEBUG) cout << "good projection: " << nGood << endl;
//    cout << "T21 = \n" << T21.matrix() << endl;

    // in order to help you debug, we plot the projected pixels here
/*    cv::Mat img1_show, img2_show;
    cv::cvtColor(img1, img1_show, CV_GRAY2BGR);
    cv::cvtColor(img2, img2_show, CV_GRAY2BGR);
    float half_width = float(img1.cols / 2); 
    float half_height = float(img1.rows / 2);
    float max_distance = half_width * half_width + half_height * half_height;
    for (auto &px: px_ref) {
	float kesi = ((px[0] - half_width) * (px[0] - half_width) + (px[1] - half_height) * (px[1] - half_height)) / max_distance;
        cv::rectangle(img1_show, cv::Point2f(px[0] - 2, px[1] - 2), cv::Point2f(px[0] + 2, px[1] + 2),
                      cv::Scalar(255 * (1-kesi), 255, 255 * kesi));
    }
    for (auto &px: goodProjection) {
	float kesi = ((px[0] - half_width) * (px[0] - half_width) + (px[1] - half_height) * (px[1] - half_height)) / max_distance;
        cv::rectangle(img2_show, cv::Point2f(px[0] - 2, px[1] - 2), cv::Point2f(px[0] + 2, px[1] + 2),
                      cv::Scalar(255 * (1-kesi), 255, 255 * kesi));
    }
    cv::imshow("reference", img1_show);
    cv::imshow("current", img2_show);
    cv::waitKey();*/
}

void DirectPoseEstimationMultiLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3 &T21
) {

    // parameters
    int pyramids = 4;
    double pyramid_scale = 0.5;
    double scales[] = {1.0, 0.5, 0.25, 0.125};

    // create pyramids
    vector<cv::Mat> pyr1, pyr2; // image pyramids
    // TODO START YOUR CODE HERE
    for(int i = 0; i < pyramids; i++) {
	Mat img1_temp, img2_temp;
	resize(img1, img1_temp, Size(img1.cols * scales[i], img1.rows * scales[i]));
	resize(img2, img2_temp, Size(img2.cols * scales[i], img2.rows * scales[i]));
	pyr1.push_back(img1_temp);
	pyr2.push_back(img2_temp);
	if(DEBUG) cout<<"Level "<<i<<" pyr1 size: "<<pyr1[i].cols<<" "<<pyr1[i].rows<<
		" pyr2 size: "<<pyr2[i].cols<<" "<<pyr2[i].rows<<endl;
    }
    // END YOUR CODE HERE

    double fxG = fx, fyG = fy, cxG = cx, cyG = cy;  // backup the old values
    for (int level = pyramids - 1; level >= 0; level--) {
        VecVector2d px_ref_pyr; // set the keypoints in this pyramid level
        for (auto &px: px_ref) {
            px_ref_pyr.push_back(scales[level] * px);
        }
        // TODO START YOUR CODE HERE
        // scale fx, fy, cx, cy in different pyramid levels
	fx = fxG * scales[level];
	fy = fyG * scales[level];
	cx = cxG * scales[level];
	cy = cyG * scales[level];
        // END YOUR CODE HERE
        DirectPoseEstimationSingleLayer(pyr1[level], pyr2[level], px_ref_pyr, depth_ref, T21);
    }
}

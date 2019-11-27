#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/so3.h"
#include "sophus/se3.h"

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "ceres/autodiff.h"

#include "tools/rotation.h"
#include "common/projection.h"

using namespace Eigen;
using namespace Sophus;
using namespace g2o;

#define DEBUG false

class VetexCamInput {
public:
    VetexCamInput(){}
    VetexCamInput(VectorXd camera){
	//exchange angle-axis and t,
	//because in se3() the head is t, the tail is R
	Vector6d se3;
	se3.head<3>() = camera.block<3,1>(3,0);
	se3.tail<3>() = camera.head<3>();
	_SE3 = SE3::exp(se3);
	_f  = camera[6];
	_k1 = camera[7];
	_k2 = camera[8]; 	
    }

public:
    SE3 _SE3;
    double _f;
    double _k1;
    double _k2;
 
};

//Input: SE3
//Output: Puv
inline bool CamProjectionWithDistortion2(VetexCamInput camin, Vector3d Pw, double* predictions){
    Vector3d Pc;
    Pc = camin._SE3 * Pw;

    // Compute the center fo distortion
    double xp = -Pc[0]/Pc[2];
    double yp = -Pc[1]/Pc[2];

    double r2 = xp*xp + yp*yp;
    double distortion = double(1.0) + r2 * (camin._k1 + camin._k2 * r2);

    predictions[0] = camin._f * distortion * xp;
    predictions[1] = camin._f * distortion * yp;

    return true;
}

class VertexCameraBAL : public g2o::BaseVertex<9, VetexCamInput>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexCameraBAL() {}

    virtual bool read ( std::istream& /*is*/ )
    {
        return false;
    }

    virtual bool write ( std::ostream& /*os*/ ) const
    {
        return false;
    }

    virtual void setToOriginImpl() {}

    virtual void oplusImpl ( const double* update )
    {
	if(DEBUG) cout<<"1 VertexCameraBAL update: "<<update[0]<<update[1]<<update[2]<<update[3]<<update[4]<<update[5]<<update[6]<<update[7]<<update[8]<<endl;
        //Eigen::VectorXd::ConstMapType v ( update, VertexCameraBAL::Dimension );
        //_estimate += v;
	Vector6d update_se3;
	//Attention: head is t, tail is R
	update_se3 << update[0],update[1],update[2],update[3],update[4],update[5];
	_estimate._SE3 = SE3::exp(update_se3) * estimate()._SE3;
	_estimate._f  += update[6];
	_estimate._k1 += update[7];
	_estimate._k2 += update[8];
    }

};


class VertexPointBAL : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexPointBAL() {}

    virtual bool read ( std::istream& /*is*/ )
    {
        return false;
    }

    virtual bool write ( std::ostream& /*os*/ ) const
    {
        return false;
    }

    virtual void setToOriginImpl() {}

    virtual void oplusImpl ( const double* update )
    {
        Eigen::Vector3d::ConstMapType v ( update );
        _estimate += v;
    }
};

class EdgeObservationBAL : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexCameraBAL, VertexPointBAL>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeObservationBAL() {}

    virtual bool read ( std::istream& /*is*/ )
    {
        return false;
    }

    virtual bool write ( std::ostream& /*os*/ ) const
    {
        return false;
    }

    virtual void computeError() override   // The virtual function comes from the Edge base class. Must define if you use edge.
    {
        const VertexCameraBAL* cam = static_cast<const VertexCameraBAL*> ( vertex ( 0 ) );
        const VertexPointBAL* point = static_cast<const VertexPointBAL*> ( vertex ( 1 ) );

	double predictions[2];
	CamProjectionWithDistortion2(cam->estimate(), point->estimate(), predictions);
	_error[0] = double(measurement()(0)) - predictions[0];
	_error[1] = double(measurement()(1)) - predictions[1]; 
    }


    virtual void linearizeOplus() override
    {
        VertexCameraBAL* cam = static_cast<VertexCameraBAL*> ( vertex ( 0 ) );
       	VertexPointBAL* point = static_cast<VertexPointBAL*> ( vertex ( 1 ) );
   	
	Vector3d Pc;
    	Pc = cam->estimate()._SE3 * point->estimate();
	// camera frame
	double xc = Pc[0];
	double yc = Pc[1];
	double zc = Pc[2];
    	// nomorlized camera frame
    	double xc1 = -xc/zc;
    	double yc1 = -yc/zc;
	double zc1 = -1;

    	// Apply second and fourth order radial distortion
    	const double& k1 = cam->estimate()._k1;
    	const double& k2 = cam->estimate()._k2;

    	double r2 = xc1 * xc1 + yc1 * yc1;
    	double d = double(1.0) + k1 * r2 + k2 * r2 * r2;

    	const double& f = cam->estimate()._f;
    	//predictions[0] = -focal * distortion * xp;
    	//predictions[1] = -focal * distortion * yp;
	Matrix<double, 2, 6> J_e_kesi;
	Matrix<double, 2, 3> J_e_pc;
	Matrix<double, 3, 6> J_pc_kesi;// = new Eigen::Matrix<double,3,6>::Zero();
	Matrix<double, 2, 1> J_e_f;
	Matrix<double, 2, 2> J_e_k;
	double zc_2 = zc * zc;
	double zc_3 = zc_2 * zc;
	double d2 = k1 + 2 * k2 * r2;

	J_e_pc(0,0) = f / zc * d + 2 * f * xc * xc / zc_3 * d2;
	J_e_pc(0,1) = 2 * f * xc * yc /zc_3 * d2;
	J_e_pc(0,2) = -f * xc/zc_2 * d - 2 * f * xc * r2 / zc_2 * d2;
	J_e_pc(1,0) = 2 * f * xc * yc / zc_3 * d2;
	J_e_pc(1,1) = f / zc * d + 2 * f * yc * yc /zc_3 * d2;
	J_e_pc(1,2) = -f * yc/zc_2 * d - 2 * f * yc * r2 / zc_2 * d2;
	if(DEBUG) cout<<"J_e_pc: \n"<<J_e_pc.matrix()<<endl;

	J_pc_kesi(0,0) = 1;
	J_pc_kesi(0,1) = 0;
	J_pc_kesi(0,2) = 0;
	J_pc_kesi(0,3) = 0;
	J_pc_kesi(0,4) = zc;
	J_pc_kesi(0,5) = -yc;

	J_pc_kesi(1,0) = 0;
	J_pc_kesi(1,1) = 1;
	J_pc_kesi(1,2) = 0;
	J_pc_kesi(1,3) = -zc;
	J_pc_kesi(1,4) = 0;
	J_pc_kesi(1,5) = xc;

	J_pc_kesi(2,0) = 0;
	J_pc_kesi(2,1) = 0;
	J_pc_kesi(2,2) = 1;
	J_pc_kesi(2,3) = yc;
	J_pc_kesi(2,4) = -xc;
	J_pc_kesi(2,5) = 0;
	if(DEBUG) cout<<"J_pc_kesi: \n"<<J_pc_kesi.matrix()<<endl;

	J_e_kesi = J_e_pc * J_pc_kesi;
	
	J_e_f(0,0) = xc / zc * d;
	J_e_f(1,0) = yc / zc * d;
	if(DEBUG) cout<<"J_e_f:\n"<<J_e_f<<endl;

	J_e_k(0,0) = f * xc * r2 / zc;
	J_e_k(0,1) = f * xc * r2 * r2 / zc;
	J_e_k(1,0) = f * yc * r2 / zc;
	J_e_k(1,1) = f * yc * r2 * r2 / zc;
	if(DEBUG) cout<<"J_e_k:\n"<<J_e_k<<endl;

	_jacobianOplusXi.block<2,6>(0,0) = J_e_kesi;
	_jacobianOplusXi.block<2,1>(0,6) = J_e_f;
	_jacobianOplusXi.block<2,2>(0,7) = J_e_k;
	if(DEBUG) cout<<"_jacobianOplusXi:\n"<<_jacobianOplusXi<<endl;
	
	Matrix<double, 2, 3> J_e_pw;
	J_e_pw = J_e_pc * cam->estimate()._SE3.rotation_matrix();
	_jacobianOplusXj = J_e_pw; 
	if(DEBUG) cout<<"_jacobianOplusXj:\n"<<_jacobianOplusXj<<endl;
    }
};

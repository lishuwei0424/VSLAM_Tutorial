#include <iostream>
#include <Eigen/Core>
#include "sophus/se3.h"
#include <fstream>
#include <pangolin/pangolin.h>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <pangolin/pangolin.h>

using namespace std;
using namespace Eigen;
using namespace Sophus;
using namespace pangolin;
using namespace g2o;
using namespace pangolin;

typedef vector<Vector3d, aligned_allocator<Vector3d> > vVector3d;

vVector3d vt_w_cg;
vVector3d vt_w_ce;
vVector3d vt_w_ce2;

string sFilePath = "../compare.txt";
ifstream ifFile;

Quaterniond q_w_cg;
Vector3d t_w_cg;
//SE3 T_wg;

Quaterniond q_w_ce;
Vector3d t_w_ce;
//SE3 T_we;

double timestamp = 0;

//pose
Matrix3d R_wg_we = Matrix3d::Identity();
Vector3d t_wg_we = Vector3d::Zero();
Isometry3d isoT_wg_we;

void DrawTrajectory(vVector3d pose1, vVector3d pose2);

class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Vector3d, VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeProjectXYZRGBDPoseOnly(const Vector3d& p_we): _p_we( p_we ){};
    virtual void computeError()
    {
    	const VertexSE3Expmap* T_wg_we = static_cast<const VertexSE3Expmap*>(_vertices[0]);
	_error = _measurement - T_wg_we->estimate().map( _p_we ); 
    }
    virtual void linearizeOplus()
    {
	VertexSE3Expmap* T_wg_we = static_cast<VertexSE3Expmap*>( _vertices[0]);
	Vector3d p_wg = T_wg_we->estimate().map( _p_we );
        double x = p_wg[0];
	double y = p_wg[1];
	double z = p_wg[2];
	
	_jacobianOplusXi(0,0) = 0;
	_jacobianOplusXi(0,1) = -z;
	_jacobianOplusXi(0,2) = y;
	_jacobianOplusXi(0,3) = -1;
	_jacobianOplusXi(0,4) = 0;
	_jacobianOplusXi(0,5) = 0;

	_jacobianOplusXi(1,0) = z;
	_jacobianOplusXi(1,1) = 0;
	_jacobianOplusXi(1,2) = -x;
	_jacobianOplusXi(1,3) = 0;
	_jacobianOplusXi(1,4) = -1;
	_jacobianOplusXi(1,5) = 0;

	_jacobianOplusXi(2,0) = -y;
	_jacobianOplusXi(2,1) = x;
	_jacobianOplusXi(2,2) = 0;
	_jacobianOplusXi(2,3) = 0;
	_jacobianOplusXi(2,4) = 0;
	_jacobianOplusXi(2,5) = -1;
    }
    bool read(istream& in) {};
    bool write(ostream& out) const {};
protected:
    Vector3d _p_we;
};


void ICP_BundleAdjustment()
{
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>();
    Block* solver_ptr = new Block(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr);
    
    g2o::SparseOptimizer optimizer;    
    optimizer.setAlgorithm(solver);
 
    //vertex T_wg_we
    g2o::VertexSE3Expmap* T_wg_we = new g2o::VertexSE3Expmap();
    T_wg_we -> setId(0);
    T_wg_we -> setEstimate(g2o::SE3Quat(R_wg_we, t_wg_we ));
    optimizer.addVertex(T_wg_we);
 
    //edge
    int index = 1;
    vector<EdgeProjectXYZRGBDPoseOnly*> edges;
    for(size_t i = 0; i < vt_w_ce.size(); i++)
    {
    	EdgeProjectXYZRGBDPoseOnly* edge = new EdgeProjectXYZRGBDPoseOnly(vt_w_ce[i]);
	edge->setId(index);
	edge->setVertex(0, dynamic_cast<VertexSE3Expmap*>(T_wg_we));
	edge->setMeasurement(vt_w_cg[i]);
	edge->setInformation(Matrix3d::Identity() * 1e4);
	optimizer.addEdge(edge); 
	index++;
	edges.push_back(edge);
    }   
    
    //optimize
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    isoT_wg_we = Isometry3d(T_wg_we->estimate());
    cout<< "T_wg_we: \n"<< isoT_wg_we.matrix()<<endl;
}


int main()
{
    cout<<"ICP main..."<<endl;
    ifFile.open(sFilePath.c_str());
    if(!ifFile.is_open()) {
    	cerr<<"isFile is not open! sFilePath: " << sFilePath.c_str() << endl;
	return -1;
    }
    string sFile;
    while(getline(ifFile,sFile) && !sFile.empty()) {
	istringstream iss(sFile);
	iss >> timestamp >> t_w_ce[0] >> t_w_ce[1] >> t_w_ce[2] >>
		q_w_ce.x() >> q_w_ce.y() >> q_w_ce.z() >> q_w_ce.w() >>
		timestamp >> t_w_cg[0] >> t_w_cg[1] >> t_w_cg[2] >>
		q_w_cg.x() >> q_w_cg.y() >> q_w_cg.z() >> q_w_cg.w();
	//cout << "t_we: " << t_we.transpose() << "q_we: " << q_we.coeffs().transpose() << endl;
	//cout << "t_wg: " << t_wg.transpose() <<" q_wg: "<< q_wg.coeffs().transpose() << endl;
	vt_w_cg.push_back(t_w_cg);
	vt_w_ce.push_back(t_w_ce);
    }

    cout<<"vt_w_cg size: "<<vt_w_cg.size() <<" vt_w_ce size: "<<vt_w_ce.size()<<endl;
    
    DrawTrajectory(vt_w_cg, vt_w_ce);    
    
    ICP_BundleAdjustment();
    for(int i = 0; i< vt_w_ce.size();i++) {
    	vt_w_ce2.push_back(isoT_wg_we * vt_w_ce[i]);
    }
//    DrawTrajectory(vt_w_cg, vt_w_ce2);
    return 0;
}

void DrawTrajectory(vVector3d pose1, vVector3d pose2)
{
    if(pose1.empty() || pose2.empty()) {
    	cerr << "pose1 or pose2 is empty!" << endl;
	return;
    }

    CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    OpenGlRenderState s_cam(
	ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
	ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));
   
    View &d_cam = CreateDisplay()
	.SetBounds(0.0, 1.0, Attach::Pix(175), 1.0, -1024.0f / 768.0f)
	.SetHandler(new Handler3D(s_cam));

    while(ShouldQuit() == false) 
    {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	d_cam.Activate(s_cam);
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glLineWidth(4);
	for(size_t i = 0; i< pose1.size() -1 ; i++)
	{
	    glColor3f(1.0f, 0.0f, 0.0f);
	    glBegin(GL_LINES);
	    auto p1_1 = pose1[i]; auto p1_2 = pose1[i+1];
	    glVertex3d(p1_1[0], p1_1[1], p1_1[2]);
	    glVertex3d(p1_2[0], p1_2[1], p1_2[2]);
	    glEnd();
	}

	for(size_t i = 0; i < pose2.size() -1; i++)
	{
	    glColor3f(0.0f, 0.0f,1.0f);
	    glBegin(GL_LINES);
	    auto p2_1 = pose2[i]; auto p2_2 = pose2[i+1];
	    glVertex3d(p2_1[0], p2_1[1], p2_1[2]);
	    glVertex3d(p2_2[0], p2_2[1], p2_2[2]);
	    glEnd();
	}
	FinishFrame();
	usleep(5000);
    }  
}

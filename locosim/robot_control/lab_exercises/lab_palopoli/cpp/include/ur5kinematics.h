#ifndef UR5KIMENATICS_H
#define UR5KIMENATICS_H

#define SCALE_FACTOR 10
#define OMEGA 1
#define SAMPLES 500
#define DELTAT 0.02
#define LAPS 5.0
#define TMIN 0.0
#define TMAX 1 
#define HTAVOLO 0.75                    //per invDiffKinematicControlSimComplete
//#define TMAX 2*M_PI/OMEGA*LAPS    per invDiffKinematicControlSim


#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

typedef Matrix<double,6,1> Vector6d;
typedef Matrix<double,8,1> Vector8d;
typedef Matrix<double,8,6> Matrix86d;
typedef Matrix<double,6,6> Matrix6d;
typedef Matrix<double,3,6> Matrix36d;
typedef Matrix<double,6,3> Matrix63d;



Vector3d xe0,phie0,xef,phief;
Matrix3d Kp,Kphi;


Vector6d A;
Vector6d D;

Matrix4d T65m;
Matrix4d T54m;
Matrix4d T43m;
Matrix4d T32m;
Matrix4d T21m;
Matrix4d T10m;

Matrix3d Re;
Vector6d pos_attuale;

Matrix4d T10(double th);
Matrix4d T21(double th);
Matrix4d T32(double th);
Matrix4d T43(double th);
Matrix4d T54(double th);
Matrix4d T65(double th);

Matrix86d ur5Inverse(Vector3d pe);
Vector3d ur5Direct(Vector6d Th);
Matrix6d ur5Jac(Vector6d Th);
Vector3d XD(Vector3d xe0,Vector3d xef,double t);
Vector3d Phid(Vector3d phie0,Vector3d phief,double t);
Vector3d rotm2eulFDR(Matrix3d R);
Matrix3d eul2rotmFDR(Vector3d eulXYZ);
Vector3d computeOrientationErrorW(Matrix3d w_R_e, Matrix3d w_R_d);
//Vector6d invDiffKinematiControl(Matrix86d qk, Vector3d xe, Vector3d xd,  Vector3d vd, Matrix3d K);
//Eigen::MatrixXd invDiffKinematicControlSim(Vector3d xd, Vector6d TH0, Matrix3d K, double minT, double maxT, double Dt);
MatrixXd invDiffKinematicControlSimComplete(Vector3d xe0,Vector3d xef,Vector3d phie0,Vector3d phief,Vector6d TH0, Matrix3d Kp, Matrix3d Kphi, double minT, double maxT, double Dt);
Vector6d invDiffKinematiControlComplete(Vector6d q, Vector3d xe, Vector3d xd, Vector3d vd, Matrix3d w_R_e, Vector3d phid, Vector3d phiddot, Matrix3d Kp, Matrix3d Kphi);

#endif

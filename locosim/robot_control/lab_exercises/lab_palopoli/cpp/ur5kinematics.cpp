#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include "include/ur5kinematics.h"

using namespace std ;
using namespace Eigen ;


Matrix4d T10(double th){
        Matrix4d T10f;
        T10f << cos(th), -sin(th), 0, 0,
                sin(th), cos(th), 0, 0,
                0, 0, 1, D(0),
                0, 0, 0, 1;
        return T10f;
}
Matrix4d T21(double th){
        Matrix4d T21f;
        T21f << cos(th), -sin(th), 0, 0,
                0, 0, -1, 0,
                sin(th), cos(th), 0, 0,
                0, 0, 0, 1;
        return T21f;
}
Matrix4d T32(double th){
        Matrix4d T32f;
        T32f << cos(th), -sin(th), 0, A(1),
                sin(th), cos(th), 0, 0,
                0, 0, 1, D(2),
                0, 0, 0, 1;
        return T32f;
}
Matrix4d T43(double th){
        Matrix4d T43f;
        T43f << cos(th), -sin(th), 0, A(2),
                sin(th), cos(th), 0, 0,
                0, 0, 1, D(3),
                0, 0, 0, 1;
        return T43f;
}
Matrix4d T54(double th){
        Matrix4d T54f;
        T54f << cos(th), -sin(th), 0, 0,
                0, 0, -1, -D(4),
                sin(th), cos(th), 0, 0,
                0, 0, 0, 1;
        return T54f;        
}
Matrix4d T65(double th){
        Matrix4d T65f;
        T65f << cos(th), -sin(th), 0, 0,
                0, 0, 1, D(5),
                -sin(th), -cos(th), 0, 0,
                0, 0, 0, 1;
        return T65f;        
}


Vector3d ur5Direct(Vector6d Th){

        A << 0.0, -0.425, -0.3922, 0.0, 0.0, 0.0; 
        D << 0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996;

        Vector6d alfa;
        alfa << 0, M_PI/2, 0, 0, M_PI/2, -M_PI/2;

        T10m=T10(Th(0));
        T21m=T21(Th(1));
        T32m=T32(Th(2));
        T43m=T43(Th(3));
        T54m=T54(Th(4));
        T65m=T65(Th(5));

        Matrix4d T06 =T10m*T21m*T32m*T43m*T54m*T65m;

        Vector3d pe;
        pe << T06(0,3),T06(1,3),T06(2,3);

        Re << T06(0,0),T06(0,1),T06(0,2),
              T06(1,0),T06(1,1),T06(1,2),
              T06(2,0),T06(2,1),T06(2,2);


        return pe; 

}

Matrix86d ur5Inverse(Vector3d pe){

        

        A << 0.0, -0.425, -0.3922, 0.0, 0.0, 0.0; 
        D << 0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996;
        
        Matrix4d R;
        R << -1,0,0,0,
              0,1,0,0,
              0,0,-1,0,
              0,0,0,1;

        Vector4d pe_homogeneous;
        pe_homogeneous << pe, 1;
        Eigen::Vector4d result=R*pe_homogeneous;
        Vector3d peInv = result.head(3);

        Matrix4d T60;
        T60 <<  1,0,0,peInv(0),
                0,1,0,peInv(1),
                0,0,1,peInv(2),
                0,0,0,1;

        Matrix4d T60I=T60;

        Vector4d mul;
        mul << 0,0,-D(5),1;

        Vector4d p50;
        p50=T60*mul;

        double th1_1 = real(atan2(p50(1), p50(0)) + acos(complex<double>(D(3)/hypot(p50(1), p50(0)),0.0)).real())+M_PI/2;
        double th1_2 = real(atan2(p50(1), p50(0)) - acos(complex<double>(D(3)/hypot(p50(1), p50(0)),0.0)).real())+M_PI/2;

        double th5_1 = +real(acos(complex<double>((pe(0)*sin(th1_1) - pe(1)*cos(th1_1)-D(3),0.0)) / D(5)).real());
        double th5_2 = -real(acos(complex<double>((pe(0)*sin(th1_1) - pe(1)*cos(th1_1)-D(3),0.0)) / D(5)).real());
        double th5_3 = +real(acos(complex<double>((pe(0)*sin(th1_2) - pe(1)*cos(th1_2)-D(3),0.0)) / D(5)).real());
        double th5_4 = -real(acos(complex<double>((pe(0)*sin(th1_2) - pe(1)*cos(th1_2)-D(3),0.0)) / D(5)).real());

        Matrix4d T06=T60.inverse();

        Vector3d Xhat;
        Xhat << T06(0,0),T06(1,0),T06(2,0);

        Vector3d Yhat;
        Yhat << T06(0,1),T06(1,1),T06(2,1);

        double th6_1 = real(atan2(((-Xhat(1)*sin(th1_1)+Yhat(1)*cos(th1_1)))/sin(th5_1), ((Xhat(0)*sin(th1_1)-Yhat(0)*cos(th1_1))/sin(th5_1))));
        double th6_2 = real(atan2(((-Xhat(1)*sin(th1_1)+Yhat(1)*cos(th1_1)))/sin(th5_2), ((Xhat(0)*sin(th1_1)-Yhat(0)*cos(th1_1))/sin(th5_2))));
        double th6_3 = real(atan2(((-Xhat(1)*sin(th1_2)+Yhat(1)*cos(th1_2)))/sin(th5_3), ((Xhat(0)*sin(th1_2)-Yhat(0)*cos(th1_2))/sin(th5_3))));
        double th6_4 = real(atan2(((-Xhat(1)*sin(th1_2)+Yhat(1)*cos(th1_2)))/sin(th5_4), ((Xhat(0)*sin(th1_2)-Yhat(0)*cos(th1_2))/sin(th5_4))));

        Matrix4d T41m=T10(th1_1).inverse()*T60*T65(th6_1).inverse()*T54(th5_1).inverse();
        Vector3d p41_1;
        p41_1 << T41m(0,3),T41m(1,3),T41m(2,3);
        double p41xz_1 = hypot(p41_1(0), p41_1(2));

        T41m=T10(th1_1).inverse()*T60*T65(th6_2).inverse()*T54(th5_2).inverse();
        Vector3d p41_2;
        p41_2 << T41m(0,3),T41m(1,3),T41m(2,3);
        double p41xz_2 = hypot(p41_2(0), p41_2(2));
 
        T41m=T10(th1_2).inverse()*T60*T65(th6_3).inverse()*T54(th5_3).inverse();
        Vector3d p41_3;
        p41_3 << T41m(0,3),T41m(1,3),T41m(2,3);
        double p41xz_3 = hypot(p41_3(0), p41_3(2));

        T41m=T10(th1_2).inverse()*T60*T65(th6_4).inverse()*T54(th5_4).inverse();
        Vector3d p41_4;
        p41_4 << T41m(0,3),T41m(1,3),T41m(2,3);
        double p41xz_4 = hypot(p41_4(0), p41_4(2));

        double th3_1 = real(acos(complex<double>((pow(p41xz_1,2)-pow(A(1),2)-pow(A(2),2)),0.0)/(2*A(1)*A(2))).real());
        double th3_2 = real(acos(complex<double>((pow(p41xz_2,2)-pow(A(1),2)-pow(A(2),2)),0.0)/(2*A(1)*A(2))).real());
        double th3_3 = real(acos(complex<double>((pow(p41xz_3,2)-pow(A(1),2)-pow(A(2),2)),0.0)/(2*A(1)*A(2))).real());
        double th3_4 = real(acos(complex<double>((pow(p41xz_4,2)-pow(A(1),2)-pow(A(2),2)),0.0)/(2*A(1)*A(2))).real());

        double th3_5 = -th3_1;
        double th3_6 = -th3_2;
        double th3_7 = -th3_3;
        double th3_8 = -th3_4;

        double th2_1 = real(atan2(-p41_1(2), -p41_1(0))-asin((-A(2)*sin(th3_1))/p41xz_1));
        double th2_2 = real(atan2(-p41_2(2), -p41_2(0))-asin((-A(2)*sin(th3_2))/p41xz_2));
        double th2_3 = real(atan2(-p41_3(2), -p41_3(0))-asin((-A(2)*sin(th3_3))/p41xz_3));
        double th2_4 = real(atan2(-p41_4(2), -p41_4(0))-asin((-A(2)*sin(th3_4))/p41xz_4));

        double th2_5 = real(atan2(-p41_1(2), -p41_1(0))-asin((A(2)*sin(th3_1))/p41xz_1));
        double th2_6 = real(atan2(-p41_2(2), -p41_2(0))-asin((A(2)*sin(th3_2))/p41xz_2));
        double th2_7 = real(atan2(-p41_3(2), -p41_3(0))-asin((A(2)*sin(th3_3))/p41xz_3));
        double th2_8 = real(atan2(-p41_4(2), -p41_4(0))-asin((A(2)*sin(th3_4))/p41xz_4));

        Matrix4d T43m=T32(th3_1).inverse()*T21(th2_1).inverse()*T10(th1_1).inverse()*T60*T65(th6_1).inverse()*T54(th5_1).inverse();
        Vector3d Xhat43;
        Xhat43 << T43m(0,0), T43m(1,0), T43m(2,0);
        double th4_1 = real(atan2(Xhat43(1), Xhat43(0)));

        T43m=T32(th3_2).inverse()*T21(th2_2).inverse()*T10(th1_1).inverse()*T60*T65(th6_2).inverse()*T54(th5_2).inverse();
        Xhat43 << T43m(0,0), T43m(1,0), T43m(2,0);
        double th4_2 = real(atan2(Xhat43(1), Xhat43(0)));

        T43m=T32(th3_3).inverse()*T21(th2_3).inverse()*T10(th1_2).inverse()*T60*T65(th6_3).inverse()*T54(th5_3).inverse();
        Xhat43 << T43m(0,0), T43m(1,0), T43m(2,0);
        double th4_3 = real(atan2(Xhat43(1), Xhat43(0)));

        T43m=T32(th3_4).inverse()*T21(th2_4).inverse()*T10(th1_2).inverse()*T60*T65(th6_4).inverse()*T54(th5_4).inverse();
        Xhat43 << T43m(0,0), T43m(1,0), T43m(2,0);
        double th4_4 = real(atan2(Xhat43(1), Xhat43(0)));

        T43m=T32(th3_5).inverse()*T21(th2_5).inverse()*T10(th1_1).inverse()*T60*T65(th6_1).inverse()*T54(th5_1).inverse();
        Xhat43 << T43m(0,0), T43m(1,0), T43m(2,0);
        double th4_5 = real(atan2(Xhat43(1), Xhat43(0)));

        T43m=T32(th3_6).inverse()*T21(th2_6).inverse()*T10(th1_1).inverse()*T60*T65(th6_2).inverse()*T54(th5_2).inverse();
        Xhat43 << T43m(0,0), T43m(1,0), T43m(2,0);
        double th4_6 = real(atan2(Xhat43(1), Xhat43(0)));

        T43m=T32(th3_7).inverse()*T21(th2_7).inverse()*T10(th1_2).inverse()*T60*T65(th6_3).inverse()*T54(th5_3).inverse();
        Xhat43 << T43m(0,0), T43m(1,0), T43m(2,0);
        double th4_7 = real(atan2(Xhat43(1), Xhat43(0)));

        T43m=T32(th3_8).inverse()*T21(th2_8).inverse()*T10(th1_2).inverse()*T60*T65(th6_4).inverse()*T54(th5_4).inverse();
        Xhat43 << T43m(0,0), T43m(1,0), T43m(2,0);
        double th4_8 = real(atan2(Xhat43(1), Xhat43(0)));

        Matrix86d Th;
        Th <<   th1_1, th2_1, th3_1, th4_1, th5_1, th6_1,
                th1_1, th2_2, th3_2, th4_2, th5_2, th6_2,
                th1_2, th2_3, th3_3, th4_3, th5_3, th6_3,
                th1_2, th2_4, th3_4, th4_4, th5_4, th6_4,
                th1_1, th2_5, th3_5, th4_5, th5_1, th6_1,
                th1_1, th2_6, th3_6, th4_6, th5_2, th6_2,
                th1_2, th2_7, th3_7, th4_7, th5_3, th6_3,
                th1_2, th2_8, th3_8, th4_8, th5_4, th6_4;

        // Th = Th * R.block(0, 0, 6, 4);
        return Th;
}


Matrix6d ur5Jac(Vector6d Th){
        
        A << 0.0, -0.425, -0.3922, 0.0, 0.0, 0.0; 
        D << 0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996;

        double A1 = A(0); double A2 = A(1); double A3 = A(2); double A4 = A(3); double A5 = A(4);  double A6 = A(5);
        double D1 = D(0); double D2 = D(1); double D3 = D(2); double D4 = D(3); double D5 = D(4);  double D6 = D(5);

        double th1 = Th(0);
        double th2 = Th(1);
        double th3 = Th(2);
        double th4 = Th(3);
        double th5 = Th(4);
        double th6 = Th(5);

        Vector6d J1,J2,J3,J4,J5,J6;

        J1 <<   D5*(cos(th1)*cos(th5) + cos(th2 + th3 + th4)*sin(th1)*sin(th5)) + D3*cos(th1) + D4*cos(th1) - A3*cos(th2 + th3)*sin(th1) - A2*cos(th2)*sin(th1) - D5*sin(th2 + th3 + th4)*sin(th1),
                D5*(cos(th5)*sin(th1) - cos(th2 + th3 + th4)*cos(th1)*sin(th5)) + D3*sin(th1) + D4*sin(th1) + A3*cos(th2 + th3)*cos(th1) + A2*cos(th1)*cos(th2) + D5*sin(th2 + th3 + th4)*cos(th1),
                0,
                0,
                0,
                1;

        J2 <<   -cos(th1)*(A3*sin(th2 + th3) + A2*sin(th2) + D5*(sin(th2 + th3)*sin(th4) - cos(th2 + th3)*cos(th4)) - D5*sin(th5)*(cos(th2 + th3)*sin(th4) + sin(th2 + th3)*cos(th4))),
                -sin(th1)*(A3*sin(th2 + th3) + A2*sin(th2) + D5*(sin(th2 + th3)*sin(th4) - cos(th2 + th3)*cos(th4)) - D5*sin(th5)*(cos(th2 + th3)*sin(th4) + sin(th2 + th3)*cos(th4))),
                A3*cos(th2 + th3) - (D5*sin(th2 + th3 + th4 + th5))/2 + A2*cos(th2) + (D5*sin(th2 + th3 + th4 - th5))/2 + D5*sin(th2 + th3 + th4),
                sin(th1),
                -cos(th1),
                0;

        J3 <<   cos(th1)*(D5*cos(th2 + th3 + th4) - A3*sin(th2 + th3) + D5*sin(th2 + th3 + th4)*sin(th5)),
                sin(th1)*(D5*cos(th2 + th3 + th4) - A3*sin(th2 + th3) + D5*sin(th2 + th3 + th4)*sin(th5)),
                A3*cos(th2 + th3) - (D5*sin(th2 + th3 + th4 + th5))/2 + (D5*sin(th2 + th3 + th4 - th5))/2 + D5*sin(th2 + th3 + th4),
                sin(th1),
                -cos(th1),
                0;

        J4 <<   D5*cos(th1)*(cos(th2 + th3 + th4) + sin(th2 + th3 + th4)*sin(th5)),
                D5*sin(th1)*(cos(th2 + th3 + th4) + sin(th2 + th3 + th4)*sin(th5)),
                D5*(sin(th2 + th3 + th4 - th5)/2 + sin(th2 + th3 + th4) - sin(th2 + th3 + th4 + th5)/2),
                sin(th1),
                -cos(th1),
                0; 

        J5 <<  -D5*sin(th1)*sin(th5) - D5*cos(th2 + th3 + th4)*cos(th1)*cos(th5),
                D5*cos(th1)*sin(th5) - D5*cos(th2 + th3 + th4)*cos(th5)*sin(th1),
                -D5*(sin(th2 + th3 + th4 - th5)/2 + sin(th2 + th3 + th4 + th5)/2),
                sin(th2 + th3 + th4)*cos(th1),
                sin(th2 + th3 + th4)*sin(th1),
                -cos(th2 + th3 + th4);

        J6 <<   0,
                0,
                0,
                cos(th5)*sin(th1) - cos(th2 + th3 + th4)*cos(th1)*sin(th5),
                -cos(th1)*cos(th5) - cos(th2 + th3 + th4)*sin(th1)*sin(th5),
                -sin(th2 + th3 + th4)*sin(th5);  

        Matrix6d J;
        J << J1,J2,J3,J4,J5,J6;

        return J;
}
/*
Vector3d XD(double t){
        return Vector3d(0.2*cos(OMEGA*t), 0.01, 0.2*sin(OMEGA*t));
}
*/
Vector3d XD(Vector3d xe0,Vector3d xef,double t){
        //xe0 << 0.3, 0.3, 0.1;
        //xef << 0.5, 0.5, 0.5;
        if (t > 1)
                return xef;
        else
                return t*xef + (1-t)*xe0;
}

Vector3d Phid(Vector3d phie0,Vector3d phief,double t){
        //phie0 << 0, 0, 0;
        //phief << M_PI/4, M_PI/4, M_PI/4;
        if (t > 1)
                return phief;
        else
                return t*phief + (1-t)*phie0;
}

Vector3d rotm2eulFDR(Matrix3d R){
    //the output of rotm2eul is a vector with ZYX coordinates so I need to
    //convert it
    Vector3d eulZYX=R.eulerAngles(2,1,0);
    Vector3d eulXYZ;
    eulXYZ(2) = eulZYX(0);
    eulXYZ(1) = eulZYX(1);
    eulXYZ(0) = eulZYX(2);

    return eulXYZ;
}

Matrix3d eul2rotmFDR(Vector3d eulXYZ){
    //the input of eul2rotm is a vector with ZYX coordinates so I need to
    //convert it
    /*
    Vector3d eulZYX;
    
    eulZYX(2) = eulXYZ(0);
    eulZYX(1) = eulXYZ(1);
    eulZYX(0) = eulXYZ(2);
    Matrix3d R = EulerAnglesZYX(eulZYX);
    */

    Vector3d eulZYX;
    
    eulZYX(2) = eulXYZ(0);
    eulZYX(1) = eulXYZ(1);
    eulZYX(0) = eulXYZ(2);

    double c1 = cos(eulZYX(0));
    double s1 = sin(eulZYX(0));
    double c2 = cos(eulZYX(1));
    double s2 = sin(eulZYX(1));
    double c3 = cos(eulZYX(2));
    double s3 = sin(eulZYX(2));
    Matrix3d R;
    // Rotation matrix using Euler angles
    R << c2*c3, c3*s1*s2 - c1*s3, s1*s3 + c1*c3*s2,
         c2*s3, c1*c3 + s1*s2*s3, c1*s2*s3 - c3*s1,
         -s2, c2*s1, c1*c2;
    return R;
}

Vector3d computeOrientationErrorW(Matrix3d w_R_e, Matrix3d w_R_d){
        Matrix3d e_R_d = w_R_e.transpose()*w_R_d;

        double cos_dtheta = (e_R_d(0,0)+e_R_d(1,1)+e_R_d(2,2)-1.0)/2.0;

        Vector3d vecsin_dtheta;
        vecsin_dtheta << (e_R_d(2,1) -e_R_d(1,2)), (e_R_d(0,2) -e_R_d(2,0)), (e_R_d(1,0) -e_R_d(0,1));
        double sin_dtheta = (vecsin_dtheta*0.5).norm();

        double dtheta= atan2(sin_dtheta, cos_dtheta);

        Vector3d errorW,axis;

        if(dtheta==0){
                errorW << 0,0,0;
        }else{
                axis = 1.0/(2.0*sin_dtheta)*vecsin_dtheta;
                errorW = w_R_e * axis*dtheta;
        }

        return errorW;
}

//definire xef e xe0, phied e phief
/*

Matrix63d pseudoInversa(Matrix36d A){
        //cout<<A<<"\n";
        cout<<((A.transpose()*A)).determinant();
        Matrix63d pinv=((A.transpose()*A).inverse())*A.transpose();
        return pinv;
}

Vector6d invDiffKinematiControl(Vector6d qk, Vector3d xe, Vector3d xd,  Vector3d vd, Matrix3d K){
        
        Matrix6d J=ur5Jac(qk);
        Matrix36d J1;
        J1 <<  J.block(0, 0, 3, 6);
        //cout <<J1<<"\n";

        Matrix63d dotQ=pseudoInversa(J1);
        //cout <<"cane\n";
        //cout<< dotQ;
        Vector3d prod=(vd+K*(xd-xe));
        return dotQ*prod;
}

MatrixXd invDiffKinematicControlSim(Vector3d xd, Vector6d TH0, Matrix3d K, double minT, double maxT, double Dt){
        //cout <<"gatto\n";
        int L=(int)(maxT-minT)/Dt+1;
        
        VectorXd T;

        for(int i=0;i<L;i++){ 
              T.conservativeResize(T.rows()+1, T.cols());
              T.row(T.rows()-1) << i*Dt;
        }
        cout << T <<"endl";
        //cout <<"gatto\n";
        Vector6d qk=TH0;
        MatrixXd q=qk;

        for(int t=1;t<L;t++){
                Vector3d xe=ur5Direct(qk);

                Vector3d vd;
                vd=(XD(T(t))-XD(T(t)-Dt))/Dt;
                cout <<"gatto\n";
                Vector6d dotqk = invDiffKinematiControl(qk, xe, XD(T(t)),  vd, K);
                cout <<"cane\n";
                Vector6d qk1 = qk + dotqk*Dt; 

                

                //primo modo (non funziona)
                //q << q,qk1;
                //qk << qk1;


                //altro modo (verificare quale funziona) (non funziona il push_back)
                //q.push_back(qk1);
                //qk = qk1;

                //q.resize(q.rows()+1, NoChange);
                //q.row(q.rows()-1) = qk1;
                
        }

        return q;

}
*/
Vector6d invDiffKinematiControlComplete(Vector6d q, Vector3d xe, Vector3d xd, Vector3d vd, Matrix3d w_R_e, Vector3d phid, Vector3d phiddot, Matrix3d Kp, Matrix3d Kphi){
        //cout<<"f\n";
        
        Matrix3d w_R_d = eul2rotmFDR(phid);

        Vector3d error_o = computeOrientationErrorW(w_R_e, w_R_d);

        Matrix6d J=ur5Jac(q);
        double psid = phid(0);//psi
        double thetad = phid(1); //theta
        double phidd = phid(2); //phi

        Matrix3d T;
        T <<    cos(thetad)*cos(phidd), -sin(phidd), 0,
                cos(thetad)*sin(phidd), cos(phidd), 0,
                -sin(thetad), 0, 1;

        
        Vector3d omega_dot = T*phiddot;

        Vector3d v1=vd+Kp*(xd-xe);
        Vector3d v2=omega_dot+Kphi*error_o;

        Vector6d v1v2;
        v1v2 << v1(0),v1(1),v1(2),v2(0),v2(1),v2(2);

        Matrix6d Id = Matrix6d::Identity()*1e-06;
        Vector6d dotQ=((J+Id).inverse())*v1v2;

        return dotQ;

}



MatrixXd invDiffKinematicControlSimComplete(Vector3d xe0,Vector3d xef,Vector3d phie0,Vector3d phief,Vector6d TH0, Matrix3d Kp, Matrix3d Kphi, double minT, double maxT, double Dt){
        cout<<"entra1\n";
        //cout<<Dt<<endl;
        int L=(int)(maxT-minT)/Dt+1;
        
        VectorXd T;

        for(int i=0;i<L;i++){ 
              T.conservativeResize(T.rows()+1, T.cols());
              T.row(T.rows()-1) << i*Dt;

        }
        //cout<<T<<"\n";

        Vector6d qk=TH0;
        MatrixXd q=qk.transpose();

        //cout<<q<<"\n";

        for(int t=1;t<L;t++){

              Vector3d xe=ur5Direct(qk);
              Vector3d phie=rotm2eulFDR(Re);

              //cout<<phie.transpose()<<endl;

              Vector3d vd;
              vd << (XD(xe0,xef,T(t))-XD(xe0,xef,T(t)-Dt))/Dt;
              
              Vector3d phiddot;
              phiddot << (Phid(phie0,phief,T(t))-Phid(phie0,phief,T(t)-Dt))/Dt;

              Vector6d dotqk = invDiffKinematiControlComplete(qk, xe, XD(xe0,xef,T(t)), vd, Re, Phid(phie0,phief,T(t)),phiddot, Kp, Kphi);
               

              Vector6d qk1 = qk + dotqk*Dt;

                //cout<<"funz\n"; 
                //ultima parte
                //primo modo (non funziona)
                //q << q,qk1;
                //qk << qk1;
/*
                q.conservativeResize(q.rows()+1, NoChange);
                //q << q,qk1.transpose();
                //q.row(q.rows()-1) = qk1.transpose();
                q.row(q.rows()-1) << qk1(0);
                q.row(q.rows()-1) << qk1(1);
                q.row(q.rows()-1) << qk1(2);
                q.row(q.rows()-1) << qk1(3);
                q.row(q.rows()-1) << qk1(4);
                q.row(q.rows()-1) << qk1(5);
*/
             
                
                //cout<<qk1.transpose()<<endl;

                q.conservativeResize(q.rows()+1, NoChange);
                for(int i=0;i<6;i++){
                        q(t,i) = qk1(i);
                }


                qk=qk1;
                //cout<<qk.transpose()<<endl;
                //cout<<q;
                //cout<<"----\n";
                //break;

        }

        //cout << "The matrix q is of size "<< q.rows() << "x" << q.cols() << std::endl;

        return q;

}


int main(){
        A << 0.0, -0.425, -0.3922, 0.0, 0.0, 0.0; 
        D << 0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996;


        cout<<"Direct kinematics\n";
        Vector6d Th;
        Th << -0.3223527113543909, -0.7805794638446351, -2.5675506591796875, -1.6347843609251917, -1.5715253988849085, -1.0017417112933558;

        Vector3d pe=ur5Direct(Th);

        cout<<pe.transpose()<<"\n";

        cout<<"Inverse kinematics\n";

        Vector3d peIniziale;
        peIniziale << 0.151846, -0.191179,  0.450473;   //-0.3223527113543909, -0.7805794638446351, -2.5675506591796875, -1.6347843609251917, -1.5715253988849085, -1.0017417112933558
        //peIniziale << 0.3, 0.3, 0.1;


        Vector3d pe1;
        pe1 << 0.5, 0.4, 0.85;   //3.51465 -0.760302   1.84325   1.98053   1.20247  -4.69833

        Matrix86d Thresult=ur5Inverse(peIniziale);

        for(int i=0;i<8;i++){
                for(int j=0;j<6;j++){
                        cout<<Thresult(i,j)<<"\t";
                }
                cout<<"\n";
        }

        cout<<"\n";
        cout<<"\n";

        for(int i=0;i<3;i++){
                for(int j=0;j<3;j++){
                        cout<<Re(i,j)<<"\t";
                }
                cout<<"\n";
        }

        Matrix3d w_R_1 = eul2rotmFDR(Vector3d(0.1,M_PI/2, 0.3));
        Matrix3d wR2 = Matrix3d::Identity();

        Vector3d err=computeOrientationErrorW(wR2,w_R_1);
        cout<<err<<endl;


        cout<<"hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh\n";

        Matrix3d Kp = Matrix3d::Identity()*5;
        Matrix3d Kphi = Matrix3d::Identity()*0.1;
        Vector6d ThFirstRow;
        ThFirstRow << Thresult(0,0),Thresult(0,1),Thresult(0,2),Thresult(0,3),Thresult(0,4),Thresult(0,5);

        //MatrixXd differentialTH=invDiffKinematicControlSim(pe1,ThFirstRow,K, TMIN, TMAX, DELTAT);

        MatrixXd differentialTH=invDiffKinematicControlSimComplete(peIniziale,pe1,rotm2eulFDR(Re),Vector3d(0.0,0.0,M_PI),ThFirstRow,Kp,Kphi, TMIN, TMAX, DELTAT);

        cout<<differentialTH<<"\n";

        cout<<ur5Direct(differentialTH.row(differentialTH.rows()-1))<<endl;
        cout<<differentialTH.rows()<<endl;
        //cout<<differentialTH.row(differentialTH.rows()-1)<<endl;

}
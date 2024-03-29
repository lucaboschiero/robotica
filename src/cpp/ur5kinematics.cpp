#include "include/ur5kinematics.h"


Matrix4d T10(double th){                                       //homogeneoux transformation matrixes
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


Vector3d ur5Direct(Vector6d Th){                                   //cinematica diretta

        A << 0.0, -0.425, -0.3922, 0.0, 0.0, 0.0; 
        D << 0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996;

        T10m=T10(Th(0));
        T21m=T21(Th(1));
        T32m=T32(Th(2));
        T43m=T43(Th(3));
        T54m=T54(Th(4));
        T65m=T65(Th(5));

        Matrix4d T06 =T10m*T21m*T32m*T43m*T54m*T65m;              //homogeneoux transormation matrix 

        Vector3d pe;
        pe << T06(0,3),T06(1,3),T06(2,3);                           //posizione end-effector

        Re << T06(0,0),T06(0,1),T06(0,2),                          //matrice di rotazione end-effector
              T06(1,0),T06(1,1),T06(1,2),
              T06(2,0),T06(2,1),T06(2,2);


        return pe; 

}
  
Matrix86d ur5Inverse(Vector3d pe){                              //cinematica inversa

        

        A << 0.0, -0.425, -0.3922, 0.0, 0.0, 0.0; 
        D << 0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996;
        
        Matrix4d R;
        R << -1,0,0,0,
              0,1,0,0,
              0,0,-1,0,
              0,0,0,1;


        Matrix4d T60;
        T60 <<  1,0,0,pe(0),
                0,1,0,pe(1),
                0,0,1,pe(2),
                0,0,0,1;

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

        return Th;
}


Matrix6d ur5Jac(Vector6d Th){                              //calclolo matrice Jacobiana
        
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



Vector3d XD(Vector3d xe0,Vector3d xef,double t){                    //calcolo posizione desiderata al tempo t

        if (t > TMAX)
                return xef;
        else
                return (t/TMAX)*xef + (1-(t/TMAX))*xe0;
}

Vector3d Phid(Vector3d phie0,Vector3d phief,double t){            //calcolo orientazione desiderata al tempo t 
   
        if (t > TMAX)
                return phief;
        else
                return (t/TMAX)*phief + (1-(t/TMAX))*phie0;
}

Vector3d rotm2eulFDR(Matrix3d R){                                   //conversione da matrice di rotazione a EulerAngles
    //the output of rotm2eul is a vector with ZYX coordinates so I need to
    //convert it
    Vector3d eulZYX=R.eulerAngles(2,1,0);
    Vector3d eulXYZ;
    eulXYZ << eulZYX(2), eulZYX(1), eulZYX(0);

    return eulXYZ;
}

Matrix3d eul2rotmFDR(Vector3d eulXYZ){                            //conversione da EulerAngles a matrice di rotazione
    //the input of eul2rotm is a vector with ZYX coordinates so I need to
    //convert it
    
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

Vector3d computeOrientationErrorW(Matrix3d w_R_e, Matrix3d w_R_d){                //calcolo errore di orientazione tra orientazione attuale e desiderata
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


Vector6d invDiffKinematiControlComplete(Vector6d q, Vector3d xe, Vector3d xd, Vector3d vd, Matrix3d w_R_e, Vector3d phid, Vector3d phiddot, Matrix3d Kp, Matrix3d Kphi){            //calcolo dotq per cinematica differenziale inversa
        
        Matrix3d w_R_d = eul2rotmFDR(phid);

        Vector3d error_o = computeOrientationErrorW(w_R_e, w_R_d);      //errore di orientazione per applicare il controllo 

        
        if(error_o.norm()>0.1){
             error_o=0.1*error_o.normalized();   
        }

        Matrix6d J=ur5Jac(q);                              //calcolo matrice jacobiana
        double psid = phid(0);   //psi
        double thetad = phid(1); //theta
        double phidd = phid(2);  //phi

        Matrix3d T;
        T <<    cos(thetad)*cos(phidd), -sin(phidd), 0,
                cos(thetad)*sin(phidd), cos(phidd), 0,
                -sin(thetad), 0, 1;

        
        Vector3d omega_dot = T*phiddot;

        Vector3d v1=vd+Kp*(xd-xe);                          //fattore per controllo posizione
        //Vector3d v2=omega_dot+Kphi*error_o;
        Vector3d v2=Kphi*error_o;                          //fattore per controllo orientazione
        

        Vector6d v1v2;
        v1v2 << v1(0),v1(1),v1(2),v2(0),v2(1),v2(2);

        Matrix6d Id = Matrix6d::Identity()*1e-06;
        Vector6d dotQ=((J+Id).inverse())*v1v2;                //calcolo dotq con controllo

        return dotQ;

}

 
MatrixXd invDiffKinematicControlSimComplete(Vector3d xe0,Vector3d xef,Vector3d phie0,Vector3d phief,Vector6d TH0, Matrix3d Kp, Matrix3d Kphi, double minT, double maxT, double Dt){           //cinematica differenziale inversa
        
        int L=(int)(maxT-minT)/Dt+1;
        
        VectorXd T;

        for(int i=0;i<L;i++){ 
              T.conservativeResize(T.rows()+1, T.cols());
              T.row(T.rows()-1) << i*Dt;

        }
        //cout<<T<<"\n";

        Vector6d qk=TH0;
        MatrixXd q=qk.transpose();            //joint values di partenza  

        //cout<<q<<"\n";

        for(int t=1;t<L;t++){

              Vector3d xe=ur5Direct(qk);             //calcolo posizione attuale con DK
              Vector3d phie=rotm2eulFDR(Re);         //conversione in eulerAngles


              Vector3d vd;
              vd = (XD(xe0,xef,T(t)+Dt)-XD(xe0,xef,T(t)))/Dt;            //calcolo velocità desiderata
              
              Vector3d phiddot;
              phiddot = (Phid(phie0,phief,T(t)+Dt)-Phid(phie0,phief,T(t)))/Dt;        //calcolo velocità angolare desiderata

              Vector6d dotqk = invDiffKinematiControlComplete(qk, xe, XD(xe0,xef,T(t)+Dt), vd, Re, Phid(phie0,phief,T(t)+Dt),phiddot, Kp, Kphi);       //calcolo dotq
               

              Vector6d qk1 = qk + dotqk*Dt;       //integrazione numerica per trovare i joint values al tempo t+1


              Vector3d p_temp=ur5Direct(qk1);
              double p_temp_z=check_z(p_temp(2));          //controllo che l'end effector non si abbassi troppo
        
              Vector2d p_check=checkCircleandTranslate(Vector2d(p_temp(0),p_temp(1)));      //controllo che non vada in singolarità
              Vector3d p_temp1;
              p_temp1 << p_check(0),p_check(1),p_temp_z;

              qk1=ur5Inverse(p_temp1).row(7);
     

              q.conservativeResize(q.rows()+1, NoChange);
              for(int i=0;i<6;i++){
                    q(t,i) = qk1(i);           //aggiorno la matrice generando la traiettoria
              }


              qk=qk1;

        }

        //cout << "The matrix q is of size "<< q.rows() << "x" << q.cols() << std::endl;

        return q;

}

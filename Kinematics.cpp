#include "Kinematics.h"
#include <stdio.h>
#include <iostream>




MTransform kinematics_forward_head(const double *q)
{
    MTransform t;
    return t;
}

MTransform kinematics_forward(const int limbID, const double *q)
{
    MTransform t;
    switch(limbID)
    {
    case ARM_LEFT:
        t = kinematics_forward_larm(q);
        break;
    case ARM_RIGHT:
        t = kinematics_forward_rarm(q);
        break;
    case LEG_LEFT:
        t = kinematics_forward_lleg(q);
        break;
    case LEG_RIGHT:
        t = kinematics_forward_rleg(q);
        break;
    default:
        std::cout<< "The ID for the body part is unknown" << std::endl;
    }
    return t;
}

MTransform kinematics_forward_larm(const double *q)
{
    MTransform t;
    t.mDH(-PI/2, d_su_y, q[0], d_su_x).mDH(-PI/2, d_ue_x, -PI/4+q[1], -d_ue_z).mDH(0, d_eh, PI/4+q[2], 0);
    return t;
}

MTransform kinematics_forward_rarm(const double *q)
{
    MTransform t;
    t.mDH(PI/2, d_su_y, q[0], d_su_x).mDH(PI/2, d_ue_x, PI/4+q[1], -d_ue_z).mDH(0, d_eh, -PI/4+q[2], 0);
    return t;
}

void kinematics_forward_larm_exp(const double *q)
{
    // This function is used for testing the equations of the forward kinematics
    // double x = r3*ca*cb*cr + r3*sa*sr + r2*ca*cb + d2*sa + r1*ca;
    // double y = r3*sa*cb*cr - r3*ca*sr + r2*sa*cb - d2*ca + r1*sa;
    // double z = -r3*sb*cr - d3*cb - r2*sb + d1;

    double d1 = d_su_x;
    double d2 = d_ue_z;
    double r1 = d_su_y;
    double r2 = d_ue_x;
    double r3 = d_eh;
    double d3 = 0;

    double sa = sin(q[0]);
    double ca = cos(q[0]);
    double sb = sin(q[1]-PI/4);
    double cb = cos(q[1]-PI/4);
    double sr = sin(q[2]+PI/4);
    double cr = cos(q[2]+PI/4);

    double x = r3*ca*cb*cr + r3*sa*sr - d3*ca*sb + r2*ca*cb + d2*sa + r1*ca;
    double y = r3*sa*cb*cr - r3*ca*sr - d3*sa*sb + r2*sa*cb - d2*ca + r1*sa;
    double z = -r3*sb*cr - d3*cb - r2*sb + d1;

    std::cout<< x << " " << y << " " << z << std::endl;

}

void kinematics_forward_rarm_exp(const double *q)
{
    // This function is used for testing the equations of the forward kinematics
    //double x = r3*ca*cb*cr + r3*sa*sr + r2*ca*cb - d2*sa + r1*ca;
    //double y = r3*sa*cb*cr - r3*ca*sr + r2*sa*cb + d2*ca + r1*sa;
    //double z = r3*sb*cr + r2*sb + d1;
    double d1 = d_su_x;
    double d2 = d_ue_z;
    double r1 = d_su_y;
    double r2 = d_ue_x;
    double r3 = d_eh;

    double sa = sin(q[0]);
    double ca = cos(q[0]);
    double sb = sin(q[1]+PI/4);
    double cb = cos(q[1]+PI/4);
    double sr = sin(q[2]-PI/4);
    double cr = cos(q[2]-PI/4);

    double x = r3*ca*cb*cr + r3*sa*sr + r2*ca*cb - d2*sa + r1*ca;
    double y = r3*sa*cb*cr - r3*ca*sr + r2*sa*cb + d2*ca + r1*sa;
    double z = r3*sb*cr + r2*sb + d1;

    std::cout<< x << " " << y << " " << z << std::endl;

}

void kinematics_forward_lleg_exp(const double * q)
{

  double d1 = d_hu;
  double r3 = d_uk;
  double r4 = d_ka;

  double s1 = sin(PI/2+q[0]);
  double c1 = cos(PI/2+q[0]);
  double s2 = sin(PI/2+q[1]);
  double c2 = cos(PI/2+q[1]);
  double s3 = sin(-delta_upperleg+q[2]);
  double c3 = cos(-delta_upperleg+q[2]);
  //double s4 = sin(-delta_upperleg-delta_knee+q[3]);
  //double c4 = cos(-delta_upperleg-delta_knee+q[3]);
  double s34 = sin(q[2]-q[3]+delta_knee);
  double c34 = cos(q[2]-q[3]+delta_knee);

  double x = c1*c2*(r4*c34 + r3*c3) + s1*(r4*s34 + r3*s3);
  double y = s1*c2*(r4*c34 + r3*c3) - c1*(r4*s34 + r3*s3);
  double z = s2*(r4*c34 + r3*c3) + d1;

  std::cout<< x << " " << y << " " << z << std::endl;
}


MTransform kinematics_forward_lleg(const double *q)
{
    MTransform t;
    t.mDH(PI/2, 0, PI/2+q[0], d_hu)
    .mDH(PI/2, 0, PI/2+q[1], 0)
    .mDH(-PI, d_uk, -delta_upperleg+q[2], 0)
    .mDH(0, d_ka, -delta_upperleg-delta_knee+q[3], 0)
    .mDH(-PI/2, 0, delta_knee+q[4], 0)
    .mDH(0, 0, q[5], 0);
    return t;
}

MTransform kinematics_forward_rleg(const double *q)
{
    MTransform t;
    return t;
}

void test_kinematics_forward_arm(int arm)
{
    std::cout << "Input three DOFs (0~1023) for current pose!" << std::endl;
    int *q = new int[numDOF_ARM];
    double *angle = new double[numDOF_ARM];
    for (int ind=0; ind<numDOF_ARM; ind++)
    {
        std::cin >> q[ind];
        angle[ind] = servo_to_radian(q[ind]);
    }
    MTransform t = kinematics_forward(arm, angle);
    t.print();

}

void test_kinematics_inverse_arm(int arm)
{
    std::cout << "Input three DOFs (0~1023) for current pose!" << std::endl;
    int *q = new int[numDOF_ARM];
    double *angle = new double[numDOF_ARM];
    for (int ind=0; ind<numDOF_ARM; ind++)
    {
        std::cin >> q[ind];
        angle[ind] = servo_to_radian(q[ind]);
    }
    MTransform t = kinematics_forward(arm, angle);
    t.print();

    std::cout << "Input the target position!" << std::endl;
    Vector3d targetPos;
    for (int ind=0; ind<numDOF_ARM; ind++)
    {
        std::cin >> targetPos[ind];
    }

    double* targetAngle = kinematics_inverse_arm(arm, targetPos, angle);
    for (int ind=0; ind<numDOF_ARM; ind++)
    {
        std::cout << radian_to_servo(targetAngle[ind]) << std::endl;
    }
    t = kinematics_forward(arm, targetAngle);
    t.print();
}

void test_kinematics_forward_leg(int leg)
{
    MTransform t;

    std::cout << "Input six DOFs (0~1023) for current pose!" << std::endl;
    int *q = new int[numDOF_LEG];
    double *angle = new double[numDOF_LEG];
    for (int ind=0; ind<numDOF_LEG; ind++)
    {
        std::cin >> q[ind];
        angle[ind] = servo_to_radian(q[ind]);
    }

    t = kinematics_forward(leg, angle);
    t.print();

    kinematics_forward_lleg_exp(angle);
}

void test_kinematics_inverse_leg(int leg)
{
    std::cout << "Input Six DOFs (0~1023) for current pose!" << std::endl;
    int *q = new int[numDOF_LEG];
    double *angle = new double[numDOF_LEG];
    for (int ind=0; ind<numDOF_LEG; ind++)
    {
        std::cin >> q[ind];
        angle[ind] = servo_to_radian(q[ind]);
    }
    MTransform t = kinematics_forward(leg, angle);
    t.print();

    std::cout << "Input the target position!" << std::endl;

    double *targetPos_in = new double[3];
    for (int ind=0; ind<3; ind++)
    {
        std::cin >> targetPos_in[ind];
    }
    Vector3d targetPos(targetPos_in[0], targetPos_in[1], targetPos_in[2]);

    double* targetAngle = kinematics_inverse_leg(leg, targetPos, angle);
    for (int ind=0; ind<numDOF_LEG; ind++)
    {
        std::cout << radian_to_servo(targetAngle[ind]) << std::endl;
    }
    t = kinematics_forward(leg, targetAngle);
    t.print();

}





Matrix3d jacobian_larm(const double *q)
{
//    double d1 = d_su_x;
    double d2 = d_ue_z;
    double r1 = d_su_y;
    double r2 = d_ue_x;
    double r3 = d_eh;

    double sa = sin(q[0]);
    double ca = cos(q[0]);
    double sb = sin(q[1]-PI/4);
    double cb = cos(q[1]-PI/4);
    double sr = sin(q[2]+PI/4);
    double cr = cos(q[2]+PI/4);

    //double x = r3*ca*cb*cr + r3*sa*sr + r2*ca*cb + d2*sa + r1*ca;
    //double y = r3*sa*cb*cr - r3*ca*sr + r2*sa*cb - d2*ca + r1*sa;
    //double z = -r3*sb*cr - d3*cb - r2*sb + d1;


    double m11 = -r3*sa*cb*cr + r3*ca*sr - r2*sa*cb + d2*ca - r1*sa;
    double m12 = -r3*ca*sb*cr - r2*ca*sb;
    double m13 = -r3*ca*cb*sr + r3*sa*cr;
    double m21 = r3*ca*cb*cr + r3*sa*sr + r2*ca*cb + d2*sa + r1*ca;
    double m22 = -r3*sa*sb*cr - r2*sa*sb;
    double m23 = -r3*sa*cb*sr - r3*ca*cr;
    double m31 = 0;
    double m32 = -r3*cb*cr - r2*cb;
    double m33 = r3*sb*sr;
    Matrix3d jac;
    jac << m11, m12, m13, m21, m22, m23, m31, m32, m33;
    return jac;
}

Matrix3d jacobian_rarm(const double *q)
{
//    double d1 = d_su_x;
    double d2 = d_ue_z;
    double r1 = d_su_y;
    double r2 = d_ue_x;
    double r3 = d_eh;

    double sa = sin(q[0]);
    double ca = cos(q[0]);
    double sb = sin(q[1]+PI/4);
    double cb = cos(q[1]+PI/4);
    double sr = sin(q[2]-PI/4);
    double cr = cos(q[2]-PI/4);

    //double x = r3*ca*cb*cr + r3*sa*sr + r2*ca*cb - d2*sa + r1*ca;
    //double y = r3*sa*cb*cr - r3*ca*sr + r2*sa*cb + d2*ca + r1*sa;
    //double z = r3*sb*cr + r2*sb + d1;


    double m11 = -r3*sa*cb*cr + r3*ca*sr - r2*sa*cb - d2*ca - r1*sa;
    double m12 = -r3*ca*sb*cr - r2*ca*sb;
    double m13 = -r3*ca*cb*sr + r3*sa*cr;
    double m21 = r3*ca*cb*cr + r3*sa*sr + r2*ca*cb - d2*sa + r1*ca;
    double m22 = -r3*sa*sb*cr - r2*sa*sb;
    double m23 = -r3*sa*cb*sr - r3*ca*cr;
    double m31 = 0;
    double m32 = r3*cb*cr + r2*cb;
    double m33 = -r3*sb*sr;
    Matrix3d jac;
    jac << m11, m12, m13, m21, m22, m23, m31, m32, m33;
    return jac;
}

MatrixXd jacobian_lleg(const double *q)
{

  MatrixXd jac(3, 6);

//  double d1 = d_hu;
  double r3 = d_uk;
  double r4 = d_ka;

  double s1 = sin(PI/2+q[0]);
  double c1 = cos(PI/2+q[0]);
  double s2 = sin(PI/2+q[1]);
  double c2 = cos(PI/2+q[1]);
  double s3 = sin(-delta_upperleg+q[2]);
  double c3 = cos(-delta_upperleg+q[2]);
  double s34 = sin(q[2]-q[3]+delta_knee);
  double c34 = cos(q[2]-q[3]+delta_knee);


  double m11 = -s1*c2*(r4*c34+r3*c3) + c1*(r4*s34+r3*s3);
  double m12 = -c1*s2*(r4*c34+r3*c3);
  double m13 = c1*c2*(-r4*s34-r3*s3) + s1*(r4*c34+r3*c3);
  double m14 = c1*c2*r4*s34 - s1*r4*c34;
  double m15 = 0;
  double m16 = 0;
  double m21 = c1*c2*(r4*c34+r3*c3) + s1*(r4*s34+r3*s3);
  double m22 = -s1*s2*(r4*c34+r3*c3);
  double m23 = s1*c2*(-r4*s34-r3*s3) - c1*(r4*c34+r3*c3);
  double m24 = s1*c2*r4*s34 + c1*r4*c34;
  double m25 = 0;
  double m26 = 0;
  double m31 = 0;
  double m32 = c2*(r4*c34+r3*c3);
  double m33 = s2*(-r4*s34-r3*s3);
  double m34 = s2*r4*s34;
  double m35 = 0;
  double m36 = 0;


  jac <<  m11, m12, m13, m14, m15, m16,
          m21, m22, m23, m24, m25, m26,
          m31, m32, m33, m34, m35, m36;

  return jac;
}

MatrixXd jacobian_rleg(const double *q)
{
  MatrixXd jac(3, 6);
  return jac;
}

double* get_current_angle(int leg)
{
    double* q = new double[3];
    // read current joint value
    return q;
}

Vector3d get_current_position(int limbID, const double* q)
{
    MTransform t;
    t = kinematics_forward(limbID, q);
    double* tl = t.getTranslation();
    Vector3d tv(tl[0], tl[1], tl[2]);
    return tv;
}

Matrix3d get_current_jacobian(int limbID, const double * q)
{
    Matrix3d jac;
    if (limbID == ARM_LEFT)
        jac = jacobian_larm(q);
    else if (limbID == ARM_RIGHT)
        jac = jacobian_rarm(q);
    return jac;
}

MatrixXd get_current_jacobian_leg(int limbID, const double *q)
{
  MatrixXd jac(3, 6);
  if (limbID == LEG_LEFT)
    jac = jacobian_lleg(q);
  else if (limbID == LEG_RIGHT)
    jac = jacobian_rleg(q);
  return jac;
}


Matrix3d compute_pseudo_inverse(Matrix3d m)
{
    //Matrix3d mt = m.transpose();
    Matrix3d m_o = m.transpose()*(m*m.transpose()).inverse();
    return m_o;
}

MatrixXd compute_pseudo_inverse_xd(MatrixXd m)
{
  MatrixXd m_o(6, 3);
  m_o = m.transpose()*(m*m.transpose()).inverse();
  return m_o;
}

double* kinematics_inverse_arm(const int arm, const Vector3d pArm, const double* qArm_now)
{
    double th_m = 0.001;
    double th_e = 0.0005;
    Matrix3d eye;
    eye << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    double* qArm = new double[3]; // Init the 3 angles with value 0
    for (int ind=0; ind<3; ind++)
        qArm[ind] = qArm_now[ind];
    Vector3d pArm_now = get_current_position(arm, qArm);
    Vector3d dArm = pArm - pArm_now;

    int numLoops = 0;

    while ((dArm.norm() > th_m)&&(numLoops<500))
    {
        Matrix3d jac = get_current_jacobian(arm, qArm);
        Matrix3d jac_inv = compute_pseudo_inverse(jac);
        Vector3d error_vec = (eye - jac*jac_inv)*dArm;
        while(error_vec.norm() > th_e)
        {
            dArm = dArm/2;
            error_vec = (eye - jac*jac_inv)*dArm;
        }
        Vector3d temp = jac_inv*dArm;
        for (int ind=0; ind<3; ind++)
        {
            qArm[ind] = qArm[ind] + temp[ind];
            qArm[ind] = clamp_limits(qArm[ind]);
        }
        pArm_now = get_current_position(arm, qArm);
        dArm = pArm - pArm_now;
        numLoops ++;
    }
    return qArm;
}

double* kinematics_inverse_leg(const int leg, const Vector3d pLeg, const double* qLeg_now)
{
  double th_m = 0.001;
  double th_e = 0.0005;

  double* qLeg = new double[numDOF_LEG];
  for (int ind=0; ind<numDOF_LEG; ind++)
    qLeg[ind] = qLeg_now[ind];
  Vector3d pLeg_now = get_current_position(leg, qLeg);
  Vector3d dLeg = pLeg - pLeg_now;
  Matrix3d eye;
  eye << 1, 0, 0, 0, 1, 0, 0, 0, 1;

  int numLoops = 0;
  while ((dLeg.norm() > th_m)&&(numLoops<500))
    {
        MatrixXd jac = get_current_jacobian_leg(leg, qLeg); //Error
        MatrixXd jac_inv = compute_pseudo_inverse_xd(jac);
        Vector3d error_vec = (eye - jac*jac_inv)*dLeg; //Error
        while(error_vec.norm() > th_e)
        {
            dLeg = dLeg/2;
            error_vec = (eye - jac*jac_inv)*dLeg;
        }
        VectorXd temp(6, 1);
        temp = jac_inv*dLeg; //Error
        for (int ind=0; ind<numDOF_LEG; ind++)
        {
            qLeg[ind] = qLeg[ind] + temp[ind];
            qLeg[ind] = clamp_limits(qLeg[ind]);
        }
        pLeg_now = get_current_position(leg, qLeg);
        dLeg = pLeg - pLeg_now;
        numLoops ++;
    }
    return qLeg;

}


double servo_to_radian(int servo)
{
    return (servo-512)/1023.0*300.0/180*PI;
}

int radian_to_servo(double radian)
{
    return int(radian/PI*180/300*1023) + 512;
}


double clamp_limits(double q)
{
    if (q > PI/6*5)
        q = PI/6*5;
    if (q < -PI/6*5)
        q = -PI/6*5;
    return q;
}


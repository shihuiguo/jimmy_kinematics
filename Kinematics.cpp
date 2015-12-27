#include "Kinematics.h"
#include "Transform.h"
#include <math.h>
#include <stdio.h>
#include <iostream>


Transform
kinematics_forward_head(const double *q)
{
  Transform t;
  return t;
}

Transform
kinematics_forward_larm(const double *q)
{
  Transform t;
  t.mDH(-PI/2, d_su_y, q[0], d_su_x).mDH(-PI/2, d_ue_x, -PI/4+q[1], -d_ue_z).mDH(0, d_eh, PI/4+q[2], 0);
  return t;
}

Transform
kinematics_forward_rarm(const double *q)
{
  Transform t;
  t.mDH(PI/2, d_su_y, q[0], d_su_x).mDH(PI/2, d_ue_x, PI/4+q[1], -d_ue_z).mDH(0, d_eh, -PI/4+q[2], 0);
  return t;
}

void test_kinematics_forward_larm(const double *q)
{


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

Transform
kinematics_forward_lleg(const double *q)
{
  Transform t;
  return t;
}

Transform
kinematics_forward_rleg(const double *q)
{
  Transform t;
  return t;
}

std::vector<double>
kinematics_inverse_leg(
			   Transform trLeg,
			   int leg, double unused)
{
  std::vector<double> qLeg(6);
  return qLeg;
}

std::vector<double>
kinematics_inverse_lleg(Transform trLeg, double unused)
{
  return kinematics_inverse_leg(trLeg, LEG_LEFT, unused);
}

std::vector<double>
kinematics_inverse_rleg(Transform trLeg, double unused)
{
  return kinematics_inverse_leg(trLeg, LEG_RIGHT, unused);
}

std::vector<double>
kinematics_inverse_legs(
			    const double *pLLeg,
			    const double *pRLeg,
			    const double *pTorso,
			    int legSupport)
{
  std::vector<double> qLLeg(12), qRLeg;
  Transform trLLeg = transform6D(pLLeg);
  Transform trRLeg = transform6D(pRLeg);
  Transform trTorso = transform6D(pTorso);

  Transform trTorso_LLeg = inv(trTorso)*trLLeg;
  Transform trTorso_RLeg = inv(trTorso)*trRLeg;

  qLLeg = kinematics_inverse_lleg(trTorso_LLeg, 0);
  qRLeg = kinematics_inverse_rleg(trTorso_RLeg, 0);

  qLLeg.insert(qLLeg.end(), qRLeg.begin(), qRLeg.end());
  return qLLeg;
}

Mat3 jacobian_arm(const double *q)
{
  double d1 = d_su_x;
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
  Mat3 jac(m11, m12, m13, m21, m22, m23, m31, m32, m33);
  return jac;
}

double* get_current_angle(int leg){
  double* q = new double[3];
  // read current joint value
  return q;
}

Vec3 get_current_position(int arm, const double* q){
  Transform t;
  if (arm == ARM_LEFT)
    t = kinematics_forward_larm(q);
  else
    t = kinematics_forward_rarm(q);
  double* tl = t.getTranslation();
  Vec3 tv(tl[0], tl[1], tl[2]);
  return tv;
}

Mat3 compute_pseudo_inverse(Mat3 m){
  Mat3 mt = m.transpose();
  Mat3 m_o = m.transpose()*(m*m.transpose()).inverse();
  return m_o;
}

double* kinematics_inverse_arm(
          const int arm,
			    const Vec3 pArm,
			    const double* qArm_now
			    )
{
  double th_m = 0.001;
  double th_e = 0.0005;
  Mat3 eye(1, 0, 0, 0, 1, 0, 0, 0, 1);
  double th_m_2 = th_m*th_m;
  double th_e_2 = th_e*th_e;
  double* qArm = new double[3]; // Init the 3 angles with value 0
  for (int ind=0; ind<3; ind++)
    qArm[ind] = qArm_now[ind];
  Vec3 pArm_now = get_current_position(arm, qArm);
  Vec3 dArm = pArm - pArm_now;

  int numLoops = 0;

  while ((dArm.Length() > th_m)&&(numLoops<500)) {
    Mat3 jac = jacobian_arm(qArm);
    Mat3 jac_inv = compute_pseudo_inverse(jac);
    Vec3 error_vec = (eye - jac*jac_inv)*dArm;
    while(error_vec.Length() > th_e){
      dArm = dArm/2;
      error_vec = (eye - jac*jac_inv)*dArm;
    }
    Vec3 temp = jac_inv*dArm;
    for (int ind=0; ind<3; ind++)
      qArm[ind] = qArm[ind] + temp[ind];
    pArm_now = get_current_position(arm, qArm);
    dArm = pArm - pArm_now;
    numLoops ++;
  }
  return qArm;
}

double servo_to_radian(int servo){
  return (servo-512)/1023.0*300.0/180*PI;
}

int radian_to_servo(double radian){
  return int(radian/PI*180/300*1023) + 512;
}

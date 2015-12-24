#include "Kinematics.h"
#include "Transform.h"
#include <math.h>



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
/*
Mat3 jacobian_arm(const double *q)
{
  double x_us = d_su_x;
  double y_us  = -d_su_y;
  double z_us = d_su_z;
  double x_eu = d_eu_x;
  double y_eu = d_eu_y;
  double z_eu = d_eu_z;
  double x_he = d_eh_x;
  double y_he = d_eh_y;
  double z_he = d_eh_z;

  double sa = sin(q[0]);
  double ca = cos(q[0]);
  double sb = sin(q[1]);
  double cb = cos(q[1]);
  double sr = sin(q[2]);
  double cr = cos(q[2]);

  double Ar = cr*x_he - sr*z_he + x_eu;
  double Br = sr*x_he + cr*z_he + z_eu;
  double D = y_he + y_eu;
  double ArD = -sr*x_he - cr*z_he;
  double BrD = cr*x_he - sr*z_he;

  double m11 = 0;
  double m12 = -Ar*sb+D*cb;
  double m13 = cr*ArD;
  double m21 = -sa*sb*Ar - sa*cb*D - ca*Br + (-sa*y_us - ca*z_us);
  double m22 = ca*cb*Ar - ca*sb*D;
  double m23 = ca*sb*ArD - sa*BrD;
  double m31 = ca*sb*Ar + ca*cb*D - sa*Br + (ca*y_us - sa*z_us);
  double m32 = sa*cb*Ar - sa*sb*D;
  double m33 = sa*sb*ArD + ca*BrD;
  Mat3 jac(m11, m12, m13, m21, m22, m23, m31, m32, m33);
  return jac;
}

double* get_current_angle(int leg){
  double* q = new double[3];
  // read current joint value
  return q;
}

double* get_current_position(int leg, const double* q){
  Transform t;
  if (leg == LEG_LEFT){
    t = kinematics_forward_larm(q);
  else
    t = kinematics_forward_rarm(q);
  return t.getTranslation();
}

Mat3 compute_pseudo_inverse(Mat3 m){
  Mat3 m_o = m.transpose()*(m*m.transpose()).inverse();
}

std::vector<double> kinematics_inverse_arm(
			    Vec3 dArm
			    )
{
  double th_m = 0.1;
  double th_e = 0.1;
  Mat3 eye(1, 0, 0, 0, 1, 0, 0, 0, 1);
  double th_m_2 = th_m*th_m;
  double th_e_2 = th_e*th_e;
  Vec3 qArm; // Init the 3 angles with value 0
  Vec3 qArm_now = get_current_angle();
  Vec3 dArm_now = get_current_position();
  Vec3 err = dArm - dArm_now;
  while (err.Length > th_m_2){
    Mat3 jac = jacobian_arm(qArm_now);
    Mat3 jac_inv = compute_pseudo_inverse(jac);
    Vec3 new_vec = (I - jac*jac_inv)*err;
    while(new_vec.Length() > th_e_2){
      err = err/2;
      new_vec = (I - jac*jac_inv)*err;
    }
    qArm = q_Arm_now + jac_inv*err;
  }
  return qArm;
}
*/

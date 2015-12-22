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
  Transform t, t_s, t_u, t_e, t_h;
  t_s.rotateX(q[0]);
  t_u.rotateZ(q[1]-PI*0.250).translateOrigin(su_h, -su_v, 0);
  t_e.rotateY(q[2]-PI*0.250).translateOrigin(ue_h, 0, ue_v);
  t_h.translateOrigin(eh, 0, 0);
  t = t_s*t_u*t_e*t_h;
  return t;
}

Transform
kinematics_forward_rarm(const double *q)
{
  Transform t, t_s, t_u, t_e, t_h;
  t_s.rotateX(q[0]);
  t_u.rotateZ(q[1]+PI*0.250).translateOrigin(-su_h, -su_v, 0);
  t_e.rotateY(q[2]-PI*0.750).translateOrigin(-ue_h, 0, ue_v);
  t_h.translateOrigin(eh, 0, 0);
  t = t_s*t_u*t_e*t_h;
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

std::vector<double> kinematics_inverse_arm(
			    const double *dArm
			    )
{

  std::vector<double> qArm(3,-999); // Init the 3 angles with value 0


  return qArm;

}

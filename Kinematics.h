#ifndef jimmyKinematics_h_DEFINED
#define jimmyKinematics_h_DEFINED

#include <math.h>
#include <vector>
#include "Transform.h"

enum {LEG_LEFT = 0, LEG_RIGHT = 1};

//const double PI = 3.14159265358979323846;
const double PI = 2*asin(1);
const double SQRT2 = sqrt(2);

const double su = 0.027;
const double su_h = 0.023;
const double su_v = 0.015;
const double ue_h = 0.063;
const double ue_v = 0.015;
const double eh = 0.078;

Transform kinematics_forward_head(const double *q);
Transform kinematics_forward_larm(const double *q);
Transform kinematics_forward_rarm(const double *q);
Transform kinematics_forward_lleg(const double *q);
Transform kinematics_forward_rleg(const double *q);

std::vector<double>
kinematics_inverse_leg(
			   const Transform trLeg,
			   const int leg,
			   double unused=0);

std::vector<double>
kinematics_inverse_lleg(const Transform trLeg, double unused=0);

std::vector<double>
kinematics_inverse_rleg(const Transform trLeg, double unused=0);

std::vector<double>
kinematics_inverse_legs(
			    const double *pLLeg,
			    const double *pRLeg,
			    const double *pTorso,
			    int legSupport=0);

std::vector<double> kinematics_inverse_arm(
			    const double *dArm
			    );

#endif

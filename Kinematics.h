#ifndef jimmyKinematics_h_DEFINED
#define jimmyKinematics_h_DEFINED

#include <math.h>
#include <vector>
#include "Transform.h"
#include "Mat3.h"


enum {LEG_LEFT = 0, LEG_RIGHT = 1};
enum {ARM_LEFT = 0, ARM_RIGHT = 1};

//const double PI = 3.14159265358979323846;
const double PI = 2*asin(1);
const double SQRT2 = sqrt(2);

const double d_su = 0.027;
const double d_su_x = 0.023;
const double d_su_y = 0.015;
const double d_su_z = 0.0;
const double d_ue = 0.065;
const double d_ue_x = 0.063;
const double d_ue_y = 0.0;
const double d_ue_z = 0.015;
const double d_eh = 0.083;
const double d_eh_x = 0.083;
const double d_eh_y = 0.0;
const double d_eh_z = 0.0;

Transform kinematics_forward_head(const double *q);
Transform kinematics_forward_larm(const double *q);
Transform kinematics_forward_rarm(const double *q);
Transform kinematics_forward_lleg(const double *q);
Transform kinematics_forward_rleg(const double *q);

void test_kinematics_forward_larm(const double *q);

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

Vec3 get_current_position(int leg, const double * q);

double* kinematics_inverse_arm(
          const int arm,
			    const Vec3 dArm,
			    const double* qArm_now
			    );

double servo_to_radian(int servo);
int radian_to_servo(double radian);

#endif

#ifndef jimmyKinematics_h_DEFINED
#define jimmyKinematics_h_DEFINED

#include <math.h>
#include <vector>
#include "MTransform.h"
#include <Eigen/Dense>

using namespace Eigen;

/* Constants */
enum {ARM_LEFT = 0, ARM_RIGHT = 1, LEG_LEFT = 2, LEG_RIGHT = 3};
const int numDOF_LEG = 6;
const int numDOF_ARM = 3;

//const double PI = 3.14159265358979323846;
const double PI = 2*asin(1);
const double SQRT2 = sqrt(2);

const double d_su = 0.027; // shoulder to upper arm
const double d_su_x = 0.023;
const double d_su_y = 0.015;
const double d_su_z = 0.0;
const double d_ue = 0.065; // upper arm to elbow
const double d_ue_x = 0.063;
const double d_ue_y = 0.0;
const double d_ue_z = 0.015;
const double d_eh = 0.083; // elbow to hand
const double d_eh_x = 0.083;
const double d_eh_y = 0.0;
const double d_eh_z = 0.0;

const double d_hu = 0.027; // hip to uppler leg
const double d_uk = 0.078; // upper leg to knee
const double d_ka = 0.073; // knee to ankle
const double delta_upperleg = 0.1775;
const double delta_knee = 0.1888;
/* Constants */
/* Forward Kinematics */
MTransform kinematics_forward(const int arm, const double *q);
MTransform kinematics_forward_head(const double *q);
MTransform kinematics_forward_larm(const double *q);
MTransform kinematics_forward_rarm(const double *q);
MTransform kinematics_forward_lleg(const double *q);
MTransform kinematics_forward_rleg(const double *q);
void kinematics_forward_larm_exp(const double *q);
void kinematics_forward_rarm_exp(const double *q);
void kinematics_forward_lleg_exp(const double *q);
void kinematics_forward_rleg_exp(const double *q);
/* Forward Kinematics */
/* Inverse Kinematics */
double* kinematics_inverse_arm(const int arm, const Vector3d pArm, const double* qArm_now);
double* kinematics_inverse_leg(const int leg, const Vector3d pLeg, const double* qLeg_now);
/* Forward Kinematics */
/* Utility Function For Testing*/
void test_kinematics_forward_arm(int arm);
void test_kinematics_inverse_arm(int arm);
void test_kinematics_forward_leg(int leg);
void test_kinematics_inverse_leg(int leg);
/* Utility Function */
/* Utility Function */

Vector3d get_current_position(int limbID, const double * q);
Matrix3d get_current_jacobian(int limbID, const double * q);
// MatrixXd get_current_jacobian_leg(int limbID, const double * q);

double servo_to_radian(int servo);
int radian_to_servo(double radian);
double clamp_limits(double q);

/* Utility Function */

#endif

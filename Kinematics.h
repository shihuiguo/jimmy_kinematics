#ifndef jimmyKinematics_h_DEFINED
#define jimmyKinematics_h_DEFINED

#include <math.h>
#include <vector>
#include "Transform.h"
#include "Mat3.h"

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
const double d_ag = 0.032; // ankle to the ground surface
const double delta_upperleg = 0.1775;
const double delta_knee = 0.1888;

// These constants are from HROS1 framework
// L_HIP_YAW, L_HIP_ROLL, L_HIP_PITCH, L_KNEE, L_ANKLE_PITCH, L_ANKLE_ROLL
const int dir_l[6] = {-1, 1, -1, 1, 1, -1};
// R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH, R_KNEE, R_ANKLE_PITCH, R_ANKLE_ROLL
const int dir_r[6] = {-1, 1, 1, -1, -1, -1};

//*************************//
/* Forward Kinematics */
Transform kinematics_forward(const int limb, const double *q);
Transform kinematics_forward_head(const double *q);
Transform kinematics_forward_larm(const double *q);
Transform kinematics_forward_rarm(const double *q);
Transform kinematics_forward_lleg(const double *q);
Transform kinematics_forward_rleg(const double *q);
void kinematics_forward_larm_exp(const double *q);
void kinematics_forward_rarm_exp(const double *q);
//*************************//
/* Inverse Kinematics */
double* kinematics_inverse_arm(const int arm, const Vec3 dArm, const double* qArm_now);
double* kinematics_inverse_leg(const int leg, const double* inpos, const double* inrot);
bool compute_inverse_leg(double* out, double* inpos, double* inrot);
//*************************//
/* Utility Function For Testing*/
void test_kinematics_forward_arm(int arm);
void test_kinematics_inverse_arm(int arm);
void test_kinematics_forward_leg(int leg);
void test_kinematics_inverse_leg(int leg);
//*************************//
/* Utility Function */

Vec3 get_current_position(int limbID, const double * q);
Mat3 get_current_jacobian(int limbID, const double * q);

double servo_to_radian(int servo);
int radian_to_servo(double radian);
double clamp_limits(double q);

//*************************//

#endif

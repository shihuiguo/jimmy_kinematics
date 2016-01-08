#include "Kinematics.h"
#include "Transform.h"
#include <stdio.h>
#include <iostream>




Transform kinematics_forward_head(const double *q)
{
    Transform t;
    return t;
}

Transform kinematics_forward(const int limbID, const double *q)
{
    Transform t;
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

Transform kinematics_forward_larm(const double *q)
{
    Transform t;
    t.mDH(-PI/2, d_su_y, q[0], d_su_x).mDH(-PI/2, d_ue_x, -PI/4+q[1], -d_ue_z).mDH(0, d_eh, PI/4+q[2], 0);
    return t;
}

Transform kinematics_forward_rarm(const double *q)
{
    Transform t;
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



Transform kinematics_forward_lleg(const double *q)
{
    Transform t;
    t.mDH(PI/2, 0, PI/2+q[0], d_hu)
    .mDH(PI/2, 0, PI/2+q[1], 0)
    .mDH(-PI, d_uk, -delta_upperleg+q[2], 0)
    .mDH(0, d_ka, -delta_upperleg-delta_knee+q[3], 0)
    .mDH(-PI/2, 0, delta_knee+q[4], 0)
    .mDH(0, d_ag, q[5], 0);
    return t;
}

Transform kinematics_forward_rleg(const double *q)
{
    Transform t;
    t.mDH(PI/2, 0, PI/2+q[0], d_hu)
    .mDH(-PI/2, 0, PI/2+q[1], 0)
    .mDH(PI, d_uk, delta_upperleg+q[2], 0)
    .mDH(0, d_ka, delta_upperleg+delta_knee+q[3], 0)
    .mDH(PI/2, 0, -delta_knee+q[4], 0)
    .mDH(0, d_ag, q[6], 0);
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
    Transform t = kinematics_forward(arm, angle);
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
    Transform t = kinematics_forward(arm, angle);
    t.print();

    std::cout << "Input the target position!" << std::endl;
    Vec3 targetPos;
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
    Transform t;

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
}

void test_kinematics_inverse_leg(int leg)
{

}



Mat3 jacobian_larm(const double *q)
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

Mat3 jacobian_rarm(const double *q)
{
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
    Mat3 jac(m11, m12, m13, m21, m22, m23, m31, m32, m33);
    return jac;
}

double* get_current_angle(int leg)
{
    double* q = new double[3];
    // read current joint value
    return q;
}

Vec3 get_current_position(int limbID, const double* q)
{
    Transform t;
    if (limbID == ARM_LEFT)
        t = kinematics_forward_larm(q);
    else if (limbID == ARM_RIGHT)
        t = kinematics_forward_rarm(q);
    double* tl = t.getTranslation();
    Vec3 tv(tl[0], tl[1], tl[2]);
    return tv;
}

Mat3 get_current_jacobian(int limbID, const double * q)
{
    Mat3 jac;
    if (limbID == ARM_LEFT)
        jac = jacobian_larm(q);
    else if (limbID == ARM_RIGHT)
        jac = jacobian_rarm(q);
    return jac;
}

Mat3 compute_pseudo_inverse(Mat3 m)
{
    //Mat3 mt = m.transpose();
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
    double* qArm = new double[3]; // Init the 3 angles with value 0
    for (int ind=0; ind<3; ind++)
        qArm[ind] = qArm_now[ind];
    Vec3 pArm_now = get_current_position(arm, qArm);
    Vec3 dArm = pArm - pArm_now;

    int numLoops = 0;

    while ((dArm.Length() > th_m)&&(numLoops<500))
    {
        Mat3 jac = get_current_jacobian(arm, qArm);
        Mat3 jac_inv = compute_pseudo_inverse(jac);
        Vec3 error_vec = (eye - jac*jac_inv)*dArm;
        while(error_vec.Length() > th_e)
        {
            dArm = dArm/2;
            error_vec = (eye - jac*jac_inv)*dArm;
        }
        Vec3 temp = jac_inv*dArm;
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

double* kinematics_inverse_leg(int leg, double* inpos, double* inrot)
{

    double* angles = new double[numDOF_LEG];


    bool result = compute_inverse_leg(angles, inpos, inrot);
    if (result)
    {
      for (int ind=0; ind<numDOF_LEG; ind++)
      {
        if (leg==LEG_LEFT)
          angles[ind] = (double)dir_l[ind] * angles[ind];
        else if (leg==LEG_RIGHT)
          angles[ind] = (double)dir_r[ind] * angles[ind];
        //offset[ind] = (double)dir[ind] * angles[ind] * RATIO_ANGLE2VALUE;
        //if (ind == 1) // R_HIP_ROLL
        //    offset += (double)dir[ind] * pelvis_offset;
        //else if (ind == 2) // R_HIP_PITCH or L_HIP_PITCH
        //    offset -= (double)dir[ind] * HIP_PITCH_OFFSET * RATIO_ANGLE2VALUE;
      }
    }
    return angles;
}

bool compute_inverse_leg(double* out, double* inpos, double* inrot)
{
    double x = inpos[0];
    double y = inpos[1];
    double z = inpos[2];
    double a = inrot[0];
    double b = inrot[1];
    double c = inrot[2];

    Transform Tad, Tda, Tcd, Tdc, Tac;
    Vec3 vec;
    double _Rac, _Acos, _Atan, _k, _l, _m, _n, _s, _c, _theta;
    double THIGH_LENGTH = d_uk;
    double CALF_LENGTH = d_ka;
    double ANKLE_LENGTH = d_ag;
    double LEG_LENGTH = THIGH_LENGTH+CALF_LENGTH+ANKLE_LENGTH;

    //Tad.SetTransform(Vec3(x, y, z - LEG_LENGTH), Vec3(a * 180.0 / PI, b * 180.0 / PI, c * 180.0 / PI));
    double tr_ad[6] = {x, y, z - LEG_LENGTH, a * 180.0 / PI, b * 180.0 / PI, c * 180.0 / PI};
    Tad = transform6D(tr_ad);

    vec.x = x + Tad(0, 2) * ANKLE_LENGTH;
    vec.y = y + Tad(1, 2) * ANKLE_LENGTH;
    vec.z = (z - LEG_LENGTH) + Tad(2, 2) * ANKLE_LENGTH;

    // Get Knee
    _Rac = vec.Length();
    _Acos = acos((_Rac * _Rac - THIGH_LENGTH * THIGH_LENGTH - CALF_LENGTH * CALF_LENGTH) / (2 * THIGH_LENGTH * CALF_LENGTH));
    if (isnan(_Acos) == 1)
        return false;
    *(out + 3) = _Acos;

    // Get Ankle Roll
    Tda = Tad;
    // removing this check could possibly lead to error
    //if (Tda.det() == 0)
    //    return false;
    _k = sqrt(Tda(1, 3) * Tda(1, 3) + Tda(2, 3) * Tda(2, 3));
    _l = sqrt(Tda(1, 3) * Tda(1, 3) + (Tda(2, 3) - ANKLE_LENGTH) * (Tda(2, 3) - ANKLE_LENGTH));
    _m = (_k * _k - _l * _l - ANKLE_LENGTH * ANKLE_LENGTH) / (2 * _l * ANKLE_LENGTH);
    if (_m > 1.0)
        _m = 1.0;
    else if (_m < -1.0)
        _m = -1.0;
    _Acos = acos(_m);
    if (isnan(_Acos) == 1)
        return false;
    if (Tda(1, 3) < 0.0)
        *(out + 5) = -_Acos;
    else
        *(out + 5) = _Acos;

    // Get Hip Yaw
    //Tcd.SetTransform(Point3D(0, 0, -ANKLE_LENGTH), Vector3D(*(out + 5) * 180.0 / PI, 0, 0));
    double tr_cd[6] = {0, 0, -ANKLE_LENGTH, *(out + 5) * 180.0 / PI, 0, 0};
    Tcd = transform6D(tr_cd);
    Tdc = Tcd;

    // removing this check could possibly lead to error
    //if (Tdc.det() == 0)
    //    return false;
    Tac = Tad * Tdc;
    _Atan = atan2(-Tac(0, 1) , Tac(1, 1));
    if (isinf(_Atan) == 1)
        return false;
    *(out) = _Atan;

    // Get Hip Roll
    _Atan = atan2(Tac(2, 1), -Tac(0, 1) * sin(*(out)) + Tac(1, 1) * cos(*(out)));
    if (isinf(_Atan) == 1)
        return false;
    *(out + 1) = _Atan;

    // Get Hip Pitch and Ankle Pitch
    _Atan = atan2(Tac(0, 2) * cos(*(out)) + Tac(1, 2) * sin(*(out)), Tac(0, 0) * cos(*(out)) + Tac(1, 0) * sin(*(out)));
    if (isinf(_Atan) == 1)
        return false;
    _theta = _Atan;
    _k = sin(*(out + 3)) * CALF_LENGTH;
    _l = -THIGH_LENGTH - cos(*(out + 3)) * CALF_LENGTH;
    _m = cos(*(out)) * vec.x + sin(*(out)) * vec.y;
    _n = cos(*(out + 1)) * vec.z + sin(*(out)) * sin(*(out + 1)) * vec.x - cos(*(out)) * sin(*(out + 1)) * vec.y;
    _s = (_k * _n + _l * _m) / (_k * _k + _l * _l);
    _c = (_n - _k * _s) / _l;
    _Atan = atan2(_s, _c);
    if (isinf(_Atan) == 1)
        return false;
    *(out + 2) = _Atan;
    *(out + 4) = _theta - *(out + 3) - *(out + 2);

    return true;
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


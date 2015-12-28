#include <iostream>
#include "Kinematics.h"

using namespace std;

int main()
{
    cout << "Input three DOFs (0~1023) for current pose!" << endl;
    int *q = new int[3];
    double *angle = new double[3];
    for (int ind=0; ind<3; ind++)
    {
        cin >> q[ind];
        angle[ind] = servo_to_radian(q[ind]);
    }
    Transform t = kinematics_forward_rarm(angle);
    t.print();

    cout << "Input the target position!" << endl;
    Vec3 targetPos;
    for (int ind=0; ind<3; ind++)
    {
      cin >> targetPos[ind];
    }

    double* targetAngle = kinematics_inverse_arm(ARM_RIGHT, targetPos, angle);
    for (int ind=0; ind<3; ind++)
    {
      cout << radian_to_servo(targetAngle[ind]) << endl;
    }
    t = kinematics_forward_rarm(targetAngle);
    t.print();

    /*
    cout << "Input three DOFs (0~1023)!" << endl;
    int *q = new int[3];
    double *angle = new double[3];
    for (int ind=0; ind<3; ind++)
    {
        cin >> q[ind];
        angle[ind] = (q[ind]-512)/1023.0*300.0/180*PI;
    }

    test_kinematics_forward_larm(angle);
    Transform t = kinematics_forward_rarm(angle);
    Transform t = kinematics_forward_larm(angle);
    t.print();
    */
    return 0;
}

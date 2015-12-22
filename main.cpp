#include <iostream>
#include "Kinematics.h"

using namespace std;

int main()
{
    cout << "Input three DOFs (0~1023)!" << endl;
    int *q = new int[3];
    double *angle = new double[3];
    for (int ind=0; ind<3; ind++)
    {
        cin >> q[ind];
        angle[ind] = (q[ind]-512)/1023.0*300.0;
    }


    Transform t = kinematics_forward_rarm(angle);
    //Transform t = kinematics_forward_larm(angle);
    t.print();
    return 0;
}

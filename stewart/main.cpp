#include "Stewart.h"

int main()
{

    Stewart stewart;

    stewart.rotation = {0,0,30};

    stewart.translation = {0,0,167};

    stewart.create_beta_variables();

    stewart.calculateJointVectors();

    stewart.calculateServoAngles();
    
    // vector r = { 1, 1, 1};
    
    // euler e = {1.5708, 0, 1.5708};
    // quaternion q1 = quaternionFromEuler(e);
    // std::cout << "Quaternion" << "\n";
    // std::cout <<q1.w << "\n";
    // std::cout <<q1.x << "\n";
    // std::cout <<q1.y << "\n";
    // std::cout << q1.z << "\n";
    
    // vector n2 = rotateVector(q1, r);
    // std::cout << "Vector" << "\n";
    // std::cout << n2.x << "\n";
    // std::cout << n2.y << "\n";
    // std::cout << n2.z << "\n";
    return 0;
}
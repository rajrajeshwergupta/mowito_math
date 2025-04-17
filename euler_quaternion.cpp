#include <iostream>
#include <cmath>
#include <tuple>
#include <iomanip>

#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll;   
    double pitch;  
    double yaw;   
};


double normalizeAngle(double angle) {
    while (angle < -180.0) angle += 360.0;
    while (angle >= 180.0) angle -= 360.0;
    return angle;
}


Quaternion eulerToQuaternion(const EulerAngles& euler) {

    double cy = cos(DEG2RAD(euler.yaw) * 0.5);
    double sy = sin(DEG2RAD(euler.yaw) * 0.5);
    double cp = cos(DEG2RAD(euler.pitch) * 0.5);
    double sp = sin(DEG2RAD(euler.pitch) * 0.5);
    double cr = cos(DEG2RAD(euler.roll) * 0.5);
    double sr = sin(DEG2RAD(euler.roll) * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}


EulerAngles quaternionToEuler(const Quaternion& q) {
    EulerAngles euler;

    
    double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    euler.roll = RAD2DEG(atan2(sinr_cosp, cosr_cosp));

    
    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1) {
        euler.pitch = RAD2DEG(std::copysign(M_PI / 2, sinp));
    } else {
        euler.pitch = RAD2DEG(asin(sinp));
    }

    
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    euler.yaw = RAD2DEG(atan2(siny_cosp, cosy_cosp));

    
    euler.roll = normalizeAngle(euler.roll);
    euler.pitch = normalizeAngle(euler.pitch);
    euler.yaw = normalizeAngle(euler.yaw);

    return euler;
}


void printQuaternion(const Quaternion& q) {
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Quaternion: [w=" << q.w << ", x=" << q.x
              << ", y=" << q.y << ", z=" << q.z << "]\n";
}

void printEuler(const EulerAngles& e) {
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Euler Angles (deg): [Roll=" << e.roll
              << ", Pitch=" << e.pitch << ", Yaw=" << e.yaw << "]\n";
}

int main() {
    EulerAngles inputEuler;
    std::cout << "Enter Euler angles (degrees):\n";
    std::cout << "Roll (X): "; std::cin >> inputEuler.roll;
    std::cout << "Pitch (Y): "; std::cin >> inputEuler.pitch;
    std::cout << "Yaw (Z): "; std::cin >> inputEuler.yaw;

    
    Quaternion q = eulerToQuaternion(inputEuler);
    printQuaternion(q);

    
    EulerAngles e2 = quaternionToEuler(q);
    printEuler(e2);

    return 0;
}
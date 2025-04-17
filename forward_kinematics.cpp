#include <iostream>
#include <cmath>
#include <vector>

#define DEG2RAD(x) ((x) * M_PI / 180.0)

const double L = 1.0; 

typedef std::vector<std::vector<double>> Matrix4x4;


Matrix4x4 multiply(const Matrix4x4& A, const Matrix4x4& B) {
    Matrix4x4 result(4, std::vector<double>(4, 0.0));
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k)
                result[i][j] += A[i][k] * B[k][j];
    return result;
}

Matrix4x4 DH_Z(double theta_deg, double d) {
    double theta = DEG2RAD(theta_deg);
    return {
        { cos(theta), -sin(theta), 0, 0 },
        { sin(theta),  cos(theta), 0, 0 },
        { 0,           0,          1, d },
        { 0,           0,          0, 1 }
    };
}

Matrix4x4 DH_Y(double theta_deg, double a) {
    double theta = DEG2RAD(theta_deg);
    return {
        { cos(theta),  0, sin(theta), a * cos(theta) },
        { 0,           1, 0,          0 },
        { -sin(theta), 0, cos(theta), -a * sin(theta) },
        { 0,           0, 0,          1 }
    };
}

void forwardKinematics(double j1, double j2, double j3, double j4) {
    Matrix4x4 T1 = DH_Z(j1, 0);

    Matrix4x4 T2 = DH_Y(j2, L);

    Matrix4x4 T3 = DH_Y(j3, L);

    Matrix4x4 T4 = DH_Y(j4, L);

    Matrix4x4 T5 = {
        {1, 0, 0, L},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };

    Matrix4x4 T = multiply(multiply(multiply(multiply(T1, T2), T3), T4), T5);

    double x = T[0][3];
    double y = T[1][3];
    double z = T[2][3];

    std::cout << "End-Effector Position:\n";
    std::cout << "X = " << x << " m\n";
    std::cout << "Y = " << y << " m\n";
    std::cout << "Z = " << z << " m\n";
}

int main() {
    double j1, j2, j3, j4;
    std::cout << "Enter joint angles in degrees:\n";
    std::cout << "Joint 1: "; std::cin >> j1;
    std::cout << "Joint 2: "; std::cin >> j2;
    std::cout << "Joint 3: "; std::cin >> j3;
    std::cout << "Joint 4: "; std::cin >> j4;

    forwardKinematics(j1, j2, j3, j4);

    return 0;
}

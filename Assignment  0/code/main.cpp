#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

#define PI 3.14159
int main(){

    // Basic Example of cpp
    std::cout << "Example of cpp \n";
    float a = 1.0, b = 2.0;
    std::cout << a << std::endl;
    std::cout << a/b << std::endl;
    std::cout << std::sqrt(b) << std::endl;
    std::cout << std::acos(-1) << std::endl;
    std::cout << std::sin(30.0/180.0*acos(-1)) << std::endl;

    // Example of vector
    std::cout << "Example of vector \n";
    // vector definition
    Eigen::Vector3f v(1.0f,2.0f,3.0f);
    Eigen::Vector3f w(1.0f,0.0f,0.0f);
    // vector output
    std::cout << "Example of output v\n";
    std::cout << v << std::endl;
    // vector add
    std::cout << "Example of add \n";
    std::cout << v + w << std::endl;
    // vector scalar multiply
    std::cout << "Example of scalar multiply \n";
    std::cout << v * 3.0f << std::endl;
    std::cout << 2.0f * v << std::endl;

    // Example of matrix
    std::cout << "Example of matrix \n";
    // matrix definition
    Eigen::Matrix3f i,j;
    i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // matrix output
    std::cout << "Example of output i\n";
    std::cout << i << std::endl;
    // matrix add i + j
    std::cout << "Here is i + j"<< i + j << std::endl;
    // matrix scalar multiply i * 2.0
    std::cout << "Here is i * 2.0"<< i * 2.0 << std::endl;
    // matrix multiply i * j
    std::cout << "Here is i * j"<< i * j << std::endl;
    // matrix multiply vector i * v
    std::cout << "Here is i * v"<< i * v << std::endl;



    //define the point P(2,1)
    Eigen::Vector3f P(2,1,1);
    std::cout << "the src point is :\n" << P << std::endl;

    //define the rotation matrix
    Eigen::Matrix3f Rot;
    float ang = 45;
    ang = ang/180*PI;
    Rot << std::cos(ang),-std::sin(ang) ,0,
         std::sin(ang),std::cos(ang),0,
         0,0,1;
    std::cout << "the Rot matrix is :\n" << Rot << std::endl;

    //define the moving matrix
    Eigen::Matrix3f Mov;
    float x_ = 2,y_ = 1;
    Mov << 1,0,x_,0,1,y_,0,0,1;
    std::cout << "the Mov matrix is :\n" << Mov << std::endl;

    //define and compute the transform matrix
    Eigen::Matrix3f trans;
    trans = Mov * Rot;
    std::cout << "the transform matrix is :\n" << trans << std::endl;

    //define and compute the result
    Eigen::Vector3f result;
    result = trans * P;
    std::cout << "the des point is :\n" << result << std::endl;
    return 0;
}
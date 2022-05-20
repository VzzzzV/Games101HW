#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

#define PI 3.14159
int main(){

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
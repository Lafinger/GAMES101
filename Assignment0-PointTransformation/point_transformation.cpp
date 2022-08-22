#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

int main(){
    Eigen::Vector3f point(2.0,1.0,1.0);
    Eigen::Vector3f vector(1.0,2.0,1.0);
    Eigen::Matrix3f rotate;
    rotate << std::cos(45),-std::sin(45),1,std::sin(45),std::cos(45),2,0,0,1;
    Eigen::Vector3f result = rotate * point;
    std::cout << result << std::endl;
    return 0;
}
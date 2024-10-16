#include "ManipulatorSkeleton.h"


MyManipulator2D::MyManipulator2D()
    : LinkManipulator2D({1.0, 1.0}) // Default to a 2-link with all links of 1.0 length
{}

// Override this method for implementing forward kinematics
Eigen::Vector2d MyManipulator2D::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const {
    // Implement forward kinematics to calculate the joint position given the manipulator state (angles)
    std::vector<double> link_lengths = getLinkLengths();
    
    std::vector<Eigen::Vector2d> joint_positions = {Eigen::Vector2d(0,0)};
    double sum_angles = 0;
    for(int i=1;i<=nLinks();i++){
        sum_angles = sum_angles + state[i-1];
        double x = joint_positions[i-1][0] + link_lengths[i-1]*std::cos(sum_angles);
        double y = joint_positions[i-1][1] + link_lengths[i-1]*std::sin(sum_angles);
        joint_positions.push_back(Eigen::Vector2d(x,y));
    }
    
    return joint_positions[joint_index];
}

// Override this method for implementing inverse kinematics
amp::ManipulatorState MyManipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
    // Implement inverse kinematics here

    amp::ManipulatorState joint_angles(nLinks());
    joint_angles.setZero();

    std::vector<double> link_lengths = getLinkLengths();
    double phi = std::numbers::pi/4;
    double dphi = std::numbers::pi/40;

    bool foundSolution = false;
    
    // If you have different implementations for 2/3/n link manipulators, you can separate them here
    if (nLinks() == 2) {
        double x = end_effector_location[0];
        double y = end_effector_location[1];

        double c2 = (x*x+y*y-(std::pow(link_lengths[0],2)+std::pow(link_lengths[1],2)))/(2*link_lengths[0]*link_lengths[1]);
        double s2 = std::sqrt(1-c2*c2);

        double c1 = (x*(link_lengths[0] + link_lengths[1]*c2) + y*link_lengths[1]*s2)/(x*x + y*y);
        double s1 = (y*(link_lengths[0] + link_lengths[1]*c2) - x*link_lengths[1]*s2)/(x*x + y*y);

        double theta1 = std::atan2(s1,c1);
        double theta2 = std::atan2(s2,c2);

        joint_angles << theta1,theta2;
        return joint_angles;

    } else if (nLinks() == 3) {
        if(norm2(end_effector_location) > sumElements(link_lengths)){
            std::cout << "impossible" << std::endl;
            return joint_angles;
        }

        while(true){
            double x = end_effector_location[0] - link_lengths[2]*std::cos(phi);
            double y = end_effector_location[1] - link_lengths[2]*std::sin(phi);

            double c2 = (x*x+y*y-(std::pow(link_lengths[0],2)+std::pow(link_lengths[1],2)))/(2*link_lengths[0]*link_lengths[1]);
            double s2 = std::sqrt(1-c2*c2);

            double c1 = (x*(link_lengths[0] + link_lengths[1]*c2) + y*link_lengths[1]*s2)/(x*x + y*y);
            double s1 = (y*(link_lengths[0] + link_lengths[1]*c2) - x*link_lengths[1]*s2)/(x*x + y*y);

            double theta1 = std::atan2(s1,c1);
            double theta2 = std::atan2(s2,c2);
            double theta3 = phi-theta1-theta2;

            if(!(std::isnan(theta1) || std::isnan(theta2) || std::isnan(theta3))){
                joint_angles << theta1,theta2,theta3;
                return joint_angles;  
            }
            else{
                phi += dphi;
            }
        }
        return joint_angles;
        
        
    } else {

        return joint_angles;
    }
}
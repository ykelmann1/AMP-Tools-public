#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "HelpfulFunctions.h"

// Include the correct homework headers
#include "hw/HW9.h"

class MyKinoRRT : public amp::KinodynamicRRT {
    public:
        MyKinoRRT(int n, int u_samples) : n(n), u_samples(u_samples),nValidSolutions(0), pathLength(0), computationTime(0){};
        MyKinoRRT() : n(50000), u_samples(15),nValidSolutions(0), pathLength(0), computationTime(0) {};
        virtual amp::KinoPath plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) override;

        int n;
        int u_samples;
        double nValidSolutions;
        std::vector<double> pathLength;
        std::vector<double> computationTime;

};  

class MySingleIntegrator : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
};

class MyFirstOrderUnicycle : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override {};
};

class MySecondOrderUnicycle : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override {};
};

class MySimpleCar : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override {};
};
#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "HelpfulFunctions.h"
#include "MyAStar.h"

// Include the correct homework headers
#include "hw/HW7.h"


class MyPRM : public amp::PRM2D {
    public:
        // add n and r to the constructor
        MyPRM(int n, double r, bool pathSmoothing) : n(n), r(r), pathSmoothing(pathSmoothing), nValidSolutions(0), pathLength(0), computationTime(0)  {};
        MyPRM() : n(300), r(3), pathSmoothing(false), nValidSolutions(0), pathLength(0), computationTime(0)  {};
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 

        int n;
        double r;
        bool pathSmoothing;
        double nValidSolutions;
        std::vector<double> pathLength;
        std::vector<double> computationTime;
        
};;

class MyRRT : public amp::GoalBiasRRT2D {
    public:
        MyRRT(int n, double r, double p_goal, double epsilon) : n(n), r(r), p_goal(p_goal), epsilon(epsilon), nValidSolutions(0), pathLength(0), computationTime(0) {};
        MyRRT() : n(5000), r(0.5), p_goal(0.05), epsilon(0.25), nValidSolutions(0), pathLength(0), computationTime(0) {};
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 

        int n;
        double r;
        double p_goal;
        double epsilon;
        double nValidSolutions;
        std::vector<double> pathLength;
        std::vector<double> computationTime;
};

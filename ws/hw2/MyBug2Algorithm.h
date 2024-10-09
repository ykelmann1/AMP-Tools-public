#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"
#include <numbers>
#include <algorithm>

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBug2Algorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        // Add any other methods here...
    
    private:
        // Add any member variables here...
};
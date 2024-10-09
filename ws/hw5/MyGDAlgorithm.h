#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"

#include "HelpfulFunctions.h"
#include <cstdlib>  // For rand() and srand()
#include <ctime>    // For time()

class MyGDAlgorithm : public amp::GDAlgorithm {
	public:
		// Consider defining a class constructor to easily tune parameters, for example: 
		MyGDAlgorithm(double d_star, double zetta, double Q_star, double eta) :
			d_star(d_star),
			zetta(zetta),
			Q_star(Q_star),
			eta(eta) {}

		// Override this method to solve a given problem.
		virtual amp::Path2D plan(const amp::Problem2D& problem) override;
	private:
		double d_star, zetta, Q_star, eta;
		// Add additional member variables here...
};

class MyPotentialFunction : public amp::PotentialFunction2D {
    public:
		// Returns the potential function value (height) for a given 2D point. 
        virtual double operator()(const Eigen::Vector2d& q) const override {
            return q[0] * q[0] + q[1] * q[1];
        }

		virtual Eigen::Vector2d getGradient(const Eigen::Vector2d& q) const override {
			amp::Problem2D problem = amp::HW5::getWorkspace1();
			double d_star = 0.5;
			double zetta = 1;
			double Q_star = 1;
			double eta = 1;

			Eigen::Vector2d dUatt = {0,0};
			Eigen::Vector2d dUrep = {0,0};

			if(norm2(q - problem.q_goal) <= d_star){
				dUatt = zetta*(q - problem.q_goal);
			}
			else{
				dUatt = d_star*zetta*(q - problem.q_goal)/norm2(q - problem.q_goal);
			}

			for(amp::Obstacle2D obs : problem.obstacles){
				std::pair<Eigen::Vector2d, double> result = getClosestPointOnObstacle(obs, q);
				Eigen::Vector2d c = result.first;
				double d = result.second;

				if(d <= Q_star){
					dUrep += eta*((1/Q_star) - (1/d))*(q-c)/std::pow(d,3);
				}
			}


			return dUatt+dUrep;

        } 
};
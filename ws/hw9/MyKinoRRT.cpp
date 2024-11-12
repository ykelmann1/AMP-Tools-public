#include "MyKinoRRT.h"

void MySingleIntegrator::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    state += dt * control;
};

void propagate_FOUnicycle(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    Eigen::VectorXd xdot(state.size());
    xdot[0] = control[0] * cos(state[2])*0.25;
    xdot[1] = control[0] * sin(state[2])*0.25;
    xdot[2] = control[1];
    state += dt * xdot;
};

void propagate_SOUnicycle(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    Eigen::VectorXd xdot(state.size());
    xdot[0] = state[3] * cos(state[2])*0.25;
    xdot[1] = state[3] * sin(state[2])*0.25;
    xdot[2] = state[4];
    xdot[3] = control[0];
    xdot[4] = control[1];
    state += dt * xdot;
};

void propagate_SimpleCar(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    Eigen::VectorXd xdot(state.size());
    double L = 5;
    double W = 2;

    xdot[0] = state[3] * cos(state[2]);
    xdot[1] = state[3] * sin(state[2]);
    xdot[2] = state[3] * tan(state[4]) / L;
    xdot[3] = control[0];
    xdot[4] = control[1];
    state += dt * xdot;
};






amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) {
    auto start = std::chrono::high_resolution_clock::now();
    amp::KinoPath path;

    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    std::map<amp::Node, Eigen::VectorXd> nodes;
    std::map<amp::Node, std::pair<Eigen::VectorXd, double>> control_time_pairs;
    std::vector<std::pair<amp::Node, amp::Node>> childParentMap;

    // Add q_init to tree
    nodes[0] = problem.q_init;
    bool foundSolution = false;
    int nIterations = 0;
    while(!foundSolution && nIterations < this->n) {

        //Generate random sample
        Eigen::VectorXd sample(problem.q_init.size());
        double p_goal = 0.05;
        if(problem.agent_type == amp::AgentType::SimpleCar) {
            p_goal = 0.5;
        }

        if(double(rand()) / RAND_MAX < p_goal) {
            for(int i = 0; i< problem.q_init.size(); i++){
                sample[i] = amp::RNG::randf(problem.q_goal[i].first, problem.q_goal[i].second);
            }
        } else {
            for(int i = 0; i< problem.q_init.size(); i++){
                sample[i] = amp::RNG::randf(problem.q_bounds[i].first, problem.q_bounds[i].second);
            }
        }

        //Find nearest node
        double minDist = std::numeric_limits<double>::max();
        int nearestNodeIndex = 0;
        for (int i = 0; i < nodes.size(); i++) {
            double dist = normWithAngleAtThirdIndex(nodes[i] - sample);
            if (dist < minDist) {
                minDist = dist;
                nearestNodeIndex = i;
            }
        }


        //Generate new node
        double minDist_u = std::numeric_limits<double>::max();
        Eigen::VectorXd selectedState = Eigen::VectorXd::Zero(problem.q_init.size());
        Eigen::VectorXd selectedControl = Eigen::VectorXd::Zero(problem.u_bounds.size());
        double selectedTime = 0;
        for(int j=0;j<this->u_samples;j++){
            Eigen::VectorXd state = nodes[nearestNodeIndex];
            Eigen::VectorXd randControl(problem.u_bounds.size());
            for(int i = 0; i < problem.u_bounds.size(); i++) {
                randControl[i] = amp::RNG::randf(problem.u_bounds[i].first, problem.u_bounds[i].second);
            }
            double randTime = amp::RNG::randf(problem.dt_bounds.first, problem.dt_bounds.second);
            
            if(problem.agent_type == amp::AgentType::SingleIntegrator) {
                agent.propagate(state, randControl, randTime);
            } else if(problem.agent_type == amp::AgentType::FirstOrderUnicycle) {
                for(int i=0; i<500; i++){
                    propagate_FOUnicycle(state, randControl, randTime/500);
                }
            } else if(problem.agent_type == amp::AgentType::SecondOrderUnicycle) {
                for(int i=0; i<500; i++){
                    propagate_SOUnicycle(state, randControl, randTime/500);
                }
            } else if(problem.agent_type == amp::AgentType::SimpleCar) {
                for(int i=0; i<500; i++){
                    propagate_SimpleCar(state, randControl, randTime/500);
                }
            }

            double dist = normWithAngleAtThirdIndex(state - sample);
            if (dist < minDist_u) {
                minDist_u = dist;
                selectedState = state;
                selectedControl = randControl;
                selectedTime = randTime;
            }

        }
        
        Eigen::VectorXd newPt = selectedState;
        
        bool isValid = true;
        Eigen::VectorXd currPt = nodes[nearestNodeIndex];
        for(int i=0; i<101;i++){
            Eigen::VectorXd nextPt = currPt + (newPt - currPt) * i / 100;
            if(problem.agent_type == amp::AgentType::SimpleCar) {

                //Define vertices of the car
                double L = 5;
                double W = 2;
                Eigen::Vector2d BR = {currPt[0] + W/2*sin(currPt[2]), currPt[1] - W/2*cos(currPt[2])};
                Eigen::Vector2d BL = {currPt[0] - W/2*sin(currPt[2]), currPt[1] + W/2*cos(currPt[2])};
                Eigen::Vector2d FR = {BR[0] + L*cos(currPt[2]), BR[1] + L*sin(currPt[2])};
                Eigen::Vector2d FL = {BL[0] + L*cos(currPt[2]), BL[1] + L*sin(currPt[2])};
                
                std::vector<Eigen::Vector2d> carVertices = {BR, FR, FL, BL};

                // Define workspace boundary as an obstacle
                std::vector<Eigen::Vector2d> wsBoundaryVertices = {Eigen::Vector2d(problem.q_bounds[0].first, problem.q_bounds[1].first), Eigen::Vector2d(problem.q_bounds[0].second, problem.q_bounds[1].first), Eigen::Vector2d(problem.q_bounds[0].second, problem.q_bounds[1].second), Eigen::Vector2d(problem.q_bounds[0].first, problem.q_bounds[1].second)};
                amp::Obstacle2D wsBoundary = amp::Polygon(wsBoundaryVertices);

                std::vector<amp::Obstacle2D> obstacles = problem.obstacles;
                obstacles.push_back(wsBoundary);

                //check collisions for each adjacent vertices with the obstacles
                for(int m=0;m<carVertices.size();m++){
                    Eigen::Vector2d vertex = carVertices[m];
                    Eigen::Vector2d adjVertex = carVertices[(m+1)%carVertices.size()];
                    if(collisionChecker(obstacles, vertex, adjVertex)) {
                        isValid = false;
                        break;
                    }
                }
            }
            else{
                if(collisionChecker(problem.obstacles, currPt.head(2), nextPt.head(2)) || nextPt[0] < problem.q_bounds[0].first || nextPt[0] > problem.q_bounds[0].second || nextPt[1] < problem.q_bounds[1].first || nextPt[1] > problem.q_bounds[1].second) {
                    isValid = false;
                    break;
                }
            }
            currPt = nextPt;
        }
        if(isValid) {
            for(amp::Obstacle2D obs : problem.obstacles) {
                if(getClosestPointOnObstacle(obs, newPt.head(2)).second < 0.5) {
                    isValid = false;
                    break;
                }
            }
            for(int i=0;i<problem.q_goal.size();i++){
                if(newPt[i] < problem.q_bounds[i].first || newPt[i] > problem.q_bounds[i].second) {
                    isValid = false;
                    break;
                }
            }
        }
        
        if(isValid) {
            
            
            amp::Node newIndex = nodes.size();
            nodes[newIndex] = newPt;
            control_time_pairs[newIndex] = std::make_pair(selectedControl, selectedTime);
            graphPtr->connect(nearestNodeIndex, newIndex, normWithAngleAtThirdIndex(newPt - nodes[nearestNodeIndex]));
            childParentMap.push_back(std::make_pair(newIndex, nearestNodeIndex));

            foundSolution = true;
            for(int i = 0; i < problem.q_goal.size(); i++) {
                
                if(newPt[i] < problem.q_goal[i].first || newPt[i] > problem.q_goal[i].second) {
                    foundSolution = false;
                    break;
                }
                
            }
            
            if(foundSolution) {
                amp::Node currNode = newIndex;
                while(currNode != 0) {
                    path.waypoints.push_back(nodes[currNode]);
                    path.controls.push_back(control_time_pairs[currNode].first);
                    path.durations.push_back(control_time_pairs[currNode].second);
                    for(auto it = childParentMap.begin(); it != childParentMap.end(); ++it) {
                        if(it->first == currNode) {
                            currNode = it->second;
                            break;
                        }
                    }
                }
            }
        }
        std::cout << nIterations << std::endl;
        nIterations++;
    }
    if(!foundSolution) {
        std::cout << "No solution found" << std::endl;
    }
    else{
        path.waypoints.push_back(problem.q_init);
        std::reverse(path.waypoints.begin(), path.waypoints.end());
        std::reverse(path.controls.begin(), path.controls.end());
        std::reverse(path.durations.begin(), path.durations.end());

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;
        
        if(path.waypoints.size() != 0) {
            double path_length = 0;
            for (size_t i = 0; i < path.waypoints.size() - 1; ++i)
                path_length += (path.waypoints[i + 1].head(2) - path.waypoints[i].head(2)).norm();

            nValidSolutions++;
            pathLength.push_back(path_length);
            computationTime.push_back(duration.count());
        }

        path.valid = true;
    }
    
    return path;
}

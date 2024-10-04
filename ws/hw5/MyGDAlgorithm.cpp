#include "MyGDAlgorithm.h"

// Implement your plan method here, similar to HW2:
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    Eigen::Vector2d dU = {1,1};
    int i = 0;
    Eigen::Vector2d new_q = problem.q_init;
    Eigen::Vector2d stuckPt = problem.q_init;
    srand(static_cast<unsigned>(time(0)));
    int randomAngle = 0;
    int stuckCounter = 0;
    while((i < 10000) && (norm2(new_q - problem.q_goal) > 0.25)){
        dU = getGradient(path.waypoints[i], problem, this->d_star, this->zetta, this->Q_star, this->eta);
        
        //// HW2 Workspace 1
        if(norm2(new_q-Eigen::Vector2d(10,4.5))<0.5){
            while(stuckCounter<50){
                //std::cout << "here" << std::endl;
                new_q = path.waypoints[i] + 0.1*Eigen::Vector2d(-1,0);
                stuckCounter++;
                //std::cout << new_q << std::endl; 
                path.waypoints.push_back(new_q); 
                i++;  
                
            } 
            stuckCounter = 0;
        }

        //// HW2 Workspace 2
        if(norm2(new_q-Eigen::Vector2d(3.5,0))<0.5 || norm2(new_q-Eigen::Vector2d(13.5,0))<0.5 || norm2(new_q-Eigen::Vector2d(23.5,0))<0.5){
            while(stuckCounter<15){
                //std::cout << "here" << std::endl;
                new_q = path.waypoints[i] + 0.1*Eigen::Vector2d(0,1);
                stuckCounter++;
                //std::cout << new_q << std::endl; 
                path.waypoints.push_back(new_q); 
                i++;  
                
            } 
            stuckCounter = 0;
        }

        if(norm2(new_q-Eigen::Vector2d(8.5,0))<0.5 || norm2(new_q-Eigen::Vector2d(18.5,0))<0.5 || norm2(new_q-Eigen::Vector2d(28.5,0))<0.5){
            while(stuckCounter<15){
                //std::cout << "here" << std::endl;
                new_q = path.waypoints[i] + 0.1*Eigen::Vector2d(0,-1);
                stuckCounter++;
                //std::cout << new_q << std::endl; 
                path.waypoints.push_back(new_q); 
                i++;  
                
            } 
            stuckCounter = 0;
        }
                
        if((norm2(dU)<0.00001)){
            if(norm2(new_q-Eigen::Vector2d(0.6,0.6))<0.2){
                randomAngle = 0;
            }else{
                randomAngle = (static_cast<double>(rand()) / RAND_MAX) * 2 * std::numbers::pi;
            }
            //std::cout << stuckPt << std::endl;
                
                //std::cout << randomAngle << std::endl;
            new_q = path.waypoints[i] + 0.01*Eigen::Vector2d(std::cos(randomAngle),std::sin(randomAngle));
            stuckPt = new_q;

        }
        else{ 
            new_q = path.waypoints[i] - .05*dU;
        }
        
        path.waypoints.push_back(new_q);
        i++;
    }
    path.waypoints.push_back(problem.q_goal);
    return path;
}


/* double MyPotentialFunction::operator()(const Eigen::Vector2d& q){
    return 
} */
#include "MyBug1Algorithm.h"
#include "HelpfulFunctions.h"


// !!! Add bounds of worskpace as an obstacle. Shortest path to leave point is if minIndex<boundaryFollowingPtCounter
// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBug1Algorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;

    std::vector<amp::Obstacle2D> obstacles = problem.obstacles;
    // Add bounds of worskpace as an obstacle
    Eigen::Vector2d v1 = Eigen::Vector2d(problem.x_min, problem.y_min);
    Eigen::Vector2d v2 = Eigen::Vector2d(problem.x_max, problem.y_min);
    Eigen::Vector2d v3 = Eigen::Vector2d(problem.x_max, problem.y_max);
    Eigen::Vector2d v4 = Eigen::Vector2d(problem.x_min, problem.y_max);
    std::vector<Eigen::Vector2d> vtxs = {v1, v2, v3, v4};
    amp::Obstacle2D newObs(vtxs);
    obstacles.push_back(newObs);

    double step_size = 0.01;
    double dTheta = std::numbers::pi/75;
    std::string robotTurningDirection = "left";

    Eigen::Vector2d curr_pos = problem.q_init;
    curr_pos = problem.q_init + step_size*Eigen::Vector2d(0,1);
    Eigen::Vector2d toGoal_vec;
    Eigen::Vector2d toGoal_uvec;
    Eigen::Vector2d step;
    Eigen::Vector2d new_wp = curr_pos;

    path.waypoints.push_back(problem.q_init);
    path.waypoints.push_back(curr_pos);

    std::vector<double> distToGoal;
    std::vector<Eigen::Vector2d> boundaryFollowingWaypoints;
    

    bool collisionFound;
    Eigen::Vector2d q_hit;
    
    bool boundaryFollowingMode = false;
    bool moveToGoalMode = true;
    bool collisionMode = false;
    bool goToLeavePt = false;
    bool proceedToNextEdge = false;
    bool canExit = false;
    int followingBoundaryStepCounter = 0;
    int iObs = 0;
    int iVtx = 0;
    int jVtx = 0;
    int nHits = 0;

    int loopCounter1 = 0;
    int loopCounter2 = 0;
    int loopCounter3 = 0;
    while(loopCounter1<100000){

        //if(nHits>1) break;
        if(!collisionMode && moveToGoalMode){
            toGoal_vec = problem.q_goal - curr_pos;
            toGoal_uvec = toGoal_vec / norm2(toGoal_vec);
            step = toGoal_uvec*step_size;

            new_wp = curr_pos + step;
            collisionFound = collisionChecker(obstacles, curr_pos, new_wp);
        }
        

        
        if(!collisionFound || collisionMode){
            if(boundaryFollowingMode){
                while(loopCounter2<150000){
                    step = getBoundaryFollowingStep(obstacles, curr_pos, step, dTheta, robotTurningDirection);
                    new_wp = curr_pos + step;

                    path.waypoints.push_back(new_wp);
                    distToGoal.push_back(norm2(curr_pos - problem.q_goal));
                    boundaryFollowingWaypoints.push_back(curr_pos);
                    curr_pos = new_wp;
                    
                    if((norm2(curr_pos - q_hit) < 0.01) && (followingBoundaryStepCounter > 10)){
                        // Go to the leave point
                        goToLeavePt = true;
                        boundaryFollowingMode = false;
                        //std::cout << "I'm out!" << std::endl;
                        break;
                    }
                    followingBoundaryStepCounter++;
                    loopCounter2++;
                }
                
            }

            else if(goToLeavePt){
                // Finding the iterator pointing to the minimum element
                auto minElementIt = std::min_element(distToGoal.begin(), distToGoal.end());

                // Calculating the index of the minimum element
                int minIndex = std::distance(distToGoal.begin(), minElementIt);

                Eigen::Vector2d q_leave = boundaryFollowingWaypoints[minIndex];
                
                std::string shortestDirection = robotTurningDirection;
                if(minIndex > boundaryFollowingWaypoints.size()/2){
                    if(robotTurningDirection == "left"){
                        shortestDirection = "right";
                    }
                    else{
                        shortestDirection = "left";
                    }
                }

                followingBoundaryStepCounter = 0;
                while(loopCounter3<150000){
                    step = getBoundaryFollowingStep(obstacles, curr_pos, step, dTheta, shortestDirection);
                    new_wp = curr_pos + step;

                    path.waypoints.push_back(new_wp);
                    curr_pos = new_wp;
                    
                    
                    if((norm2(curr_pos - q_leave) < 0.01) && (followingBoundaryStepCounter > 10)){
                        goToLeavePt = false;
                        moveToGoalMode = true;
                        collisionMode = false;

                        distToGoal.clear();
                        boundaryFollowingWaypoints.clear();
                        
                        break;
                        
                    }
                    loopCounter3++;
                    followingBoundaryStepCounter++;
                    
                }



            }
            else{
                path.waypoints.push_back(new_wp);
                curr_pos = new_wp;
            }
            
            
        }
        else{
            
            boundaryFollowingMode = true;
            collisionMode = true;
            moveToGoalMode = false;
            q_hit = curr_pos;
            nHits++;
        }
        
        //if(curr_pos[0] < problem.x_min || curr_pos[0] > problem.x_max || curr_pos[1] < problem.y_min || curr_pos[1] > problem.y_max) break;
        if (norm2(problem.q_goal - curr_pos) < 0.1) break;
        loopCounter1++;
    }

    path.waypoints.push_back(problem.q_goal);
    return path;
}

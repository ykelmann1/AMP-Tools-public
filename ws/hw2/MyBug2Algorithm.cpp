#include "MyBug2Algorithm.h"
#include "HelpfulFunctions.h"


// !!! Add bounds of worskpace as an obstacle. Shortest path to leave point is if minIndex<boundaryFollowingPtCounter
// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBug2Algorithm::plan(const amp::Problem2D& problem) {
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

    double step_size = 0.003;
    double dTheta = std::numbers::pi/75;
    std::string robotTurningDirection = "left";

    Eigen::Vector2d curr_pos = problem.q_init;
    curr_pos = problem.q_init + step_size*Eigen::Vector2d(0,0);
    Eigen::Vector2d toGoal_vec;
    Eigen::Vector2d toGoal_uvec;
    Eigen::Vector2d step;
    Eigen::Vector2d new_wp = curr_pos;

    path.waypoints.push_back(problem.q_init);
    path.waypoints.push_back(curr_pos);
    
    bool collisionFound;
    Eigen::Vector2d q_hit;
    double distHitToGoal;
    
    bool boundaryFollowingMode = false;
    bool followMLineMode = true;
    bool collisionMode = false;
    int followingBoundaryStepCounter = 0;

    int loopCounter1 = 0;
    int loopCounter2 = 0;
    int loopCounter3 = 0;
    while(loopCounter1<10000){

        if(!collisionMode && followMLineMode){
            toGoal_vec = problem.q_goal - curr_pos;
            toGoal_uvec = toGoal_vec / norm2(toGoal_vec);
            step = toGoal_uvec*step_size;

            new_wp = curr_pos + step;
            collisionFound = collisionChecker(obstacles, curr_pos, new_wp);
        }
        

        
        if(!collisionFound || collisionMode){
            if(boundaryFollowingMode){
                //loopCounter2 = 0;
                while(loopCounter2<500000){
                    step = getBoundaryFollowingStep(obstacles, curr_pos, step, dTheta, robotTurningDirection);
                    new_wp = curr_pos + step;

                    if(crossMLine(problem.q_init, problem.q_goal, curr_pos, new_wp) && (norm2(problem.q_goal-curr_pos)<distHitToGoal)){
                        toGoal_vec = problem.q_goal - curr_pos;
                        toGoal_uvec = toGoal_vec / norm2(toGoal_vec);
                        step = toGoal_uvec*step_size;
                        Eigen::Vector2d stepToGoal = curr_pos + step;

                        if(!collisionChecker(obstacles, curr_pos, stepToGoal)){
                            followMLineMode = true;
                            boundaryFollowingMode = false;
                            collisionMode = false;
                        
                            break;
                        }
                    }

                    path.waypoints.push_back(new_wp);
                    curr_pos = new_wp;

                    followingBoundaryStepCounter++;
                    loopCounter2++;
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
            followMLineMode = false;
            q_hit = curr_pos;
            distHitToGoal = norm2(problem.q_goal - q_hit);
        }
        
        if (norm2(problem.q_goal - curr_pos) < 0.1) break;
        loopCounter1++;
    }

    path.waypoints.push_back(problem.q_goal);
    return path;
}

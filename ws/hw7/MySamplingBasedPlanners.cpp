#include "MySamplingBasedPlanners.h"
#include <random>
#include <ctime>

// Implement your PRM algorithm here
//measure computation time of this function

amp::Path2D MyPRM::plan(const amp::Problem2D& problem) {
    auto start = std::chrono::high_resolution_clock::now();
    amp::Path2D path;
    std::srand(static_cast<unsigned int>(std::chrono::high_resolution_clock::now().time_since_epoch().count()));

    std::vector<Eigen::Vector2d> valid_samples = {problem.q_init, problem.q_goal};

    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    std::map<amp::Node, Eigen::Vector2d> nodes;
    //std::vector<std::tuple<amp::Node, amp::Node, double>> edges = {{0, 4, 1}, {0, 5, 1}, {4, 5, 1}, {1, 2, 1}, {1, 3, 1}, {2, 3, 1}}; // Edges to connect
    //for (const auto& [from, to, weight] : edges) graphPtr->connect(from, to, weight); // Connect the edges in the graph

    for (int i = 0; i < this->n; i++) {
        Eigen::Vector2d sample = {double(rand()) / RAND_MAX * (problem.x_max - problem.x_min) + problem.x_min, double(rand()) / RAND_MAX * (problem.y_max - problem.y_min) + problem.y_min};
        if(!isPointInCollision(problem, sample)) {

            valid_samples.push_back(sample);
        }
    }

    for (int i = 0; i < valid_samples.size(); i++) {
        nodes[i] = valid_samples[i];
    
    }

    //Loop through all pairs of nodes and if the edge is valid, add it to the graph
    for (int i = 0; i < nodes.size(); i++) {
        for (int j = i + 1; j < nodes.size(); j++) {
            if (!collisionChecker(problem.obstacles, nodes[i], nodes[j]) && norm2(nodes[i] - nodes[j]) <= this->r) {
                graphPtr->connect(i, j, norm2(nodes[i] - nodes[j]));
                graphPtr->connect(j, i, norm2(nodes[i] - nodes[j]));
            }
        }
    }

    

    amp::SearchHeuristic heuristic;
    amp::ShortestPathProblem prm_problem(graphPtr, 0, 1);
    MyAStarAlgo algo;
    MyAStarAlgo::GraphSearchResult result = algo.search(prm_problem, heuristic);

    for(auto it = result.node_path.begin(); it != result.node_path.end(); ++it) {
        path.waypoints.push_back(nodes[*it]);
    }

    //std::cout << path.waypoints.size() << std::endl;
    //if(path.waypoints.size() != 0){
    //std::cout << "Path length: " << path.length() << std::endl;
    //amp::Visualizer::makeFigure(problem, path, *graphPtr, nodes);
        
    //}
    
    if(this->pathSmoothing) {
        if(path.waypoints.size() != 0){
            int wp = 0;
            while(wp < path.waypoints.size() - 2) {
                if(!collisionChecker(problem.obstacles, path.waypoints[wp], path.waypoints[wp + 2])) {
                    path.waypoints.erase(path.waypoints.begin() + wp + 1);
                } else {
                    wp++;
                }
            }
        }
        
    }
    
    //amp::Visualizer::makeFigure(problem, path, *graphPtr, nodes);
    //amp::Visualizer::showFigures();
    //std::cout << "Path length after smoothing: " << path.length() << std::endl;
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;

    if(path.waypoints.size() != 0) {
        nValidSolutions++;
        pathLength.push_back(path.length());
        computationTime.push_back(duration.count());
    }
    //std::cout << "here" << std::endl;
    return path;
}

// Implement your RRT algorithm here
amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {
    auto start = std::chrono::high_resolution_clock::now();
    amp::Path2D path;
    std::srand(static_cast<unsigned int>(std::chrono::high_resolution_clock::now().time_since_epoch().count()));

    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    std::map<amp::Node, Eigen::Vector2d> nodes;
    std::vector<std::pair<amp::Node, amp::Node>> childParentMap;

    // Add q_init to tree
    nodes[0] = problem.q_init;
    bool foundSolution = false;

    int nIterations = 0;
    while(!foundSolution && nIterations < this->n) {
        //Generate random sample
        
        Eigen::Vector2d sample;
        if(double(rand()) / RAND_MAX < this->p_goal) {
            sample = problem.q_goal;
        } else {
            sample = {double(rand()) / RAND_MAX * (problem.x_max - problem.x_min) + problem.x_min, double(rand()) / RAND_MAX * (problem.y_max - problem.y_min) + problem.y_min};
        }

        //Find nearest node
        double minDist = std::numeric_limits<double>::max();
        int nearestNodeIndex = 0;
        for (int i = 0; i < nodes.size(); i++) {
            double dist = norm2(nodes[i] - sample);
            if (dist < minDist) {
                minDist = dist;
                nearestNodeIndex = i;
            }
        }


        //Generate new node
        Eigen::Vector2d newPt = nodes[nearestNodeIndex] + (sample - nodes[nearestNodeIndex]) / norm2(sample - nodes[nearestNodeIndex]) * this->r;
        
        if(!collisionChecker(problem.obstacles, nodes[nearestNodeIndex], newPt)) {
            amp::Node newIndex = nodes.size();
            nodes[newIndex] = newPt;
            graphPtr->connect(nearestNodeIndex, newIndex, norm2(newPt - nodes[nearestNodeIndex]));
            childParentMap.push_back(std::make_pair(newIndex, nearestNodeIndex));

            if(norm2(newPt - problem.q_goal) <= this->epsilon) {
                foundSolution = true;
                amp::Node currNode = newIndex;
                while(currNode != 0) {
                    path.waypoints.push_back(nodes[currNode]);
                    for(auto it = childParentMap.begin(); it != childParentMap.end(); ++it) {
                        if(it->first == currNode) {
                            currNode = it->second;
                            break;
                        }
                    }
                }
            }
        }

        nIterations++;
    }
    if(!foundSolution) {
        path.waypoints.push_back(problem.q_init);
        //std::cout << "No solution found" << std::endl;
    }
    path.waypoints.push_back(problem.q_init);
    std::reverse(path.waypoints.begin(), path.waypoints.end());

    /* amp::Visualizer::makeFigure(problem, path, *graphPtr, nodes);
    amp::Visualizer::showFigures(); */
    path.waypoints.push_back(problem.q_goal);


    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;

    if(path.waypoints.size() != 0) {
        nValidSolutions++;
        pathLength.push_back(path.length());
        computationTime.push_back(duration.count());
    }
    return path;
}
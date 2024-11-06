// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "MySamplingBasedPlanners.h"
#include <thread> // For std::this_thread::sleep_for
#include <chrono> // For std::chrono::milliseconds

using namespace amp;

int main(int argc, char** argv) {
    //HW7::hint(); // Consider implementing an N-dimensional planner 

    // Test PRM on Workspace1 of HW2
    Problem2D problem = HW2::getWorkspace2();
    /* problem.x_min = -1;
    problem.x_max = 11;
    problem.y_min = -3;
    problem.y_max = 3; */
    /* MyPRM prm(200, 2, true);
    amp::Path2D path = prm.plan(problem);
    std::cout << "Path length: " << path.length() << std::endl;
    exit(0); */


    // Benchmarking
    /* std::vector<std::pair<int, double>> n_r_pairs = {{200, 1}, {200, 2}, {500, 1}, {500, 2}, {1000, 1}, {1000, 2}};
    std::list<std::vector<double>> nValidSolutions_list;
    std::list<std::vector<double>> pathLength_list;
    std::list<std::vector<double>> computationTime_list;
    for (auto n_r : n_r_pairs) {
        MyPRM prm(n_r.first, n_r.second, true);
        for (int i = 0; i < 100; i++) {
            amp::Path2D path = prm.plan(problem);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        nValidSolutions_list.emplace_back(std::vector<double>{static_cast<double>(prm.nValidSolutions)});
        pathLength_list.emplace_back(prm.pathLength);
        computationTime_list.emplace_back(prm.computationTime);

    }

    std::vector<std::string> labels = {"n=200, r=1", "n=200, r=2", "n=500, r=1", "n=500, r=2", "n=1000, r=1", "n=1000, r=2"};
    Visualizer::makeBoxPlot(nValidSolutions_list, labels, "PRM Benchmarking - # of Valid Solutions", "Parameters", "# oF Valid Solutions");
    Visualizer::makeBoxPlot(pathLength_list, labels, "PRM Benchmarking - Path Length", "Parameters", "Path Length");
    Visualizer::makeBoxPlot(computationTime_list, labels, "PRM Benchmarking - Computation Time", "Parameters", "Computation Time (s)");
    Visualizer::showFigures(); */

    //Visualizer::makeFigure(problem, prm.plan(problem), *graphPtr, nodes);

    // Generate a random problem and test RRT
    /* MyRRT rrt;
     amp::Path2D path = rrt.plan(problem);
    std::cout << "Path length: " << path.length() << std::endl;
    exit(0);  */
    //HW7::generateAndCheck(rrt, path, problem);
    //Visualizer::makeFigure(problem, path, *graphPtr, nodes);
    //Visualizer::showFigures();

    // Benchmarking
    std::vector<std::pair<double, double>> r_p_pairs = {{0.5, 0.05}, {0.5, 0.15}, {1, 0.05}, {1, 0.1}, {2, 0.05}, {2, 0.1}};
    std::list<std::vector<double>> nValidSolutions_list;
    std::list<std::vector<double>> pathLength_list;
    std::list<std::vector<double>> computationTime_list;
    for (auto r_p : r_p_pairs) {
        MyRRT rrt(5000, r_p.first, r_p.second, 0.25);
        for (int i = 0; i < 100; i++) {
            amp::Path2D path = rrt.plan(problem);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        nValidSolutions_list.emplace_back(std::vector<double>{static_cast<double>(rrt.nValidSolutions)});
        pathLength_list.emplace_back(rrt.pathLength);
        computationTime_list.emplace_back(rrt.computationTime);

    }

    std::vector<std::string> labels = {"r=0.5, p_goal=0.05", "r=0.5, p_goal=0.15", "r=1, p_goal=0.05", "r=1, p_goal=0.1", "r=2, p_goal=0.05", "r=2, p_goal=0.1"};
    Visualizer::makeBoxPlot(nValidSolutions_list, labels, "RRT Benchmarking - # of Valid Solutions", "Parameters", "# oF Valid Solutions");
    Visualizer::makeBoxPlot(pathLength_list, labels, "RRT Benchmarking - Path Length", "Parameters", "Path Length");
    Visualizer::makeBoxPlot(computationTime_list, labels, "RRT Benchmarking - Computation Time", "Parameters", "Computation Time (s)");
    Visualizer::showFigures();

    // Grade method
    //HW7::grade<MyPRM, MyRRT>("yarden.kelmann@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}
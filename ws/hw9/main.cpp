// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW9.h"
#include "hw/HW2.h"
#include "MyKinoRRT.h"
#include <thread> // For std::this_thread::sleep_for
#include <chrono> // For std::chrono::milliseconds

using namespace amp;

// Load problems and map agent for quick testing
std::vector<KinodynamicProblem2D> problems = {HW9::getStateIntProblemWS1(), HW9::getStateIntProblemWS2(), HW9::getFOUniProblemWS1(), HW9::getFOUniProblemWS2(), HW9::getSOUniProblemWS1(), HW9::getSOUniProblemWS2(), HW9::getCarProblemWS1(), HW9::getParkingProblem()};
std::unordered_map<AgentType, std::function<std::shared_ptr<amp::DynamicAgent>()>> agentFactory = {
    {AgentType::SingleIntegrator, []() { return std::make_shared<MySingleIntegrator>(); }},
    {AgentType::FirstOrderUnicycle, []() { return std::make_shared<MyFirstOrderUnicycle>(); }},
    {AgentType::SecondOrderUnicycle, []() { return std::make_shared<MySecondOrderUnicycle>(); }},
    {AgentType::SimpleCar, []() { return std::make_shared<MySimpleCar>(); }}
};

int main(int argc, char** argv) {
    // Select problem, plan, check, and visualize
    int select = 7;
    KinodynamicProblem2D prob = problems[select];

    MyKinoRRT kino_planner;
    KinoPath path;// = kino_planner.plan(prob, *agentFactory[prob.agent_type]());
    //HW9::check(path, prob);
    if (path.valid){
        //Visualizer::makeFigure(prob, path, false); // Set to 'true' to render animation
    }
    //Visualizer::showFigures();
    //HW9::grade<MyKinoRRT, MySingleIntegrator, MyFirstOrderUnicycle, MySecondOrderUnicycle, MySimpleCar>("yarden.kelmann@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple());
    //exit(0);
    
    
    // Get path length
    /* double path_length = 0;
    for (size_t i = 0; i < path.waypoints.size() - 1; ++i)
        path_length += (path.waypoints[i + 1].head(2) - path.waypoints[i].head(2)).norm();
    std::cout << "Path length: " << path_length << std::endl;
    std::cout << "Computation time: " << kino_planner.computationTime[0] << " s" << std::endl;

    // write path.control to a csv file
    std::ofstream file("/home/ykelmann/AMP-Tools-public/ws/hw9/path_control.csv");
    if (file.is_open()) {
        for (size_t i = 0; i < path.waypoints.size(); ++i) {
            file << path.durations[i] << "," << path.waypoints[i][3] << "," << path.waypoints[i][4] << std::endl;
        }
        file.close();
    } else {
        std::cerr << "Unable to open file for writing" << std::endl;
    }


    exit(0); */









    // Benchmarking
    /* std::vector<std::pair<double, double>> n_u_pairs = {{50000, 1}, {50000, 5}, {50000, 10}, {50000, 15}};
    std::list<std::vector<double>> nValidSolutions_list;
    std::list<std::vector<double>> pathLength_list;
    std::list<std::vector<double>> computationTime_list;
    for (auto n_u : n_u_pairs) {
        MyKinoRRT kino_rrt(n_u.first, n_u.second);
        for (int i = 0; i < 50; i++) {
            std::cout << i << std::endl;
            path = kino_rrt.plan(prob, *agentFactory[prob.agent_type]());
        }
        nValidSolutions_list.emplace_back(std::vector<double>{static_cast<double>(kino_rrt.nValidSolutions)});
        pathLength_list.emplace_back(kino_rrt.pathLength);
        computationTime_list.emplace_back(kino_rrt.computationTime);

    }

    std::vector<std::string> labels = {"n=50000, u=1", "n=50000, u=5", "n=50000, u=10", "n=50000, u=15"};
    Visualizer::makeBoxPlot(nValidSolutions_list, labels, "Kino-RRT Benchmarking - # of Valid Solutions", "Parameters", "# oF Valid Solutions");
    Visualizer::makeBoxPlot(pathLength_list, labels, "Kino-RRT Benchmarking - Path Length", "Parameters", "Path Length");
    Visualizer::makeBoxPlot(computationTime_list, labels, "Kino-RRT Benchmarking - Computation Time", "Parameters", "Computation Time (s)");
    Visualizer::showFigures();
    return 0; */

    // Benchmarking
    std::vector<std::pair<double, double>> n_u_pairs = {{50000, 5}};
    std::list<std::vector<double>> nValidSolutions_list;
    std::list<std::vector<double>> pathLength_list;
    std::list<std::vector<double>> computationTime_list;
    for (auto n_u : n_u_pairs) {
        MyKinoRRT kino_rrt(n_u.first, n_u.second);
        for (int i = 0; i < 10; i++) {
            std::cout << i << std::endl;
            prob.u_bounds[4] = std::make_pair(-std::numbers::pi/4, std::numbers::pi/4);
            path = kino_rrt.plan(prob, *agentFactory[prob.agent_type]());
        }
        nValidSolutions_list.emplace_back(std::vector<double>{static_cast<double>(kino_rrt.nValidSolutions)});
        pathLength_list.emplace_back(kino_rrt.pathLength);
        computationTime_list.emplace_back(kino_rrt.computationTime);

    }

    std::vector<std::string> labels = {"u=5"};
    Visualizer::makeBoxPlot(nValidSolutions_list, labels, "Kino-RRT Benchmarking - # of Valid Solutions", "Parameters", "# oF Valid Solutions");
    Visualizer::makeBoxPlot(pathLength_list, labels, "Kino-RRT Benchmarking - Path Length", "Parameters", "Path Length");
    Visualizer::makeBoxPlot(computationTime_list, labels, "Kino-RRT Benchmarking - Computation Time", "Parameters", "Computation Time (s)");
    Visualizer::showFigures();
    return 0;
}
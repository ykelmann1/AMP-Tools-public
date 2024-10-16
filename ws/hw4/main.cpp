// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the headers for HW4 code
#include "CSpaceSkeleton.h"
#include "ManipulatorSkeleton.h"

// Include the header of the shared class
#include "HelpfulClass.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    ///////////////////////////
    /////// Exercise 1 ////////
    ///////////////////////////
    Obstacle2D obstacle = HW4::getEx1TriangleObstacle();

    std::vector<Eigen::Vector2d> robotVertices = {Eigen::Vector2d(0,0),Eigen::Vector2d(1,2),Eigen::Vector2d(0,2)};
    Obstacle2D cSpaceObstacle = computeCSpaceObstacle2d(robotVertices, obstacle);
    //Visualizer::makeFigure({cSpaceObstacle});

    double dtheta = std::numbers::pi*2/12;
    std::vector<double> thetas = {};
    for(int i=0;i<12;i++){
        thetas.push_back(i*dtheta);
    }

    std::vector<Obstacle2D> cSpaceObstacles = {};
    for(int i=0;i<12;i++){
        std::vector<Eigen::Vector2d> rotatedRobotVertices = robotVertices;;
        for(int j=0;j<3;j++){
            rotatedRobotVertices[j] = rotate(robotVertices[j],thetas[i]);
        }
        cSpaceObstacle = computeCSpaceObstacle2d(rotatedRobotVertices, obstacle);
        cSpaceObstacles.push_back(cSpaceObstacle);
    } 
    
    Visualizer::makeFigure(cSpaceObstacles, thetas);





    ///////////////////////////
    /////// Exercise 2 ////////
    ///////////////////////////
    
    //// Part a
    MyManipulator2D manipulator = MyManipulator2D({0.5, 1, 0.5});
    amp::ManipulatorState state(3);
    double pi = std::numbers::pi;
    state << pi/6, pi/3, 7*pi/4;

    //std::cout << manipulator.getJointLocation(state,3) << std::endl;
    //Visualizer::makeFigure(manipulator, state);

    //// Part b
    manipulator = MyManipulator2D({1, 0.5, 1});
    state = manipulator.getConfigurationFromIK(Eigen::Vector2d(2,0));

    std::cout << state << std::endl;
    Visualizer::makeFigure(manipulator, state);



    ///////////////////////////
    /////// Exercise 3 ////////
    ///////////////////////////

    manipulator = MyManipulator2D({1.0, 1.0});
    
    // Create the collision space constructor
    std::size_t n_cells = 300;
    MyManipulatorCSConstructor cspace_constructor(n_cells);

    // Create the collision space using a given manipulator and environment
    std::unique_ptr<amp::GridCSpace2D> cspace = cspace_constructor.construct(manipulator, HW4::getEx3Workspace3());

    // Visualize workspace
    //Visualizer::makeFigure(HW4::getEx3Workspace3(),manipulator,state.setZero());

    // You can visualize your cspace 
    Visualizer::makeFigure(*cspace);

    Visualizer::showFigures();

    // Grade method
    //amp::HW4::grade<MyManipulator2D>(cspace_constructor, "yarden.kelmann@colorado.edu", argc, argv);
    return 0;
}
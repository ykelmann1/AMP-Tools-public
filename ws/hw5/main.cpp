// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"
#include "hw/HW2.h"

// Include any custom headers you created in your workspace
#include "MyGDAlgorithm.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    //amp::RNG::seed(amp::RNG::randiUnbounded());

    // Test your gradient descent algorithm on a random problem.
    MyGDAlgorithm algo(0.0001,1,1,1);
    amp::Problem2D prob = HW5::getWorkspace1();
    amp::Path2D path = algo.plan(prob);
    //bool success = HW5::generateAndCheck(algo, path, prob);
    Visualizer::makeFigure(prob, path);
    std::cout << "Path length = " << path.length() << std::endl;


    // Visualize your potential function
    amp::Visualizer::makeFigure(MyPotentialFunction{}, prob, 50);
    //amp::Visualizer::makeFigure(MyPotentialFunction{}, prob.x_min, prob.x_max, prob.y_min, prob.y_max, 20);
    Visualizer::showFigures();
    
    // Arguments following argv correspond to the constructor arguments of MyGDAlgorithm:
    HW5::grade<MyGDAlgorithm>("yarden.kelmann@colorado.edu", argc, argv, 0.5,1,5,1);
    return 0;
}
#include "MyCSConstructors.h"


//////////////////////////////////////////////////////////////

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyPointAgentCSConstructor::construct(const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    size_t nX0cells = (env.x_max-env.x_min)/0.5;
    size_t nX1cells = (env.y_max-env.y_min)/0.5;
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(nX0cells, nX1cells, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    std::cout << "Constructing C-space for point agent" << std::endl;
    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
    
    for(int i = 0; i < 10*nX0cells; i++){
        for(int j = 0; j < 10*nX1cells; j++){
            double x0 = env.x_min + i*(env.x_max - env.x_min)/(10*nX0cells);
            double x1 = env.y_min + j*(env.y_max - env.y_min)/(10*nX1cells);
            
            std::pair<std::size_t, std::size_t> cell_index = cspace.getCellFromPoint(x0, x1);
            //cspace(cell_index.first, cell_index.second) = isPointInCollision(env, Eigen::Vector2d(x0, x1));
            cspace(cell_index.first, cell_index.second) = false;
            bool inCollision = isPointInCollision(env, Eigen::Vector2d(x0, x1));
            if(!inCollision && cspace(cell_index.first, cell_index.second) == false){
                cspace(cell_index.first, cell_index.second) = false;
            }else{
                cspace(cell_index.first, cell_index.second) = true;
                if(cell_index.first > 0){
                    cspace(cell_index.first-1, cell_index.second) = true;
                }
                if(cell_index.first < nX0cells-1){
                    cspace(cell_index.first+1, cell_index.second) = true;
                }
                if(cell_index.second > 0){
                    cspace(cell_index.first, cell_index.second-1) = true;
                }
                if(cell_index.second < nX1cells-1){
                    cspace(cell_index.first, cell_index.second+1) = true;
                }
                
            }
            
        }
    }

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}

amp::Path2D MyWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, bool isManipulator) {
    // Implement your WaveFront algorithm here

    Eigen::Vector2d q_init_corrected;
    Eigen::Vector2d q_goal_corrected;
    if(isManipulator){
        // Correct angles to be bwetween 0 and 2pi
        q_init_corrected = Eigen::Vector2d(correctAngle(q_init[0]), correctAngle(q_init[1]));
        q_goal_corrected = Eigen::Vector2d(correctAngle(q_goal[0]), correctAngle(q_goal[1]));
    }else{
        q_init_corrected = q_init;
        q_goal_corrected = q_goal;
    }
    
    
    amp::Path2D path;
    size_t nX0cells = grid_cspace.size().first;
    size_t nX1cells = grid_cspace.size().second;
    /* amp::Visualizer::makeFigure(grid_cspace);
    amp::Visualizer::showFigures(); */

    Eigen::MatrixXi grid = Eigen::MatrixXi::Zero(nX0cells, nX1cells);
    //Loop through indices of grid_cspace and set grid values to 1 if in collision
    for(int i = 0; i < nX0cells; i++){
        for(int j = 0; j < nX1cells; j++){
            if(grid_cspace(i,j)){
                grid(i,j) = 1;
            }
        }
    }
    std::pair<std::size_t, std::size_t> q_goal_cell = grid_cspace.getCellFromPoint(q_goal_corrected[0], q_goal_corrected[1]);
    //set grid value at goal cell to 2
    grid(q_goal_cell.first, q_goal_cell.second) = 2;

    // Give a value of 0 to every other cell in the grid
    for(int i = 0; i < nX0cells; i++){
        for(int j = 0; j < nX1cells; j++){
            if(grid(i,j) != 1 && grid(i,j) != 2){
                grid(i,j) = 0;
            }
        }
    }

    // Give a value of 3 to all the 0-valued cells that share a facet with the 2-valued cells; etc.
    int valToAdd = 3;
    //Move q_init_corrected to the center of the cell
    std::pair<std::size_t, std::size_t> q_init_cell = grid_cspace.getCellFromPoint(q_init_corrected[0], q_init_corrected[1]);
    
    
    Eigen::Vector2d q_init_bottomLeft = Eigen::Vector2d(grid_cspace.x0Bounds().first + q_init_cell.first*(grid_cspace.x0Bounds().second - grid_cspace.x0Bounds().first)/nX0cells, grid_cspace.x1Bounds().first + q_init_cell.second*(grid_cspace.x1Bounds().second - grid_cspace.x1Bounds().first)/nX1cells);
    Eigen::Vector2d q_init_corrected_center = Eigen::Vector2d(q_init_bottomLeft[0]+(grid_cspace.x0Bounds().second - grid_cspace.x0Bounds().first)/(2*nX0cells), q_init_bottomLeft[1] + (grid_cspace.x1Bounds().second - grid_cspace.x1Bounds().first)/(2*nX1cells));
    q_init_cell = grid_cspace.getCellFromPoint(q_init_corrected_center[0], q_init_corrected_center[1]);
    std::size_t q_init_x = q_init_cell.first;
    std::size_t q_init_y = q_init_cell.second;
    bool exitLoops = false;
    int iterationCount = 0;
    while(true){
        //If isManipulator is true, change the function so that the search wraps around the bounds of the grid

        for(int i = 0; i < nX0cells; i++){
            for(int j = 0; j < nX1cells; j++){
                if(grid(i,j) == valToAdd-1){
                    //Check if reached q_init_corrected
                    if ((q_init_x == i - 1 && q_init_y == j) ||
                    (q_init_x == i + 1 && q_init_y == j) ||
                    (q_init_x == i && q_init_y == j - 1) || 
                    (q_init_x == i && q_init_y == j + 1)) {
                        grid(q_init_x, q_init_y) = valToAdd;
                        exitLoops = true;
                        break;
                    }

                    if(i > 0 && grid(i-1,j) == 0){
                        grid(i-1,j) = valToAdd;
                    } else if (isManipulator && i == 0 && grid(nX0cells-1,j) == 0) {
                        grid(nX0cells-1,j) = valToAdd;
                    }
                    if(i < nX0cells-1 && grid(i+1,j) == 0){
                        grid(i+1,j) = valToAdd;
                    } else if (isManipulator && i == nX0cells-1 && grid(0,j) == 0) {
                        grid(0,j) = valToAdd;
                    }
                    if(j > 0 && grid(i,j-1) == 0){
                        grid(i,j-1) = valToAdd;
                    } else if (isManipulator && j == 0 && grid(i,nX1cells-1) == 0) {
                        grid(i,nX1cells-1) = valToAdd;
                    }
                    if(j < nX1cells-1 && grid(i,j+1) == 0){
                        grid(i,j+1) = valToAdd;
                    } else if (isManipulator && j == nX1cells-1 && grid(i,0) == 0) {
                        grid(i,0) = valToAdd;
                    }
                }
            }
            if (exitLoops) break;
        }
        if (exitLoops) break;
        valToAdd++;
        iterationCount++;
        if (iterationCount >= 100000) {
            std::cout << "Cannot find path" << std::endl;
            path.waypoints.push_back(q_init);
            path.waypoints.push_back(q_goal);
/*             amp::Visualizer::makeFigure(grid_cspace, path); // Visualize path in cspace
            amp::Visualizer::showFigures(); */
            return path;
        }
    }   

    path.waypoints.push_back(q_init_corrected);
    path.waypoints.push_back(q_init_corrected_center);
    Eigen::Vector2d q_curr = q_init_corrected_center;

    //push the waypoints in the path from q_init_cell to q_goal_cell following the gradient
    int iterationCount2 = 0;
    while (true) {


        std::pair<std::size_t, std::size_t> q_curr_cell = grid_cspace.getCellFromPoint(q_curr[0], q_curr[1]);
        std::size_t q_curr_x = q_curr_cell.first;
        std::size_t q_curr_y = q_curr_cell.second;
        if (q_curr_cell == q_goal_cell){
            std::cout << "Reached goal" << std::endl;
            break;
        }
        if (q_curr_x > 0 && grid(q_curr_x - 1, q_curr_y) == grid(q_curr_x, q_curr_y) - 1) {
            q_curr[0] -= (grid_cspace.x0Bounds().second - grid_cspace.x0Bounds().first) / nX0cells;
        } else if (isManipulator && q_curr_x == 0 && grid(nX0cells - 1, q_curr_y) == grid(q_curr_x, q_curr_y) - 1) {
            q_curr[0] = grid_cspace.x0Bounds().second - (grid_cspace.x0Bounds().second - grid_cspace.x0Bounds().first) /(2*nX0cells);
        } else if (q_curr_x < nX0cells - 1 && grid(q_curr_x + 1, q_curr_y) == grid(q_curr_x, q_curr_y) - 1) {
            q_curr[0] += (grid_cspace.x0Bounds().second - grid_cspace.x0Bounds().first) / nX0cells;
        } else if (isManipulator && q_curr_x == nX0cells - 1 && grid(0, q_curr_y) == grid(q_curr_x, q_curr_y) - 1) {
            q_curr[0] = grid_cspace.x0Bounds().first + (grid_cspace.x0Bounds().second - grid_cspace.x0Bounds().first) /(2*nX0cells);
        } else if (q_curr_y > 0 && grid(q_curr_x, q_curr_y - 1) == grid(q_curr_x, q_curr_y) - 1) {
            q_curr[1] -= (grid_cspace.x1Bounds().second - grid_cspace.x1Bounds().first) / nX1cells;
        } else if (isManipulator && q_curr_y == 0 && grid(q_curr_x, nX1cells - 1) == grid(q_curr_x, q_curr_y) - 1) {
            q_curr[1] = grid_cspace.x1Bounds().second - (grid_cspace.x1Bounds().second - grid_cspace.x1Bounds().first) /(2*nX1cells);
        } else if (q_curr_y < nX1cells - 1 && grid(q_curr_x, q_curr_y + 1) == grid(q_curr_x, q_curr_y) - 1) {
            q_curr[1] += (grid_cspace.x1Bounds().second - grid_cspace.x1Bounds().first) / nX1cells;
        } else if (isManipulator && q_curr_y == nX1cells - 1 && grid(q_curr_x, 0) == grid(q_curr_x, q_curr_y) - 1) {
            q_curr[1] = grid_cspace.x1Bounds().first + (grid_cspace.x1Bounds().second - grid_cspace.x1Bounds().first) /(2*nX1cells);
        }
        path.waypoints.push_back(q_curr);
    }
    
    path.waypoints.pop_back();
    path.waypoints.push_back(q_goal_corrected);
    if (isManipulator) {
        Eigen::Vector2d bounds0 = Eigen::Vector2d(0.0, 0.0);
        Eigen::Vector2d bounds1 = Eigen::Vector2d(2*M_PI, 2*M_PI);
        amp::unwrapWaypoints(path.waypoints, bounds0, bounds1);
    }
    if(!isManipulator){
        /* std::cout << "q_initial =" << q_init_corrected[0] << " " << q_init_corrected[1] << std::endl;
        for(int i=0; i< path.waypoints.size(); i++){
            std::cout << path.waypoints[i][0] << " " << path.waypoints[i][1] << std::endl;
        } */
        /* amp::Visualizer::makeFigure(grid_cspace, path); // Visualize path in cspace
        amp::Visualizer::showFigures(); */
        
        
    }
    
    return path;
}

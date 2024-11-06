#include "CSpaceSkeleton.h"

// Override this method for returning whether or not a point is in collision

std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    // Access the size of the cell in each dimension for the env.x_min, env.x_max, etc
    double sizePerCellDim0 = (this->m_x0_max - this->m_x0_min) / this->m_x0_cells;
    double sizePerCellDim1 = (this->m_x1_max - this->m_x1_min) / this->m_x1_cells;

    std::size_t cell_x = std::floor((x0 - this->m_x0_min)  / sizePerCellDim0); // x index of cell
    std::size_t cell_y = std::floor((x1 - this->m_x1_min) / sizePerCellDim1); // y index of cell
    return {cell_x, cell_y};
}

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, 0, 2*std::numbers::pi, 0, 2*std::numbers::pi);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;

    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
    double theta1 = 0;
    double theta2 = 0;
    double dtheta = 2*std::numbers::pi/400;

    while(theta1 < 2*std::numbers::pi){
        theta2 = 0;
        while(theta2 < 2*std::numbers::pi){
            amp::ManipulatorState state(2);
            state << theta1, theta2;

            Eigen::Vector2d joint_1_pos = manipulator.getJointLocation(state, 0);
            Eigen::Vector2d joint_2_pos = manipulator.getJointLocation(state, 1);
            Eigen::Vector2d joint_3_pos = manipulator.getJointLocation(state, 2);
            bool inCollisionLink1 = collisionChecker(env.obstacles, joint_1_pos, joint_2_pos);
            bool inCollisionLink2 = collisionChecker(env.obstacles, joint_2_pos, joint_3_pos);

            if(inCollisionLink1 || inCollisionLink2){
                std::pair<std::size_t, std::size_t> cell_index = cspace.getCellFromPoint(theta1,theta2);
                cspace(cell_index.first,cell_index.second) = true;
            }


            theta2 += dtheta;
        }
        theta1 += dtheta;
    }

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}

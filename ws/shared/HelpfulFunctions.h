#include "AMPCore.h"
#include <vector>
#include <algorithm>
#include <limits>

double norm2(Eigen::Vector2d vec2d);
double sumElements(std::vector<double> vec);

//std::pair<bool, std::vector<int>> collisionChecker(std::vector<amp::Obstacle2D> obstacles, Eigen::Vector2d curr_pos, Eigen::Vector2d next_pos);
bool onSegment(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r);
int orientation(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r);
bool collisionChecker(std::vector<amp::Obstacle2D> obstacles, Eigen::Vector2d curr_pos, Eigen::Vector2d next_pos);

bool collisionCheckerPrimitive(Eigen::Vector2d q_init, Eigen::Vector2d q_goal, Eigen::Vector2d curr_pos, Eigen::Vector2d next_pos);

Eigen::Vector2d getBoundaryFollowingStep(std::vector<amp::Obstacle2D> obstacles, Eigen::Vector2d curr_pos, Eigen::Vector2d curr_step, double dTheta, std::string robotTurningDirection);

Eigen::Vector2d rotate(Eigen::Vector2d vec, double theta);

bool crossMLine(Eigen::Vector2d q_init, Eigen::Vector2d q_goal, Eigen::Vector2d curr_pos, Eigen::Vector2d next_pos);

// Cspace obstacle helpers
double angleBetweenVertices(Eigen::Vector2d v1, Eigen::Vector2d v2);
int getIndexBottomLeft(std::vector<Eigen::Vector2d> vertices);
std::vector<Eigen::Vector2d> rearrangeVertices(std::vector<Eigen::Vector2d> vertices);
amp::Obstacle2D computeCSpaceObstacle2d(std::vector<Eigen::Vector2d> R, amp::Obstacle2D obs);

// Compute gradient for gradient descent
std::pair<Eigen::Vector2d, double> getClosestPointOnObstacle(amp::Obstacle2D obs, Eigen::Vector2d q);
Eigen::Vector2d getGradient(Eigen::Vector2d q, const amp::Problem2D& problem, double d_star, double zetta, double Q_star, double eta);



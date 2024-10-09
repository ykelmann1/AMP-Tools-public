#include "HelpfulFunctions.h"

double norm2(Eigen::Vector2d vec2d){
    return std::sqrt(std::pow(vec2d[0], 2) + std::pow(vec2d[1], 2));
}

double sumElements(std::vector<double> vec){
    double sum = 0;
    for(int i=0;i<vec.size();i++){
        sum += vec[i];
    }
    return sum;
}

bool onSegment(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r) 
{ 
    if (q[0] <= std::max(p[0], r[0]) && q[0] >= std::min(p[0], r[0]) && 
        q[1] <= std::max(p[1], r[1]) && q[1] >= std::min(p[1], r[1])) 
       return true; 
  
    return false; 
} 
  
// To find orientation of ordered triplet (p, q, r). 
// The function returns following values 
// 0 --> p, q and r are collinear 
// 1 --> Clockwise 
// 2 --> Counterclockwise 
int orientation(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r) 
{ 
    // See https://www.geeksforgeeks.org/orientation-3-ordered-points/ 
    // for details of below formula. 
    double val = (q[1] - p[1]) * (r[0] - q[0]) - 
              (q[0] - p[0]) * (r[1] - q[1]); 
  
    if (val == 0) return 0;  // collinear 
  
    return (val > 0)? 1: 2; // clock or counterclock wise 
} 
  

bool collisionChecker(std::vector<amp::Obstacle2D> obstacles, Eigen::Vector2d curr_pos, Eigen::Vector2d next_pos){
    for(amp::Obstacle2D obs : obstacles){
        std::vector<Eigen::Vector2d> vtxs = obs.verticesCW();
        for(int i = 0; i < vtxs.size(); i++){
            int j = (i == vtxs.size()-1) ? 0 : i+1;
            Eigen::Vector2d p1 = vtxs[i];
            Eigen::Vector2d q1 = vtxs[j];
            Eigen::Vector2d p2 = curr_pos;
            Eigen::Vector2d q2 = next_pos;

            // Find the four orientations needed for general and 
            // special cases 
            int o1 = orientation(p1, q1, p2); 
            int o2 = orientation(p1, q1, q2); 
            int o3 = orientation(p2, q2, p1); 
            int o4 = orientation(p2, q2, q1); 
        
            // General case 
            if (o1 != o2 && o3 != o4){
                return true;
            } 
                 
        
            // Special Cases 
            // p1, q1 and p2 are collinear and p2 lies on segment p1q1 
            if (o1 == 0 && onSegment(p1, p2, q1)){ return true;} 
        
            // p1, q1 and q2 are collinear and q2 lies on segment p1q1 
            if (o2 == 0 && onSegment(p1, q2, q1)){ return true;}
        
            // p2, q2 and p1 are collinear and p1 lies on segment p2q2 
            if (o3 == 0 && onSegment(p2, p1, q2)){  return true;} 
        
            // p2, q2 and q1 are collinear and q1 lies on segment p2q2 
            if (o4 == 0 && onSegment(p2, q1, q2)){  return true;} 



        }
    }

    return false;
}

bool collisionCheckerPrimitive(Eigen::Vector2d q_init, Eigen::Vector2d q_goal, Eigen::Vector2d curr_pos, Eigen::Vector2d next_pos){
    double f1;
    double f2;

    if(q_init[0] == q_goal[0]){
        f1 = curr_pos[0] - q_init[0];
        f2 = next_pos[0] - q_init[0];

    }
    else{
        double m = (q_goal[1]-q_init[1])/(q_goal[0]-q_init[0]);

        f1 = curr_pos[1] - m*(curr_pos[0]-q_init[0]) + q_init[1];
        f2 = next_pos[1] - m*(next_pos[0]-q_init[0]) + q_init[1];
    }

    if((f1<0 && f2>0) || (f1>0 && f2<0) || f1 == 0 || f2 == 0) return true;

    return false;
}

Eigen::Vector2d rotate(Eigen::Vector2d vec, double theta){
    double R11 = std::cos(theta);
    double R12 = -std::sin(theta);
    double R21 = std::sin(theta);
    double R22 = std::cos(theta);

    Eigen::Vector2d vecp = Eigen::Vector2d(R11*vec[0] + R12*vec[1], R21*vec[0] + R22*vec[1]);
    return vecp;
}



Eigen::Vector2d getBoundaryFollowingStep(std::vector<amp::Obstacle2D> obstacles, Eigen::Vector2d curr_pos, Eigen::Vector2d curr_step,  double dTheta, std::string robotTurningDirection){
    Eigen::Vector2d sight_pt = curr_pos + curr_step;
    Eigen::Vector2d touch_pt;
    if(robotTurningDirection == "left"){
        touch_pt = curr_pos + rotate(curr_step, -std::numbers::pi/4);

        // Case where sight pt in obstacle and touch pt outside obstacle, or sight pt and touch pt in obstacle
        while((collisionChecker(obstacles, curr_pos, sight_pt) && !collisionChecker(obstacles, curr_pos, touch_pt)) || (collisionChecker(obstacles, curr_pos, sight_pt) && collisionChecker(obstacles, curr_pos, touch_pt))){
            sight_pt = rotate(sight_pt - curr_pos, dTheta) + curr_pos;
            touch_pt = rotate(touch_pt - curr_pos, dTheta) + curr_pos;
            
        }
        
        while((!collisionChecker(obstacles, curr_pos, sight_pt) && !collisionChecker(obstacles, curr_pos, touch_pt))){
            //std::cout << "CASE 2" << std::endl;
            sight_pt = rotate(sight_pt - curr_pos, -dTheta) + curr_pos;
            touch_pt = rotate(touch_pt - curr_pos, -dTheta) + curr_pos;
            

        }
        

    }
    else{
        touch_pt = curr_pos + rotate(curr_step, std::numbers::pi/4);

        // Case where sight pt in obstacle and touch pt outside obstacle, or sight pt and touch pt in obstacle
        while((collisionChecker(obstacles, curr_pos, sight_pt) && !collisionChecker(obstacles, curr_pos, touch_pt)) || (collisionChecker(obstacles, curr_pos, sight_pt) && collisionChecker(obstacles, curr_pos, touch_pt))){
            sight_pt = rotate(sight_pt - curr_pos, -dTheta) + curr_pos;
            touch_pt = rotate(touch_pt - curr_pos, -dTheta) + curr_pos;
            
        }
        
        while((!collisionChecker(obstacles, curr_pos, sight_pt) && !collisionChecker(obstacles, curr_pos, touch_pt))){
            sight_pt = rotate(sight_pt - curr_pos, dTheta) + curr_pos;
            touch_pt = rotate(touch_pt - curr_pos, dTheta) + curr_pos;
            
        }
    }

    if(!collisionChecker(obstacles, curr_pos, sight_pt) && collisionChecker(obstacles, curr_pos, touch_pt)){
        return sight_pt - curr_pos;
    }

    
    

}





bool crossMLine(Eigen::Vector2d q_init, Eigen::Vector2d q_goal, Eigen::Vector2d curr_pos, Eigen::Vector2d next_pos){
    return collisionCheckerPrimitive(q_init, q_goal, curr_pos, next_pos);
}




double angleBetweenVertices(Eigen::Vector2d v1, Eigen::Vector2d v2){
    double angle = std::atan2(v2[1]-v1[1],v2[0]-v1[0]);
    if(angle < -0.0000000001){
        angle += 2*std::numbers::pi;
    }

    return angle;
}

int getIndexBottomLeft(std::vector<Eigen::Vector2d> vertices){
    double minx = vertices[0][0];
    double miny = vertices[0][1];
    int minIdx = 0;

    for(int i=1;i<vertices.size();i++){
        if(vertices[i][1]-miny<-0.000000000001){
            miny = vertices[i][1];
            minx = vertices[i][0];
            minIdx = i;
            
        }
        else if(std::abs(vertices[i][1]-miny)<0.000000000001){
            if(vertices[i][0]<minx){
                minx = vertices[i][0];
                minIdx = i;
            }
        }
    }
    return minIdx;
}

std::vector<Eigen::Vector2d> rearrangeVertices(std::vector<Eigen::Vector2d> vertices){
    int minIdx = getIndexBottomLeft(vertices);
    std::vector<int> idxs = {};
    for(int i=0;i<vertices.size();i++){
        idxs.push_back(i);
    }

    std::rotate(idxs.begin(), idxs.begin()+minIdx, idxs.end());

    std::vector<Eigen::Vector2d> rearrangedVertices = {};
    for(int j=0;j<vertices.size();j++){
        rearrangedVertices.push_back(vertices[idxs[j]]);
    }
    return rearrangedVertices;
}


amp::Obstacle2D computeCSpaceObstacle2d(std::vector<Eigen::Vector2d> R, amp::Obstacle2D obs){
    // Robot vertices are ccw and must begin with the reference point 
    // Obstacle vertices are ccw and must begin with A vertex with the smallest y-coordinate
    amp::Obstacle2D cSpaceObstacle = {};

    // Place the reference point at the bottom-left vertex of the obstacle and take the negative of the robot
    std::vector<Eigen::Vector2d> negR = R;
    for(int i=0;i<R.size();i++){
        negR[i] += -R[0];
        negR[i] *= -1;
    }

    int j = 0;
    int k = 0;

    // Rearrange the negative robot vertices so that the first in the vector is the bottom left vertex
    negR = rearrangeVertices(negR); 
    negR.push_back(negR[0]);

    std::vector<Eigen::Vector2d> obsVertices = obs.verticesCCW();
    obsVertices.push_back(obsVertices[0]);        

    // Calculate the Cspace obstacle vertices using the angle comparison algorithm
    std::vector<Eigen::Vector2d> cSpaceObsVertices = {};
    int verticesAdded = 0;
    while(verticesAdded < negR.size()+obsVertices.size()-2){
        cSpaceObsVertices.push_back(negR[j]+obsVertices[k]);
        verticesAdded++;

        if(angleBetweenVertices(negR[j],negR[j+1]) < angleBetweenVertices(obsVertices[k],obsVertices[k+1])){
            j++;
        }
        else if(angleBetweenVertices(negR[j],negR[j+1]) > angleBetweenVertices(obsVertices[k],obsVertices[k+1])){
            k++;
        }
        else{
            j++;
            k++;
        }
    }

    return amp::Polygon(cSpaceObsVertices);
}









std::pair<Eigen::Vector2d, double> getClosestPointOnObstacle(amp::Obstacle2D obs, Eigen::Vector2d q){
    std::vector<Eigen::Vector2d> vertices = obs.verticesCCW();

    Eigen::Vector2d closestPt;
    double minDistance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < vertices.size(); ++i) {
        Eigen::Vector2d A = vertices[i];
        Eigen::Vector2d B = vertices[(i + 1) % vertices.size()]; // Wrap around for the last vertex

        Eigen::Vector2d AB = B - A;
    
        // Vector AQ
        Eigen::Vector2d AQ = q - A;

        // Project AQ onto AB, computing the parameter t that minimizes the distance
        double AB_AB = AB[0] * AB[0] + AB[1] * AB[1]; // AB • AB
        double AQ_AB = AQ[0] * AB[0] + AQ[1] * AB[1]; // AQ • AB
        double t = AQ_AB / AB_AB;

        // Clamp t to the [0, 1] range to ensure the point is on the segment
        t = std::max(0.0, std::min(1.0, t));

        // Compute the closest point using t
        Eigen::Vector2d candidate = {A[0] + t * AB[0], A[1] + t * AB[1]};

        // Calculate the distance to the candidate point
        double dist = norm2(candidate-q);

        // Update if we found a closer point
        if (dist < minDistance) {
            minDistance = dist;
            closestPt = candidate;
        }
    }

    return {closestPt, minDistance};

}









Eigen::Vector2d getGradient(Eigen::Vector2d q, amp::Problem2D& problem, double d_star, double zetta, double Q_star, double eta){
    Eigen::Vector2d dUatt = {0,0};
    Eigen::Vector2d dUrep = {0,0};

    if(norm2(q - problem.q_goal) <= d_star){
        dUatt = zetta*(q - problem.q_goal);
    }
    else{
        dUatt = d_star*zetta*(q - problem.q_goal)/norm2(q - problem.q_goal);
    }
   // std::cout << "dUatt = <" << dUatt[0] << ", " << dUatt[1] << ">" << std::endl;

     for(amp::Obstacle2D obs : problem.obstacles){
        std::pair<Eigen::Vector2d, double> result = getClosestPointOnObstacle(obs, q);
        Eigen::Vector2d c = result.first;
        double d = result.second;
        //std::cout << "q = <" << q[0] << ", " << q[1] << ">" << std::endl;
        /* std::cout << "c = <" << c[0] << ", " << c[1] << ">" << std::endl;
        std::cout << "d = " << d << std::endl; */

        if(d <= Q_star){
            dUrep += eta*((1/Q_star) - (1/d))*(q-c)/std::pow(d,3);
            //std::cout << "dUrep = <" << dUrep[0] << ", " << dUrep[1] << ">" << std::endl; 
        }
     }

     return dUatt + dUrep;


}



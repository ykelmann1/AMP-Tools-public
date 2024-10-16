#include "MyAStar.h"
#include <queue>
#include <vector>
#include <tuple> // For std::tuple
#include <functional> // For std::greater
#include <algorithm> // For std::find

/* class myNode{
    public:
        myNode(amp::Node node, bool visited = false, double total_path_length) : node(node), visited(visited), total_path_length(total_path_length) {}
        amp::Node node;
        bool visited;
        double total_path_length;
} */

// Implement the search method for the A* algorithm
MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
    std::cout << "Starting A* Graph Search: Init --> goal | " << problem.init_node << " --> " << problem.goal_node << std::endl;
    GraphSearchResult result = {false, {}, 0.0}; // initialize the results object

//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
    /* amp::Node what = 4;
    const std::vector<amp::Node>& children = problem.graph->children(what);

    // Print the children
    NEW_LINE;
    INFO("Children of node " << what << ":");
    for (amp::Node n : children)
        INFO(" - " << n);

    // Look at the outgoing edges of node `1`
    const auto& outgoing_edges = problem.graph->outgoingEdges(what);

    // Print the outgoing edges (notice they are always in the same order as the children nodes)
    NEW_LINE;
    INFO("Outgoing edges of node " << what << ":");
    for (const auto& edge : outgoing_edges)
        INFO(" - " << edge);
    exit(0); */

//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////

    // Create queue for open nodes. Each element contains the node, priority value, length of path from start to node, and parent node
    std::vector<std::tuple<amp::Node, double, double, amp::Node>> open_list = {};
    // Create a list of closed nodes. Each element contains the node and the total path length from the start node
    std::vector<std::tuple<amp::Node, double, double, amp::Node>> closed_list = {};
    //create myNode instance

    open_list.push_back(std::make_tuple(problem.init_node, heuristic(problem.init_node), 0.0, problem.init_node));

    int nIterations = 0;
    while(!open_list.empty()){
        // Sort open_list by the second element of the tuple in ascending order
        std::sort(open_list.begin(), open_list.end(), [](const std::tuple<amp::Node, double, double, amp::Node>& a, const std::tuple<amp::Node, double, double, amp::Node>& b) {
            return std::get<1>(a) < std::get<1>(b);
        });

        /* std::cout << "New iteration after sorting:" << std::endl;
        for(auto element : open_list){
            std::cout << "Node: " << std::get<0>(element) << " | " << "utility: " << std::get<1>(element) << " | " << "total_path_length: " << std::get<2>(element) << " | " << "parent: " << std::get<3>(element) << std::endl << std::endl;
        } */

        amp::Node n_best = std::get<0>(open_list.front());
        closed_list.push_back(std::make_tuple(n_best,std::get<1>(open_list.front()), std::get<2>(open_list.front()), std::get<3>(open_list.front())));
        open_list.erase(open_list.begin());
        

        if(n_best == problem.goal_node){
            result.success = true;
            break;
        }

        std::vector<amp::Node> children = problem.graph->children(n_best);

        int childIndex = 0;
        for(amp::Node n : children){
            //check if node is in closed list
            auto it_closed = std::find_if(closed_list.begin(), closed_list.end(), [n](std::tuple<amp::Node, double, double, amp::Node> element) {
                return std::get<0>(element) == n;
            });
            if(it_closed != closed_list.end()){
                childIndex++;
                continue;
            }

            auto it = std::find_if(open_list.begin(), open_list.end(), [n](std::tuple<amp::Node, double, double, amp::Node> element) {
                return std::get<0>(element) == n;
            });
            
            auto parent = std::find_if(closed_list.begin(), closed_list.end(), [n_best](std::tuple<amp::Node, double, double, amp::Node> element) {
                    return std::get<0>(element) == n_best;
            });
            
            if(it == open_list.end()){
                //get the total_path_length value of the parent node
                double total_path_length = std::get<2>(*parent) + problem.graph->outgoingEdges(n_best)[childIndex];
                double h = heuristic(n);
                // For Dijkstra's Algorithm, h = 0
                //double h = 0;
                double utility = total_path_length + h;
                open_list.push_back(std::make_tuple(n, utility, total_path_length, n_best));
                
            }
            else if(std::get<2>(*it) > std::get<2>(*parent) + problem.graph->outgoingEdges(n_best)[childIndex]){
                //change value of utility, path length, and parent node
                std::get<1>(*it) = std::get<2>(*parent) + problem.graph->outgoingEdges(n_best)[childIndex] + heuristic(n);
                std::get<2>(*it) = std::get<2>(*parent) + problem.graph->outgoingEdges(n_best)[childIndex];
                std::get<3>(*it) = n_best;
                
            }

            
            childIndex++;
        }
        nIterations++;
    }











    // Starting from the goal node, follow the parent nodes to the start node
    amp::Node node = problem.goal_node;
    //set path cost to total_path_length of goal node
    result.path_cost = std::get<2>(*std::find_if(closed_list.begin(), closed_list.end(), [&node](std::tuple<amp::Node, double, double, amp::Node>& element) {
        return std::get<0>(element) == node;
    }));

    // Backtrack and add the nodes to the path
    while(node != problem.init_node){
        result.node_path.push_back(node);
        node = std::get<3>(*std::find_if(closed_list.begin(), closed_list.end(), [&node](std::tuple<amp::Node, double, double, amp::Node>& element) {
            return std::get<0>(element) == node;
        }));
    }
    result.node_path.push_back(problem.init_node);
    std::reverse(result.node_path.begin(), result.node_path.end());

    //std::cout << "A* Graph Search Complete: " << nIterations << " iterations | Path Length: " << result.path_cost << std::endl;
    result.print();
    return result;
}

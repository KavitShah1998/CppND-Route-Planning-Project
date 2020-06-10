#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);

    start_node->g_value = 0;
    start_node->h_value = CalculateHValue(start_node);
    start_node->visited = true;// i totally missed this one :<)
    
}


// TODO 3

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// TODO 4

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    for(RouteModel::Node* each_neighbor: current_node->neighbors){

        if(each_neighbor->visited!=true){
            each_neighbor->parent  = current_node;
            each_neighbor->g_value = current_node->g_value + each_neighbor->distance(*current_node);
            each_neighbor->h_value = CalculateHValue(each_neighbor);
            each_neighbor->visited = true;
            open_list.emplace_back(each_neighbor);
        }
        else{ // node already added to the open list
            if(each_neighbor->closed!=true)// node is in the open list but not yet explored
            {  

                float new_g_value = current_node->g_value + each_neighbor->distance(*current_node);
                float new_h_value = CalculateHValue(each_neighbor);

                if((new_g_value + new_h_value) < (each_neighbor->g_value + each_neighbor->h_value)) // a better parent available which reduces the f value of a node in the open list.
                {  
                    each_neighbor->g_value = new_g_value;
                    each_neighbor->h_value = new_h_value;
                    each_neighbor->parent = current_node;
                }
                //else // the current parent to the node is the best parent for the node.
                
                
            }
            //else // the neighbor has already been visited & EXPLORED!
        }
    }
}


// TODO 5

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), [](RouteModel::Node* const &nodeA, RouteModel::Node* const &nodeB){return (nodeA->g_value + nodeA->h_value) > (nodeB->g_value + nodeB->h_value); });

    RouteModel::Node* best_node = open_list.back();
    best_node->closed = true;
    open_list.pop_back();

    return best_node;
}


// TODO 6

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {

    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO

    while (current_node != start_node){
        distance += current_node->distance(*(current_node->parent));
        path_found.emplace_back(*current_node);
        current_node = current_node->parent;
    }
    // current_node = start_node;
    path_found.emplace_back(*current_node);


    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    std::cout<<"\n End of A*\n";
    return path_found;

}


// TODO 7

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO

    open_list.emplace_back(start_node);

    while(!open_list.empty()){
        current_node = NextNode();
        if(current_node->distance(*end_node) == 0)
        {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        
        AddNeighbors(current_node);
    }

    std::cout<<"\n No path Found. ";
    std::cout<<"\n End of A*\n";
 

}
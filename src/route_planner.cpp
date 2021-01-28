#include "route_planner.h"
#include <algorithm>

//EXERCISE 6-4
// RoutePlanner constructor
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y) : m_Model(model){ // initialize m_Model using input model
    // scale the input floats to percentages
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    // find the closest nodes to start and end using m_Model.FindClosestNode()
    // store the node into RoutePlanner's private attributes start_node and end_node
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}



// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

// EXERCISE 6-7
float RoutePlanner::CalculateHValue(const RouteModel::Node *node){
    // return the Euclidean distance from current node to end_node
    return node->distance(*(this->end_node));
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

// EXERCISE 6-9
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node){
    // populate the neighbors vector of the current_node
    current_node->FindNeighbors();
    // for each neighbor in current_node's neighbors
    for(auto neighbor : current_node->neighbors){
        // set the neighbor's parent to current_node
        neighbor->parent = current_node;
        // set the neighbor's g_value
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        // set the neighbor's h_value
        neighbor->h_value = CalculateHValue(neighbor);
        // push the neighbor to the back of the open_list
        open_list.push_back(neighbor);
        // mark the neighbor as visited
        neighbor->visited = true;
    }

}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

// EXERCISE 6-8
// helper function for sorting
// return true if f value of node1 > f value of node2
bool Compare(RouteModel::Node* node1, RouteModel::Node* node2){
    if((node1->h_value + node1->g_value) > (node2->h_value + node2->g_value)) return true;
    return false;
}

RouteModel::Node* RoutePlanner::NextNode(){
   // - Sort the open_list in descending according to the sum of the h value and g value.
    sort(open_list.begin(), open_list.end(), Compare);
// - Create a pointer to the node in the list with the lowest sum.
    RouteModel::Node* lowest_node = open_list.back();
// - Remove that node from the open_list.
    open_list.pop_back();
// - Return the pointer.
    return lowest_node;
}



// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

// EXERCISE 6-5
vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node){
    // initialize path_found vector and set distance as 0
    vector<RouteModel::Node> path_found;
    this->distance = 0.0f;
    // iterate and store node parents until starting node whose parent is nullptr
    while(current_node->parent != nullptr){
        // store current node
        path_found.push_back(*current_node);
        // add distance between current node and its parent
        this->distance += current_node->distance(*(current_node->parent));
        // move to the parent node
        current_node = current_node->parent;
    }
    // add the starting node
    path_found.push_back(*current_node);
    // reverse the order of path_found
    reverse(path_found.begin(), path_found.end());
    // scale the distance
    this->distance *= this->m_Model.MetricScale();
    return path_found;
}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

// EXERCISE 6-6
// void RoutePlanner::AStarSearch(){
//     // Set the parent of end_node to the start_node.
//     this->end_node->parent = this->start_node;
//     // Set m_Model.path to the result of calling ConstructFinalPath on end_node.
//     this->m_Model.path = ConstructFinalPath(this->end_node);
//     return;
// }

// EXERCISE 6-10
void RoutePlanner::AStarSearch(){
    // set start_node->visited to be true
    start_node->visited = true;
    // push start_node to open_list
    open_list.push_back(start_node);
    // create a pointer to the current node
    RouteModel::Node *current_node = nullptr;
    // A* search
    while(!open_list.empty()){
        // set the current_node to the results of NextNode(), 
        // which is node in open_list with lowest f-value
        current_node = NextNode();
        // if current node is end node
        if(current_node->distance(*end_node) == 0){
            // constructFinalPath and store it
            m_Model.path = ConstructFinalPath(end_node);
            // exit search
            return;
        }
        // else, add neighbors with the current_node
        else{
            AddNeighbors(current_node);
        }
    }
}
#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"

using namespace std;


class RoutePlanner {
  public:
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);
    // Add public variables or methods declarations here.
    // EXERCISE 6-3
    // getter function should be constant function
    float GetDistance() const {return distance;};
     // EXERCISE 6-5
    vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *);
    // EXERCISE 6-6
    void AStarSearch();
    

    // The following methods have been made public so we can test them individually.
    // EXERCISE 6-7
    float CalculateHValue(const RouteModel::Node*);
    // EXERCISE 6-8
    RouteModel::Node* NextNode();
    // EXERCISE 6-9
    void AddNeighbors(RouteModel::Node *current_node);

  private:
    // Add private variables or methods declarations here.
    RouteModel &m_Model; // refern to the model that A* search will perform on
    // EXERCISE 6-8
    vector<RouteModel::Node*> open_list;
    // EXERCISE 6-3
    RouteModel::Node *start_node;
    RouteModel::Node *end_node;
    // hold the total distance for the route that A* search 
    // finds from start_node to end_node
    float distance = 0.0f; 
    

    
};

#endif
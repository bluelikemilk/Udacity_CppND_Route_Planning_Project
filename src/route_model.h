#ifndef ROUTE_MODEL_H
#define ROUTE_MODEL_H

#include <limits>
#include <cmath>
#include <unordered_map>
#include "model.h"
#include <iostream>

using namespace std;

class RouteModel : public Model { // RouteModel inherites from Model

  public:
    class Node : public Model::Node { //Node class of RouteModel inherites from Model::Node
      public:
       // EXERCISE 5-7
        Node *parent = nullptr;
        float h_value = std::numeric_limits<float>::max();
        float g_value = 0.0;
        bool visited = false;
        vector<Node*> neighbors;

      // EXERCISE 5 - 10
        float distance(Node other) const{
          // x and y inherited from Model::Node
          return std::sqrt(std::pow((x - other.x), 2) + std::pow((y - other.y), 2));
        }

      // EXERCISE 5-15
      // This method will be called from route_planner.cpp, so the method needs to be public.
      void FindNeighbors();

      // 2 constructors
      Node(){} // default constructor that sets default value for Node
      Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {} // construcotr with initializer list

      private:
      // private Node variables and methods
      // EXERCISE 5-7
        int index; // used to keep track of the nodes
        RouteModel * parent_model = nullptr; // allows node access data in the parent model
      // EXERCISE 5-14
        Node* FindNeighbor(vector<int> node_indicies);
    };

    RouteModel(const std::vector<std::byte> &xml);
    std::vector<Node> path; // stores the path found by A* search

    // EXERCISE 5-6: Add a public "getter" method SNodes
    //   This method should return a reference to the vector of Nodes stored as m_Nodes.
    vector<Node>& SNodes() {return m_Nodes;};

    // EXERCISE 5 - 13
    auto &GetNodeToRoadMap() {return node_to_road;};

    // EXERCISE 5 - 16
    RouteModel::Node &FindClosestNode(float x, float y);
    
  private:
    // EXERCISE 5-6: Add a private vector of Node objects named m_Nodes
    vector<Node> m_Nodes;
    // EXERCISE 5-13
    unordered_map<int, vector<const Model::Road*>> node_to_road;
    void CreateNodeToRoadHashmap();
};

#endif

#include "route_model.h"
#include <iostream>
using namespace std;
// RouteModel constructor
// EXERCISE 5-9
RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml){ // extands Model constructor, so Model constructor will be called to load xml as Model::Node type into this->Nodes
    int counter = 0; // cnt needed by RouteModel::Node
    // load each node of Model::Node, use these information plus counter and RouteModel reference 
    // to create a RouteModel::Node, and push the created RouteModel::Node into m_Nodes
    for(Model::Node node : this->Nodes()){ // this measn this RouteModel instance, because RouteModel extends Model, this->Nodes() returns a vector of Model::Node
        //create RouteModel::Node use its constructor with intializer list
        m_Nodes.emplace_back(Node(counter, this, node)); 
        counter++;
    }

    // EXERCISE 5-13
    // create hash map for each loaded node
    CreateNodeToRoadHashmap();
}

// EXERCISE 5-13
void RouteModel::CreateNodeToRoadHashmap(){ // remember to write RouteModel::
    //iterates through the vector given by calling Roads()
    for(const Model::Road &road : Roads()){
        // check the type is not a foot way
        if(road.type != Model::Road::Type::Footway){
            // Loop over each node_idx in the way that the road belongs to
            for(int idx : Ways()[road.way].nodes){
                // if the idx is not in the hashmap node_to_road yet, 
                if(node_to_road.find(idx) == node_to_road.end()){
                    // add it with an empty vector of const Model::Road* objects
                    node_to_road[idx] = vector<const Model::Road*> (); // use () for constructor
                }
                // Push a pointer to the current road in the loop to the back of the vector given by the node_idx key in node_to_road
                node_to_road[idx].push_back(&road);
            }
        }
    }
}

//EXERCISE 5-14
RouteModel::Node* RouteModel::Node::FindNeighbor(vector<int> node_indicies){
    // loop through the node_indices to find the closest unvisited node
    Node *closest_node = nullptr;
    Node node;
    for(int index:node_indicies){
        node = parent_model->SNodes()[index]; // index is among all nodes in parent_model
        // check the ndoe hase not been visited and not the current node (i.e. distance = 0)
        if(!node.visited && this->distance(node) != 0){
            // include the case that closest_node is not intialized
            if(closest_node == nullptr || this->distance(node) < this->distance(*closest_node))
                closest_node = &parent_model->SNodes()[index];
        }
    }
    return closest_node;
}

// EXERCISE 5-15
void RouteModel::Node::FindNeighbors(){
    // all roads that the current node is on
    // parent_model is the model of the current node, node_to_road is a map <node index, roads containes this index>
    for(const Model::Road* road : parent_model->node_to_road[this->index]){
        // get all nodes on current road and find the closest_node from them
        RouteModel::Node *closest_node = this->FindNeighbor(parent_model->Ways()[road->way].nodes); 
        // if the closest_node is not null, add it to neighbors of the current node
        if(closest_node != nullptr) this->neighbors.emplace_back(closest_node);
    } 
}

// EXERCISE 5 - 16
RouteModel::Node &RouteModel::FindClosestNode(float x, float y){
    // use the struct Node inheritated from Model::Node
    // take user input
    Node input;
    input.x = x;
    input.y = y;
    // init the min distance and closest index
    float min_dist = std::numeric_limits<float>::max();
    int closest_idx;
    // iterates through road vector given by calling Roads()
    for(auto &road : Roads()){ // Roads() returns all road of the current model
        // check the current road is not footway
        if(road.type != Model::Road::Type::Footway){
            // loop over each node on the way that this road belongs to
            for(int node_idx : Ways()[road.way].nodes){
                // calculate distance and comapre with min_dist
                float dist = input.distance(SNodes()[node_idx]);
                if(dist < min_dist){
                    min_dist = dist;
                    closest_idx = node_idx;
                }
            }
        }
    }
    return SNodes()[closest_idx];
}





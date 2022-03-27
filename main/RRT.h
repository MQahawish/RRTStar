#include <iostream>
#include <math.h>
#include <cmath>
#include <vector>
#include <random>
#include "Node.h"
#include "Obstacle.h"
#include "Area.h"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

class RRTstar {
    public :
    Node start;
    Node goal;
    std::vector<Obstacle> obstacle_list;
    std::vector<Node> node_list;
    double rand_area_max;
    double rand_area_min;
    double expand_dis;
    double path_resolution;
    int max_iter;
    Area area;
    double goal_sample_rate;
    double connect_circle_dist;
    bool search_until_max_iter;
    bool animation;
    
    RRTstar(Node& start, Node& goal, std::vector<Obstacle> obstacles, double rand_area_max,
        double rand_area_min, Area area,
        double expand_dis_, double path_resolution_,
        int max_iter_, double connect_circle_dist_, bool search_until_max_iter_
        ,double goal_sample_rate_);//constructor
   ~RRTstar();                             //destructor
   void Planning(std::vector<Node>& path); //finding and planning a new path
   Node steer(Node* A,Node* B,double extend_length = std::numeric_limits<double>::max()); //guiding to choosing the next node
   void generateFinalCourse(double goal_ind,std::vector<Node>& path);  //generates the final path
   Node getRandomNode();                 //get random node
   Node& getNearestNode(Node ran_Node);  //gets the nearest node
   bool checkIfOutsidePlayArea(Node node);  //checks if it's outside the configured Area
   bool checkCollision(Node& node);  //checks if there is any obstacles that collide with the path
   double calcDistance(Node from,Node to);  //calculate the distance between 2 nodes
   double calcAngle(Node from,Node to);     //calculate the angle between 2 nodes
   std::vector<double> findNearNodes(Node node);  //finds the nearest neighboring nodes to the current node
   void rewire(Node& node,std::vector<double> near_nodes_ind); //redraw the path
   void searchBestGoalNode(double& index);         //search for the best goal node
   double chooseParent(double& index_, Node& new_node, std::vector<double>& near_node_inds);  //chose a parent node on the path to the current node with the lowest cost
   double calcNewCost(Node& from,Node& to);   //calculate the new cost
   void propagateCostToLeaves(Node& parent); //passes on the cost to the leaves

};
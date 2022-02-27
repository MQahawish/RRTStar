#include <iostream>
#include <math.h>
#include <cmath>
#include <vector>
#include <list>
#include <time.h>
#include "Node.h"
#include "Obstacle.h"
#include "AreaBounds.h"

using namespace std;

class RRTstar {
    public :
    Node start;
    Node goal;
    vector<Obstacle> obstacle_list;
    vector<Node> node_list;
    double rand_area_max;
    double rand_area_min;
    double expand_dis;
    double path_resolution;
    double goal_sample_rate;
    int max_iter;
    Area play_Area;
    double connect_circle_dist;
    bool search_until_max_iter;
    

   RRTstar( Node  start, Node  goal,  vector<Obstacle> obstacles, double rand_area_max,double rand_area_min, double expand_dis, double path_resolution , double goal_sample_rate, int max_iter, Area play_area,double connect_circle_dist,bool search_until_max_iter);//constructor
   ~RRTstar(); //destructor
   vector<Node>* Planning(); //finding and planning a new path
   Node* steer(Node A,Node B,double extend_length=std::numeric_limits<float>::max()); //guiding to choosing the next node
   vector<Node>* generateFinalCourse(int goal_ind);  //generates the final path
   double calcDistToGoal(int x,int y);  //calculates distance from goal node
   Node getRandomNode();  //get random node
   int getNearestNodeIndex(vector<Node> nodes,Node ran_Node);  //gets the index of the nearest node
   bool checkIfOutsidePlayArea(Node node,Area playArea);  //checks if it's outside the configured Area
   bool checkCollision(Node node);  //checks if there is any obstacles that collide with the path
   double calcDistance(Node from,Node to);  //calculate the distance between 2 nodes
   double calcAngle(Node from,Node to);     //calculate the angle between 2 nodes
   vector<double> findNearNodes(Node node);  //finds the nearest neighboring nodes to the current node
   void rewire(Node node,vector<int> near_nodes_ind); //redraw the path
   int searchBestGoalNode();     //search for the best goal node
   Node* chooseParent(Node new_node,vector<double> near_node_inds);  //chose a parent node on the path to the current node with the lowest cost
   int calcNewCost(Node from,Node to);   //calculate the new cost
   void propagateCostToLeaves(Node parent); //passes on the cost to the leaves
};
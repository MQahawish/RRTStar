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
    

   RRTstar( Node  start, Node  goal,  vector<Obstacle> obstacles, double rand_area_max,double rand_area_min, double expand_dis, double path_resolution , double goal_sample_rate, int max_iter, Area play_area,double connect_circle_dist,bool search_until_max_iter);
   ~RRTstar();
   vector<Node>* Planning();
   Node* steer(Node A,Node B,double extend_length=std::numeric_limits<float>::max()); //+
   vector<Node>* generateFinalCourse(int goal_ind);  //+
   double calcDistToGoal(int x,int y);  //+
   Node getRandomNode();  //+
   int getNearestNodeIndex(vector<Node> nodes,Node ran_Node);  //+
   bool checkIfOutsidePlayArea(Node node,Area playArea);  //+
   bool checkCollision(Node node);  //+
   double calcDistance(Node from,Node to);  //+
   double calcAngle(Node from,Node to);     //+
   vector<double> findNearNodes(Node node);  //+ 
   void rewire(Node node,vector<int> near_nodes_ind); //+
   int searchBestGoalNode();     //+
   Node* chooseParent(Node new_node,vector<double> near_node_inds);  //+
   int calcNewCost(Node from,Node to);   //+
   void propagateCostToLeaves(Node parent); //+
};
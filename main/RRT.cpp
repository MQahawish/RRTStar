#include "RRT.h"
#include <algorithm>

RRTstar::RRTstar(Node& start_, Node& goal_, std::vector<Obstacle> obstacles_, double rand_area_max_,
 double rand_area_min_, Area area_, double expand_dis_, double path_resolution_,
  int max_iter_, double connect_circle_dist_ , bool search_until_max_iter_
  ,double goal_sample_rate_)
{
   start = start_;
   goal = goal_;
   obstacle_list = obstacles_;
   rand_area_max = rand_area_max_;
   rand_area_min = rand_area_min_;
   expand_dis = expand_dis_;
   path_resolution = path_resolution_;
   max_iter = max_iter_;
   area = area_;
   connect_circle_dist = connect_circle_dist_;
   search_until_max_iter = search_until_max_iter_;
   goal_sample_rate=goal_sample_rate_;
}

RRTstar::~RRTstar(){}

//Calculating the distance between two Nodes
double RRTstar::calcDistance(Node from, Node to)
{
   double dx = to.x - from.x;
   double dy = to.y - from.y;
   return hypot(dx, dy);
}

//Calculating angle between two nodes
double RRTstar::calcAngle(Node from, Node to)
{
   double dx = to.x - from.x;
   double dy = to.y - from.y;
   return atan2(dx, dy);
}

//Calculating new cost
double RRTstar::calcNewCost(Node& from, Node& to)
{
    return from.cost + calcDistance(from, to);
}

//checking if Node collides with obstacle
bool RRTstar::checkCollision(Node& node)
{
   for (auto obstacle : obstacle_list)
   {
       for(int i =0 ; i < node.pathX.size();i++) {
           if (hypot(node.pathX[i] - obstacle.x, node.pathY[i] - obstacle.y) <= obstacle.radius) {
               return false;
           }
       }
   }
   return true;
}

//checking if node is outside play area
bool RRTstar::checkIfOutsidePlayArea(Node node)
{
    if(area.Free)
        return true;
    for(auto x : node.pathX)
        if(x<area.xmin||x>area.xmax)
            return false;
    for(auto y : node.pathY)
    {
        if(y<area.ymin||y>area.ymax)
            return false;
    }
    return true;
}

//generating random node
Node RRTstar::getRandomNode()
{
      double number;
      Node rand_node;
      std::random_device rd;
      std::uniform_real_distribution<double> ud(rand_area_min, rand_area_max);
    std::uniform_real_distribution<double> td(0, 100);
    std::mt19937 mt(rd());
    number = td(mt);
    if(number>goal_sample_rate) {
        rand_node.x = ud(mt);
        rand_node.y = ud(mt);

    }
    else
    {
        rand_node.x=goal.x;
        rand_node.y=goal.y;
    }
   return rand_node;
}   

//getting the nearest node out of list of existing node
Node& RRTstar::getNearestNode(Node ran_Node)
{
    double currdist,min_dist = std::numeric_limits<double>::max(), index=-1;
    for (unsigned int  i = 0; i < node_list.size(); i++)
    {
        currdist = calcDistance(node_list[i], ran_Node);
        if ( currdist <= min_dist)
        {
            min_dist = currdist;
            index = i;
        }
    }
    return node_list[index];
}

//building final path
void RRTstar::generateFinalCourse(double goal_ind,std::vector<Node>& path)
{
    path.push_back(goal);
    Node node = node_list[goal_ind];
    node.pathX.push_back(goal.x);
    node.pathY.push_back(goal.y);
    while (node.parent_index != -1)
    {
        path.push_back(node);
        node = node_list[node.parent_index];
    }
    path.push_back(node);
    path.push_back(goal);
    std::reverse(path.begin(),path.end());
}

//updating cost to nodes in path
void RRTstar::propagateCostToLeaves(Node& parent)
{
   for (auto node : node_list)
   {
      if (node.parent_index!=-1 && node_list[node.parent_index] == parent)
      {
         node.cost = calcNewCost(parent, node);
         propagateCostToLeaves(node);
      }
   }
}

//rewiring near nodes to new node
void RRTstar::rewire(Node& new_node, std::vector<double> near_nodes_ind)
{
   for (int index = 0; index < near_nodes_ind.size(); index++)
   {
       Node edge_node;
       edge_node =  steer(&new_node, &node_list[near_nodes_ind[index]]);
      edge_node.cost = calcNewCost(new_node, node_list[near_nodes_ind[index]]);
      bool no_collision = checkCollision(edge_node);
      bool improved_cost = node_list[near_nodes_ind[index]].cost > edge_node.cost;
      if (no_collision && improved_cost)
      {
         node_list[near_nodes_ind[index]] = edge_node;
         propagateCostToLeaves(new_node);
      }
   }
}

//checking if there is an existing node close enough to goal node
void RRTstar::searchBestGoalNode(double& index)
{
    std::vector<double> safe_goal_inds;
    for (int i = 0; i < node_list.size(); i++)
    {
        if (calcDistance(node_list[i], goal) <= expand_dis)
        {
            Node t_node;
            t_node=steer(&node_list[i], &goal);
            if (checkCollision(t_node))
            {
                safe_goal_inds.push_back(i);
            }
        }
    }
    double min_cost = std::numeric_limits<double>::max();
    for (int j = 0; j < safe_goal_inds.size(); j++)
    {
        if (node_list[safe_goal_inds[j]].cost < min_cost)
        {
            index = safe_goal_inds[j];
            min_cost = node_list[safe_goal_inds[j]].cost;
        }
    }
}

//choosing new parent to new_node
double RRTstar::chooseParent(double& index_, Node& new_node, std::vector<double>& near_node_inds)
{
    if (near_node_inds.size() == 0)
        return std::numeric_limits<double>::max();
   std::vector<double> costs;
   for (auto ind : near_node_inds)
   {
      Node t_node;
      t_node =  steer(&node_list[ind], &new_node);
      if (checkCollision(t_node))
         costs.push_back(calcNewCost(node_list[ind], t_node));
      else
         costs.push_back(std::numeric_limits<double>::max());
   }
   double min_cost = std::numeric_limits<double>::max();
   for (int i = 0; i < costs.size(); i++)
      if (costs[i] < min_cost)
      {
         min_cost = costs[i];
         index_ = i;
      }

   return min_cost;
}

//finding near nodes in existing list of nodes
std::vector<double> RRTstar::findNearNodes(Node new_node)
{
   std::vector<double> near_inds;
   double n_nodes = node_list.size();
   double r = connect_circle_dist * sqrt(log(n_nodes) / n_nodes);
   r = connect_circle_dist < r ? connect_circle_dist : r ;
   for (int index = 0; index < node_list.size(); index++)
   {
       if (calcDistance(new_node, node_list[index]) < r)
       {
           near_inds.push_back(index);
       }
   }
   return near_inds;
}

//generating new node between two nodes by steering according to angle and distance
Node RRTstar::steer(Node* from, Node* to, double extend_length)
{
   Node new_node(from->x,from->y);
   double distance = calcDistance(new_node, *to);
   double angle = calcAngle(new_node, *to);
   extend_length = extend_length > distance ? distance : extend_length;
   double n_expand = floor(extend_length / path_resolution);
   for (int i = 0; i < n_expand; i++)
   {
      new_node.x += path_resolution * cos(angle);
      new_node.y += path_resolution * sin(angle);
      new_node.pathX.push_back(new_node.x);
      new_node.pathY.push_back(new_node.y);
   }
   distance = calcDistance(new_node, *to);
   if (distance < path_resolution)
   {
       new_node.pathX.push_back(to->x);
       new_node.pathY.push_back(to->y);
      new_node.x = to->x;
      new_node.y = to->y;
   }
   for (int cnt = 0; cnt < node_list.size(); cnt++)
   {
       if (node_list[cnt] == *from)
       {
           new_node.parent_index = cnt;
       }
   }
   return new_node;
}
 
//planning the path
void RRTstar::Planning(std::vector<Node>& path){
    double lastindex = -1, n_parent_index = -1;
   node_list.push_back(start);
   double i=0;
   while (i < max_iter)
   {
       Node rnd_node = getRandomNode();
       Node nearest_node = getNearestNode(rnd_node);
       Node new_node;
       new_node =  steer(&nearest_node, &rnd_node, expand_dis);
       new_node.cost = nearest_node.cost + calcDistance(new_node, nearest_node);
       if (checkCollision(new_node) && checkIfOutsidePlayArea(new_node)) {
           std::vector<double> near_inds = findNearNodes(new_node);
           double new_cost = chooseParent(n_parent_index,new_node, near_inds);
           if (new_cost != std::numeric_limits<double>::max())
           {
                   new_node = steer(&node_list[n_parent_index], &new_node,expand_dis);
                   new_node.cost = new_cost;
                   rewire(new_node, near_inds);
                   node_list.push_back(new_node);
           }
           else {
               node_list.push_back(new_node);
           }
       }
       searchBestGoalNode(lastindex);
       if (lastindex != -1 && !search_until_max_iter)
       {
           generateFinalCourse(lastindex,path);
           return;
       }
       i++;
   }
   searchBestGoalNode(lastindex);
   if (lastindex != -1 && !search_until_max_iter)
   {
       generateFinalCourse(lastindex, path);
       return;
   }
   std::cout << "Can't Find Path!" << std::endl;
}



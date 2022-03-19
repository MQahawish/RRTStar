#include "RRT.h"


RRTstar::RRTstar(Node& start_, Node& goal_, std::vector<Obstacle> obstacles_, double rand_area_max_,
 double rand_area_min_, Area area_, double expand_dis_, double path_resolution_,
  int max_iter_, double connect_circle_dist_ , bool search_until_max_iter_)
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
}

RRTstar::~RRTstar(){}

double RRTstar::calcDistance(Node from, Node to)
{
   double dx = to.x - from.x;
   double dy = to.y - from.y;
   return hypot(dx, dy);
}

double RRTstar::calcAngle(Node from, Node to)
{
   double dx = to.x - from.x;
   double dy = to.y - from.y;
   return atan2(dx, dy);
}

double RRTstar::calcDistToGoal(double x, double y)
{
   double dx = goal.x - x;
   double dy = goal.y - y;
   return hypot(dx, dy);
}

double RRTstar::calcNewCost(Node& from, Node& to)
{
    return from.cost + calcDistance(from, to);
}

bool RRTstar::checkCollision(Node& node)
{
   for (auto obstacle : obstacle_list)
   {
       if (hypot(node.x - obstacle.x, node.y - obstacle.y) <= obstacle.radius)
       {
           return false;
       }
   }
   return true;
}

bool RRTstar::checkIfOutsidePlayArea(Node node)
{
   return !(node.x < area.xmin || node.y < area.ymin || node.x > area.xmax || node.y > area.ymax)||area.Free;
}

Node RRTstar::getRandomNode()
{
      std::random_device rd;
      std::uniform_real_distribution<double> ud(rand_area_min, rand_area_max);
      std::mt19937 mt(rd());
      Node rand_node;
          rand_node.x = ud(mt);
          rand_node.y = ud(mt);
   return rand_node;
}   

Node& RRTstar::getNearestNode(Node ran_Node)
{
    double currdist,min_dist = std::numeric_limits<double>::max(), index=-1;
    for (unsigned int  i = 0; i < node_list.size(); i++)
    {
        currdist = calcDistance(node_list[i], ran_Node);
        if ( currdist < min_dist)
        {
            min_dist = currdist;
            index = i;
        }
    }
    return node_list[index];
}

void RRTstar::generateFinalCourse(double goal_ind,std::vector<Node>& path)
{
    path.push_back(goal);
    Node node = node_list[goal_ind];
    while (node.parent_index != -1)
    {
        path.push_back(node);
        node = node_list[node.parent_index];
    }
    path.push_back(node);
}


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

Node& RRTstar::steer(Node* from, Node* to, double extend_length)
{
   Node new_node;
   new_node.x = from->x;
   new_node.y = from->y;
   double distance = calcDistance(new_node, *to);
   double angle = calcAngle(new_node, *to);
   extend_length = extend_length > distance ? distance : extend_length;
   double n_expand = floor(extend_length / path_resolution);
   for (int i = 0; i < n_expand; i++)
   {
      new_node.x += path_resolution * cos(angle);
      new_node.y += path_resolution * sin(angle);
   }
   distance = calcDistance(new_node, *to);
   if (distance < path_resolution)
   {
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
           else
           {
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


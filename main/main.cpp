#include "RRT.h"
#include "matplotlibcpp.h"

/****Parameters for RRT ******\
* 1) starting Node
* 2) goal Node
* 3) Obstacle List
* 4) upper limit for randem number generating
* 5) lower limir for random number generating
* 6) play area
* 7) expand distination 
* 8) path resolution - the value of pivoting from current node to another
* 9) max number of itterations
* 10) connect cirle destination
* 11) search until max number of itterations ?
* */

int main() {

	Node start(0, 0);
	Node goal(7, 1);
	std::vector<Obstacle> obstacles;
	obstacles.push_back(Obstacle(2,2,1));
	obstacles.push_back(Obstacle(2, 3, 1));
	obstacles.push_back(Obstacle(5, 3, 2));
	obstacles.push_back(Obstacle(5, 1, 1));;
	Area area(0, 10, 0, 10, 0);
	RRTstar rrt_star(start, goal, obstacles, 10, 0, area, 1, 3, 100000, 3, false);
	std::vector<Node> path;
	rrt_star.Planning(path);
	reverse(path.begin(), path.end());
	std::cout << " The path is : \n";
	for (auto node : path)
	{
		std::cout  << node.x << "," << node.y << std::endl;
	}
	return 0;
}


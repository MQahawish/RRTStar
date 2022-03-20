#include "RRT.h"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

/****Parameters for RRT ******\
* 1) starting Node
* 2) goal Node
* 3) Obstacle List
* 4) upper limit for random number generating
* 5) lower limit for random number generating
* 6) play area
* 7) expand destination
* 8) path resolution - the value of pivoting from current node to another
* 9) max number of iterations
* 10) connect circle destination
* 11) search until max number of iterations ?
* */

/*
*  recommendation : expand destination and connect circle =~  the smallest obstacle radius
*  so that the algorithm doesn't jump over obstacles
*/

void draw_graph(std::vector<Node>& path, std::vector<Obstacle>& obstacles);
void draw_circle(Obstacle& obstacle);

int main() {
	Node start(0, 0);
	Node goal(5,5);
	std::vector<Obstacle> obstacles;
	obstacles.emplace_back(Obstacle(2,2,1));

	Area area(0, 10, 0, 10, false);
	RRTstar rrt_star(start, goal, obstacles, 10, 0, area, 0.5, 2, 100000,2, false);
	std::vector<Node> path;
	rrt_star.Planning(path);
	reverse(path.begin(), path.end());
	draw_graph(path, obstacles);
	return 0;
}

void draw_graph(std::vector<Node>& path, std::vector<Obstacle>& obstacles)
{
	for (auto obstacle : obstacles)
	{
		draw_circle(obstacle);
	}
	std::vector<double> x;
	std::vector<double> y;
	for (auto const &node : path)
	{
		x.push_back(node.x);
		y.push_back(node.y);
	}
	plt::plot(x, y, "-r");
}

void draw_circle(Obstacle& obstacle)
{
	std::vector<double> x, y;
	for (double deg = 0; deg <= 360; deg ++)
	{
		x.push_back(obstacle.x + obstacle.radius * cos(2 * 3.14 * (deg / 360)));
		y.push_back(obstacle.y + obstacle.radius * sin(2 * 3.14 * (deg / 360)));
	}
	plt::plot(x, y, "-b");
}





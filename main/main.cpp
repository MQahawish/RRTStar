#include "RRT.h"
#include <chrono>

void draw_circle(double x, double y, double r, std::string color);

void animation(RRTstar r, std::vector<Node> &path);

//generating random obstacles with radius in range [1,radius] , becareful that generated obstacles might include start/goal!
void generate_random_obstacles(double num, double radius, std::vector<Obstacle> &obstacles, Area &area);


int main() {
    Node start(50, 50);
    Node goal(1, 1);
    std::vector<Obstacle> obstacles;
    Area area(0, 50, 0, 50, false);
    generate_random_obstacles(10, 4, obstacles, area);
    auto startTimer = std::chrono::high_resolution_clock::now();
    RRTstar rrt_star(start, goal, obstacles, 100, -100, area, 2, 1,
                     100000, 2, false, 10);
    std::vector<Node> path;
    rrt_star.Planning(path);
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - startTimer);
    std::cout << "duration: " << duration.count() << std::endl;
    animation(rrt_star, path);
    return 0;
}




void animation(RRTstar r, std::vector<Node> &path) {
    
    //drawing the goal node is a small circle
    draw_circle(r.goal.x, r.goal.y, 0.1, "-g");
    if (!r.area.Free) {
        plt::ylim(r.area.ymin, r.area.ymax);
        plt::xlim(r.area.xmin, r.area.xmax);
    }
    
    //drawing obstacles
    for (auto obstacle: r.obstacle_list) {
        draw_circle(obstacle.x, obstacle.y, obstacle.radius, "-b");
    }

   //drawing all nodes that were computed in the process (might take a while!)
   for(auto node: r.node_list)
    {
           for (int j = 0; j < node.pathY.size(); j++) {
               draw_circle(node.pathX[j], node.pathY[j], 0.01, "black");
           }
           plt::plot(node.pathX, node.pathY, "black");
           plt::pause(0.0001);
    }


    //drawing final path found in green
    for (auto node: path) {
        for (int j = 0; j < node.pathY.size(); j++) {
            draw_circle(node.pathX[j], node.pathY[j], 0.01, "-g");
        }
        plt::plot(node.pathX, node.pathY, "-g");
        plt::pause(0.0001);
    }
    plt::show();
}

void draw_circle(double x, double y, double r, std::string color) {
    std::vector<double> xs, ys;
    for (double deg = 0; deg <= 360; deg += 0.1) {
        xs.push_back(x + r * cos(2 * M_PI * (deg / 360)));
        ys.push_back(y + r * sin(2 * M_PI * (deg / 360)));
    }
    plt::plot(xs, ys, color);
}

void generate_random_obstacles(double num, double radius, std::vector<Obstacle> &obstacles, Area &area) {
    std::random_device rd;
    std::uniform_real_distribution<double> x(area.xmin + radius, area.xmax - radius);
    std::uniform_real_distribution<double> y(area.ymin + radius, area.ymax - radius);
    std::uniform_real_distribution<double> r(1, radius);
    std::mt19937 mt(rd());
    for (int i = 0; i < num; i++) {
        obstacles.emplace_back(x(mt), y(mt), r(mt));
    }
}








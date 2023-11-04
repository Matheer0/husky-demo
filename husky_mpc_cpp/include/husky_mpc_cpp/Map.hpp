#ifndef MAP_HPP
#define MAP_HPP

#include <vector>
#include <casadi/casadi.hpp>

class Obstacle
{
public:
    Obstacle(double x, double y, double radius);
    double x_;
    double y_;
    double radius_;
};

class Map
{
public:
    Map () = default;
    Map(double x_init, double y_init, double x_target, double y_target, double theta_target, double x_center, double y_center, double offset);
    
    double x_init_;
    double y_init_;
    double x_target_;
    double y_target_;
    double theta_target_;

    double x_center_;
    double y_center_;
    double offset_;

    double x_upper_limit_;
    double x_lower_limit_;
    double y_upper_limit_;
    double y_lower_limit_;

    std::vector<Obstacle> obstacle_list;
    std::vector<Obstacle> dummy_obstacles;

    void generate_circular_obstacles();
};

#endif // MAP_HPP
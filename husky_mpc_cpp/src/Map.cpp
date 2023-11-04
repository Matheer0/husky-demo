#include "husky_mpc_cpp/Map.hpp"

Obstacle::Obstacle(double x, double y, double radius) : x_(x), y_(y), radius_(radius)
{
}

Map::Map(double x_init, double y_init, double x_target, double y_target, double theta_target, double x_center, double y_center, double offset):
    x_init_(x_init), y_init_(y_init), x_target_(x_target), y_target_(y_target), theta_target_(theta_target), x_center_(x_center), y_center_(y_center), offset_(offset)
{
    y_upper_limit_ = y_center_ + offset_;
    y_lower_limit_ = y_center_ - offset_;

    x_lower_limit_ = x_center_ - offset_;
	x_upper_limit_ = x_center_ + offset_;
}
		
void Map::generate_circular_obstacles()
{
    double default_radius = 1;
    Obstacle obs(15, 0, default_radius);
    obstacle_list.push_back(obs);
}

	



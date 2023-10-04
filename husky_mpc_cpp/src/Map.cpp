#include "husky_mpc_cpp/Map.hpp"

Obstacle::Obstacle(double x, double y, double radius) : x_(x), y_(y), radius_(radius)
{
}

Map::Map(double x_init, double y_init, double x_target, double y_target, double theta_target, double y_center, double y_offset):
    x_init_(x_init), y_init_(y_init), x_target_(x_target), y_target_(y_target), theta_target_(theta_target), y_center_(y_center), y_offset_(y_offset)
{
    y_upper_limit_ = y_center_ + y_offset_;
    y_lower_limit_ = y_center_ - y_offset_;
}
		
void Map::generate_circular_obstacles()
{
    Obstacle obs(15, 0, 2);
    obstacle_list.push_back(obs);
}

	



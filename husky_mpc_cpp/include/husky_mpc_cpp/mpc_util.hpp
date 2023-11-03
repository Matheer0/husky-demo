#ifndef UTIL_HPP
#define UTIL_HPP

#include <casadi/casadi.hpp>

casadi::Function dynamics(const casadi::SX& states);

#endif // UTIL_HPP
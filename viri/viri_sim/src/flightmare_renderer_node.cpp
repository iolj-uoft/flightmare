#include <ros/ros.h>

#include "viri_sim/flightmare_renderer.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "flightmare_renderer");
  viri_sim::FlightmareRenderer renderer(ros::NodeHandle(), ros::NodeHandle("~"));
  ros::spin();
  return 0;
}


#include <ros/ros.h>
#include <gscam/gspipeline.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "gspipeline_publisher");
  ros::NodeHandle nh, nh_private("~");

  gscam::GSPipeline gspipeline_driver(nh, nh_private);
  gspipeline_driver.run();

  return 0;
}


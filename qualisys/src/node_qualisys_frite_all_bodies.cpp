#include <ros/ros.h>
#include <qualisys/QualisysDriver_frite_all_bodies.h>

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "qualisys frite_all_bodies");
  ros::NodeHandle nh("~");

  qualisys::QualisysDriverFriteAllBodies qualisys_driver_frite_all_bodies(nh);
  
  if(!qualisys_driver_frite_all_bodies.init()) {
    ROS_INFO("Initialization of the qualisys driver frite failed!");
    return -1;
  }

  while(ros::ok())
  {
    qualisys_driver_frite_all_bodies.run();
    ros::spinOnce();
  }

  ROS_INFO("Shutting down");
  qualisys_driver_frite_all_bodies.disconnect();

  return 0;
}

#include <ros/ros.h>
#include <qualisys/QualisysDriver_frite.h>

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "qualisys frite");
  ros::NodeHandle nh("~");

  qualisys::QualisysDriverFrite qualisys_driver_frite(nh);
  if(!qualisys_driver_frite.init()) {
    ROS_INFO("Initialization of the qualisys driver frite failed!");
    return -1;
  }

  while(ros::ok())
  {
    qualisys_driver_frite.run();
    ros::spinOnce();
  }

  ROS_INFO("Shutting down");
  qualisys_driver_frite.disconnect();

  return 0;
}

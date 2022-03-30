#include <qualisys/QualisysDriver_frite_all_bodies.h>
#include <algorithm>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

using namespace std;

namespace qualisys{

double QualisysDriverFriteAllBodies::deg2rad = M_PI / 180.0;

QualisysDriverFriteAllBodies::QualisysDriverFriteAllBodies(const ros::NodeHandle& n):
  nh(n),
  publish_tf(false)
{
  return;
}

bool QualisysDriverFriteAllBodies::init() {
  // The base port (as entered in QTM, TCP/IP port number, in the RT output tab
  // of the workspace options
  nh.param("server_address", server_address, string("192.168.254.1"));
  nh.param("server_base_port", base_port, 22222);
  nh.param("publish_tf", publish_tf, false);

  // Connecting to the server
  ROS_INFO_STREAM("Connecting to the Qualisys Motion Tracking system specified at: "
      << server_address << ":" << base_port);

  if(!port_protocol.Connect((char *)server_address.data(), base_port, 0, 1, 7)) {
    ROS_FATAL_STREAM("Could not find the Qualisys Motion Tracking system at: "
        << server_address << ":" << base_port);
    return false;
  }
  ROS_INFO_STREAM("Connected to " << server_address << ":" << base_port);

  // Get 6DOF settings
  port_protocol.Read6DOFSettings();
  
  pose_all_bodies_publisher = nh.advertise<geometry_msgs::PoseArray>("/PoseAllBodies", 10);
  marker_pose_array_publisher = nh.advertise<visualization_msgs::MarkerArray>("/VisualizationPoseArrayMarkers", 10);
  
  return true;
}

void QualisysDriverFriteAllBodies::disconnect() {
  ROS_INFO_STREAM("Disconnected with the server "
      << server_address << ":" << base_port);
  port_protocol.StreamFramesStop();
  port_protocol.Disconnect();
  return;
}

void QualisysDriverFriteAllBodies::handlePacketData(CRTPacket* prt_packet) {

  // Number of rigid bodies
  int body_count = prt_packet->Get6DOFEulerBodyCount();
  
  visualization_msgs::MarkerArray  marker_array_msg;
  visualization_msgs::Marker  simple_marker_msg;
  
  geometry_msgs::PoseArray  pose_array_msg;
  geometry_msgs::Pose pose_msg;
  
  geometry_msgs::Quaternion geometry_quaternion_body;
  geometry_msgs::Point geometry_point_body;
  tf::Vector3 vector3_point_body;
  
  pose_array_msg.header.stamp = ros::Time::now(); // timestamp of creation of the msg
  pose_array_msg.header.frame_id = "map"; // frame id in which the array is published
  
  simple_marker_msg.header.frame_id = "map";
  simple_marker_msg.header.stamp = ros::Time::now();
  simple_marker_msg.ns = "points_and_lines";
  simple_marker_msg.action = visualization_msgs::Marker::ADD;
  
  simple_marker_msg.type = visualization_msgs::Marker::SPHERE;
  simple_marker_msg.scale.x = 0.02;
  simple_marker_msg.scale.y = 0.02;
  simple_marker_msg.scale.z = 0.02;
  // Points are read
  simple_marker_msg.color.r = 1.0f;
  simple_marker_msg.color.a = 1.0;
  
  float x, y, z, roll, pitch, yaw;
  
  // Publish data for each rigid body
  for (int i = 0; i < body_count; ++i) 
  {
	  
    prt_packet->Get6DOFEulerBody(i, x, y, z, roll, pitch, yaw);

    if(isnan(x) || isnan(y) || isnan(z) || isnan(roll) || isnan(pitch) || isnan(yaw)) 
	{
      ROS_WARN_STREAM_THROTTLE(3, "Rigid-body " << i + 1 << "/" << body_count << " not detected");
      continue;
    }

    // Qualisys sometimes flips 180 degrees around the x axis
    if(roll > 90)
      roll -= 180;
    else if(roll < -90)
      roll += 180;
      
      
    geometry_quaternion_body = tf::createQuaternionMsgFromRollPitchYaw(roll * deg2rad, pitch * deg2rad, yaw * deg2rad);
	vector3_point_body = tf::Vector3(x, y, z) / 1000.;
	tf::pointTFToMsg(vector3_point_body, geometry_point_body);
	
	pose_msg.position = geometry_point_body;
	pose_msg.orientation = geometry_quaternion_body;
	pose_array_msg.poses.push_back(pose_msg);
	
	
	simple_marker_msg.id = i;
	simple_marker_msg.pose.position = geometry_point_body;
	simple_marker_msg.pose.orientation = geometry_quaternion_body;
	
	marker_array_msg.markers.push_back(simple_marker_msg);
	
  }

  pose_all_bodies_publisher.publish(pose_array_msg);
  marker_pose_array_publisher.publish(marker_array_msg);
 
  return;
}

void QualisysDriverFriteAllBodies::run() {

  CRTPacket* prt_packet = port_protocol.GetRTPacket();
  CRTPacket::EPacketType e_type;
  port_protocol.GetCurrentFrame(CRTProtocol::Component6dEuler);

  if(port_protocol.ReceiveRTPacket(e_type, true)) {

    switch(e_type) {
      // Case 1 - sHeader.nType 0 indicates an error
      case CRTPacket::PacketError:
        ROS_ERROR_STREAM_THROTTLE(
            1, "Error when streaming frames: "
            << port_protocol.GetRTPacket()->GetErrorString());
        break;

      // Case 2 - No more data
      case CRTPacket::PacketNoMoreData:
        ROS_WARN_STREAM_THROTTLE(1, "No more data");
        break;

      // Case 3 - Data received
      case CRTPacket::PacketData:
        handlePacketData(prt_packet);
        break;

      default:
        ROS_ERROR_THROTTLE(1, "Unknown CRTPacket case");
    }
  }

  return;
}

}


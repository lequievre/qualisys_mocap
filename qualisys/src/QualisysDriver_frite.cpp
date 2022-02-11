#include <qualisys/QualisysDriver_frite.h>
#include <algorithm>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

using namespace std;

namespace qualisys{

double QualisysDriverFrite::deg2rad = M_PI / 180.0;

QualisysDriverFrite::QualisysDriverFrite(const ros::NodeHandle& n):
  nh(n),
  publish_tf(false){
  return;
}

bool QualisysDriverFrite::init() {
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
  
  pose_array_publisher = nh.advertise<geometry_msgs::PoseArray>("/PoseMarkers", 10);
  marker_line_strip_publisher = nh.advertise<visualization_msgs::Marker>("/VisualizationLineStripMarkers", 10);
  marker_points_publisher = nh.advertise<visualization_msgs::Marker>("/VisualizationPointsMarkers", 10);
  marker_pose_array_publisher = nh.advertise<visualization_msgs::MarkerArray>("/VisualizationPoseArrayMarkers", 10);
  
  return true;
}

void QualisysDriverFrite::disconnect() {
  ROS_INFO_STREAM("Disconnected with the server "
      << server_address << ":" << base_port);
  port_protocol.StreamFramesStop();
  port_protocol.Disconnect();
  return;
}

void QualisysDriverFrite::checkPublishers(const int& body_count) {
  map<string, bool> subject_indicator;

  for (auto it = subject_publishers.begin();
      it != subject_publishers.end(); ++it)
    subject_indicator[it->first] = false;

  // Check publishers for each body
  for(int i = 0; i < body_count; ++i) {
    //std::stringstream name;
    //name << port_protocol.Get6DOFBodyName(i);
    string name(port_protocol.Get6DOFBodyName(i));

    // Create a publisher for the rigid body
    // if it does not have one.
    if (subject_publishers.find(name) ==
          subject_publishers.end())
      subject_publishers[name] =
        nh.advertise<qualisys::Subject>(name, 10);

    subject_indicator[name] = true;
  }

  for (auto it = subject_indicator.begin();
      it != subject_indicator.end(); ++it) {
    if (it->second == false)
      subject_publishers.erase(it->first);
  }

  return;
}

void QualisysDriverFrite::handlePacketData(CRTPacket* prt_packet) {

  // Number of rigid bodies
  int body_count = prt_packet->Get6DOFEulerBodyCount();
  
  
  geometry_msgs::PoseArray  pose_array_msg;
  geometry_msgs::Pose pose_msg;
  
  visualization_msgs::Marker line_strip_msg, points_msg, simple_marker_msg;
  
  visualization_msgs::MarkerArray  marker_array_msg;
  
  
  
  // Check the publishers for the rigid bodies
  checkPublishers(body_count);
  
  unsigned nbMarkers;
  
  //CRTProtocol::SPoint sPoint;
  
  pose_array_msg.header.stamp = ros::Time::now(); // timestamp of creation of the msg
  pose_array_msg.header.frame_id = "map"; // frame id in which the array is published
  
  line_strip_msg.header.frame_id = "map";
  line_strip_msg.header.stamp = ros::Time::now();
  line_strip_msg.id = 1;
  line_strip_msg.ns = "points_and_lines";
  line_strip_msg.action = visualization_msgs::Marker::ADD;
  line_strip_msg.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip_msg.scale.x = 0.01;
  line_strip_msg.scale.y = 0.01;
  line_strip_msg.scale.z = 0.01;
  // Line strip is blue
  line_strip_msg.color.b = 1.0;
  line_strip_msg.color.a = 1.0;
  
  line_strip_msg.pose.orientation.w = 1.0;
  
  
  points_msg.header.frame_id = "map";
  points_msg.header.stamp = ros::Time::now();
  points_msg.id = 0;
  points_msg.ns = "points_and_lines";
  points_msg.action = visualization_msgs::Marker::ADD;
  
  points_msg.type = visualization_msgs::Marker::POINTS;
  points_msg.scale.x = 0.02;
  points_msg.scale.y = 0.02;
  points_msg.scale.z = 0.02;
  // Points are green
  points_msg.color.g = 1.0f;
  points_msg.color.a = 1.0;
  
  points_msg.pose.orientation.w = 1.0;
   
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
  
  
  
  geometry_msgs::Point point_msg;
  

  // Publish data for each rigid body
  for (int i = 0; i < body_count; ++i) {
    float x, y, z, roll, pitch, yaw;
    prt_packet->Get6DOFEulerBody(i, x, y, z, roll, pitch, yaw);

    if(isnan(x) || isnan(y) || isnan(z) ||
        isnan(roll) || isnan(pitch) || isnan(yaw)) {
      ROS_WARN_STREAM_THROTTLE(3, "Rigid-body " << i + 1 << "/"
          << body_count << " not detected");
      continue;
    }

    // ROTATION: GLOBAL (FIXED) X Y Z (R P Y)
    //std::stringstream name;
    //name << port_protocol.Get6DOFBodyName(i);
    string subject_name(port_protocol.Get6DOFBodyName(i));

    // Qualisys sometimes flips 180 degrees around the x axis
    if(roll > 90)
      roll -= 180;
    else if(roll < -90)
      roll += 180;

    // Send transform
    tf::StampedTransform stamped_transform = tf::StampedTransform(
        tf::Transform(
            tf::createQuaternionFromRPY(
                roll * deg2rad, pitch * deg2rad, yaw * deg2rad),
            tf::Vector3(x, y, z) / 1000.),
        ros::Time::now(), "qualisys", subject_name);
    if (publish_tf)
      tf_publisher.sendTransform(stamped_transform);
      
      
    // Send Subject msg
    geometry_msgs::TransformStamped geom_stamped_transform;
    tf::transformStampedTFToMsg(stamped_transform,
        geom_stamped_transform);

    qualisys::Subject subject_msg;
    
    subject_msg.header =
      geom_stamped_transform.header;
    subject_msg.name = subject_name;
    subject_msg.position.x =
        geom_stamped_transform.transform.translation.x;
    subject_msg.position.y =
        geom_stamped_transform.transform.translation.y;
    subject_msg.position.z =
        geom_stamped_transform.transform.translation.z;
    subject_msg.orientation =
        geom_stamped_transform.transform.rotation;
        
    //ROS_INFO("%s :  Point: X = %9f  Y = %9f  Z = %9f\n", subject_msg.name.c_str(), subject_msg.position.x, subject_msg.position.y, subject_msg.position.z);
     
	pose_msg.position.x = geom_stamped_transform.transform.translation.x;
	pose_msg.position.y = geom_stamped_transform.transform.translation.y;
	pose_msg.position.z = geom_stamped_transform.transform.translation.z;
	
	pose_msg.orientation = geom_stamped_transform.transform.rotation;
	// push in array (in C++ a vector, in python a list)
	pose_array_msg.poses.push_back(pose_msg); 
	
	
	point_msg.x = geom_stamped_transform.transform.translation.x;
	point_msg.y = geom_stamped_transform.transform.translation.y;
	point_msg.z = geom_stamped_transform.transform.translation.z;
	
	line_strip_msg.points.push_back(point_msg);
	points_msg.points.push_back(point_msg);
	
	simple_marker_msg.id = 2 + i;
	simple_marker_msg.pose.position.x = geom_stamped_transform.transform.translation.x;
	simple_marker_msg.pose.position.y = geom_stamped_transform.transform.translation.y;
	simple_marker_msg.pose.position.z = geom_stamped_transform.transform.translation.z;
	simple_marker_msg.pose.orientation = geom_stamped_transform.transform.rotation;
	
	marker_array_msg.markers.push_back(simple_marker_msg);
	
    subject_publishers[subject_name].publish(subject_msg);
  }

  pose_array_publisher.publish(pose_array_msg);
  marker_line_strip_publisher.publish(line_strip_msg);
  marker_points_publisher.publish(points_msg);
  marker_pose_array_publisher.publish(marker_array_msg);
  
  return;
}

void QualisysDriverFrite::run() {

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


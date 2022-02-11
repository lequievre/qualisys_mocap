#include <qualisys/QualisysDriver3D.h>
#include <algorithm>

using namespace std;

namespace qualisys{

double QualisysDriver3D::deg2rad = M_PI / 180.0;

QualisysDriver3D::QualisysDriver3D(const ros::NodeHandle& n):
  nh(n),
  publish_tf(false){
  return;
}

bool QualisysDriver3D::init() {
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
  //port_protocol.Read6DOFSettings();
  port_protocol.Read3DSettings();
  
  subject_publisher = nh.advertise<qualisys::Subject>("/3DMarkers", 10);
  
  //port_protocol.StreamFrames(CRTProtocol::RateAllFrames, 0, 0, NULL, CRTProtocol:: ComponentAll);

  return true;
}

void QualisysDriver3D::disconnect() {
  ROS_INFO_STREAM("Disconnected with the server "
      << server_address << ":" << base_port);
  port_protocol.StreamFramesStop();
  port_protocol.Disconnect();
  return;
}



void QualisysDriver3D::handlePacketData(CRTPacket* prt_packet) {


  
 int nbPacketMarkers = prt_packet->Get3DMarkerCount();
    //ROS_INFO(" Nb PACKET Markers = %d",nbPacketMarkers);
 
 qualisys::Subject subject_msg;
 
 
 subject_msg.markers.resize(nbPacketMarkers);
 subject_msg.header.stamp = ros::Time::now();
            
  float m_x,m_y,m_z;   
  
  for (int j = 0; j < nbPacketMarkers; ++j) {    
      prt_packet->Get3DMarker(j, m_x, m_y, m_z);
      subject_msg.markers[j].position.x = m_x;
      subject_msg.markers[j].position.y = m_y;
      subject_msg.markers[j].position.z = m_z;
      subject_msg.markers[j].name = port_protocol.Get3DLabelName(j);
      
      
      ROS_INFO("%s :  Point: X = %9f  Y = %9f  Z = %9f\n", subject_msg.markers[j].name.c_str(), subject_msg.markers[j].position.x, subject_msg.markers[j].position.y, subject_msg.markers[j].position.z);
      
      //ROS_INFO("label=%s, m_x=%f, m_y=%f, m_z=%f",port_protocol.Get3DLabelName(j),m_x, m_y, m_z);
      
  }
   
    subject_publisher.publish(subject_msg);

  return;
}

void QualisysDriver3D::run() {

  CRTPacket* prt_packet = port_protocol.GetRTPacket();
  CRTPacket::EPacketType e_type;
  port_protocol.GetCurrentFrame(CRTProtocol::Component3d);

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


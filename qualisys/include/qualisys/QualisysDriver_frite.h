#ifndef QUALISYS_DRIVER_FRITE_H
#define QUALISYS_DRIVER_FRITE_H

#include <sstream>
#include <cmath>
#include <string>

// Including ROS
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <qualisys/Subject.h>
#include <qualisys/RTProtocol.h>


namespace qualisys{

class QualisysDriverFrite{

  public:
    /*
     * @brief Constructor
     * @param nh Ros node
     */
    QualisysDriverFrite(const ros::NodeHandle& n);

    /*
     * @brief Destructor
     */
    ~QualisysDriverFrite() {
      disconnect();
    }

    /*
     * @brief init Initialize the object
     * @return True if successfully initialized
     */
    bool init();

    /*
     * @brief run Start acquiring data from the server
     */
    void run();

    /*
     * @brief disconnect Disconnect to the server
     * @Note The function is called automatically when the
     *  destructor is called.
     */
    void disconnect();

  private:
    // Disable the copy constructor and assign operator
    QualisysDriverFrite(const QualisysDriverFrite& );
    QualisysDriverFrite& operator=(const QualisysDriverFrite& );

    // Initialize publishers
    void checkPublishers(const int& body_count);

    // Handle data packet
    void handlePacketData(CRTPacket* prt_packet);

    // Unit converter
    static double deg2rad;

    // Address and port of the server
    std::string server_address;
    int base_port;

    // Protocol to connect to the server
    CRTProtocol port_protocol;

    // Ros related
    ros::NodeHandle nh;

    // Publishers
    std::map<std::string, ros::Publisher> subject_publishers;
    tf::TransformBroadcaster tf_publisher;
    
    ros::Publisher pose_array_publisher;
    ros::Publisher marker_line_strip_publisher;
    
    ros::Publisher marker_points_publisher;
    ros::Publisher marker_pose_array_publisher;
    

    // If publish tf msgs
    bool publish_tf;


};
}


#endif

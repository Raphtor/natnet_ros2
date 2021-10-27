#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>


#include <unistd.h>
#include <termios.h>


#include <vector>

#include <chrono>
#include <functional>
#include <algorithm>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "NatNetTypes.h"
#include "NatNetCAPI.h"
#include "NatNetClient.h"
#include <stdexcept>
typedef geometry_msgs::msg::Pose posemsg_t;
typedef geometry_msgs::msg::PoseStamped sposemsg_t;

typedef rclcpp::Publisher<posemsg_t>::SharedPtr posepub_t;
typedef rclcpp::Publisher<sposemsg_t>::SharedPtr sposepub_t;

class NatNetRosClient : public rclcpp::Node
{
    public: 
        // Methods
        NatNetRosClient();
        virtual ~NatNetRosClient();
        void Init();
        void resetClient();
        int ConnectClient();
        NatNetDiscoveryHandle autoDiscoverServer();
        int DiscoverRigidBodies();
        void set_data(sRigidBodyData);

        // ROS Interfaces
        rclcpp::TimerBase::SharedPtr timer_;
        // Pose publisher
        std::map<u_int32_t, posepub_t> publishers_;
        std::map<u_int32_t, posemsg_t> data_;
        // StampedPose publisher
        std::map<u_int32_t, sposepub_t> spublishers_;
        std::map<u_int32_t, sposemsg_t> sdata_;


        
        // ROS parameters
        bool parameter_use_timestamps_;
        std::string parameter_world_frame_id_;
        float parameter_timeout_;

        // Connection parameters
        NatNetClient* g_pClient = NULL;
        static const ConnectionType kDefaultConnectionType = ConnectionType_Multicast;
        sServerDescription g_serverDescription;
        std::map<u_int32_t, std::string> markernames;
        std::vector< sNatNetDiscoveredServer > g_discoveredServers;
        sNatNetClientConnectParams g_connectParams;
        char g_discoveredMulticastGroupAddr[kNatNetIpv4AddrStrLenMax] = NATNET_DEFAULT_MULTICAST_ADDRESS;
        int g_analogSamplesPerMocapFrame = 0;
        
    
        
        
};
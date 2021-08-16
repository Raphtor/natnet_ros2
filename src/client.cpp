/* 
Copyright © 2012 NaturalPoint Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. */


/*

SampleClient.cpp

This program connects to a NatNet server, receives a data stream, and writes that data stream
to an ascii file.  The purpose is to illustrate using the NatNetClient class.

Usage [optional]:

	SampleClient [ServerIP] [LocalIP] [OutputFilename]

	[ServerIP]			IP address of the server (e.g. 192.168.0.107) ( defaults to local machine)
	[OutputFilename]	Name of points file (pts) to write out.  defaults to Client-output.pts

*/

#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>


#include <unistd.h>
#include <termios.h>


#include <vector>

#include <chrono>
#include <string>
#include <functional>
#include <math.h>

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "../include/natnet/NatNetTypes.h"
#include "../include/natnet/NatNetCAPI.h"
#include "../include/natnet/NatNetClient.h"
#include <memory>
#include <string>
#include <stdexcept>
using namespace std::chrono_literals;
// The code snippet below is licensed under CC0 1.0. see https://stackoverflow.com/questions/2342162/stdstring-formatting-like-sprintf
template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
    int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
    auto size = static_cast<size_t>( size_s );
    auto buf = std::make_unique<char[]>( size );
    std::snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}
// End licensed code

typedef geometry_msgs::msg::Pose posemsg_t;
typedef geometry_msgs::msg::PoseStamped sposemsg_t;

typedef rclcpp::Publisher<posemsg_t>::SharedPtr posepub_t;
typedef rclcpp::Publisher<sposemsg_t>::SharedPtr sposepub_t;

class NatNetRosClient : public rclcpp::Node
{
    public: 
        NatNetRosClient();
        
        virtual ~NatNetRosClient();
        void Init();
        void resetClient();
        int ConnectClient();
        
        NatNetDiscoveryHandle autoDiscoverServer();
        int DiscoverRigidBodies();

        std::map<u_int32_t, posepub_t> publishers_;
        std::map<u_int32_t, posemsg_t> data_;

        std::map<u_int32_t, sposepub_t> spublishers_;
        std::map<u_int32_t, sposemsg_t> sdata_;


        sServerDescription g_serverDescription;
        std::map<u_int32_t, std::string> markernames;
        bool parameter_use_timestamps_;
        std::string parameter_world_frame_id_;
        float parameter_timeout_;
        // void NATNET_CALLCONV ServerDiscoveredCallback( const sNatNetDiscoveredServer* pDiscoveredServer, void* pUserContext );
        // void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData);    // receives data from the server
        // void NATNET_CALLCONV MessageHandler(Verbosity msgType, const char* msg);      // receives NatNet error messages
        
        

        static const ConnectionType kDefaultConnectionType = ConnectionType_Multicast;

        NatNetClient* g_pClient = NULL;
        FILE* g_outputFile;

        std::vector< sNatNetDiscoveredServer > g_discoveredServers;
        sNatNetClientConnectParams g_connectParams;
        char g_discoveredMulticastGroupAddr[kNatNetIpv4AddrStrLenMax] = NATNET_DEFAULT_MULTICAST_ADDRESS;
        int g_analogSamplesPerMocapFrame = 0;
        void set_data(sRigidBodyData);
        rclcpp::TimerBase::SharedPtr timer_;
        
};

// I hate this, but callbacks need to be function pointers, so this is my temp solution


// Global pointer to node
std::shared_ptr<NatNetRosClient> node;
void NATNET_CALLCONV MessageHandler(Verbosity msgType, const char* msg )
// MessageHandler receives NatNet error/debug messages
{
    rclcpp::Logger logger = node->get_logger();
    switch ( msgType )
    {
        case Verbosity_Debug:
            RCLCPP_DEBUG(logger, "%s\n", msg);
            break;
        case Verbosity_Info:
            RCLCPP_INFO(logger, "%s\n", msg);
            break;
        case Verbosity_Warning:
            RCLCPP_WARN(logger, "%s\n", msg);
            break;
        case Verbosity_Error:
            RCLCPP_ERROR(logger, "%s\n", msg);
            break;
        default:
            RCLCPP_ERROR(logger, "???", msg);
            break;
    }

    
}
void NATNET_CALLCONV ServerDiscoveredCallback( const sNatNetDiscoveredServer* pDiscoveredServer, void* pUserContext )
{
    rclcpp::Logger logger = node->get_logger();
    char serverHotkey = '.';
    if ( node->g_discoveredServers.size() < 9 )
    {char serverHotkey = static_cast<char>('1' + node->g_discoveredServers.size());
    }

    RCLCPP_DEBUG(logger, "[%c] %s %d.%d at %s ",
        serverHotkey,
        pDiscoveredServer->serverDescription.szHostApp,
        pDiscoveredServer->serverDescription.HostAppVersion[0],
        pDiscoveredServer->serverDescription.HostAppVersion[1],
        pDiscoveredServer->serverAddress );

    if ( pDiscoveredServer->serverDescription.bConnectionInfoValid )
    {
        RCLCPP_DEBUG(logger, "(%s)\n", pDiscoveredServer->serverDescription.ConnectionMulticast ? "multicast" : "unicast" );
    }
    else
    {
        RCLCPP_DEBUG(logger, "(WARNING: Legacy server, could not autodetect settings. Auto-connect may not work reliably.)\n" );
    }

    node->g_discoveredServers.push_back( *pDiscoveredServer );
}
// DataHandler receives data from the server
// This function is called by NatNet when a frame of mocap data is available
void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData)
{
    rclcpp::Logger logger = node->get_logger();
    rclcpp::Clock clock = *node->get_clock();
    NatNetClient* pClient = (NatNetClient*) pUserData;

    // Software latency here is defined as the span of time between:
    //   a) The reception of a complete group of 2D frames from the camera system (CameraDataReceivedTimestamp)
    // and
    //   b) The time immediately prior to the NatNet frame being transmitted over the network (TransmitTimestamp)
    //
    // This figure may appear slightly higher than the "software latency" reported in the Motive user interface,
    // because it additionally includes the time spent preparing to stream the data via NatNet.
    const uint64_t softwareLatencyHostTicks = data->TransmitTimestamp - data->CameraDataReceivedTimestamp;
    const double softwareLatencyMillisec = (softwareLatencyHostTicks * 1000) / static_cast<double>(node->g_serverDescription.HighResClockFrequency);

    // Transit latency is defined as the span of time between Motive transmitting the frame of data, and its reception by the client (now).
    // The SecondsSinceHostTimestamp method relies on NatNetClient's internal clock synchronization with the server using Cristian's algorithm.
    const double transitLatencyMillisec = pClient->SecondsSinceHostTimestamp( data->TransmitTimestamp ) * 1000.0;

    
    std::stringstream debug_msg;
    int i=0;

    debug_msg << " " << string_format("FrameID : %d\n", data->iFrame);
    debug_msg << " " << string_format("Timestamp : %3.2lf\n", data->fTimestamp);
    debug_msg << " " << string_format("Software latency : %.2lf milliseconds\n", softwareLatencyMillisec);

    // Only recent versions of the Motive software in combination with ethernet camera systems support system latency measurement.
    // If it's unavailable (for example, with USB camera systems, or during playback), this field will be zero.
    const bool bSystemLatencyAvailable = data->CameraMidExposureTimestamp != 0;

    if ( bSystemLatencyAvailable )
    {
        // System latency here is defined as the span of time between:
        //   a) The midpoint of the camera exposure window, and therefore the average age of the photons (CameraMidExposureTimestamp)
        // and
        //   b) The time immediately prior to the NatNet frame being transmitted over the network (TransmitTimestamp)
        const uint64_t systemLatencyHostTicks = data->TransmitTimestamp - data->CameraMidExposureTimestamp;
        const double systemLatencyMillisec = (systemLatencyHostTicks * 1000) / static_cast<double>(node->g_serverDescription.HighResClockFrequency);

        // Client latency is defined as the sum of system latency and the transit time taken to relay the data to the NatNet client.
        // This is the all-inclusive measurement (photons to client processing).
        const double clientLatencyMillisec = pClient->SecondsSinceHostTimestamp( data->CameraMidExposureTimestamp ) * 1000.0;

        // You could equivalently do the following (not accounting for time elapsed since we calculated transit latency above):
        //const double clientLatencyMillisec = systemLatencyMillisec + transitLatencyMillisec;

        debug_msg << " " << string_format("System latency : %.2lf milliseconds\n", systemLatencyMillisec );
        debug_msg << " " << string_format("Total client latency : %.2lf milliseconds (transit time +%.2lf ms)\n", clientLatencyMillisec, transitLatencyMillisec );
    }
    else
    {
        debug_msg << " " << string_format("Transit latency : %.2lf milliseconds\n", transitLatencyMillisec );
    }

    // FrameOfMocapData params
    bool bIsRecording = ((data->params & 0x01)!=0);
    bool bTrackedModelsChanged = ((data->params & 0x02)!=0);
    if(bIsRecording)
        RCLCPP_DEBUG(logger,"RECORDING\n");
    if(bTrackedModelsChanged)
        RCLCPP_DEBUG(logger,"Models Changed.\n");

    // timecode - for systems with an eSync and SMPTE timecode generator - decode to values
    int hour, minute, second, frame, subframe;
    NatNet_DecodeTimecode( data->Timecode, data->TimecodeSubframe, &hour, &minute, &second, &frame, &subframe );
    // decode to friendly string
    char szTimecode[128] = "";
    NatNet_TimecodeStringify( data->Timecode, data->TimecodeSubframe, szTimecode, 128 );
    debug_msg << " " << string_format("Timecode : %s\n", szTimecode);

    // Rigid Bodies
    debug_msg << " " << string_format("Rigid Bodies [Count=%d]\n", data->nRigidBodies);
    for(i=0; i < data->nRigidBodies; i++)
    {
        // params
        // 0x01 : bool, rigid body was successfully tracked in this frame
        bool bTrackingValid = data->RigidBodies[i].params & 0x01;
        u_int32_t ID = data->RigidBodies[i].ID;
        if (node->markernames.find(ID) != node->markernames.end()){

        
            posemsg_t msg = node->data_[ID];
            sRigidBodyData dat = data->RigidBodies[i];
            
            msg.position.x = dat.x;
            msg.position.y = dat.y;
            msg.position.z = dat.z;
            msg.orientation.x = dat.qx;
            msg.orientation.y = dat.qy;
            msg.orientation.z = dat.qz;
            msg.orientation.w = dat.qw;
            if (node->parameter_use_timestamps_){
                sposemsg_t stamped_msg = node->sdata_[ID];
                stamped_msg.pose = msg;
                stamped_msg.header.frame_id = node->parameter_world_frame_id_;
                uint32_t secs = (uint32_t) floor(data->fTimestamp);
                stamped_msg.header.stamp.sec = secs;
                stamped_msg.header.stamp.nanosec = secs - (uint32_t) floor(data->fTimestamp * 1000000000.0);
                node->spublishers_[ID]->publish(stamped_msg);
            }
            else
            {
                node->publishers_[ID]->publish(msg);
            }
        }
        else
        {

            RCLCPP_WARN_THROTTLE(logger, clock, 1, "Rigidbody %d does not have a publisher: ", ID);
        }
        // RCLCPP_DEBUG(logger,"Rigid Body [Name=%s, ID=%d  Error=%3.2f  Valid=%d]\n", node->markernames[ID], ID, dat.MeanError, bTrackingValid);
        // RCLCPP_DEBUG(logger,"\tx\ty\tz\tqx\tqy\tqz\tqw\n");
        // RCLCPP_DEBUG(logger,"\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
        //     data->RigidBodies[i].x,
        //     data->RigidBodies[i].y,
        //     data->RigidBodies[i].z,
        //     data->RigidBodies[i].qx,
        //     data->RigidBodies[i].qy,
        //     data->RigidBodies[i].qz,
        //     data->RigidBodies[i].qw);
    }

    // Skeletons
    debug_msg << " " << string_format("Skeletons [Count=%d]\n", data->nSkeletons);
    // TODO do something with this
    // for(i=0; i < data->nSkeletons; i++)
    // {
    //     sSkeletonData skData = data->Skeletons[i];
    //     RCLCPP_DEBUG(logger,"Skeleton [ID=%d  Bone count=%d]\n", skData.skeletonID, skData.nRigidBodies);
    //     for(int j=0; j< skData.nRigidBodies; j++)
    //     {
    //         sRigidBodyData rbData = skData.RigidBodyData[j];
    //         RCLCPP_DEBUG(logger,"Bone %d\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
    //             rbData.ID, rbData.x, rbData.y, rbData.z, rbData.qx, rbData.qy, rbData.qz, rbData.qw );
    //     }
    // }

    // labeled markers - this includes all markers (Active, Passive, and 'unlabeled' (markers with no asset but a PointCloud ID)
    // bool bOccluded;     // marker was not visible (occluded) in this frame
    // bool bPCSolved;     // reported position provided by point cloud solve
    // bool bModelSolved;  // reported position provided by model solve
    // bool bHasModel;     // marker has an associated asset in the data stream
    // bool bUnlabeled;    // marker is 'unlabeled', but has a point cloud ID that matches Motive PointCloud ID (In Motive 3D View)
    // bool bActiveMarker; // marker is an actively labeled LED marker

    debug_msg << " " << string_format("Markers [Count=%d]\n", data->nLabeledMarkers);
    // TODO do something with this
    // for(i=0; i < data->nLabeledMarkers; i++)
    // {
    //     // Should this be a pointcloud??
    //     bOccluded = ((data->LabeledMarkers[i].params & 0x01)!=0);
    //     bPCSolved = ((data->LabeledMarkers[i].params & 0x02)!=0);
    //     bModelSolved = ((data->LabeledMarkers[i].params & 0x04) != 0);
    //     bHasModel = ((data->LabeledMarkers[i].params & 0x08) != 0);
    //     bUnlabeled = ((data->LabeledMarkers[i].params & 0x10) != 0);
    //     bActiveMarker = ((data->LabeledMarkers[i].params & 0x20) != 0);

    //     sMarker marker = data->LabeledMarkers[i];

    //     // Marker ID Scheme:
    //     // Active Markers:
    //     //   ID = ActiveID, correlates to RB ActiveLabels list
    //     // Passive Markers: 
    //     //   If Asset with Legacy Labels
    //     //      AssetID 	(Hi Word)
    //     //      MemberID	(Lo Word)
    //     //   Else
    //     //      PointCloud ID
    //     int modelID, markerID;
    //     NatNet_DecodeID( marker.ID, &modelID, &markerID );
        
    //     char szMarkerType[512];
    //     if (bActiveMarker)
    //         strcpy(szMarkerType, "Active");
    //     else if(bUnlabeled)
    //         strcpy(szMarkerType, "Unlabeled");
    //     else
    //         strcpy(szMarkerType, "Labeled");

    //     // RCLCPP_DEBUG(logger,"%s Marker [ModelID=%d, MarkerID=%d] [size=%3.2f] [pos=%3.2f,%3.2f,%3.2f]\n",
    //     //     szMarkerType, modelID, markerID, marker.size, marker.x, marker.y, marker.z);
    // }

    // force plates
    debug_msg << " " << string_format("Force Plate [Count=%d]\n", data->nForcePlates);
    // TODO do something with this
    // for(int iPlate=0; iPlate < data->nForcePlates; iPlate++)
    // {
    //     RCLCPP_DEBUG(logger,"Force Plate %d\n", data->ForcePlates[iPlate].ID);
    //     for(int iChannel=0; iChannel < data->ForcePlates[iPlate].nChannels; iChannel++)
    //     {
    //         RCLCPP_DEBUG(logger,"\tChannel %d:\t", iChannel);
    //         if(data->ForcePlates[iPlate].ChannelData[iChannel].nFrames == 0)
    //         {
    //             RCLCPP_DEBUG(logger,"\tEmpty Frame\n");
    //         }
    //         else if(data->ForcePlates[iPlate].ChannelData[iChannel].nFrames != node->g_analogSamplesPerMocapFrame)
    //         {
    //             RCLCPP_DEBUG(logger,"\tPartial Frame [Expected:%d   Actual:%d]\n", node->g_analogSamplesPerMocapFrame, data->ForcePlates[iPlate].ChannelData[iChannel].nFrames);
    //         }
    //         for(int iSample=0; iSample < data->ForcePlates[iPlate].ChannelData[iChannel].nFrames; iSample++)
    //             RCLCPP_DEBUG(logger,"%3.2f\t", data->ForcePlates[iPlate].ChannelData[iChannel].Values[iSample]);
    //         RCLCPP_DEBUG(logger,"\n");
    //     }
    // }

    // devices
    debug_msg << " " << string_format("Device [Count=%d]\n", data->nDevices);
    // TODO do something with this
    // for (int iDevice = 0; iDevice < data->nDevices; iDevice++)
    // {
    //     RCLCPP_DEBUG(logger,"Device %d\n", data->Devices[iDevice].ID);
    //     for (int iChannel = 0; iChannel < data->Devices[iDevice].nChannels; iChannel++)
    //     {
    //         RCLCPP_DEBUG(logger,"\tChannel %d:\t", iChannel);
    //         if (data->Devices[iDevice].ChannelData[iChannel].nFrames == 0)
    //         {
    //             RCLCPP_DEBUG(logger,"\tEmpty Frame\n");
    //         }
    //         else if (data->Devices[iDevice].ChannelData[iChannel].nFrames != node->g_analogSamplesPerMocapFrame)
    //         {
    //             RCLCPP_DEBUG(logger,"\tPartial Frame [Expected:%d   Actual:%d]\n",node-> g_analogSamplesPerMocapFrame, data->Devices[iDevice].ChannelData[iChannel].nFrames);
    //         }
    //         for (int iSample = 0; iSample < data->Devices[iDevice].ChannelData[iChannel].nFrames; iSample++)
    //             RCLCPP_DEBUG(logger,"%3.2f\t", data->Devices[iDevice].ChannelData[iChannel].Values[iSample]);
    //         RCLCPP_DEBUG(logger,"\n");
    //     }
    // }
    RCLCPP_DEBUG(logger, debug_msg.str());
}



NatNetRosClient::NatNetRosClient() : Node("natnet_client")
    {

        this->declare_parameter<std::string>("server_address", "");
        this->declare_parameter<std::string>("local_address", "");
        this->declare_parameter<float>("discovery_timeout", 30.0f);
        this->declare_parameter<bool>("use_timestamps", true);
        this->declare_parameter<std::string>("world_frame_id", "world");
    }
int NatNetRosClient::DiscoverRigidBodies(){
    if (g_pClient == NULL){
        return 1;
    }
    sDataDescriptions* pDataDefs = NULL;
    parameter_world_frame_id_ = this->get_parameter("world_frame_id").as_string();
    int iResult = g_pClient->GetDataDescriptionList(&pDataDefs);
    if (iResult != ErrorCode_OK || pDataDefs == NULL)
    {
        RCLCPP_ERROR(this->get_logger()," Unable to retrieve Data Descriptions.\n");
        return 1;
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(),"Received %d Data Descriptions:\n", pDataDefs->nDataDescriptions );
        for(int i=0; i < pDataDefs->nDataDescriptions; i++)
        {
            // RCLCPP_DEBUG(this->get_logger(),"Data Description # %d (type=%d)\n", i, pDataDefs->arrDataDescriptions[i].type);
            if(pDataDefs->arrDataDescriptions[i].type == Descriptor_MarkerSet)
            {
                // TODO do something with this
                // // MarkerSet
                // sMarkerSetDescription* pMS = pDataDefs->arrDataDescriptions[i].Data.MarkerSetDescription;
                // RCLCPP_DEBUG(this->get_logger(),"MarkerSet Name : %s\n", pMS->szName);
                // for(int i=0; i < pMS->nMarkers; i++)
                //     RCLCPP_DEBUG(this->get_logger(),"%s\n", pMS->szMarkerNames[i]);

            }
            else if(pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody)
            {

                

                // RigidBody
                sRigidBodyDescription* pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
                int32_t ID = pRB->ID;
                markernames[ID]= pRB->szName;
                if (parameter_use_timestamps_)
                {
                    std::map<uint32_t, sposepub_t>::iterator it = spublishers_.find(ID);
                    if ( spublishers_.end() == it ) { 
                        std::string topicname = string_format("%s/pose_stamped", pRB->szName);
                        spublishers_[ID] = this->create_publisher<sposemsg_t>(topicname, 10);
                    }
                }
                else
                {
                    std::map<uint32_t, posepub_t>::iterator it = publishers_.find(ID);
                    if ( publishers_.end() == it ) { 
                        std::string topicname = string_format("%s/pose", pRB->szName);
                        publishers_[ID] = this->create_publisher<posemsg_t>(topicname, 10);
                    }
                }
                
                RCLCPP_DEBUG(this->get_logger(),"RigidBody Name : %s\n", pRB->szName);
                RCLCPP_DEBUG(this->get_logger(),"RigidBody ID : %d\n", pRB->ID);
                RCLCPP_DEBUG(this->get_logger(),"RigidBody Parent ID : %d\n", pRB->parentID);
                RCLCPP_DEBUG(this->get_logger(),"Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);

                if ( pRB->MarkerPositions != NULL && pRB->MarkerRequiredLabels != NULL )
                {
                    for ( int markerIdx = 0; markerIdx < pRB->nMarkers; ++markerIdx )
                    {
                        const MarkerData& markerPosition = pRB->MarkerPositions[markerIdx];
                        const int markerRequiredLabel = pRB->MarkerRequiredLabels[markerIdx];

                        RCLCPP_DEBUG(this->get_logger(), "\tMarker #%d:\n", markerIdx );
                        RCLCPP_DEBUG(this->get_logger(), "\t\tPosition: %.2f, %.2f, %.2f\n", markerPosition[0], markerPosition[1], markerPosition[2] );

                        if ( markerRequiredLabel != 0 )
                        {
                            RCLCPP_DEBUG(this->get_logger(), "\t\tRequired active label: %d\n", markerRequiredLabel );
                        }
                    }
                }
            }
            else if(pDataDefs->arrDataDescriptions[i].type == Descriptor_Skeleton)
            {
                // Skeleton
                sSkeletonDescription* pSK = pDataDefs->arrDataDescriptions[i].Data.SkeletonDescription;
                RCLCPP_DEBUG(this->get_logger(),"Skeleton Name : %s\n", pSK->szName);
                RCLCPP_DEBUG(this->get_logger(),"Skeleton ID : %d\n", pSK->skeletonID);
                RCLCPP_DEBUG(this->get_logger(),"RigidBody (Bone) Count : %d\n", pSK->nRigidBodies);
                for(int j=0; j < pSK->nRigidBodies; j++)
                {
                    sRigidBodyDescription* pRB = &pSK->RigidBodies[j];
                    RCLCPP_DEBUG(this->get_logger(),"  RigidBody Name : %s\n", pRB->szName);
                    RCLCPP_DEBUG(this->get_logger(),"  RigidBody ID : %d\n", pRB->ID);
                    RCLCPP_DEBUG(this->get_logger(),"  RigidBody Parent ID : %d\n", pRB->parentID);
                    RCLCPP_DEBUG(this->get_logger(),"  Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);
                }
            }
            else if(pDataDefs->arrDataDescriptions[i].type == Descriptor_ForcePlate)
            {
                // Force Plate
                sForcePlateDescription* pFP = pDataDefs->arrDataDescriptions[i].Data.ForcePlateDescription;
                RCLCPP_DEBUG(this->get_logger(),"Force Plate ID : %d\n", pFP->ID);
                RCLCPP_DEBUG(this->get_logger(),"Force Plate Serial : %s\n", pFP->strSerialNo);
                RCLCPP_DEBUG(this->get_logger(),"Force Plate Width : %3.2f\n", pFP->fWidth);
                RCLCPP_DEBUG(this->get_logger(),"Force Plate Length : %3.2f\n", pFP->fLength);
                RCLCPP_DEBUG(this->get_logger(),"Force Plate Electrical Center Offset (%3.3f, %3.3f, %3.3f)\n", pFP->fOriginX,pFP->fOriginY, pFP->fOriginZ);
                for(int iCorner=0; iCorner<4; iCorner++)
                    RCLCPP_DEBUG(this->get_logger(),"Force Plate Corner %d : (%3.4f, %3.4f, %3.4f)\n", iCorner, pFP->fCorners[iCorner][0],pFP->fCorners[iCorner][1],pFP->fCorners[iCorner][2]);
                RCLCPP_DEBUG(this->get_logger(),"Force Plate Type : %d\n", pFP->iPlateType);
                RCLCPP_DEBUG(this->get_logger(),"Force Plate Data Type : %d\n", pFP->iChannelDataType);
                RCLCPP_DEBUG(this->get_logger(),"Force Plate Channel Count : %d\n", pFP->nChannels);
                for(int iChannel=0; iChannel<pFP->nChannels; iChannel++)
                    RCLCPP_DEBUG(this->get_logger(),"\tChannel %d : %s\n", iChannel, pFP->szChannelNames[iChannel]);
            }
            else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Device)
            {
                // Peripheral Device
                sDeviceDescription* pDevice = pDataDefs->arrDataDescriptions[i].Data.DeviceDescription;
                RCLCPP_DEBUG(this->get_logger(),"Device Name : %s\n", pDevice->strName);
                RCLCPP_DEBUG(this->get_logger(),"Device Serial : %s\n", pDevice->strSerialNo);
                RCLCPP_DEBUG(this->get_logger(),"Device ID : %d\n", pDevice->ID);
                RCLCPP_DEBUG(this->get_logger(),"Device Channel Count : %d\n", pDevice->nChannels);
                for (int iChannel = 0; iChannel < pDevice->nChannels; iChannel++)
                    RCLCPP_DEBUG(this->get_logger(),"\tChannel %d : %s\n", iChannel, pDevice->szChannelNames[iChannel]);
            }
            else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Camera)
            {
                // Camera
                sCameraDescription* pCamera = pDataDefs->arrDataDescriptions[i].Data.CameraDescription;
                RCLCPP_DEBUG(this->get_logger(),"Camera Name : %s\n", pCamera->strName);
                RCLCPP_DEBUG(this->get_logger(),"Camera Position (%3.2f, %3.2f, %3.2f)\n", pCamera->x, pCamera->y, pCamera->z);
                RCLCPP_DEBUG(this->get_logger(),"Camera Orientation (%3.2f, %3.2f, %3.2f, %3.2f)\n", pCamera->qx, pCamera->qy, pCamera->qz, pCamera->qw);
            }
            else
            {
                RCLCPP_DEBUG(this->get_logger(),"Unknown data type.\n");
                // Unknown
            }
        }
        return 0;      
    }

}
void NatNetRosClient::Init(){
        // print version info
        unsigned char ver[4];
        NatNet_GetVersion( ver );
        RCLCPP_DEBUG(this->get_logger(), "NatNet Sample Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3] );

        // Install logging callback using global node
        

        // create NatNet client
        g_pClient = new NatNetClient();

        

        // If no arguments were specified on the command line,
        // attempt to discover servers on the local network.
        std::string server_address = this->get_parameter("server_address").as_string();
        std::string local_address = this->get_parameter("local_address").as_string();
        parameter_use_timestamps_ = this->get_parameter("use_timestamps").as_bool();
        if ( server_address.empty() && local_address.empty())
        {
            // Do asynchronous server discovery.
            RCLCPP_DEBUG(this->get_logger(), "Looking for servers on the local network.\n" );


            NatNetDiscoveryHandle discovery = autoDiscoverServer();
            this->set_parameter( rclcpp::Parameter("server_address", g_connectParams.serverAddress));
            this->set_parameter( rclcpp::Parameter("local_address", g_connectParams.localAddress));
        }
        else
        {

            g_connectParams.serverAddress = server_address.c_str();
            g_connectParams.localAddress = local_address.c_str();
            
            
        }

        int iResult;

        // Connect to Motive
        iResult = ConnectClient();
        if (iResult != ErrorCode_OK)
        {
            RCLCPP_DEBUG(this->get_logger(),"Error initializing client. See log for details. Exiting.\n");
            
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(),"Client initialized and ready.\n");
        }


        // Send/receive test request
        void* response;
        int nBytes;
        RCLCPP_DEBUG(this->get_logger()," Sending Test Request\n");
        iResult = g_pClient->SendMessageAndWait("TestRequest", &response, &nBytes);
        if (iResult == ErrorCode_OK)
        {
            RCLCPP_DEBUG(this->get_logger()," Received: %s\n", (char*)response);
        }

        // Retrieve Data Descriptions from Motive
        RCLCPP_DEBUG(this->get_logger(),"Requesting Data Descriptions...\n");
        iResult = DiscoverRigidBodies();
        if (iResult != ErrorCode_OK)
        {
            RCLCPP_ERROR(this->get_logger(),"Failed to properly discover bodies. Is client initialized?");
        }
        std::stringstream markerlist;
        markerlist << "Markers:";
        std::map<uint32_t, std::string>::iterator itr;
        for (itr=markernames.begin(); itr!=markernames.end(); ++itr){
            markerlist << " " << itr->second;
        }
        RCLCPP_INFO(this->get_logger(),"Found %d markers",markernames.size());
        RCLCPP_INFO(this->get_logger(),markerlist.str());
        // Start handling messages after everything is done
        NatNet_SetLogCallback(MessageHandler);
        // set the frame callback handler after initialization has been done.
        g_pClient->SetFrameReceivedCallback( DataHandler, g_pClient );	// this function will receive data from the server

        // Ready to receive marker stream!
        RCLCPP_DEBUG(this->get_logger(),"Client is connected to server and listening for data...\n");
        timer_ = this->create_wall_timer(500ms, std::bind(&NatNetRosClient::DiscoverRigidBodies, this));
    }

NatNetRosClient::~NatNetRosClient()
{
    // Done - clean up.
    if (g_pClient)
    {
        g_pClient->Disconnect();
        delete g_pClient;
        g_pClient = NULL;
    }
}

NatNetDiscoveryHandle NatNetRosClient::autoDiscoverServer()
{
    NatNetDiscoveryHandle discovery;
    NatNet_CreateAsyncServerDiscovery( &discovery, ServerDiscoveredCallback );
    double parameter_timeout_ = this->get_parameter("discovery_timeout").as_double();
    rclcpp::Duration elapsed = rclcpp::Duration(0,0);
    rclcpp::Time start = this->get_clock()->now();
    rclcpp::Time end;
    while (1)
    {
        end = this->get_clock()->now();
        elapsed = end - start;
        if ( elapsed.seconds() < parameter_timeout_)
        {
            // Use first server found
            const size_t serverIndex = 0;
            if ( serverIndex < g_discoveredServers.size() )
            {
                const sNatNetDiscoveredServer& discoveredServer = g_discoveredServers[serverIndex];

                if ( discoveredServer.serverDescription.bConnectionInfoValid )
                {
                    // Build the connection parameters.

                    RCLCPP_DEBUG(this->get_logger(),

                        g_discoveredMulticastGroupAddr, sizeof g_discoveredMulticastGroupAddr,
                        "%" PRIu8 ".%" PRIu8".%" PRIu8".%" PRIu8"",
                        discoveredServer.serverDescription.ConnectionMulticastAddress[0],
                        discoveredServer.serverDescription.ConnectionMulticastAddress[1],
                        discoveredServer.serverDescription.ConnectionMulticastAddress[2],
                        discoveredServer.serverDescription.ConnectionMulticastAddress[3]
                    );

                    g_connectParams.connectionType = discoveredServer.serverDescription.ConnectionMulticast ? ConnectionType_Multicast : ConnectionType_Unicast;
                    g_connectParams.serverCommandPort = discoveredServer.serverCommandPort;
                    g_connectParams.serverDataPort = discoveredServer.serverDescription.ConnectionDataPort;
                    g_connectParams.serverAddress = discoveredServer.serverAddress;
                    g_connectParams.localAddress = discoveredServer.localAddress;
                    g_connectParams.multicastAddress = g_discoveredMulticastGroupAddr;
                }
                else
                {
                    // We're missing some info because it's a legacy server.
                    // Guess the defaults and make a best effort attempt to connect.
                    g_connectParams.connectionType = kDefaultConnectionType;
                    g_connectParams.serverCommandPort = discoveredServer.serverCommandPort;
                    g_connectParams.serverDataPort = 0;
                    g_connectParams.serverAddress = discoveredServer.serverAddress;
                    g_connectParams.localAddress = discoveredServer.localAddress;
                    g_connectParams.multicastAddress = NULL;
                }

                break;
            }
        }
        else
        {   
            RCLCPP_ERROR(this->get_logger(), "Timed out while trying to discover server");
        }
        
}
NatNet_FreeAsyncServerDiscovery( discovery );
return discovery;
}
// Establish a NatNet Client connection
int NatNetRosClient::ConnectClient()
{
    // Release previous server
    g_pClient->Disconnect();

    // Init Client and connect to NatNet server
    int retCode = g_pClient->Connect( g_connectParams );
    if (retCode != ErrorCode_OK)
    {
        RCLCPP_DEBUG(this->get_logger(),"Unable to connect to server.  Error code: %d. Exiting.\n", retCode);
        return ErrorCode_Internal;
    }
    else
    {
        // connection succeeded

        void* pResult;
        int nBytes = 0;
        ErrorCode ret = ErrorCode_OK;

        // print server info
        memset( &g_serverDescription, 0, sizeof( g_serverDescription ) );
        ret = g_pClient->GetServerDescription( &g_serverDescription );
        if ( ret != ErrorCode_OK || ! g_serverDescription.HostPresent )
        {
            RCLCPP_DEBUG(this->get_logger(),"Unable to connect to server. Host not present. Exiting.\n");
            return 1;
        }
        RCLCPP_DEBUG(this->get_logger(),"\n[SampleClient] Server application info:\n");
        RCLCPP_DEBUG(this->get_logger(),"Application: %s (ver. %d.%d.%d.%d)\n", g_serverDescription.szHostApp, g_serverDescription.HostAppVersion[0],
            g_serverDescription.HostAppVersion[1], g_serverDescription.HostAppVersion[2], g_serverDescription.HostAppVersion[3]);
        RCLCPP_DEBUG(this->get_logger(),"NatNet Version: %d.%d.%d.%d\n", g_serverDescription.NatNetVersion[0], g_serverDescription.NatNetVersion[1],
            g_serverDescription.NatNetVersion[2], g_serverDescription.NatNetVersion[3]);
        RCLCPP_DEBUG(this->get_logger(),"Client IP:%s\n", g_connectParams.localAddress );
        RCLCPP_DEBUG(this->get_logger(),"Server IP:%s\n", g_connectParams.serverAddress );
        RCLCPP_DEBUG(this->get_logger(),"Server Name:%s\n", g_serverDescription.szHostComputerName);

        // get mocap frame rate
        ret = g_pClient->SendMessageAndWait("FrameRate", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            float fRate = *((float*)pResult);
            RCLCPP_DEBUG(this->get_logger(),"Mocap Framerate : %3.2f\n", fRate);
        }
        else
            RCLCPP_DEBUG(this->get_logger(),"Error getting frame rate.\n");

        // get # of analog samples per mocap frame of data
        ret = g_pClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            g_analogSamplesPerMocapFrame = *((int*)pResult);
            RCLCPP_DEBUG(this->get_logger(),"Analog Samples Per Mocap Frame : %d\n", g_analogSamplesPerMocapFrame);
        }
        else
            RCLCPP_DEBUG(this->get_logger(),"Error getting Analog frame rate.\n");
    }

    return ErrorCode_OK;
}

void NatNetRosClient::resetClient()
    {
        int iSuccess;

        RCLCPP_DEBUG(this->get_logger(),"\n\nre-setting Client\n\n.");

        iSuccess = g_pClient->Disconnect();
        if(iSuccess != 0)
            RCLCPP_DEBUG(this->get_logger(),"error un-initting Client\n");

        iSuccess = g_pClient->Connect( g_connectParams );
        if(iSuccess != 0)
            RCLCPP_DEBUG(this->get_logger(),"error re-initting Client\n");
    }



int main( int argc, char** argv )
{   
    rclcpp::init(argc, argv);
    node = std::make_shared<NatNetRosClient>();
    node->Init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}





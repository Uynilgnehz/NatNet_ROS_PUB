#include <ros/ros.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#   include <conio.h>
#else
#   include <unistd.h>
#   include <termios.h>
#endif

#include <vector>

#include<NatNetTypes.h>
#include<NatNetClient.h>
#include<NatNetRequests.h>
#include<NatNetCAPI.h>

#include<geometry_msgs/Vector3.h>


//靶球ID
int marker_num=50840;

//有关motive的声明和定义
#ifndef _WIN32
char getch();//ubuntu下没有conio库
#endif

void resetClient();
int ConnectClient();
NatNetClient* g_pClient = NULL;
void NATNET_CALLCONV ServerDiscoveredCallback( const sNatNetDiscoveredServer* pDiscoveredServer, void* pUserContext );
void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData);    // receives data from the server

static const ConnectionType kDefaultConnectionType = ConnectionType_Multicast;

std::vector< sNatNetDiscoveredServer > g_discoveredServers;
sNatNetClientConnectParams g_connectParams;
char g_discoveredMulticastGroupAddr[kNatNetIpv4AddrStrLenMax] = NATNET_DEFAULT_MULTICAST_ADDRESS;
int g_analogSamplesPerMocapFrame = 0;
sServerDescription g_serverDescription;

bool Get_Marker_coord(int argc,char** argv);
bool init_motive(int argc,char** argv);
bool clear_motive();
 //

double marker_coord[3];

int main(int argc, char ** argv)
{
	// ROS节点初始化
	ros::init(argc, argv, "coord_pub");

	// 创建节点句柄
	ros::NodeHandle n;

	ros::Publisher coord_pub = n.advertise<geometry_msgs::Vector3>("target_coordinate", 10);

	// 设置循环的频率
	ros::Rate loop_rate(10);

	int count = 0;
    init_motive(argc,argv);
    Get_Marker_coord(argc, argv);

	while (ros::ok())
	{
		geometry_msgs::Vector3 coord;
		// coord.x = 0.1;
		// coord.y = 0.2;
		// coord.z = 0.3;
        coord.x = marker_coord[0];
		coord.y = marker_coord[1];
		coord.z = marker_coord[2];

	    // 发布消息
		coord_pub.publish(coord);
		ROS_INFO("Publish target coordinates:[%0.6f, %0.6f, %0.6f]",coord.x, coord.y, coord.z);

	    // 按照循环频率延时
	    loop_rate.sleep();
	}

    clear_motive();
	return 0;
}


//以下为从motive服务器获取靶标球坐标
bool init_motive(int argc,char** argv)
{
    g_pClient = new NatNetClient();
    g_pClient->SetFrameReceivedCallback( DataHandler, g_pClient );	// this function will receive data from the server

    if ( argc == 1 )
    {
        // An example of synchronous server discovery.
#if 0
        const unsigned int kDiscoveryWaitTimeMillisec = 5 * 1000; // Wait 5 seconds for responses.
        const int kMaxDescriptions = 10; // Get info for, at most, the first 10 servers to respond.
        sNatNetDiscoveredServer servers[kMaxDescriptions];
        int actualNumDescriptions = kMaxDescriptions;
        NatNet_BroadcastServerDiscovery( servers, &actualNumDescriptions );

        if ( actualNumDescriptions < kMaxDescriptions )
        {
            // If this happens, more servers responded than the array was able to store.
        }
#endif

        // Do asynchronous server discovery.
        printf( "Looking for servers on the local network.\n" );
        printf( "Press the number key that corresponds to any discovered server to connect to that server.\n" );
        printf( "Press Q at any time to quit.\n\n" );

        NatNetDiscoveryHandle discovery;
        NatNet_CreateAsyncServerDiscovery( &discovery, ServerDiscoveredCallback );

        while ( const int c = getch() )
        {
            if ( c >= '1' && c <= '9' )
            {
                const size_t serverIndex = c - '1';
                if ( serverIndex < g_discoveredServers.size() )
                {
                    const sNatNetDiscoveredServer& discoveredServer = g_discoveredServers[serverIndex];

                    if ( discoveredServer.serverDescription.bConnectionInfoValid )
                    {
                        // Build the connection parameters.
#ifdef _WIN32
                        _snprintf_s(
#else
                        snprintf(
#endif
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
            else if ( c == 'q' )
            {
                return 0;
            }
        }

        NatNet_FreeAsyncServerDiscovery( discovery );
    }

return true;

}

bool Get_Marker_coord(int argc,char** argv)
{
    // marker_coord[0] = 1.0;
    // marker_coord[1] = 2.0;
    // marker_coord[2] = 3.0;

    int iResult;

    // Connect to Motive
    iResult = ConnectClient();
    if (iResult != ErrorCode_OK)
    {
        printf("Error initializing client.  See log for details.  Exiting");
        return 1;
    }
    else
    {
        printf("Client initialized and ready.\n");
    }


	// Send/receive test request
    void* response;
    int nBytes;
	printf("[SampleClient] Sending Test Request\n");
	iResult = g_pClient->SendMessageAndWait("TestRequest", &response, &nBytes);
	if (iResult == ErrorCode_OK)
	{
		printf("[SampleClient] Received: %s", (char*)response);
	}

	// Retrieve Data Descriptions from Motive
	printf("\n\n[SampleClient] Requesting Data Descriptions...");
	sDataDescriptions* pDataDefs = NULL;
	iResult = g_pClient->GetDataDescriptionList(&pDataDefs);
	if (iResult != ErrorCode_OK || pDataDefs == NULL)
	{
		printf("[SampleClient] Unable to retrieve Data Descriptions.");
	}
	else
	{
        printf("[SampleClient] Received %d Data Descriptions:\n", pDataDefs->nDataDescriptions );
	}

	// Ready to receive marker stream!
	printf("\nClient is connected to server and listening for data...\n");
	int c;
	bool bExit = false;
	// while(c=getch())
	// {
	// 	switch(c)
	// 	{
	// 		case 'q':
	// 			bExit = true;		
	// 			break;	
	// 		case 'r':
	// 			resetClient();
	// 			break;	
    //         case 'p':
    //             sServerDescription ServerDescription;
    //             memset(&ServerDescription, 0, sizeof(ServerDescription));
    //             g_pClient->GetServerDescription(&ServerDescription);
    //             if(!ServerDescription.HostPresent)
    //             {
    //                 printf("Unable to connect to server. Host not present. Exiting.");
    //                 return 1;
    //             }
    //             break;
    //         case 's':
    //             {
    //             printf("\n\n[SampleClient] Requesting Data Descriptions...");
    //             sDataDescriptions* pDataDefs = NULL;
    //             iResult = g_pClient->GetDataDescriptionList(&pDataDefs);
    //             if (iResult != ErrorCode_OK || pDataDefs == NULL)
    //             {
    //                 printf("[SampleClient] Unable to retrieve Data Descriptions.");
    //             }
    //             else
    //             {
    //                 printf("[SampleClient] Received %d Data Descriptions:\n", pDataDefs->nDataDescriptions);
    //             }
    //             }
    //             break;
    //         case 'm':	                        // change to multicast
    //             g_connectParams.connectionType = ConnectionType_Multicast;
    //             iResult = ConnectClient();
    //             if(iResult == ErrorCode_OK)
    //                 printf("Client connection type changed to Multicast.\n\n");
    //             else
    //                 printf("Error changing client connection type to Multicast.\n\n");
    //             break;
    //         case 'u':	                        // change to unicast
    //             g_connectParams.connectionType = ConnectionType_Unicast;
    //             iResult = ConnectClient();
    //             if(iResult == ErrorCode_OK)
    //                 printf("Client connection type changed to Unicast.\n\n");
    //             else
    //                 printf("Error changing client connection type to Unicast.\n\n");
    //             break;
    //         case 'c' :                          // connect
    //             iResult = ConnectClient();
    //             break;
    //         case 'd' :                          // disconnect
    //             // note: applies to unicast connections only - indicates to Motive to stop sending packets to that client endpoint
    //             iResult = g_pClient->SendMessageAndWait("Disconnect", &response, &nBytes);
    //             if (iResult == ErrorCode_OK)
    //                 printf("[SampleClient] Disconnected");
    //             break;
	// 		default:
	// 			break;
	// 	}
	// 	if(bExit)
	// 		break;
	// }

	return true;
}

bool clear_motive()
{
	// Done - clean up.
	if (g_pClient)
	{
		g_pClient->Disconnect();
		delete g_pClient;
		g_pClient = NULL;
	}

    return true;
}

void NATNET_CALLCONV ServerDiscoveredCallback( const sNatNetDiscoveredServer* pDiscoveredServer, void* pUserContext )
{
    char serverHotkey = '.';
    if ( g_discoveredServers.size() < 9 )
    {
        serverHotkey = static_cast<char>('1' + g_discoveredServers.size());
    }

    const char* warning = "";

    if ( pDiscoveredServer->serverDescription.bConnectionInfoValid == false )
    {
        warning = " (WARNING: Legacy server, could not autodetect settings. Auto-connect may not work reliably.)";
    }

    printf( "[%c] %s %d.%d at %s%s\n",
        serverHotkey,
        pDiscoveredServer->serverDescription.szHostApp,
        pDiscoveredServer->serverDescription.HostAppVersion[0],
        pDiscoveredServer->serverDescription.HostAppVersion[1],
        pDiscoveredServer->serverAddress,
        warning );

    g_discoveredServers.push_back( *pDiscoveredServer );
}

// Establish a NatNet Client connection
int ConnectClient()
{
    // Release previous server
    g_pClient->Disconnect();

    // Init Client and connect to NatNet server
    int retCode = g_pClient->Connect( g_connectParams );
    if (retCode != ErrorCode_OK)
    {
        printf("Unable to connect to server.  Error code: %d. Exiting", retCode);
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
            printf("Unable to connect to server. Host not present. Exiting.");
            return 1;
        }
        printf("\n[SampleClient] Server application info:\n");
        printf("Application: %s (ver. %d.%d.%d.%d)\n", g_serverDescription.szHostApp, g_serverDescription.HostAppVersion[0],
            g_serverDescription.HostAppVersion[1], g_serverDescription.HostAppVersion[2], g_serverDescription.HostAppVersion[3]);
        printf("NatNet Version: %d.%d.%d.%d\n", g_serverDescription.NatNetVersion[0], g_serverDescription.NatNetVersion[1],
            g_serverDescription.NatNetVersion[2], g_serverDescription.NatNetVersion[3]);
        printf("Client IP:%s\n", g_connectParams.localAddress );
        printf("Server IP:%s\n", g_connectParams.serverAddress );
        printf("Server Name:%s\n", g_serverDescription.szHostComputerName);

        // get mocap frame rate
        ret = g_pClient->SendMessageAndWait("FrameRate", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            float fRate = *((float*)pResult);
            printf("Mocap Framerate : %3.2f\n", fRate);
        }
        else
            printf("Error getting frame rate.\n");

        // get # of analog samples per mocap frame of data
        ret = g_pClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            g_analogSamplesPerMocapFrame = *((int*)pResult);
            printf("Analog Samples Per Mocap Frame : %d\n", g_analogSamplesPerMocapFrame);
        }
        else
            printf("Error getting Analog frame rate.\n");
    }

    return ErrorCode_OK;
}

// DataHandler receives data from the server
// This function is called by NatNet when a frame of mocap data is available
void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData)
{
    NatNetClient* pClient = (NatNetClient*) pUserData;

    // Software latency here is defined as the span of time between:
    //   a) The reception of a complete group of 2D frames from the camera system (CameraDataReceivedTimestamp)
    // and
    //   b) The time immediately prior to the NatNet frame being transmitted over the network (TransmitTimestamp)
    //
    // This figure may appear slightly higher than the "software latency" reported in the Motive user interface,
    // because it additionally includes the time spent preparing to stream the data via NatNet.
    const uint64_t softwareLatencyHostTicks = data->TransmitTimestamp - data->CameraDataReceivedTimestamp;
    const double softwareLatencyMillisec = (softwareLatencyHostTicks * 1000) / static_cast<double>(g_serverDescription.HighResClockFrequency);

    // Transit latency is defined as the span of time between Motive transmitting the frame of data, and its reception by the client (now).
    // The SecondsSinceHostTimestamp method relies on NatNetClient's internal clock synchronization with the server using Cristian's algorithm.
    const double transitLatencyMillisec = pClient->SecondsSinceHostTimestamp( data->TransmitTimestamp ) * 1000.0;

    int i=0;

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
        const double systemLatencyMillisec = (systemLatencyHostTicks * 1000) / static_cast<double>(g_serverDescription.HighResClockFrequency);

        // Client latency is defined as the sum of system latency and the transit time taken to relay the data to the NatNet client.
        // This is the all-inclusive measurement (photons to client processing).
        const double clientLatencyMillisec = pClient->SecondsSinceHostTimestamp( data->CameraMidExposureTimestamp ) * 1000.0;

        // You could equivalently do the following (not accounting for time elapsed since we calculated transit latency above):
        //const double clientLatencyMillisec = systemLatencyMillisec + transitLatencyMillisec;

    }
    else
    {
        printf( "Transit latency : %.2lf milliseconds\n", transitLatencyMillisec );
    }

    // timecode - for systems with an eSync and SMPTE timecode generator - decode to values
	int hour, minute, second, frame, subframe;
    NatNet_DecodeTimecode( data->Timecode, data->TimecodeSubframe, &hour, &minute, &second, &frame, &subframe );
	// decode to friendly string
	char szTimecode[128] = "";
    NatNet_TimecodeStringify( data->Timecode, data->TimecodeSubframe, szTimecode, 128 );

	// labeled markers - this includes all markers (Active, Passive, and 'unlabeled' (markers with no asset but a PointCloud ID)
    bool bOccluded;     // marker was not visible (occluded) in this frame
    bool bPCSolved;     // reported position provided by point cloud solve
    bool bModelSolved;  // reported position provided by model solve
    bool bHasModel;     // marker has an associated asset in the data stream
    bool bUnlabeled;    // marker is 'unlabeled', but has a point cloud ID that matches Motive PointCloud ID (In Motive 3D View)
	bool bActiveMarker; // marker is an actively labeled LED marker

	for(i=0; i < data->nLabeledMarkers; i++)
	{
        bOccluded = ((data->LabeledMarkers[i].params & 0x01)!=0);
        bPCSolved = ((data->LabeledMarkers[i].params & 0x02)!=0);
        bModelSolved = ((data->LabeledMarkers[i].params & 0x04) != 0);
        bHasModel = ((data->LabeledMarkers[i].params & 0x08) != 0);
        bUnlabeled = ((data->LabeledMarkers[i].params & 0x10) != 0);
		bActiveMarker = ((data->LabeledMarkers[i].params & 0x20) != 0);

        sMarker marker = data->LabeledMarkers[i];

        // Marker ID Scheme:
        // Active Markers:
        //   ID = ActiveID, correlates to RB ActiveLabels list
        // Passive Markers: 
        //   If Asset with Legacy Labels
        //      AssetID 	(Hi Word)
        //      MemberID	(Lo Word)
        //   Else
        //      PointCloud ID
        int modelID, markerID;
        NatNet_DecodeID( marker.ID, &modelID, &markerID );
		
        char szMarkerType[512];
        if (bActiveMarker)
            strcpy(szMarkerType, "Active");
        else if(bUnlabeled)
            strcpy(szMarkerType, "Unlabeled");
        else
            strcpy(szMarkerType, "Labeled");

//找到目标靶球的ID，提取数据
    if(markerID==marker_num)
    {
        marker_coord[0]=marker.x;
        marker_coord[1]=marker.y;
        marker_coord[2]=marker.z;

    }

        // printf("%s Marker [ModelID=%d, MarkerID=%d, Occluded=%d, PCSolved=%d, ModelSolved=%d] [size=%3.2f] [pos=%3.2f,%3.2f,%3.2f]\n",
            // szMarkerType, modelID, markerID, bOccluded, bPCSolved, bModelSolved,  marker.size, marker.x, marker.y, marker.z);
            
	}

}

void resetClient()
{
	int iSuccess;

	printf("\n\nre-setting Client\n\n.");

	iSuccess = g_pClient->Disconnect();
	if(iSuccess != 0)
		printf("error un-initting Client\n");

    iSuccess = g_pClient->Connect( g_connectParams );
	if(iSuccess != 0)
		printf("error re-initting Client\n");
}

#ifndef _WIN32
char getch()
{
    char buf = 0;
    termios old = { 0 };

    fflush( stdout );

    if ( tcgetattr( 0, &old ) < 0 )
        perror( "tcsetattr()" );

    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;

    if ( tcsetattr( 0, TCSANOW, &old ) < 0 )
        perror( "tcsetattr ICANON" );

    if ( read( 0, &buf, 1 ) < 0 )
        perror( "read()" );

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;

    if ( tcsetattr( 0, TCSADRAIN, &old ) < 0 )
        perror( "tcsetattr ~ICANON" );

    //printf( "%c\n", buf );

    return buf;
}
#endif

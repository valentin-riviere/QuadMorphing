#include "Client.h"
#include <iostream>

// ********************************************************
// Les librairies
// ********************************************************
#pragma comment(lib,"ViconDataStreamSDK_CPP.lib")

// NameSpace
using namespace ViconDataStreamSDK::CPP;
using namespace std;

// Object and segment properties
#define MAX_OBJECT              20
#define MAX_SEGMENT             20
#define MAX_MARKER              20
#define MAX_NAME_LENGTH         255
#define NB_OTHER_OUTPUTS        3       // New,framenumber and latency 
//#define SEGMENT_LENGTH          12    // Rotation matrix: 9, translation: 3
#define SEGMENT_LENGTH          7       // translation: 3, Euler: 3, occluded: 1              
#define MARKER_LENGTH           3

// Connection properties
#define MAX_CONNECTION_TRY      10
#define MAX_GETFRAME_TRY        3
#define TRANSMIT_MULTICAST      0
#define MULTICAST_ADDRESS       "139.124.59.0"

// Parameters order
#define VICON_IP_NUM_PARAM      0
#define VICON_PORT_NUM_PARAM    1
#define SAMPLE_TIME_NUM_PARAM   2
#define OBJECT_MAT_NUM_PARAM    3
#define MARKER_MAT_NUM_PARAM    4
#define OBJECT_NAME_NUM_PARAM   5
#define SEGMENT_NAME_NUM_PARAM  6
#define MARKER_NAME_NUM_PARAM   7

// ERROR_CODE
#define CONNECTION_FAILED   0x01
#define NOT_ENOUGH_OBJECT   0x02
#define MISSING_OBJECT      0x04
#define MISSING_SEGMENT 	0x08

/************** Declaration of function's protoypes **************/

void GetViconAddress(SimStruct *S, char* IP_Vicon_str, int* port);

void DisplayCompleteFrame(void);

// format adaptation
std::string AdaptDeviceType( const DeviceType::Enum i_DeviceType );
std::string AdaptUnit( const Unit::Enum i_Unit );
std::string AdaptOccluded( const bool i_Value );
std::string AdaptEnable( const bool i_Value );
std::string AdaptDirection( const Direction::Enum i_Direction );

/********** Declaration of data structure and new types **********/
typedef struct Segment
{
    std::string Name;
}Segment;

typedef struct Marker
{
    std::string Name;
}Marker;

typedef struct Object
{
    std::string Name;
    unsigned int Index;
    unsigned int NbSegment; 
    unsigned int NbMarker;
    Segment Segments[MAX_SEGMENT];
    Marker Markers[MAX_MARKER];
}Object;



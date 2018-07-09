/*
 * File : timestwo.c
 * Abstract:
 *       An example C-file S-function for multiplying an input by 2,
 *       y  = 2*u
 *
 * Real-Time Workshop note:
 *   This file can be used as is (noninlined) with the Real-Time Workshop
 *   C rapid prototyping targets, or it can be inlined using the Target 
 *   Language Compiler technology and used with any target. See 
 *     matlabroot/toolbox/simulink/blocks/tlc_c/timestwo.tlc   
 *     matlabroot/toolbox/simulink/blocks/tlc_ada/timestwo.tlc 
 *   the C and Ada TLC code to inline the S-function.
 *
 * See simulink/src/sfuntmpl_doc.c
 *
 * Copyright 1990-2004 The MathWorks, Inc.
 * $Revision: 1.12.4.2 $
 */


#define S_FUNCTION_NAME  SfunCPP_RecoverVicon_SDK
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "SfunCPP_RecoverVicon_SDK.h"

// Make a new client
Client MyClient;
char HostName[30] = "";
// define number of each element
unsigned int NbObject = 0;
unsigned int NbSegment = 0;
unsigned int NbMarker = 0;
unsigned int NbUnlabeledMarker = 0;
unsigned int UnlabeledMarkerTab[MAX_MARKER];
int OutputWidth = 0;

Object ObjectList[MAX_OBJECT];

FILE* FID_debug = NULL;


/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{
    char IP_Vicon_str[16];
	int port;
    char port_str[6] = "";
    
    int buflen, status, i, j;
    char ObjectMatrix[MAX_OBJECT*MAX_NAME_LENGTH];
    char SegmentMatrix[MAX_OBJECT*MAX_SEGMENT*MAX_NAME_LENGTH];
    char MarkerMatrix[MAX_OBJECT*MAX_MARKER*MAX_NAME_LENGTH];
    
    char* segment_tok;
    char* marker_tok;
    char* object_tok;
    Output_GetFrame OutputGetFrame;
    char NbTry = 0;
    
    printf("<VICON> mdlInitializeSizes()\n");
    
    // Clear Memory
    memset(HostName, '\0', 30);
    NbObject = 0;
	NbSegment = 0;
	NbMarker = 0;
    
    /************************* GET ALL PARAMETERS ************************/
        // parameter 1 & 2
    GetViconAddress(S, IP_Vicon_str, &port);
	// copy results in HostName
    strcat(HostName, IP_Vicon_str);
    itoa(port, port_str, 10);
    strcat(HostName, ":");
    strcat(HostName, port_str);
    printf("HostName: %s\n", HostName);
    //fprintf(FID_debug, "HostName: %s\n", HostName);
    //fflush(FID_debug);
    
        // Construct the structure
    NbObject = (int)mxGetDimensions(ssGetSFcnParam(S, OBJECT_MAT_NUM_PARAM))[0];
    
    // recover the ObjectMatrix containing the name of all Object 
        buflen = (int)(mxGetN(ssGetSFcnParam(S, OBJECT_NAME_NUM_PARAM))*sizeof(mxChar)+1);
        status = mxGetString(ssGetSFcnParam(S, OBJECT_NAME_NUM_PARAM), ObjectMatrix, (mwSize)buflen);
    
    // recover the ObjectMatrix containing the name of all Segment 
        buflen = (int)(mxGetN(ssGetSFcnParam(S, SEGMENT_NAME_NUM_PARAM))*sizeof(mxChar)+1);
        status = mxGetString(ssGetSFcnParam(S, SEGMENT_NAME_NUM_PARAM), SegmentMatrix, (mwSize)buflen);
    
    // recover the ObjectMatrix containing the name of all Marker 
        buflen = (int)(mxGetN(ssGetSFcnParam(S, MARKER_NAME_NUM_PARAM))*sizeof(mxChar)+1);
        status = mxGetString(ssGetSFcnParam(S, MARKER_NAME_NUM_PARAM), MarkerMatrix, (mwSize)buflen);
         
    object_tok = strtok(ObjectMatrix,",");
    
    for (i=0; i<NbObject; i++)
    {
        // recover Object info
        ObjectList[i].Name = object_tok;
        object_tok = strtok(NULL, ","); 
    }
    
    segment_tok = strtok(SegmentMatrix,",");
    for (i=0; i<NbObject; i++)
    {
        // recover segment info
        ObjectList[i].NbSegment = (unsigned int)(mxGetPr(ssGetSFcnParam(S, OBJECT_MAT_NUM_PARAM))[i]);
        for (j=0; j<ObjectList[i].NbSegment; j++)
        {
            ObjectList[i].Segments[j].Name = segment_tok;
            segment_tok = strtok(NULL, ",");
        }
        NbSegment += ObjectList[i].NbSegment;
    }
    
    marker_tok = strtok(MarkerMatrix,",");
    for (i=0; i<NbObject; i++)
    {
        // recover marker info
        ObjectList[i].NbMarker = (int)(mxGetPr(ssGetSFcnParam(S, OBJECT_MAT_NUM_PARAM))[i+NbObject]);
        for (j=0; j<ObjectList[i].NbMarker; j++)
        {
            ObjectList[i].Markers[j].Name = marker_tok;
            marker_tok = strtok(NULL, ",");
        }
        NbMarker += ObjectList[i].NbMarker;
    }
    
    // Display Structure
    for (i=0; i<NbObject; i++)
    {
        // recover Object info
        printf("<VICON> \t Subject #%d:\n", i);
        printf("<VICON> \t        Name: %s\n", ObjectList[i].Name.c_str());
        // recover segment info
        printf("<VICON> \t        NbSegment: %u\n", ObjectList[i].NbSegment);
        for (j=0; j<ObjectList[i].NbSegment; j++)
        {
            printf("<VICON> \t           Segment #%d: %s\n", j, ObjectList[i].Segments[j].Name.c_str());
        }
        // recover marker info
        printf("<VICON> \t        NbMarker: %u\n", ObjectList[i].NbMarker);
        for (j=0; j<ObjectList[i].NbMarker; j++)
        {
            printf("<VICON> \t           Marker #%d: %s\n", j, ObjectList[i].Markers[j].Name.c_str());
        }
    }
     
        // parameter 5
    NbUnlabeledMarker = (int)mxGetDimensions(ssGetSFcnParam(S, MARKER_MAT_NUM_PARAM))[0];  
    for (i=0; i<NbUnlabeledMarker; i++)
    {
        UnlabeledMarkerTab[i] = (int)(mxGetPr(ssGetSFcnParam(S, MARKER_MAT_NUM_PARAM))[i]);
        printf("<VICON> \t UnlabeledMarker #%d: %u\n", i, UnlabeledMarkerTab[i]);
    }
    
    OutputWidth = NB_OTHER_OUTPUTS + NbSegment*SEGMENT_LENGTH + NbMarker*MARKER_LENGTH + NbUnlabeledMarker*MARKER_LENGTH;
    
    printf("<VICON> \t %d Subject to receive\n", NbObject);
    printf("<VICON> \t %d Segment to receive\n", NbSegment);
    printf("<VICON> \t %d Labeled Marker to receive\n", NbMarker);
    printf("<VICON> \t %d Unlabeled Marker to receive\n", NbUnlabeledMarker);
    printf("NB_OTHER_OUTPUTS = %d\n", NB_OTHER_OUTPUTS);
    printf("SEGMENT_LENGTH = %d\n", SEGMENT_LENGTH);
    printf("MARKER_LENGTH = %d\n", MARKER_LENGTH);

    printf("OutputWidth = %d\n", OutputWidth);
    
    /*********************************************************************/
        
    // Connect to a server
    printf("<VICON> \t Connecting to %s ... ", HostName);
    //fprintf(FID_debug, "<VICON> \t Connecting to %s ... ", HostName);
    //fflush(FID_debug);
    while( !MyClient.IsConnected().Connected && (NbTry<MAX_CONNECTION_TRY) )
    {
        // Direct connection
        MyClient.Connect( HostName );
        printf(".");
        NbTry++;
    }
    if (MyClient.IsConnected().Connected)
    {
        printf("Successfully Connected!\n");
        //fprintf(FID_debug, "Successfully Connected!\n");
        //fflush(FID_debug);
    }
    else
    {
        printf("Echec! Connection aborted!\n");
        //fprintf(FID_debug, "Echec! Connection aborted!\n");
        //fflush(FID_debug);
    }
  
    // Enable some different data types
    MyClient.EnableSegmentData();
    MyClient.EnableMarkerData();
    MyClient.EnableUnlabeledMarkerData();
    MyClient.EnableDeviceData();
    
    printf("Segment Data Enabled: %s\n", AdaptEnable( MyClient.IsSegmentDataEnabled().Enabled ));
    printf("Marker Data Enabled: %s\n", AdaptEnable( MyClient.IsMarkerDataEnabled().Enabled ));
    printf("Unlabeled Marker Data Enabled: %s\n", AdaptEnable( MyClient.IsUnlabeledMarkerDataEnabled().Enabled ));
    printf("Device Data Enabled: %s\n", AdaptEnable( MyClient.IsDeviceDataEnabled().Enabled ));

    // Set the streaming mode
    // MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPull );
    // MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPullPreFetch );
    MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ServerPush );

    // Set the global up axis
    MyClient.SetAxisMapping( Direction::Forward, 
                             Direction::Left, 
                             Direction::Up ); // Z-up
    // MyClient.SetGlobalUpAxis( Direction::Forward, 
    //                           Direction::Up, 
    //                           Direction::Right ); // Y-up

    Output_GetAxisMapping _Output_GetAxisMapping = MyClient.GetAxisMapping();
    printf("Axis Mapping: X-%s Y-%s Z-%s\n", AdaptDirection( _Output_GetAxisMapping.XAxis ), 
                                             AdaptDirection( _Output_GetAxisMapping.YAxis ),
                                             AdaptDirection( _Output_GetAxisMapping.ZAxis ));

    // Discover the version number
    Output_GetVersion _Output_GetVersion = MyClient.GetVersion();
    printf("Version: %u.%u.%u\n", _Output_GetVersion.Major,
                                  _Output_GetVersion.Minor,
                                  _Output_GetVersion.Point);
    
    if( TRANSMIT_MULTICAST )
    {
      MyClient.StartTransmittingMulticast( "localhost", MULTICAST_ADDRESS );
    }
    
    printf( "Waiting for first frame." );  
    //fprintf(FID_debug, "Waiting for first frame." );  
    //fflush(FID_debug);
        
    NbTry = 0;
    OutputGetFrame = MyClient.GetFrame();
    while((OutputGetFrame.Result!=2)&&(NbTry<MAX_GETFRAME_TRY))
    {
        printf( ". ");
        //fprintf(FID_debug, ". ");
        //fflush(FID_debug);
        OutputGetFrame = MyClient.GetFrame();
        NbTry++;
    }
    printf( "\n" );
    //fprintf(FID_debug, "\n" );  
    //fflush(FID_debug);

    printf( "Frame Number: %d\n", MyClient.GetFrameNumber().FrameNumber );
    //fprintf(FID_debug, "Frame Number: %d\n", MyClient.GetFrameNumber().FrameNumber );
    //fflush(FID_debug);
    
    DisplayCompleteFrame();
            
    if( TRANSMIT_MULTICAST )
    {
        printf("<VICON> \t stop transmitting Multicast...");
        //fprintf(FID_debug, "<VICON> \t stop transmitting Multicast...");
        //fflush(FID_debug);
        MyClient.StopTransmittingMulticast();
        printf( "done\n" );
        //fprintf(FID_debug, "done\n" );
        //fflush(FID_debug);
    }

    MyClient.DisableSegmentData();
    MyClient.DisableMarkerData();
    MyClient.DisableUnlabeledMarkerData();
    MyClient.DisableDeviceData();
    
    //Disconnect and dispose
    printf("<VICON> \t Disconnect from server...");
    //fprintf(FID_debug, "<VICON> \t Disconnect from server...");
    //fflush(FID_debug);
    MyClient.Disconnect();
    printf( "done\n" );
    //fprintf(FID_debug, "done\n" );
    //fflush(FID_debug);
    
    // Set the number of parameters
    ssSetNumSFcnParams(S, 8);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }
    
    if (!ssSetNumInputPorts(S, 0)) return;
    //ssSetInputPortWidth(S, 0, 0);
    //ssSetInputPortDirectFeedThrough(S, 0, 1);

    if (!ssSetNumOutputPorts(S,1)) return;
    ssSetOutputPortWidth(S, 0, OutputWidth);
    //ssSetOutputPortWidth(S, 0, 3);

    ssSetNumSampleTimes(S, 1);

    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    ssSetOptions(S,
                 SS_OPTION_WORKS_WITH_CODE_REUSE |
                 SS_OPTION_EXCEPTION_FREE_CODE |
                 SS_OPTION_USE_TLC_WITH_ACCELERATOR);
    
    printf("<VICON> End mdlInitializeSizes()\n");
}


/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy that we inherit our sample time from the driving block.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    
    time_T SampleTime = (time_T)(mxGetPr(ssGetSFcnParam(S, 2))[0]);
    
    ssSetSampleTime(S, 0, SampleTime);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S); 
}

/* Function: mdlOutputs =======================================================
 * Abstract:
 *    y = 2*u
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    unsigned int      i, j;
    real_T            *y    = ssGetOutputPortRealSignal(S,0);
    unsigned int OutputOffset = 0;
    double Occluded;
    
    if(MyClient.GetFrame().Result == 2)
    {
        y[0] = 1;
    }
    else
    {
        y[0] = 0;
    }
    
    y[1] = (real_T)MyClient.GetFrameNumber().FrameNumber;
    y[2] = MyClient.GetLatencyTotal().Total;
    
    for (i=0; i<NbObject; i++) 
    {
        for (j=0; j<ObjectList[i].NbSegment; j++)
        {
            //Output_GetSegmentGlobalRotationMatrix       _Output_GetSegmentGlobalRotationMatrix      = MyClient.GetSegmentGlobalRotationMatrix( ObjectList[i].Name, ObjectList[i].Segments[j].Name );
            //Output_GetSegmentGlobalRotationQuaternion   _Output_GetSegmentGlobalRotationQuaternion  = MyClient.GetSegmentGlobalRotationQuaternion( ObjectList[i].Name, ObjectList[i].Segments[j].Name );
            Output_GetSegmentGlobalRotationEulerXYZ     _Output_GetSegmentGlobalRotationEulerXYZ    = MyClient.GetSegmentGlobalRotationEulerXYZ( ObjectList[i].Name, ObjectList[i].Segments[j].Name );
            Output_GetSegmentGlobalTranslation          _Output_GetSegmentGlobalTranslation         = MyClient.GetSegmentGlobalTranslation( ObjectList[i].Name, ObjectList[i].Segments[j].Name );

            memcpy(&y[NB_OTHER_OUTPUTS+OutputOffset], _Output_GetSegmentGlobalTranslation.Translation, 24);           // copy the array of position in the output [3xdouble] = 3x8 bytes = 24 bytes
            OutputOffset += 3;
            //memcpy(&y[NB_OTHER_OUTPUTS+OutputOffset], _Output_GetSegmentGlobalRotationMatrix.Rotation, 72);        // copy the array of position in the output [9xdouble] = 9x8 bytes = 72 bytes
            //OutputOffset += 9;
            //memcpy(&y[NB_OTHER_OUTPUTS+OutputOffset], _Output_GetSegmentGlobalRotationQuaternion.Rotation, 32);    // copy the array of position in the output [4xdouble] = 4x8 bytes = 32 bytes
            //OutputOffset += 4;
            memcpy(&y[NB_OTHER_OUTPUTS+OutputOffset], _Output_GetSegmentGlobalRotationEulerXYZ.Rotation, 24);     // copy the array of position in the output [3xdouble] = 3x8 bytes = 24 bytes
            OutputOffset += 3;
            Occluded = (double)(_Output_GetSegmentGlobalTranslation.Occluded);
            memcpy(&y[NB_OTHER_OUTPUTS+OutputOffset], &Occluded, 8);     // copy the array of position in the output [1xdouble] = 1x8 bytes = 8 bytes
            OutputOffset += 1;        
            
        }
    }
    
    for (i=0; i<NbObject; i++) 
    {
        for (j=0; j<NbMarker; j++) 
        {
            Output_GetMarkerGlobalTranslation _Output_GetMarkerGlobalTranslation = MyClient.GetMarkerGlobalTranslation( ObjectList[i].Name, ObjectList[i].Markers[j].Name );
            memcpy(&y[NB_OTHER_OUTPUTS+OutputOffset], _Output_GetMarkerGlobalTranslation.Translation, 24);     // copy the array of position in the output [3xdouble] = 3x8 bytes = 24 bytes
            //printf("ObjectList[%d].Markers[%d] = %s.%s = %f, %f, %f\n", i, j, ObjectList[i].Name, ObjectList[i].Markers[j].Name, y[NB_OTHER_OUTPUTS+OutputOffset], y[NB_OTHER_OUTPUTS+OutputOffset+1], y[NB_OTHER_OUTPUTS+OutputOffset+2]);
            OutputOffset += 3;
        }
    }
    
    for (i=0; i<NbUnlabeledMarker; i++) 
    {
        // Get the global marker translation
        Output_GetUnlabeledMarkerGlobalTranslation _Output_GetUnlabeledMarkerGlobalTranslation = MyClient.GetUnlabeledMarkerGlobalTranslation( UnlabeledMarkerTab[i] );
        memcpy(&y[NB_OTHER_OUTPUTS+OutputOffset], _Output_GetUnlabeledMarkerGlobalTranslation.Translation, 24);
    }
}

#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START) 
    
    /* Function: mdlStart =======================================================
     * Abstract:
     *    This function is called once at start of model execution. If you
     *    have states that should be initialized once, this is the place
     *    to do it.
     */
    static void mdlStart(SimStruct *S)
    {
        char NbTry = 0;
                
        printf("<VICON> MdlStart()\n");
        // Connect to a server
        printf("<VICON> \t Connecting to %s ... ", HostName);
        //fprintf(FID_debug, "<VICON> \t Connecting to %s ... ", HostName);
        //fflush(FID_debug);
        while( !MyClient.IsConnected().Connected && (NbTry<MAX_CONNECTION_TRY) )
        {
            // Direct connection
            MyClient.Connect( HostName );
            printf(".");
            NbTry++;
        }
        if (MyClient.IsConnected().Connected)
        {
            printf("Successfully Connected!\n");
            //fprintf(FID_debug, "Successfully Connected!\n");
            //fflush(FID_debug);
        }
        else
        {
            printf("Echec! Connection aborted!\n");
            //fprintf(FID_debug, "Echec! Connection aborted!\n");
            //fflush(FID_debug);
        }

        // Enable some different data types
        MyClient.EnableSegmentData();
        MyClient.EnableMarkerData();
        MyClient.EnableUnlabeledMarkerData();
        MyClient.EnableDeviceData();

        printf("<VICON> \t Segment Data Enabled: %s\n", AdaptEnable( MyClient.IsSegmentDataEnabled().Enabled ));
        printf("<VICON> \t Marker Data Enabled: %s\n", AdaptEnable( MyClient.IsMarkerDataEnabled().Enabled ));
        printf("<VICON> \t Unlabeled Marker Data Enabled: %s\n", AdaptEnable( MyClient.IsUnlabeledMarkerDataEnabled().Enabled ));
        printf("<VICON> \t Device Data Enabled: %s\n", AdaptEnable( MyClient.IsDeviceDataEnabled().Enabled ));

        // Set the streaming mode
        // MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPull );
        // MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPullPreFetch );
        MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ServerPush );

        // Set the global up axis
        MyClient.SetAxisMapping( Direction::Forward, 
                                 Direction::Left, 
                                 Direction::Up ); // Z-up
        // MyClient.SetGlobalUpAxis( Direction::Forward, 
        //                           Direction::Up, 
        //                           Direction::Right ); // Y-up

        Output_GetAxisMapping _Output_GetAxisMapping = MyClient.GetAxisMapping();
        printf("<VICON> \t Axis Mapping: X-%s Y-%s Z-%s\n", AdaptDirection( _Output_GetAxisMapping.XAxis ), 
                                                 AdaptDirection( _Output_GetAxisMapping.YAxis ),
                                                 AdaptDirection( _Output_GetAxisMapping.ZAxis ));

        // Discover the version number
        Output_GetVersion _Output_GetVersion = MyClient.GetVersion();
        printf("<VICON> \t Version: %u.%u.%u\n", _Output_GetVersion.Major,
                                      _Output_GetVersion.Minor,
                                      _Output_GetVersion.Point);

        if( TRANSMIT_MULTICAST )
        {
          MyClient.StartTransmittingMulticast( "localhost", MULTICAST_ADDRESS );
        }
        
        // Fill the corresponding array
        //FillCorrespondingArray();
        
        printf("<VICON> End MdlStart()\n");
    }
#endif /*  MDL_START */


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
    printf("<VICON> MdlTerminate()\n");
    if( TRANSMIT_MULTICAST )
    {
        printf("<VICON> \t stop transmitting Multicast...");
        //fprintf(FID_debug, "<VICON> \t stop transmitting Multicast...");
        //fflush(FID_debug);
        MyClient.StopTransmittingMulticast();
        printf( "done\n" );
        //fprintf(FID_debug, "done\n" );
        //fflush(FID_debug);
    }

    MyClient.DisableSegmentData();
    MyClient.DisableMarkerData();
    MyClient.DisableUnlabeledMarkerData();
    MyClient.DisableDeviceData();
    
    //Disconnect and dispose
    printf("<VICON> \t Disconnect from server...");
    //fprintf(FID_debug, "<VICON> \t Disconnect from server...");
    //fflush(FID_debug);
    MyClient.Disconnect();
    printf( " done\n" );
    //fprintf(FID_debug, "done\n" );
    //fflush(FID_debug);
    
    printf("<VICON> End MedlTerminate()\n");
}

/*************************** Sfunction's tools ***************************/

void GetViconAddress(SimStruct *S, char* IP_Vicon_str, int* port)// recover IP_remote_str length
{
    int buflen, status;
    
    // recover IP_remote_str length
	printf("<VICON> \t Vicon's Server address: ");
	buflen = (int)(mxGetN(ssGetSFcnParam(S, 0))*sizeof(mxChar)+1);
	// recover IP_address_str
	status = mxGetString(ssGetSFcnParam(S, 0), IP_Vicon_str, (mwSize)buflen);
	printf("%s : ", IP_Vicon_str);
	// recover int port;
	*port = (int)(( mxGetPr(ssGetSFcnParam(S, 1)))[0]);
	printf("%d \n", *port);
}

void DisplayCompleteFrame(void)
{
    std::string SubjectName;
    std::string RootSegment;
    std::string SegmentName;
    std::string SegmentParentName;
    
    //fprintf(FID_debug, "Timecode\n");
    //fflush(FID_debug);
      // Get the timecode
      Output_GetTimecode _Output_GetTimecode  = MyClient.GetTimecode();

      printf( "Timecode: %u h", _Output_GetTimecode.Hours);
      printf( " %u m", _Output_GetTimecode.Minutes);
      printf( " %u s", _Output_GetTimecode.Seconds);
      printf( " %u f", _Output_GetTimecode.Frames);
      printf( " %u sf", _Output_GetTimecode.SubFrame);
      printf( " %u", (unsigned int)_Output_GetTimecode.FieldFlag);
      printf( " %u", _Output_GetTimecode.Standard);
      printf( " %u", _Output_GetTimecode.SubFramesPerFrame);
      printf( " %u\n", _Output_GetTimecode.UserBits);

      // Get the latency
      printf("Latency: %f s\n", MyClient.GetLatencyTotal().Total);
      
      // Count the number of subjects
      unsigned int SubjectCount = MyClient.GetSubjectCount().SubjectCount;
      printf("Subjects (%u):\n", SubjectCount);
      //fprintf(FID_debug, "Subjects (%u):\n", SubjectCount);
      //fflush(FID_debug);
      
      for( unsigned int SubjectIndex = 0 ; SubjectIndex < SubjectCount ; ++SubjectIndex )
      {
        printf("  Subject #%u\n", SubjectIndex);
        //fprintf(FID_debug, "  Subject #%u\n", SubjectIndex);
        //fflush(FID_debug);
        
        // Get the subject name
        SubjectName = MyClient.GetSubjectName( SubjectIndex ).SubjectName;
        printf("            Name: %s\n", SubjectName.c_str());
        //fprintf(FID_debug, "            Name: %s\n", SubjectName);
        //fflush(FID_debug);
        
        // Get the root segment
        RootSegment = MyClient.GetSubjectRootSegmentName( SubjectName ).SegmentName;
        printf("    Root Segment: %s\n", RootSegment.c_str());
        //fprintf(FID_debug, "    Root Segment: %s\n", RootSegment);
        //fflush(FID_debug);
        
        // Count the number of segments
        unsigned int SegmentCount = MyClient.GetSegmentCount( SubjectName ).SegmentCount;
        printf("    Segments (%u):\n", SegmentCount);
        //fprintf(FID_debug, "    Segments (%u):\n", SegmentCount);
        //fflush(FID_debug);
        for( unsigned int SegmentIndex = 0 ; SegmentIndex < SegmentCount ; ++SegmentIndex )
        {
            printf("      Segment #%u\n", SegmentIndex);
            //fprintf(FID_debug, "      Segment #%u\n", SegmentIndex);
            //fflush(FID_debug);
            
            // Get the segment name
            SegmentName = MyClient.GetSegmentName( SubjectName, SegmentIndex ).SegmentName;
            printf("          Name: %s\n", SegmentName.c_str());
            //fprintf(FID_debug, "          Name: %s\n", SegmentName);
            
            // Get the segment parent
            SegmentParentName = MyClient.GetSegmentParentName( SubjectName, SegmentName ).SegmentName;
            printf("        Parent: %s\n", SegmentParentName.c_str());
            //fprintf(FID_debug, "        Parent: %s\n", SegmentParentName);
            //fflush(FID_debug);
            
            // Get the segment's children
            unsigned int ChildCount = MyClient.GetSegmentChildCount( SubjectName, SegmentName ).SegmentCount;
            printf("     Children (%u):\n", ChildCount);
            //fprintf(FID_debug, "     Children (%u):\n", ChildCount);
            //fflush(FID_debug);
            
            for( unsigned int ChildIndex = 0 ; ChildIndex < ChildCount ; ++ChildIndex )
            {
                std::string ChildName = MyClient.GetSegmentChildName( SubjectName, SegmentName, ChildIndex ).SegmentName;
                printf("       %s\n", ChildName);
                //fprintf(FID_debug, "       %s\n", ChildName);
                //fflush(FID_debug);
            }

            // Get the static segment translation
            Output_GetSegmentStaticTranslation _Output_GetSegmentStaticTranslation = 
            MyClient.GetSegmentStaticTranslation( SubjectName, SegmentName );
            printf("        Static Translation: (%f, %f, %f)\n", _Output_GetSegmentStaticTranslation.Translation[ 0 ], _Output_GetSegmentStaticTranslation.Translation[ 1 ], _Output_GetSegmentStaticTranslation.Translation[ 2 ]);
            //fprintf(FID_debug, "        Static Translation: (%f, %f, %f)\n", _Output_GetSegmentStaticTranslation.Translation[ 0 ], _Output_GetSegmentStaticTranslation.Translation[ 1 ], _Output_GetSegmentStaticTranslation.Translation[ 2 ]);
            //fflush(FID_debug);
            
            // Get the static segment rotation in helical co-ordinates
            Output_GetSegmentStaticRotationHelical _Output_GetSegmentStaticRotationHelical = 
            MyClient.GetSegmentStaticRotationHelical( SubjectName, SegmentName );
            printf("        Static Rotation Helical: (%f, %f, %f)\n", _Output_GetSegmentStaticRotationHelical.Rotation[ 0 ], _Output_GetSegmentStaticRotationHelical.Rotation[ 1 ], _Output_GetSegmentStaticRotationHelical.Rotation[ 2 ]);
            //fprintf(FID_debug, "        Static Rotation Helical: (%f, %f, %f)\n", _Output_GetSegmentStaticRotationHelical.Rotation[ 0 ], _Output_GetSegmentStaticRotationHelical.Rotation[ 1 ], _Output_GetSegmentStaticRotationHelical.Rotation[ 2 ]);
            //fflush(FID_debug);
            
            // Get the static segment rotation as a matrix
            Output_GetSegmentStaticRotationMatrix _Output_GetSegmentStaticRotationMatrix = 
            MyClient.GetSegmentStaticRotationMatrix( SubjectName, SegmentName );
            printf("        Static Rotation Matrix: (%f, %f, %f\n", _Output_GetSegmentStaticRotationMatrix.Rotation[ 0 ], _Output_GetSegmentStaticRotationMatrix.Rotation[ 1 ], _Output_GetSegmentStaticRotationMatrix.Rotation[ 2 ]);
            printf("                                (%f, %f, %f\n", _Output_GetSegmentStaticRotationMatrix.Rotation[ 3 ], _Output_GetSegmentStaticRotationMatrix.Rotation[ 4 ], _Output_GetSegmentStaticRotationMatrix.Rotation[ 5 ]);
            printf("                                (%f, %f, %f\n", _Output_GetSegmentStaticRotationMatrix.Rotation[ 6 ], _Output_GetSegmentStaticRotationMatrix.Rotation[ 7 ], _Output_GetSegmentStaticRotationMatrix.Rotation[ 8 ]);
            //fprintf(FID_debug, "        Static Rotation Matrix: (%f, %f, %f\n", _Output_GetSegmentStaticRotationMatrix.Rotation[ 0 ], _Output_GetSegmentStaticRotationMatrix.Rotation[ 1 ], _Output_GetSegmentStaticRotationMatrix.Rotation[ 2 ]);
            //fprintf(FID_debug, "                                (%f, %f, %f\n", _Output_GetSegmentStaticRotationMatrix.Rotation[ 3 ], _Output_GetSegmentStaticRotationMatrix.Rotation[ 4 ], _Output_GetSegmentStaticRotationMatrix.Rotation[ 5 ]);
            //fprintf(FID_debug, "                                (%f, %f, %f\n", _Output_GetSegmentStaticRotationMatrix.Rotation[ 6 ], _Output_GetSegmentStaticRotationMatrix.Rotation[ 7 ], _Output_GetSegmentStaticRotationMatrix.Rotation[ 8 ]);
            //fflush(FID_debug);
            
            // Get the static segment rotation in quaternion co-ordinates
            Output_GetSegmentStaticRotationQuaternion _Output_GetSegmentStaticRotationQuaternion = 
            MyClient.GetSegmentStaticRotationQuaternion( SubjectName, SegmentName );
            printf("        Static Rotation Quaternion: (%f, %f, %f, %f)\n", _Output_GetSegmentStaticRotationQuaternion.Rotation[ 0 ], _Output_GetSegmentStaticRotationQuaternion.Rotation[ 1 ], _Output_GetSegmentStaticRotationQuaternion.Rotation[ 2 ], _Output_GetSegmentStaticRotationQuaternion.Rotation[ 3 ]);
            //fprintf(FID_debug, "        Static Rotation Quaternion: (%f, %f, %f, %f)\n", _Output_GetSegmentStaticRotationQuaternion.Rotation[ 0 ], _Output_GetSegmentStaticRotationQuaternion.Rotation[ 1 ], _Output_GetSegmentStaticRotationQuaternion.Rotation[ 2 ], _Output_GetSegmentStaticRotationQuaternion.Rotation[ 3 ]);
            //fflush(FID_debug);
            
            // Get the static segment rotation in EulerXYZ co-ordinates
            Output_GetSegmentStaticRotationEulerXYZ _Output_GetSegmentStaticRotationEulerXYZ = 
            MyClient.GetSegmentStaticRotationEulerXYZ( SubjectName, SegmentName );
            printf("        Static Rotation EulerXYZ: (%f, %f, %f)\n", _Output_GetSegmentStaticRotationEulerXYZ.Rotation[ 0 ], _Output_GetSegmentStaticRotationEulerXYZ.Rotation[ 1 ], _Output_GetSegmentStaticRotationEulerXYZ.Rotation[ 2 ]);
            //fprintf(FID_debug, "        Static Rotation EulerXYZ: (%f, %f, %f)\n", _Output_GetSegmentStaticRotationEulerXYZ.Rotation[ 0 ], _Output_GetSegmentStaticRotationEulerXYZ.Rotation[ 1 ], _Output_GetSegmentStaticRotationEulerXYZ.Rotation[ 2 ]);
            //fflush(FID_debug);
            
            // Get the global segment translation
            Output_GetSegmentGlobalTranslation _Output_GetSegmentGlobalTranslation = 
            MyClient.GetSegmentGlobalTranslation( SubjectName, SegmentName );
            printf("        Global Translation: (%f, %f, %f):%s\n", _Output_GetSegmentGlobalTranslation.Translation[ 0 ], _Output_GetSegmentGlobalTranslation.Translation[ 1 ], _Output_GetSegmentGlobalTranslation.Translation[ 2 ], AdaptOccluded(_Output_GetSegmentGlobalTranslation.Occluded ));
            //fprintf(FID_debug, "        Global Translation: (%f, %f, %f):%s\n", _Output_GetSegmentGlobalTranslation.Translation[ 0 ], _Output_GetSegmentGlobalTranslation.Translation[ 1 ], _Output_GetSegmentGlobalTranslation.Translation[ 2 ], AdaptOccluded(_Output_GetSegmentGlobalTranslation.Occluded ));
            //fflush(FID_debug);
            
            // Get the global segment rotation in helical co-ordinates
            Output_GetSegmentGlobalRotationHelical _Output_GetSegmentGlobalRotationHelical = 
            MyClient.GetSegmentGlobalRotationHelical( SubjectName, SegmentName );
            printf("        Global Rotation Helical: (%f, %f, %f):%s\n", _Output_GetSegmentGlobalRotationHelical.Rotation[ 0 ], _Output_GetSegmentGlobalRotationHelical.Rotation[ 1 ], _Output_GetSegmentGlobalRotationHelical.Rotation[ 2 ], AdaptOccluded(_Output_GetSegmentGlobalRotationHelical.Occluded));
            //fprintf(FID_debug, "        Global Rotation Helical: (%f, %f, %f):%s\n", _Output_GetSegmentGlobalRotationHelical.Rotation[ 0 ], _Output_GetSegmentGlobalRotationHelical.Rotation[ 1 ], _Output_GetSegmentGlobalRotationHelical.Rotation[ 2 ], AdaptOccluded(_Output_GetSegmentGlobalRotationHelical.Occluded));
            //fflush(FID_debug);
            
            // Get the global segment rotation as a matrix
            Output_GetSegmentGlobalRotationMatrix _Output_GetSegmentGlobalRotationMatrix = 
            MyClient.GetSegmentGlobalRotationMatrix( SubjectName, SegmentName );
            printf("        Global Rotation Matrix: (%f, %f, %f,\n", _Output_GetSegmentGlobalRotationMatrix.Rotation[ 0 ], _Output_GetSegmentGlobalRotationMatrix.Rotation[ 1 ], _Output_GetSegmentGlobalRotationMatrix.Rotation[ 2 ]);
            printf("                                (%f, %f, %f,\n", _Output_GetSegmentGlobalRotationMatrix.Rotation[ 3 ], _Output_GetSegmentGlobalRotationMatrix.Rotation[ 4 ], _Output_GetSegmentGlobalRotationMatrix.Rotation[ 5 ]);
            printf("                                (%f, %f, %f):%s\n", _Output_GetSegmentGlobalRotationMatrix.Rotation[ 6 ], _Output_GetSegmentGlobalRotationMatrix.Rotation[ 7 ], _Output_GetSegmentGlobalRotationMatrix.Rotation[ 8 ], AdaptOccluded(_Output_GetSegmentGlobalRotationMatrix.Occluded));
            //fprintf(FID_debug, "        Global Rotation Matrix: (%f, %f, %f,\n", _Output_GetSegmentGlobalRotationMatrix.Rotation[ 0 ], _Output_GetSegmentGlobalRotationMatrix.Rotation[ 1 ], _Output_GetSegmentGlobalRotationMatrix.Rotation[ 2 ]);
            //fprintf(FID_debug, "                                (%f, %f, %f,\n", _Output_GetSegmentGlobalRotationMatrix.Rotation[ 3 ], _Output_GetSegmentGlobalRotationMatrix.Rotation[ 4 ], _Output_GetSegmentGlobalRotationMatrix.Rotation[ 5 ]);
            //fprintf(FID_debug, "                                (%f, %f, %f):%s\n", _Output_GetSegmentGlobalRotationMatrix.Rotation[ 6 ], _Output_GetSegmentGlobalRotationMatrix.Rotation[ 7 ], _Output_GetSegmentGlobalRotationMatrix.Rotation[ 8 ], AdaptOccluded(_Output_GetSegmentGlobalRotationMatrix.Occluded));
            //fflush(FID_debug);
            
            // Get the global segment rotation in quaternion co-ordinates
            Output_GetSegmentGlobalRotationQuaternion _Output_GetSegmentGlobalRotationQuaternion = 
            MyClient.GetSegmentGlobalRotationQuaternion( SubjectName, SegmentName );
            printf("        Global Rotation Quaternion: (%f, %f, %f, %f):%s\n", _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 0 ], _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 1 ], _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 2 ], _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 3 ], AdaptOccluded( _Output_GetSegmentGlobalRotationQuaternion.Occluded));
            //fprintf(FID_debug, "        Global Rotation Quaternion: (%f, %f, %f, %f):%s\n", _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 0 ], _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 1 ], _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 2 ], _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 3 ], AdaptOccluded( _Output_GetSegmentGlobalRotationQuaternion.Occluded));
            //fflush(FID_debug);
            
            // Get the global segment rotation in EulerXYZ co-ordinates
            Output_GetSegmentGlobalRotationEulerXYZ _Output_GetSegmentGlobalRotationEulerXYZ = 
            MyClient.GetSegmentGlobalRotationEulerXYZ( SubjectName, SegmentName );
            printf("        Global Rotation EulerXYZ: (%f, %f, %f):%s\n", _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 0 ], _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 1 ], _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 2 ], AdaptOccluded(_Output_GetSegmentGlobalRotationEulerXYZ.Occluded));
            //fprintf(FID_debug, "        Global Rotation EulerXYZ: (%f, %f, %f):%s\n", _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 0 ], _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 1 ], _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 2 ], AdaptOccluded(_Output_GetSegmentGlobalRotationEulerXYZ.Occluded));
            //fflush(FID_debug);
            
            // Get the local segment translation
            Output_GetSegmentLocalTranslation _Output_GetSegmentLocalTranslation = 
            MyClient.GetSegmentLocalTranslation( SubjectName, SegmentName );
            printf("        Local Translation: (%f, %f, %f):%s\n", _Output_GetSegmentLocalTranslation.Translation[ 0 ], _Output_GetSegmentLocalTranslation.Translation[ 1 ], _Output_GetSegmentLocalTranslation.Translation[ 2 ], AdaptOccluded(_Output_GetSegmentLocalTranslation.Occluded));
            //fprintf(FID_debug, "        Local Translation: (%f, %f, %f):%s\n", _Output_GetSegmentLocalTranslation.Translation[ 0 ], _Output_GetSegmentLocalTranslation.Translation[ 1 ], _Output_GetSegmentLocalTranslation.Translation[ 2 ], AdaptOccluded(_Output_GetSegmentLocalTranslation.Occluded));
            //fflush(FID_debug);
            
            // Get the local segment rotation in helical co-ordinates
            Output_GetSegmentLocalRotationHelical _Output_GetSegmentLocalRotationHelical = 
            MyClient.GetSegmentLocalRotationHelical( SubjectName, SegmentName );
            printf("        Local Rotation Helical: (%f, %f, %f):%s\n", _Output_GetSegmentLocalRotationHelical.Rotation[ 0 ], _Output_GetSegmentLocalRotationHelical.Rotation[ 1 ], _Output_GetSegmentLocalRotationHelical.Rotation[ 2 ], AdaptOccluded(_Output_GetSegmentLocalRotationHelical.Occluded));
            //fprintf(FID_debug, "        Local Rotation Helical: (%f, %f, %f):%s\n", _Output_GetSegmentLocalRotationHelical.Rotation[ 0 ], _Output_GetSegmentLocalRotationHelical.Rotation[ 1 ], _Output_GetSegmentLocalRotationHelical.Rotation[ 2 ], AdaptOccluded(_Output_GetSegmentLocalRotationHelical.Occluded));
            //fflush(FID_debug);
            
            // Get the local segment rotation as a matrix
            Output_GetSegmentLocalRotationMatrix _Output_GetSegmentLocalRotationMatrix = 
            MyClient.GetSegmentLocalRotationMatrix( SubjectName, SegmentName );
            printf("        Local Rotation Matrix: (%f, %f, %f,\n", _Output_GetSegmentLocalRotationMatrix.Rotation[ 0 ], _Output_GetSegmentLocalRotationMatrix.Rotation[ 1 ], _Output_GetSegmentLocalRotationMatrix.Rotation[ 2 ]);
            printf("                               (%f, %f, %f,\n", _Output_GetSegmentLocalRotationMatrix.Rotation[ 3 ], _Output_GetSegmentLocalRotationMatrix.Rotation[ 4 ], _Output_GetSegmentLocalRotationMatrix.Rotation[ 5 ]);
            printf("                               (%f, %f, %f):%s\n", _Output_GetSegmentLocalRotationMatrix.Rotation[ 6 ], _Output_GetSegmentLocalRotationMatrix.Rotation[ 7 ], _Output_GetSegmentLocalRotationMatrix.Rotation[ 8 ], AdaptOccluded(_Output_GetSegmentLocalRotationMatrix.Occluded));
            //fprintf(FID_debug, "        Local Rotation Matrix: (%f, %f, %f,\n", _Output_GetSegmentLocalRotationMatrix.Rotation[ 0 ], _Output_GetSegmentLocalRotationMatrix.Rotation[ 1 ], _Output_GetSegmentLocalRotationMatrix.Rotation[ 2 ]);
            //fprintf(FID_debug, "                               (%f, %f, %f,\n", _Output_GetSegmentLocalRotationMatrix.Rotation[ 3 ], _Output_GetSegmentLocalRotationMatrix.Rotation[ 4 ], _Output_GetSegmentLocalRotationMatrix.Rotation[ 5 ]);
            //fprintf(FID_debug, "                               (%f, %f, %f):%s\n", _Output_GetSegmentLocalRotationMatrix.Rotation[ 6 ], _Output_GetSegmentLocalRotationMatrix.Rotation[ 7 ], _Output_GetSegmentLocalRotationMatrix.Rotation[ 8 ], AdaptOccluded(_Output_GetSegmentLocalRotationMatrix.Occluded));
            //fflush(FID_debug);
            
            // Get the local segment rotation in quaternion co-ordinates
            Output_GetSegmentLocalRotationQuaternion _Output_GetSegmentLocalRotationQuaternion = 
            MyClient.GetSegmentLocalRotationQuaternion( SubjectName, SegmentName );
            printf("        Local Rotation Quaternion: (%f, %f, %f)\n", _Output_GetSegmentLocalRotationQuaternion.Rotation[ 0 ], _Output_GetSegmentLocalRotationQuaternion.Rotation[ 1 ], _Output_GetSegmentLocalRotationQuaternion.Rotation[ 2 ], _Output_GetSegmentLocalRotationQuaternion.Rotation[ 3 ]);
            //fprintf(FID_debug, "        Local Rotation Quaternion: (%f, %f, %f)\n", _Output_GetSegmentLocalRotationQuaternion.Rotation[ 0 ], _Output_GetSegmentLocalRotationQuaternion.Rotation[ 1 ], _Output_GetSegmentLocalRotationQuaternion.Rotation[ 2 ], _Output_GetSegmentLocalRotationQuaternion.Rotation[ 3 ]);
            //fflush(FID_debug);
            
            // Get the local segment rotation in EulerXYZ co-ordinates
            Output_GetSegmentLocalRotationEulerXYZ _Output_GetSegmentLocalRotationEulerXYZ = 
            MyClient.GetSegmentLocalRotationEulerXYZ( SubjectName, SegmentName );
            printf("        Local Rotation EulerXYZ: (%f, %f, %f):%s\n", _Output_GetSegmentLocalRotationEulerXYZ.Rotation[ 0 ], _Output_GetSegmentLocalRotationEulerXYZ.Rotation[ 1 ], _Output_GetSegmentLocalRotationEulerXYZ.Rotation[ 2 ], AdaptOccluded(_Output_GetSegmentLocalRotationEulerXYZ.Occluded));
            //fprintf(FID_debug, "        Local Rotation EulerXYZ: (%f, %f, %f):%s\n", _Output_GetSegmentLocalRotationEulerXYZ.Rotation[ 0 ], _Output_GetSegmentLocalRotationEulerXYZ.Rotation[ 1 ], _Output_GetSegmentLocalRotationEulerXYZ.Rotation[ 2 ], AdaptOccluded(_Output_GetSegmentLocalRotationEulerXYZ.Occluded));
            //fflush(FID_debug);
        }
        
        // Count the number of markers
        unsigned int MarkerCount = MyClient.GetMarkerCount( SubjectName ).MarkerCount;
        printf("    Markers (%u)\n", MarkerCount);
        //fflush(stdout);
        //fprintf(FID_debug, "    Markers (%u)\n", MarkerCount);
        //fflush(FID_debug);
        for( unsigned int MarkerIndex = 0 ; MarkerIndex < MarkerCount ; ++MarkerIndex )
        {
            // Get the marker name
            std::string MarkerName = MyClient.GetMarkerName( SubjectName, MarkerIndex ).MarkerName;

            // Get the marker parent
            std::string MarkerParentName = MyClient.GetMarkerParentName( SubjectName, MarkerName ).SegmentName;

            // Get the global marker translation
            Output_GetMarkerGlobalTranslation _Output_GetMarkerGlobalTranslation =
            MyClient.GetMarkerGlobalTranslation( SubjectName, MarkerName );
            
            printf("      Marker #%u: %s (%f, %f, %f):%s\n", MarkerIndex, MarkerName, _Output_GetMarkerGlobalTranslation.Translation[ 0 ], _Output_GetMarkerGlobalTranslation.Translation[ 1 ], _Output_GetMarkerGlobalTranslation.Translation[ 2 ], AdaptOccluded(_Output_GetMarkerGlobalTranslation.Occluded));
            //fprintf(FID_debug, "      Marker #%u: %s (%f, %f, %f):%s\n", MarkerIndex, MarkerName, _Output_GetMarkerGlobalTranslation.Translation[ 0 ], _Output_GetMarkerGlobalTranslation.Translation[ 1 ], _Output_GetMarkerGlobalTranslation.Translation[ 2 ], AdaptOccluded(_Output_GetMarkerGlobalTranslation.Occluded));
            //fflush(FID_debug);
        }
      }

      // Get the unlabeled markers
      unsigned int UnlabeledMarkerCount = MyClient.GetUnlabeledMarkerCount().MarkerCount;
      printf("  Unlabeled Markers (%u)\n", UnlabeledMarkerCount);
      //fprintf(FID_debug, "  Unlabeled Markers (%u)\n", UnlabeledMarkerCount);
      //fflush(FID_debug);
      for( unsigned int UnlabeledMarkerIndex = 0 ; UnlabeledMarkerIndex < UnlabeledMarkerCount ; ++UnlabeledMarkerIndex )
      { 
            // Get the global marker translation
            Output_GetUnlabeledMarkerGlobalTranslation _Output_GetUnlabeledMarkerGlobalTranslation =
            MyClient.GetUnlabeledMarkerGlobalTranslation( UnlabeledMarkerIndex );

            printf("      Marker #%u: (%f, %f, %f)\n", UnlabeledMarkerIndex, _Output_GetUnlabeledMarkerGlobalTranslation.Translation[ 0 ], _Output_GetUnlabeledMarkerGlobalTranslation.Translation[ 1 ], _Output_GetUnlabeledMarkerGlobalTranslation.Translation[ 2 ]);
            //fprintf(FID_debug, "      Marker #%u: (%f, %f, %f)\n", _Output_GetUnlabeledMarkerGlobalTranslation.Translation[ 0 ], _Output_GetUnlabeledMarkerGlobalTranslation.Translation[ 1 ], _Output_GetUnlabeledMarkerGlobalTranslation.Translation[ 2 ]);
        //fflush(FID_debug);
      }
      
      // Count the number of devices
      unsigned int DeviceCount = MyClient.GetDeviceCount().DeviceCount;
      printf("  Devices (%u):\n", DeviceCount);
      for( unsigned int DeviceIndex = 0 ; DeviceIndex < DeviceCount ; ++DeviceIndex )
      {
            printf("    Device #%u", DeviceIndex);

            // Get the device name and type
            Output_GetDeviceName _Output_GetDeviceName = MyClient.GetDeviceName( DeviceIndex );
            printf("      Name: %s\n", _Output_GetDeviceName.DeviceName);
            printf("      Type: %s\n", AdaptDeviceType( _Output_GetDeviceName.DeviceType ));
        
            // Count the number of device outputs
            unsigned int DeviceOutputCount = MyClient.GetDeviceOutputCount( _Output_GetDeviceName.DeviceName ).DeviceOutputCount;
            printf("      Device Outputs (%u):\n", DeviceOutputCount);
            for( unsigned int DeviceOutputIndex = 0 ; DeviceOutputIndex < DeviceOutputCount ; ++DeviceOutputIndex )
            {
                // Get the device output name and unit
                Output_GetDeviceOutputName _Output_GetDeviceOutputName = 
                MyClient.GetDeviceOutputName( _Output_GetDeviceName.DeviceName, DeviceOutputIndex );

                // Get the device output value
                Output_GetDeviceOutputValue _Output_GetDeviceOutputValue = 
                MyClient.GetDeviceOutputValue( _Output_GetDeviceName.DeviceName, _Output_GetDeviceOutputName.DeviceOutputName );

                printf("        Device Output #%u: %s %f %s %s\n", DeviceOutputIndex, _Output_GetDeviceOutputName.DeviceOutputName, _Output_GetDeviceOutputValue.Value, AdaptUnit( _Output_GetDeviceOutputName.DeviceOutputUnit ), AdaptOccluded( _Output_GetDeviceOutputValue.Occluded ));
            }
      }
      
      // Output the force plate information.
      unsigned int ForcePlateCount = MyClient.GetForcePlateCount().ForcePlateCount;
      printf("Force Plates: %u\n", ForcePlateCount);

      for( unsigned int ForcePlateIndex = 0 ; ForcePlateIndex < ForcePlateCount ; ++ForcePlateIndex )
      {
            printf("    Force Plate #%u\n", ForcePlateIndex);
            //fprintf(FID_debug, "    Force Plate #%u\n", ForcePlateIndex);
            //fflush(FID_debug);

            Output_GetGlobalForceVector _Output_GetForceVector = MyClient.GetGlobalForceVector( ForcePlateIndex );
            printf("  Force:\n");
            printf("  X: %f\n", _Output_GetForceVector.ForceVector[ 0 ]);
            printf("  Y: %f\n", _Output_GetForceVector.ForceVector[ 1 ]);
            printf("  Z: %f\n", _Output_GetForceVector.ForceVector[ 2 ]);
            //fprintf(FID_debug,"  Force:\n");
            //fprintf(FID_debug, "  X: %f\n", _Output_GetForceVector.ForceVector[ 0 ]);
            //fprintf(FID_debug,"  Y: %f\n", _Output_GetForceVector.ForceVector[ 1 ]);
            //fprintf(FID_debug,"  Z: %f\n", _Output_GetForceVector.ForceVector[ 2 ]);
            //fflush(FID_debug);
            
            Output_GetGlobalMomentVector _Output_GetMomentVector = MyClient.GetGlobalMomentVector( ForcePlateIndex );
            printf("  Moment:\n");
            printf("  X: %f\n", _Output_GetMomentVector.MomentVector[ 0 ]);
            printf("  Y: %f\n", _Output_GetMomentVector.MomentVector[ 1 ]);
            printf("  Z: %f\n", _Output_GetMomentVector.MomentVector[ 2 ]);
            //fprintf(FID_debug,"  Moment:\n");
            //fprintf(FID_debug,"  X: %f\n", _Output_GetMomentVector.MomentVector[ 0 ]);
            //fprintf(FID_debug,"  Y: %f\n", _Output_GetMomentVector.MomentVector[ 1 ]);
            //fprintf(FID_debug,"  Z: %f\n", _Output_GetMomentVector.MomentVector[ 2 ]);
            //fflush(FID_debug);
            
            Output_GetGlobalCentreOfPressure _Output_GetCentreOfPressure = MyClient.GetGlobalCentreOfPressure( ForcePlateIndex );
            printf("  Centre Of Pressure:\n");
            printf("  X: %f\n", _Output_GetCentreOfPressure.CentreOfPressure[ 0 ]);
            printf("  Y: %f\n", _Output_GetCentreOfPressure.CentreOfPressure[ 1 ]);
            printf("  Z: %f\n", _Output_GetCentreOfPressure.CentreOfPressure[ 2 ]);
            //fprintf(FID_debug,"  Centre Of Pressure:\n");
            //fprintf(FID_debug,"  X: %f\n", _Output_GetCentreOfPressure.CentreOfPressure[ 0 ]);
            //fprintf(FID_debug,"  Y: %f\n", _Output_GetCentreOfPressure.CentreOfPressure[ 1 ]);
            //fprintf(FID_debug,"  Z: %f\n", _Output_GetCentreOfPressure.CentreOfPressure[ 2 ]);
            //fflush(FID_debug);
      }
 
}

// format adaptation
std::string AdaptDeviceType( const DeviceType::Enum i_DeviceType )
  {
    switch( i_DeviceType )
    {
      case DeviceType::ForcePlate:
        return "ForcePlate";
      case DeviceType::Unknown:
      default:
        return "Unknown";
    }
  }

std::string AdaptDirection( const Direction::Enum i_Direction )
  {
    switch( i_Direction )
    {
      case Direction::Forward:
        return "Forward";
      case Direction::Backward:
        return "Backward";
      case Direction::Left:
        return "Left";
      case Direction::Right:
        return "Right";
      case Direction::Up:
        return "Up";
      case Direction::Down:
        return "Down";
      default:
        return "Unknown";
    }
  }

std::string AdaptOccluded( const bool i_Value )
  {
    if (i_Value)
        return "Visible";
    else
        return "Occluded";
  }

std::string AdaptEnable( const bool i_Value )
  {
    if (i_Value)
        return "Enable";
    else
        return "Disable";
  }

std::string AdaptUnit( const Unit::Enum i_Unit )
  {
    switch( i_Unit )
    {
      case Unit::Meter:
        return "Meter";
      case Unit::Volt:
        return "Volt";
      case Unit::NewtonMeter:
        return "NewtonMeter";
      case Unit::Newton:
        return "Newton";
      case Unit::Unknown:
      default:
        return "Unknown";
    }
  }


/*************************************************************************/


#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif


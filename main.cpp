// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.
#include <iostream>
#include <signal.h>

#include <librealsense/rs.hpp>
#include "rs_sdk.h"
#include "version.h"
#include "pt_utils.hpp"
#include "pt_console_display.hpp"
#include "main.h"

using namespace std;

// Version number of the samples
extern constexpr auto rs_sample_version = concat("VERSION: ",RS_SAMPLE_VERSION_STR);

int detectGestures(Intel::RealSense::PersonTracking::PersonTrackingData::PersonJoints *personJoints);

int main(int argc, char** argv)
{
    state_e state = STATE_INIT;

    pt_utils pt_utils;
    unique_ptr<console_display::pt_console_display> console_view = move(console_display::make_console_pt_display());

    rs::core::video_module_interface::actual_module_config actualModuleConfig;
    rs::person_tracking::person_tracking_video_module_interface* ptModule = nullptr;


    // Initializing Camera and Person Tracking modules
    if(pt_utils.init_camera(actualModuleConfig) != rs::core::status_no_error)
    {
        cerr << "Error: Device is null." << endl << "Please connect a RealSense device and restart the application" << endl;
        return -1;
    }
    pt_utils.init_person_tracking(&ptModule);

    //Enable Pointing Gesture
    ptModule->QueryConfiguration()->QueryGestures()->Enable();
    ptModule->QueryConfiguration()->QueryGestures()->EnableAllGestures();
    ptModule->QueryConfiguration()->QueryTracking()->Enable();
    ptModule->QueryConfiguration()->QuerySkeletonJoints()->Enable();


    // Configure enabled Pointing Gesture
    if(ptModule->set_module_config(actualModuleConfig) != rs::core::status_no_error)
    {
        cerr<<"Error : Failed to configure the enabled Pointing Gesture" << endl;
        return -1;
    }

    // Start the camera
    pt_utils.start_camera();

    cout << endl << "-------- Press Esc key to exit --------" << endl << endl;

    state = STATE_IDLE;
    Intel::RealSense::PersonTracking::PersonTrackingData *trackingData = ptModule->QueryOutput();
    Intel::RealSense::PersonTracking::PersonTrackingData::PersonJoints *personJoints = nullptr;

    // Start main loop
    while(!pt_utils.user_request_exit())
    {
        rs::core::correlated_sample_set sampleSet = {};

        // Get next frame
        if (pt_utils.GetNextFrame(sampleSet) != 0)
        {
            cerr << "Error: Invalid frame" << endl;
            continue;
        }

        // Process frame
        if (ptModule->process_sample_set(sampleSet) != rs::core::status_no_error)
        {
            cerr << "Error : Failed to process sample" << endl;
            continue;
        }


        // Display color image
        auto colorImage = sampleSet[rs::core::stream_type::color];
        console_view->render_color_frames(colorImage);

        // Release color and depth image
        sampleSet.images[static_cast<uint8_t>(rs::core::stream_type::color)]->release();
        sampleSet.images[static_cast<uint8_t>(rs::core::stream_type::depth)]->release();

        // Main program FSM implementation
        int numTracked;
        switch (state)
        {
            case STATE_IDLE:
                numTracked = trackingData->QueryNumberOfPeople();
                cout << numTracked;
                if(numTracked == 1)
                {
                    // If we are tracking exactly one person, detect their gesture
                    console_view->set_tracking(ptModule);
                    state = STATE_READY;
                }

            break;

            case STATE_READY:
            // Track skeleton joints
            if(trackingData->QueryNumberOfPeople() == 0)
            {
                // If we no longer see a person, back to idle state
                state = STATE_IDLE;
            }
            else
            {
                // Start tracking the first person detected in the frame
                console_view->on_person_skeleton(ptModule);
                personJoints = console_view->on_person_skeleton(ptModule);
                detectGestures(personJoints);
            }

            break;

        case STATE_PLAYBACK:
            // @TODO Issue system call to playback video content over HDMI
            // system(...);

            // @TODO
            // If we are still detecting a person, listen for cancel gesture
            break;



        }

    }

    pt_utils.stop_camera();
    actualModuleConfig.projection->release();
    cout << "-------- Stopping --------" << endl;
    return 0;
}

int detectGestures(Intel::RealSense::PersonTracking::PersonTrackingData::PersonJoints *personJoints)
{


    int numDetectedJoints = personJoints->QueryNumJoints();
    std::vector<Intel::RealSense::PersonTracking::PersonTrackingData::PersonJoints::SkeletonPoint> skeletonPoints(personJoints->QueryNumJoints());

    personJoints->QueryJoints(skeletonPoints.data());

    //cout << "Number of Joints detected: " << numDetectedJoints << endl;

         // Temporary Decleration. Might be better to declare as struct
         int Lhandx;
         int Lhandy;
         int Lshoulderx;
         int Lshouldery;


         int Rhandx;
         int Rhandy;
         int Rshoulderx;
         int Rshouldery;

    for(int i = 0; i < numDetectedJoints ; i++) {

                     Lhandx = skeletonPoints.at(0).image.x;
                     Lhandy = skeletonPoints.at(0).image.y;
                     Rhandx = skeletonPoints.at(1).image.x;
                     Rhandy = skeletonPoints.at(1).image.y;

                     Lshoulderx = skeletonPoints.at(4).image.x;
                     Lshouldery = skeletonPoints.at(4).image.y;
                     Rshoulderx = skeletonPoints.at(5).image.x;
                     Rshouldery = skeletonPoints.at(5).image.y;




                 // 6: LH, 7: RH, 10: H, 19: S, 16: LS, 17: RS
                 cout << "LH: " << skeletonPoints.at(0).image.x << ", " << skeletonPoints.at(0).image.y
                      << "|RH: " << skeletonPoints.at(1).image.x << ", " << skeletonPoints.at(1).image.y
                      << "|LS: " << skeletonPoints.at(4).image.x << ", " << skeletonPoints.at(4).image.y
                      << "|RS: " << skeletonPoints.at(5).image.x << ", " << skeletonPoints.at(5).image.y
                      //<< "|H: " << skeletonPoints.at(2).image.x << ", " << skeletonPoints.at(2).image.y
                      //<< "|S: " << skeletonPoints.at(3).image.x << ", " << skeletonPoints.at(3).image.y
                      << endl;


                 //Power pose
                 if( ((Lshoulderx - Lhandx) <= 40) &&
                     ((Lshoulderx - Lhandx) > 0 ) &&
                     ((Lshouldery - Lhandy) <= 40) &&
                     ((Lshouldery - Lhandy) > 0) &&
                     ((Rshoulderx - Rhandx) <= 0) &&
                     ((Rshoulderx - Rhandx) > -40 ) &&
                     ((Rshouldery - Rhandy) <= 40) &&
                     ((Rshouldery - Rhandy) > 0)
                   )
                 {
                         cout << "Power Pose Detected!" << endl << endl;
                         //system("firefox goo.gl/xZPVb9");
                 }

                 if( ((Lshoulderx - Lhandx) <= 30) &&
                     ((Lshoulderx - Lhandx) >= 10 ) &&
                     ((Lshouldery - Lhandy) <= 120) &&
                     ((Lshouldery - Lhandy) >= 100) &&
                     ((Rshoulderx - Rhandx) <= 0) &&
                     ((Rshoulderx - Rhandx) >= -20 ) &&
                     ((Rshouldery - Rhandy) <= 120) &&
                     ((Rshouldery - Rhandy) >= 100)
                   )
                 {
                        cout << "Touch the Sky Pose Detected!" << endl << endl;
                        //system("firefox goo.gl/xipLSq");
                 }

                 //Usain Bolt Pose (to the left)
                 if( ((Lshoulderx - Lhandx) <= 20) &&
                     ((Lshoulderx - Lhandx) >= -20 ) &&
                     ((Lshouldery - Lhandy) <= -20) &&
                     ((Lshouldery - Lhandy) >= -60) &&
                     ((Rshoulderx - Rhandx) <= -50) &&
                     ((Rshoulderx - Rhandx) >= -100 ) &&
                     ((Rshouldery - Rhandy) <= 50) &&
                     ((Rshouldery - Rhandy) >= -10)
                   )
                 {
                        cout << "Usain Bolt Pose Detected!" << endl << endl;
                        //system("firefox goo.gl/xipLSq");
                 }

                 if( ((Lshoulderx - Lhandx) <= 90) &&
                     ((Lshoulderx - Lhandx) >= 70 ) &&
                     ((Lshouldery - Lhandy) <= -20) &&
                     ((Lshouldery - Lhandy) >= -30) &&
                     ((Rshoulderx - Rhandx) <= -90) &&
                     ((Rshoulderx - Rhandx) >= -110 ) &&
                     ((Rshouldery - Rhandy) <= 0) &&
                     ((Rshouldery - Rhandy) >= -20)
                   )
                 {
                        cout << "T Pose Detected!" << endl << endl;
                        //system("firefox goo.gl/xipLSq");
                 }

                 if( ((Lshoulderx - Lhandx) <= 5) &&
                     ((Lshoulderx - Lhandx) >= -35 ) &&
                     ((Lshouldery - Lhandy) <= 25) &&
                     ((Lshouldery - Lhandy) >= 5) &&
                     ((Rshoulderx - Rhandx) <= 5) &&
                     ((Rshoulderx - Rhandx) >= -15 ) &&
                     ((Rshouldery - Rhandy) <= 5) &&
                     ((Rshouldery - Rhandy) >= -15)
                   )
                 {
                        cout << "O Pose Detected!" << endl << endl;
                        //system("firefox goo.gl/xipLSq");
                 }

    }

    return 0;
}

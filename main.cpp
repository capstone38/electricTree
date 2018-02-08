// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

// C++ std libraries
#include <iostream>
#include <signal.h>
#include <thread>

// Boost Libraries
#include <boost/interprocess/ipc/message_queue.hpp>

// Realsense libraries
#include <librealsense/rs.hpp>
#include "rs_sdk.h"
#include "version.h"
#include "pt_utils.hpp"
#include "pt_console_display.hpp"


#include "main.h"



using namespace std;
using namespace boost::interprocess;

// Version number of the samples
extern constexpr auto rs_sample_version = concat("VERSION: ",RS_SAMPLE_VERSION_STR);

int main(int argc, char** argv)
{
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
    Intel::RealSense::PersonTracking::PersonTrackingData *trackingData = ptModule->QueryOutput();
    Intel::RealSense::PersonTracking::PersonTrackingData::PersonJoints *personJoints = nullptr;

    // Init interprocess message queue
    message_queue::remove("etree_message_queue");
    message_queue mq
            (create_only,
             "etree_message_queue",
             1,
             sizeof(bool));
    unsigned int priority;
    message_queue::size_type recvd_size;

    state_e state = STATE_IDLE;
    int numTracked;
    gestures_e gestureDetected = GESTURE_UNDEFINED;
    bool playbackFinished = false;

    // Start main loop
    while(!pt_utils.user_request_exit())
    {
        // Check for cancel request from system
        bool shouldQuit = false;
        mq.try_receive(&shouldQuit, sizeof(shouldQuit), recvd_size, priority);
        if(shouldQuit) break;

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
        switch (state)
        {
            case STATE_IDLE:
                numTracked = trackingData->QueryNumberOfPeople();
               //cout << "In idle state. " << numTracked << " people detected." << endl;

                if(numTracked == 1) // numTracked >= 1?
                {
                    // If we are tracking exactly one person, detect their gesture
                    cout << "found someone!" << endl;
                    int personId = trackingData->QueryPersonData(
                                Intel::RealSense::PersonTracking::PersonTrackingData::ACCESS_ORDER_BY_INDEX, 0)->QueryTracking()->QueryId();
                    cout << "ID before clearing database: " << personId << endl;

                    auto config = ptModule->QueryConfiguration()->QueryRecognition();
                    auto database = config->QueryDatabase();
                    database->Clear();

                    personId = trackingData->QueryPersonData(
                                                    Intel::RealSense::PersonTracking::PersonTrackingData::ACCESS_ORDER_BY_INDEX, 0)->QueryTracking()->QueryId();
                                        cout << "ID after clearing database: " << personId << endl;

                    if(personId == 0)
                    {
                        console_view->set_tracking(ptModule);
                        state = STATE_READY;
                    }
                }

            break;

        case STATE_READY:
            // Track skeleton joints
            if(trackingData->QueryNumberOfPeople() != 1)
            {
                // If we no longer see a person, back to idle state
                state = STATE_IDLE;
                //trackingData->StopTracking(0);

            }
            else
            {
                // Start tracking the first person detected in the frame
                personJoints = console_view->on_person_skeleton(ptModule);
                gestureDetected = detectGestures(personJoints);

                if(gestureDetected != GESTURE_UNDEFINED && gestureDetected != GESTURE_CANCEL)
                {
                    state = STATE_PLAYBACK_START;
                }
            }

            break;

        case STATE_PLAYBACK_START:
            // Issue system call to playback video content in a detached thread
            playbackFinished = false;
            {
                thread video(playContent, gestureDetected, ref(playbackFinished));
                video.detach();
            }

            state = STATE_PLAYBACK_UNDERWAY;
            break;

        case STATE_PLAYBACK_UNDERWAY:
            // If we are still detecting a person, listen for cancel gesture
            numTracked = trackingData->QueryNumberOfPeople();

            if(numTracked == 1)
            {
                personJoints = console_view->on_person_skeleton(ptModule);
                gestureDetected = detectGestures(personJoints);

                // @TODO Implement cancel gesture. Currently playback will cancel as soon as a person is tracked.
                if(gestureDetected == GESTURE_CANCEL)
                {
                   system("killall vlc");
                }
            }

            if(playbackFinished)
            {
                cout << "playback completed or killed!" << endl;
                state = STATE_READY;
            }
            break;
        }
    }

    pt_utils.stop_camera();
    actualModuleConfig.projection->release();
    cout << "-------- Stopping --------" << endl;
    return 0;
}

gestures_e detectGestures(Intel::RealSense::PersonTracking::PersonTrackingData::PersonJoints *personJoints)
{


    int numDetectedJoints = personJoints->QueryNumJoints();
    std::vector<Intel::RealSense::PersonTracking::PersonTrackingData::PersonJoints::SkeletonPoint> skeletonPoints(personJoints->QueryNumJoints());

    personJoints->QueryJoints(skeletonPoints.data());

    jointCoords_t jointCoords;

    //cout << "num detected joints" << numDetectedJoints << endl;

    //cout << skeletonPoints.at(0).image.x << endl;

    for(int i = 0; i < numDetectedJoints ; i++) {
                     // Populate joint coordinates values
                     jointCoords.Lhandx = skeletonPoints.at(0).image.x;
                     jointCoords.Lhandy = skeletonPoints.at(0).image.y;
                     jointCoords.Rhandx = skeletonPoints.at(1).image.x;
                     jointCoords.Rhandy = skeletonPoints.at(1).image.y;

                     jointCoords.Lshoulderx = skeletonPoints.at(4).image.x;
                     jointCoords.Lshouldery = skeletonPoints.at(4).image.y;
                     jointCoords.Rshoulderx = skeletonPoints.at(5).image.x;
                     jointCoords.Rshouldery = skeletonPoints.at(5).image.y;


                 // check for each pose sequentially.
                 // some way to make this cleaner, perhaps a helper function with just jointCoords struct as only parameter?

                 //Power pose
                 if( ((jointCoords.Lshoulderx - jointCoords.Lhandx) <= 40) &&
                     ((jointCoords.Lshoulderx - jointCoords.Lhandx) > 0 ) &&
                     ((jointCoords.Lshouldery - jointCoords.Lhandy) <= 40) &&
                     ((jointCoords.Lshouldery - jointCoords.Lhandy) > 0) &&
                     ((jointCoords.Rshoulderx - jointCoords.Rhandx) <= 0) &&
                     ((jointCoords.Rshoulderx - jointCoords.Rhandx) > -40 ) &&
                     ((jointCoords.Rshouldery - jointCoords.Rhandy) <= 40) &&
                     ((jointCoords.Rshouldery - jointCoords.Rhandy) > 0)
                   )
                 {
                         cout << "Power Pose Detected!" << endl << endl;
                         return GESTURE_POWERPOSE;
                 }

                 if( ((jointCoords.Lshoulderx - jointCoords.Lhandx) <= 70) &&
                     ((jointCoords.Lshoulderx - jointCoords.Lhandx) >= 35 ) &&
                     ((jointCoords.Lshouldery - jointCoords.Lhandy) <= 90) &&
                     ((jointCoords.Lshouldery - jointCoords.Lhandy) >= 50) &&
                     ((jointCoords.Rshoulderx - jointCoords.Rhandx) <= -20) &&
                     ((jointCoords.Rshoulderx - jointCoords.Rhandx) >= -60 ) &&
                     ((jointCoords.Rshouldery - jointCoords.Rhandy) <= 80) &&
                     ((jointCoords.Rshouldery - jointCoords.Rhandy) >= 50)
                   )
                 {
                        cout << "Victory Pose Detected!" << endl << endl;
                        return GESTURE_SKY;
                 }

                 //Usain Bolt Pose (to the left)
                 if( ((jointCoords.Lshoulderx - jointCoords.Lhandx) <= 45) &&
                     ((jointCoords.Lshoulderx - jointCoords.Lhandx) >= 20 ) &&
                     ((jointCoords.Lshouldery - jointCoords.Lhandy) <= -10) &&
                     ((jointCoords.Lshouldery - jointCoords.Lhandy) >= -45) &&
                     ((jointCoords.Rshoulderx - jointCoords.Rhandx) <= -30) &&
                     ((jointCoords.Rshoulderx - jointCoords.Rhandx) >= -90 ) &&
                     ((jointCoords.Rshouldery - jointCoords.Rhandy) <= 50) &&
                     ((jointCoords.Rshouldery - jointCoords.Rhandy) >= 15)
                   )
                 {
                        cout << "Usain Bolt Pose Detected!" << endl << endl;
                        return GESTURE_USAIN;
                 }

                 // T Pose Gesture
                 if( //((jointCoords.Lshoulderx - jointCoords.Lhandx) <= 100) &&
                     ((jointCoords.Lshoulderx - jointCoords.Lhandx) >= 75 ) &&
                     ((jointCoords.Lshouldery - jointCoords.Lhandy) <= 10) &&
                     ((jointCoords.Lshouldery - jointCoords.Lhandy) >= -10) &&
                     //((jointCoords.Rshoulderx - jointCoords.Rhandx) <= -80) &&
                     ((jointCoords.Rshoulderx - jointCoords.Rhandx) >= -110 ) &&
                     ((jointCoords.Rshouldery - jointCoords.Rhandy) <= 5) &&
                     ((jointCoords.Rshouldery - jointCoords.Rhandy) >= -15)
                   )
                 {
                        cout << "T Pose Detected!" << endl << endl;
                        return GESTURE_T;
                 }

                 if( ((jointCoords.Lshoulderx - jointCoords.Lhandx) <= 15) &&
                     ((jointCoords.Lshoulderx - jointCoords.Lhandx) >= -10 ) &&
                     ((jointCoords.Lshouldery - jointCoords.Lhandy) <= 65) &&
                     ((jointCoords.Lshouldery - jointCoords.Lhandy) >= 55) &&
                     ((jointCoords.Rshoulderx - jointCoords.Rhandx) <= 15) &&
                     ((jointCoords.Rshoulderx - jointCoords.Rhandx) >= -5 ) &&
                     ((jointCoords.Rshouldery - jointCoords.Rhandy) <= 70) &&
                     ((jointCoords.Rshouldery - jointCoords.Rhandy) >= 55)
                   )
                 {
                        cout << "O Pose Detected!" << endl << endl;
                        return GESTURE_0;
                 }

    }

    //printJointCoords(jointCoords);

    return GESTURE_UNDEFINED;
}

void printJointCoords(jointCoords_t& jc)
{
    // 6: LH, 7: RH, 10: H, 19: S, 16: LS, 17: RS
    cout << "LH: " << jc.Lhandx << ", " << jc.Lhandy
         << "|RH: " << jc.Rhandx << ", " << jc.Rhandy
         << "|LS: " << jc.Lshoulderx << ", " << jc.Lshouldery
         << "|RS: " << jc.Rshoulderx << ", " << jc.Rshouldery
         //<< "|H: " << skeletonPoints.at(2).image.x << ", " << skeletonPoints.at(2).image.y
         //<< "|S: " << skeletonPoints.at(3).image.x << ", " << skeletonPoints.at(3).image.y
         << endl;
}

void playContent(gestures_e gesture, bool &finished)
{
    // Play the specified video in fullscreen mode and close vlc when finished
    // (We should use this in our production code)
    //system("cvlc -f --play-and-exit file:///home/zac/electricTree/videos/test.mov");

    // Play the specified video on a loop (useful for testing cancel gesture)
    system("cvlc -R file:///home/zac/electricTree/videos/test.mov");

    // Set finished to true. This is a reference so its value will be seen in the main loop.
    finished = true;
    cout << "playback completed! " << endl;
}

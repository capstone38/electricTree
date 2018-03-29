// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

// C++ std libraries
#include <iostream>
#include <signal.h>
#include <thread>
#include <cstdlib>
#include <ctime>

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

vector<int> jumpVector;
int jumpCount = 0;
int jumpStartValue;


bool analyticsOn = true;
int result; // file renaming and removing error checking
char filename[] = "/home/zac/electricTree/analytics.txt";
char temp_filename[] = "/home/zac/electricTree/temp_analytics.txt";

int main(int argc, char** argv)
{
    pt_utils pt_utils;
    unique_ptr<console_display::pt_console_display> console_view = move(console_display::make_console_pt_display());

    rs::core::video_module_interface::actual_module_config actualModuleConfig;
    rs::person_tracking::person_tracking_video_module_interface* ptModule = nullptr;
    Intel::RealSense::PersonTracking::PersonTrackingData::Person *personData = nullptr;

    // Auto-Exposure Setup
    rs_context *ctx = rs_create_context(RS_API_VERSION, 0);
    rs_device *camera = rs_get_device(ctx, 0, 0);

    cout << "Enabled: " << rs_get_device_option(camera, RS_OPTION_R200_LR_AUTO_EXPOSURE_ENABLED, 0) << " || ";

//    rs_set_device_option(camera, RS_OPTION_COLOR_ENABLE_AUTO_EXPOSURE, 1, 0);
    rs_set_device_option(camera, RS_OPTION_R200_LR_AUTO_EXPOSURE_ENABLED, 1, 0);

//    rs_set_device_option(camera, RS_OPTION_R200_LR_GAIN  , 100, 0); //400
//    rs_set_device_option(camera, RS_OPTION_R200_LR_EXPOSURE , 7, 0); //164

//    rs_set_device_option(camera, RS_OPTION_R200_AUTO_EXPOSURE_MEAN_INTENSITY_SET_POINT, 1600, 0); //512 needs auto exposure enabled
//    rs_set_device_option(camera, RS_OPTION_R200_EMITTER_ENABLED, 1, 0);

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
        cerr<<"Error: Failed to configure the enabled Pointing Gesture" << endl;
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

    // Init RNG
    srand(time(nullptr));

    state_e state = STATE_IDLE;
    int cyclesSpentIdle = 0;
    int numTracked;
    gestures_e gestureDetected = GESTURE_UNDEFINED;
    gesture_states_t gesture_states;
    resetGestureStates(gesture_states);
    int cyclesSpentDetected = 0;

    int pid_in_center = INVALID_PERSONID;
    int pid_in_center_prev;

    bool shouldCancel = false;

    // move intialization to diff file
    powerpose_count = 0;
    t_count = 0;
    victory_count = 0;
    usain_count = 0;
    stop_count = 0;
    flying_count = 0;
    waving_r_count = 0;
    waving_l_count = 0;
    pointing_trf_count = 0;
    pointing_rf_count = 0;
    pointing_tlf_count = 0;
    pointing_lf_count = 0;
    pointing_tr_count = 0;
    pointing_r_count = 0;
    pointing_tl_count = 0;
    pointing_l_count = 0;

    // Start main loop
    while(!pt_utils.user_request_exit())
    {
        // Auto-exposure Prints
//        cout << "Enabled: " << rs_get_device_option(camera, RS_OPTION_R200_LR_AUTO_EXPOSURE_ENABLED, 0) << " || ";
//        cout << "Exposure: " << rs_get_device_option(camera, RS_OPTION_R200_LR_EXPOSURE, 0) << " || ";
//        cout << "Gain: " << rs_get_device_option(camera, RS_OPTION_R200_LR_GAIN, 0) << endl;

        // Check for cancel request from system
        bool shouldQuit = false;
        mq.try_receive(&shouldQuit, sizeof(shouldQuit), recvd_size, priority);
        if(shouldQuit) {
            updateAnalytics(analyticsOn, gesture_states);
            break;
        }

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


        trackingData = ptModule->QueryOutput();

        auto ids_in_frame = console_view->get_person_ids(ptModule->QueryOutput());

        pid_in_center_prev = pid_in_center;

        for(auto iter = ids_in_frame->begin(); iter != ids_in_frame->end(); ++iter){
            int id = *iter;
            //cout<< "id: "<<id<<endl;

            personData = trackingData->QueryPersonDataById(id);

            if(personData){
                Intel::RealSense::PersonTracking::PersonTrackingData::PersonTracking* personTrackData = personData->QueryTracking();
                Intel::RealSense::PersonTracking::PersonTrackingData::PointCombined centerMass = personTrackData->QueryCenterMass();


                if(personIsInCenter(centerMass))
                {
                    // If person in center, say which id
                    pid_in_center = id;
                    break;
                }
                else
                {
                    // If no one in center, indicate so.
                    pid_in_center = INVALID_PERSONID;
                    jumpVector.clear();
                    jumpCount=0;
                }
            }
            else
            {
                // If we no longer see a person, back to idle state
                pid_in_center = INVALID_PERSONID;
                jumpVector.clear();
                jumpCount=0;
            }
        }

        if(pid_in_center != pid_in_center_prev)
        {
            trackingData->StopTracking(pid_in_center_prev);

            if(pid_in_center != INVALID_PERSONID)
            {
                trackingData->StartTracking(pid_in_center);
            }
        }


        //Main program FSM implementation
        switch (state)
        {
        case STATE_IDLE:
            numTracked = trackingData->QueryNumberOfPeople();

            if(pid_in_center != INVALID_PERSONID)
            {
                // If we are tracking exactly one person, detect their gesture
                cout << "found someone!" << endl;
                cout<<"Person ID: " << pid_in_center <<" is in center!" <<endl<<endl;

                if(cyclesSpentDetected >= SEC_TO_CYCLES(3)) {
                    state = STATE_READY;

                    {
                        thread video(playContent, GESTURE_READY, shouldQuit);
                        video.detach();
                    }
                    break;
                } else {
                    cyclesSpentDetected++;
                }
            }
            else if(cyclesSpentIdle >= SEC_TO_CYCLES(0))
            {
                // We have idled for 10 seconds. Begin playback of idle video
                state = STATE_IDLEVIDEO_START;

                cyclesSpentDetected = 0;
            }

            cyclesSpentIdle++;
            break;

        case STATE_READY:
            // Track skeleton joints of person in centre,
            // or go back to idle if person has left centre

            if(pid_in_center != INVALID_PERSONID)
            {
                personJoints = personData->QuerySkeletonJoints();
                std::vector<Intel::RealSense::PersonTracking::PersonTrackingData::PersonJoints::SkeletonPoint> skeletonPoints(personJoints->QueryNumJoints());
                personJoints->QueryJoints(skeletonPoints.data());

                gestureDetected = detectGestures(personJoints, gesture_states);

                if(gestureDetected != GESTURE_UNDEFINED && gestureDetected != GESTURE_CANCEL)
                {

                    state = STATE_PLAYBACK_START;
                }
            }
            else
            {
                state = STATE_IDLE;
                cyclesSpentIdle = 0;
            }
            break;

        case STATE_PLAYBACK_START:
            // Issue system call to playback video content in a detached thread

        {
            thread video(playContent, gestureDetected, shouldQuit);
            video.detach();
        }
            shouldCancel = false;

            state = STATE_PLAYBACK_UNDERWAY;
            break;

        case STATE_PLAYBACK_UNDERWAY:
            // If we are still detecting a person, listen for cancel gesture

            if(pid_in_center != INVALID_PERSONID)
            {
                personJoints = personData->QuerySkeletonJoints();
                std::vector<Intel::RealSense::PersonTracking::PersonTrackingData::PersonJoints::SkeletonPoint> skeletonPoints(personJoints->QueryNumJoints());
                personJoints->QueryJoints(skeletonPoints.data());

                gestureDetected = detectGestures(personJoints, gesture_states);

                // Implement cancel gesture.
                if(gestureDetected == GESTURE_CANCEL)
                {
                    shouldCancel = true;
                    //system("killall vlc");

                }
            }


            if(currentVideoType() == GESTURE_UNDEFINED || shouldCancel)
            {
                cout << "playback completed or killed!" << endl;
                state = STATE_READY;

                {
                    thread video(playContent, GESTURE_READY, shouldQuit);
                    video.detach();
                }

            }
            break;


        case STATE_IDLEVIDEO_START:
            // Issue system call to playback idle video content in a detached thread

        {
            thread idlevideo(playContent, GESTURE_IDLE, shouldQuit);
            idlevideo.detach();
        }

            state = STATE_IDLEVIDEO_UNDERWAY;
            break;

        case STATE_IDLEVIDEO_UNDERWAY:
            if(pid_in_center != INVALID_PERSONID)
            {
                cout << "Person detected during idle content playback!" << endl;

                state = STATE_IDLE;
                cyclesSpentIdle = 0;
            }
            else if(currentVideoType() == GESTURE_UNDEFINED)
            {
                cout << "Idle playback completed. Starting new idle video at random" << endl;
                state = STATE_IDLEVIDEO_START;
            }

            break;
        }

    }

    system("killall vlc");


    pt_utils.stop_camera();
    actualModuleConfig.projection->release();
    cout << "-------- Stopping --------" << endl;
    return 0;
}

bool personIsInCenter(Intel::RealSense::PersonTracking::PersonTrackingData::PointCombined centerMass)
{
    if(     centerMass.world.point.x >= 0.0 &&
            centerMass.world.point.z >= 1.5 &&
            centerMass.world.point.x <= 0.2 &&
            centerMass.world.point.z <= 2.4
            )
    {

        //        cout<< "x: " << centerMass.world.point.x<<endl;
        //        cout<< "z: " << centerMass.world.point.z<<endl<<endl;
        return true;
    }

    return false;
}

gestures_e detectGestures(Intel::RealSense::PersonTracking::PersonTrackingData::PersonJoints *personJoints, gesture_states_t &gesture_states)
{
    int numDetectedJoints = personJoints->QueryNumJoints();
    std::vector<Intel::RealSense::PersonTracking::PersonTrackingData::PersonJoints::SkeletonPoint> skeletonPoints(personJoints->QueryNumJoints());


    personJoints->QueryJoints(skeletonPoints.data());

    jointCoords_t jointCoords;

    //    cout << "num detected joints" << numDetectedJoints << endl;

    //cout << skeletonPoints.at(0).image.x << endl;


    // Populate joint coordinates values
    jointCoords.Lhandx = skeletonPoints.at(0).image.x;
    jointCoords.Lhandy = skeletonPoints.at(0).image.y;
    jointCoords.Lhandz = skeletonPoints.at(0).world.z;
    jointCoords.Rhandx = skeletonPoints.at(1).image.x;
    jointCoords.Rhandy = skeletonPoints.at(1).image.y;
    jointCoords.Rhandz = skeletonPoints.at(1).world.z;

    jointCoords.Lshoulderx = skeletonPoints.at(4).image.x;
    jointCoords.Lshouldery = skeletonPoints.at(4).image.y;
    jointCoords.Lshoulderz = skeletonPoints.at(4).world.z;
    jointCoords.Rshoulderx = skeletonPoints.at(5).image.x;
    jointCoords.Rshouldery = skeletonPoints.at(5).image.y;
    jointCoords.Rshoulderz = skeletonPoints.at(5).world.z;

    jointCoords.headx = skeletonPoints.at(2).image.x;
    jointCoords.heady = skeletonPoints.at(2).image.y;
    jointCoords.headz = skeletonPoints.at(2).world.z;
    jointCoords.Spinex = skeletonPoints.at(3).image.x;
    jointCoords.Spiney = skeletonPoints.at(3).image.y;
    jointCoords.Spinez = skeletonPoints.at(3).world.z;

    // check for each pose sequentially.
    // some way to make this cleaner, perhaps a helper function with just jointCoords struct as only parameter?

    int LeftX = jointCoords.Lshoulderx - jointCoords.Lhandx;
    int LeftY = jointCoords.Lshouldery - jointCoords.Lhandy;
    int LeftZ = jointCoords.Lshoulderz - jointCoords.Lhandz;
    int RightX = jointCoords.Rshoulderx - jointCoords.Rhandx;
    int RightY = jointCoords.Rshouldery - jointCoords.Rhandy;
    int RightZ = jointCoords.Rshoulderz - jointCoords.Rhandz;

//    jumpHeadCurrX = jointCoords.headx;
//    jumpHeadCurrY = jointCoords.heady;
//    jumpHeadCurrZ = jointCoords.headz;





    /*
     * STATIC GESTURES BEGIN
     */

    // USAIN BOLT POSE
    switch(gesture_states.usain_gesture_state)
    {
    case gesture_states.USAIN_INIT: //everything increased by 10
        //        if( (LeftX <= 55) &&
        //                (LeftX >= 10 ) &&
        //                (LeftY <= 0) &&
        //                (LeftY >= -55) &&
        //                (RightX <= -20) &&
        //                (RightX >= -100 ) &&
        //                (RightY <= 60) &&
        //                (RightY >= 5)
        //                )
        //        {
        if( (LeftX <= 70) &&
                (LeftX >= 10 ) &&
                (LeftY <= 20) &&
                (LeftY >= -50) &&
                (RightX <= -60) &&
                (RightX >= -110) &&
                (RightY <= 60) &&
                (RightY >= 30))
        {
            gesture_states.usain_gesture_state = gesture_states.USAIN_DETECTING;
            cout << "DETECTING USAIN BOLT POSE..." << endl;

            gesture_states.cyclesInState_usain_detecting = 0;
        }
        break;
    case gesture_states.USAIN_DETECTING:
        if( (LeftX <= 70) &&
                (LeftX >= 10 ) &&
                (LeftY <= 20) &&
                (LeftY >= -50) &&
                (RightX <= -60) &&
                (RightX >= -90 ) &&
                (RightY <= 60) &&
                (RightY >= 30)
                )
        {
            gesture_states.cyclesInState_usain_detecting++;
            cout << "DETECTING USAIN BOLT POSE..." << gesture_states.cyclesInState_usain_detecting << " CYCLES" << endl;

            if(gesture_states.cyclesInState_usain_detecting >= STATIC_POSE_DETECTING_TIMEOUT)
            {
                cout << "USAIN BOLT POSE DETECTED!" << endl << endl;
                resetGestureStates(gesture_states);
                return GESTURE_USAIN;
            }
        }
        else
        {
            gesture_states.usain_gesture_state = gesture_states.USAIN_LOST;
            cout << "DETECTING USAIN BOLT POSE LOST..." << endl;

            gesture_states.cyclesInState_usain_lost = 0;
        }
        break;

    case gesture_states.USAIN_LOST:
        //cout << "LOST USAIN BOLT POSE..." << gesture_states.cyclesInState_usain_lost << " CYCLES" << endl;
        if( (LeftX <= 70) &&
                (LeftX >= 10 ) &&
                (LeftY <= 20) &&
                (LeftY >= -50) &&
                (RightX <= -60) &&
                (RightX >= -90 ) &&
                (RightY <= 60) &&
                (RightY >= 30)
                )
        {
            gesture_states.usain_gesture_state = gesture_states.USAIN_DETECTING;
            gesture_states.cyclesInState_usain_lost = 0;
        }
        else if(gesture_states.cyclesInState_usain_lost >= STATIC_POSE_LOST_TIMEOUT)
        {
            gesture_states.usain_gesture_state = gesture_states.USAIN_INIT;
            gesture_states.cyclesInState_usain_detecting = 0;
            gesture_states.cyclesInState_usain_lost = 0;
        }
        else
        {
            gesture_states.cyclesInState_usain_lost++;
        }
        break;
    }


    // VICTORY POSE
    switch(gesture_states.victory_gesture_state)
    {
    case gesture_states.VICTORY_INIT:
        //        if( (LeftX <= 70) &&
        //                (LeftX >= 35 ) &&
        //                (LeftY <= 90) &&
        //                (LeftY >= 50) &&
        //                (RightX <= -20) &&
        //                (RightX >= -60 ) &&
        //                (RightY <= 80) &&
        //                (RightY >= 50)
        //                )
        //        {
        if( (LeftX <= 100) &&
                (LeftX >= 45 ) &&
                (LeftY <= 90) &&
                (LeftY >= 45) &&
                (RightX <= -30) &&
                (RightX >= -80 ) &&
                (RightY <= 100) &&
                (RightY >= 50)
                )
        {
            gesture_states.victory_gesture_state = gesture_states.VICTORY_DETECTING;
            cout << "DETECTING VICTORY POSE..." << endl;

            gesture_states.cyclesInState_victory_detecting = 0;
        }
        break;
    case gesture_states.VICTORY_DETECTING:
        if( (LeftX <= 100) &&
                (LeftX >= 45 ) &&
                (LeftY <= 90) &&
                (LeftY >= 45) &&
                (RightX <= -30) &&
                (RightX >= -80 ) &&
                (RightY <= 100) &&
                (RightY >= 50)
                )
        {
            gesture_states.cyclesInState_victory_detecting++;
            cout << "DETECTING VICTORY POSE..." << gesture_states.cyclesInState_victory_detecting << " CYCLES" << endl;

            if(gesture_states.cyclesInState_victory_detecting >= STATIC_POSE_DETECTING_TIMEOUT)
            {
                cout << "VICTORY GESTURE DETECTED!" << endl << endl;
                resetGestureStates(gesture_states);
                return GESTURE_VICTORY;
            }
        }
        else
        {
            gesture_states.victory_gesture_state = gesture_states.VICTORY_LOST;
            cout << "DETECTING VICTORY POSE LOST..." << endl;

            gesture_states.cyclesInState_victory_lost = 0;
        }
        break;

    case gesture_states.VICTORY_LOST:
        //cout << "LOST VICTORY POSE " << gesture_states.cyclesInState_vpose_lost << " CYCLES" << endl;
        if( (LeftX <= 100) &&
                (LeftX >= 45 ) &&
                (LeftY <= 90) &&
                (LeftY >= 45) &&
                (RightX <= -30) &&
                (RightX >= -80 ) &&
                (RightY <= 100) &&
                (RightY >= 50)
                )
        {
            gesture_states.victory_gesture_state = gesture_states.VICTORY_DETECTING;
            gesture_states.cyclesInState_victory_lost = 0;
        }
        else if(gesture_states.cyclesInState_victory_lost >= STATIC_POSE_LOST_TIMEOUT)
        {
            gesture_states.victory_gesture_state = gesture_states.VICTORY_INIT;
            gesture_states.cyclesInState_victory_detecting = 0;
            gesture_states.cyclesInState_victory_lost = 0;
        }
        else
        {
            gesture_states.cyclesInState_victory_lost++;
        }
        break;
    }



    // POWER POSE/ FLEX POSE
    switch(gesture_states.powerpose_gesture_state)
    {
    case gesture_states.POWERPOSE_INIT:
        //        if( (LeftX <= 40) &&
        //                (LeftX > 0 ) &&
        //                (LeftY <= 40) &&
        //                (LeftY > 0) &&
        //                (RightX <= 0) &&
        //                (RightX > -40 ) &&
        //                (RightY <= 40) &&
        //                (RightY > 0)
        //                )
        //        {
        if( (LeftX <= 70) &&
                (LeftX >= 0 ) &&
                (LeftY <= 50) &&
                (LeftY >= 0) &&
                (RightX <= 0) &&
                (RightX >= -50 ) &&
                (RightY <= 50) &&
                (RightY >= 20)
                )
        {
            gesture_states.powerpose_gesture_state = gesture_states.POWERPOSE_DETECTING;
            cout << "DETECTING POWER POSE..." << endl;

            gesture_states.cyclesInState_powerpose_detecting = 0;
        }
        break;
    case gesture_states.POWERPOSE_DETECTING:
        if( (LeftX <= 70) &&
                (LeftX >= 0 ) &&
                (LeftY <= 50) &&
                (LeftY >= 0) &&
                (RightX <= 0) &&
                (RightX >= -50 ) &&
                (RightY <= 50) &&
                (RightY >= 20)
                )
        {
            gesture_states.cyclesInState_powerpose_detecting++;
            cout << "DETECTING POWER POSE..." << gesture_states.cyclesInState_powerpose_detecting << " CYCLES" << endl;

            if(gesture_states.cyclesInState_powerpose_detecting >= STATIC_POSE_DETECTING_TIMEOUT)
            {
                cout << "POWER POSE DETECTED!" << endl << endl;
                resetGestureStates(gesture_states);
                return GESTURE_POWERPOSE;
            }
        }
        else
        {
            gesture_states.powerpose_gesture_state = gesture_states.POWERPOSE_LOST;
            cout << "DETECTING POWER POSE LOST..." << endl;

            gesture_states.cyclesInState_powerpose_lost = 0;
        }
        break;

    case gesture_states.POWERPOSE_LOST:
        //cout << "LOST POWER POSE..." << gesture_states.cyclesInState_powerpose_lost << " CYCLES" << endl;
        if((LeftX <= 70) &&
                (LeftX >= 0 ) &&
                (LeftY <= 50) &&
                (LeftY >= 0) &&
                (RightX <= 0) &&
                (RightX >= -50 ) &&
                (RightY <= 50) &&
                (RightY >= 20)
                )
        {
            gesture_states.powerpose_gesture_state = gesture_states.POWERPOSE_DETECTING;
            gesture_states.cyclesInState_powerpose_lost = 0;
        }
        else if(gesture_states.cyclesInState_powerpose_lost >= STATIC_POSE_LOST_TIMEOUT)
        {
            gesture_states.powerpose_gesture_state = gesture_states.POWERPOSE_INIT;
            gesture_states.cyclesInState_powerpose_detecting = 0;
            gesture_states.cyclesInState_powerpose_lost = 0;
        }
        else
        {
            gesture_states.cyclesInState_powerpose_lost++;
        }
        break;
    }



    // T POSE
    switch(gesture_states.tpose_gesture_state)
    {
    case gesture_states.TPOSE_INIT:
        //        if( //(LeftX <= 100) &&
        //                (LeftX >= 75 ) &&
        //                //(LeftY <= 10) &&
        //                //(LeftY >= -10) &&
        //                (abs(LeftY) <= 15) &&
        //                //(RightX <= -80) &&
        //                (RightX >= -110 ) &&
        //                //(RightY <= 5) &&
        //                //(RightY >= -15)
        //                (abs(RightY) <= 15)
        //                )
        //        {
        if((LeftX >= 45 ) &&            // recall that there is only a minimum arm length
                (LeftY <= 20) &&
                (LeftY >= -20) &&
                (RightX <= -45) &&
                (RightY <= 20) &&
                (RightY >= -20)
                )
        {
            gesture_states.tpose_gesture_state = gesture_states.TPOSE_DETECTING;
            cout << "DETECTING T POSE..." << endl;

            gesture_states.cyclesInState_tpose_detecting = 0;
        }
        break;
    case gesture_states.TPOSE_DETECTING:
        if((LeftX >= 45 ) &&
                (LeftY <= 20) &&
                (LeftY >= -20) &&
                (RightX <= -45) &&
                (RightY <= 20) &&
                (RightY >= -20)
                )
        {
            gesture_states.cyclesInState_tpose_detecting++;
            cout << "DETECTING T POSE..." << gesture_states.cyclesInState_tpose_detecting << " CYCLES" << endl;

            if(gesture_states.cyclesInState_tpose_detecting >= STATIC_POSE_DETECTING_TIMEOUT)
            {
                cout << "T POSE DETECTED!" << endl << endl;
                resetGestureStates(gesture_states);
                return GESTURE_T;
            }
        }
        else
        {
            gesture_states.tpose_gesture_state = gesture_states.TPOSE_LOST;
            cout << "DETECTING T POSE LOST..." << endl;

            gesture_states.cyclesInState_tpose_lost = 0;
        }
        break;

    case gesture_states.TPOSE_LOST:
        //cout << "LOST T POSE..." << gesture_states.cyclesInState_tpose_lost << " CYCLES" << endl;
        if((LeftX >= 45 ) &&
                (LeftY <= 20) &&
                (LeftY >= -20) &&
                (RightX <= -45) &&
                (RightY <= 20) &&
                (RightY >= -20)
                )
        {
            gesture_states.tpose_gesture_state = gesture_states.TPOSE_DETECTING;
            gesture_states.cyclesInState_tpose_lost = 0;
        }
        else if(gesture_states.cyclesInState_tpose_lost >= STATIC_POSE_LOST_TIMEOUT)
        {
            gesture_states.tpose_gesture_state = gesture_states.TPOSE_INIT;
            gesture_states.cyclesInState_tpose_detecting = 0;
            gesture_states.cyclesInState_tpose_lost = 0;
        }
        else
        {
            gesture_states.cyclesInState_tpose_lost++;
        }
        break;
    }


    // STOP POSE
    switch(gesture_states.stop_gesture_state)
    {
    case gesture_states.STOP_INIT:
        if((LeftX >= -10) &&
                (LeftX <= 30) &&
                (LeftY >= 10) &&
                (LeftY <= 50) &&
                (jointCoords.Lhandz <= 1550))
        {
            gesture_states.stop_gesture_state = gesture_states.STOP_DETECTING;
            cout << "DETECTING STOP POSE..." << endl;

            gesture_states.cyclesInState_stop_detecting = 0;
        }
        break;
    case gesture_states.STOP_DETECTING:
        if((LeftX >= -10) &&
                (LeftX <= 30) &&
                (LeftY >= 10) &&
                (LeftY <= 50) &&
                (jointCoords.Lhandz <= 1550))
        {
            gesture_states.cyclesInState_stop_detecting++;
            cout << "DETECTING STOP..." << gesture_states.cyclesInState_stop_detecting << " CYCLES" << endl;

            if(gesture_states.cyclesInState_stop_detecting >= STATIC_POSE_DETECTING_TIMEOUT)
            {
                cout << "STOP DETECTED!" << endl << endl << endl << endl << endl << endl;
                resetGestureStates(gesture_states);
                return GESTURE_STOP;
            }
        }
        else
        {
            gesture_states.stop_gesture_state = gesture_states.STOP_LOST;
            cout << "DETECTING STOP LOST..." << endl;

            gesture_states.cyclesInState_stop_lost = 0;
        }
        break;

    case gesture_states.STOP_LOST:
        //cout << "LOST STOP..." << gesture_states.cyclesInState_stop_lost << " CYCLES" << endl;
        if((LeftX >= -10) &&
                (LeftX <= 30) &&
                (LeftY >= 10) &&
                (LeftY <= 50) &&
                (jointCoords.Lhandz <= 1550))
        {
            gesture_states.stop_gesture_state = gesture_states.STOP_DETECTING;
            gesture_states.cyclesInState_stop_lost = 0;
        }
        else if(gesture_states.cyclesInState_stop_lost >= STATIC_POSE_LOST_TIMEOUT)
        {
            gesture_states.stop_gesture_state = gesture_states.STOP_INIT;
            gesture_states.cyclesInState_stop_detecting = 0;
            gesture_states.cyclesInState_stop_lost = 0;
        }
        else
        {
            gesture_states.cyclesInState_stop_lost++;
        }
        break;
    }


    //POINTING POSES



    // POINTING TOP RIGHT FORWARD POSE
    switch(gesture_states.pointing_trf_gesture_state)
    {
    case gesture_states.POINTING_TRF_INIT:
        if( (LeftX <= 90) &&
                (LeftX >= 50 ) &&
                (LeftY <= 90) &&
                (LeftY >= 50) &&
                (LeftZ <= 500) &&
                (LeftZ >= 300) &&
                ((RightY < -40) || (RightY > 110)))
        {
            gesture_states.pointing_trf_gesture_state = gesture_states.POINTING_TRF_DETECTING;
            cout << "DETECTING POINTING TRF..." << endl;

            gesture_states.cyclesInState_pointing_trf_detecting = 0;
        }
        break;
    case gesture_states.POINTING_TRF_DETECTING:
        if( (LeftX <= 90) &&
                (LeftX >= 50 ) &&
                (LeftY <= 90) &&
                (LeftY >= 50) &&
                (LeftZ <= 500) &&
                (LeftZ >= 300) &&
                ((RightY < -40) || (RightY > 110)))
        {
            gesture_states.cyclesInState_pointing_trf_detecting++;
            cout << "DETECTING POINTING TRF..." << gesture_states.cyclesInState_pointing_trf_detecting << " CYCLES" << endl;

            if(gesture_states.cyclesInState_pointing_trf_detecting >= STATIC_POSE_DETECTING_TIMEOUT)
            {
                cout << "POINTING TOP RIGHT FORWARD POSE DETECTED!" << endl << endl;
                resetGestureStates(gesture_states);
                return GESTURE_POINTING_TRF;
            }
        }
        else
        {
            gesture_states.pointing_trf_gesture_state = gesture_states.POINTING_TRF_LOST;
            cout << "DETECTING POINTING TOP RIGHT FORWARD LOST" << endl;

            gesture_states.cyclesInState_pointing_trf_lost = 0;
        }
        break;

    case gesture_states.POINTING_TRF_LOST:
        //cout << "LOST POINTING TRF..." << gesture_states.cyclesInState_pointing_trf_lost << " CYCLES" << endl;
        if( (LeftX <= 90) &&
                (LeftX >= 50 ) &&
                (LeftY <= 90) &&
                (LeftY >= 50) &&
                (LeftZ <= 500) &&
                (LeftZ >= 300) &&
                ((RightY < -40) || (RightY > 110)))
        {
            gesture_states.pointing_trf_gesture_state = gesture_states.POINTING_TRF_DETECTING;
            gesture_states.cyclesInState_pointing_trf_lost = 0;
        }
        else if(gesture_states.cyclesInState_pointing_trf_lost >= STATIC_POSE_LOST_TIMEOUT)
        {
            gesture_states.pointing_trf_gesture_state = gesture_states.POINTING_TRF_INIT;
            gesture_states.cyclesInState_pointing_trf_detecting = 0;
            gesture_states.cyclesInState_pointing_trf_lost = 0;
        }
        else
        {
            gesture_states.cyclesInState_pointing_trf_lost++;
        }
        break;
    }

    // POINTING RIGHT FORWARD POSE
    switch(gesture_states.pointing_rf_gesture_state)
    {
    case gesture_states.POINTING_RF_INIT:
        if( (LeftX <= 120) &&
                (LeftX >= 60 ) &&
                (LeftY <= 30) &&
                (LeftY >= -30) &&
                (LeftZ <= 670) &&
                (LeftZ >= 230) &&
                ((RightY < -40) || (RightY > 110)))
        {
            gesture_states.pointing_rf_gesture_state = gesture_states.POINTING_RF_DETECTING;
            cout << "DETECTING POINTING RF..." << endl;

            gesture_states.cyclesInState_pointing_rf_detecting = 0;
        }
        break;
    case gesture_states.POINTING_RF_DETECTING:
        if( (LeftX <= 120) &&
                (LeftX >= 60 ) &&
                (LeftY <= 30) &&
                (LeftY >= -30) &&
                (LeftZ <= 670) &&
                (LeftZ >= 230) &&
                ((RightY < -40) || (RightY > 110)))
        {
            gesture_states.cyclesInState_pointing_rf_detecting++;
            cout << "DETECTING POINTING RF..." << gesture_states.cyclesInState_pointing_rf_detecting << " CYCLES" << endl;

            if(gesture_states.cyclesInState_pointing_rf_detecting >= STATIC_POSE_DETECTING_TIMEOUT)
            {
                cout << "POINTING RIGHT FORWARD POSE DETECTED!" << endl << endl;
                resetGestureStates(gesture_states);
                return GESTURE_POINTING_RF;
            }
        }
        else
        {
            gesture_states.pointing_rf_gesture_state = gesture_states.POINTING_RF_LOST;
            cout << "DETECTING POINTING RIGHT FORWARD LOST" << endl;

            gesture_states.cyclesInState_pointing_rf_lost = 0;
        }
        break;

    case gesture_states.POINTING_RF_LOST:
        //cout << "LOST POINTING RF..." << gesture_states.cyclesInState_pointing_rf_lost << " CYCLES" << endl;
        if( (LeftX <= 120) &&
                (LeftX >= 60 ) &&
                (LeftY <= 30) &&
                (LeftY >= -30) &&
                (LeftZ <= 670) &&
                (LeftZ >= 230) &&
                ((RightY < -40) || (RightY > 110)))
        {
            gesture_states.pointing_rf_gesture_state = gesture_states.POINTING_RF_DETECTING;
            gesture_states.cyclesInState_pointing_rf_lost = 0;
        }
        else if(gesture_states.cyclesInState_pointing_rf_lost >= STATIC_POSE_LOST_TIMEOUT)
        {
            gesture_states.pointing_rf_gesture_state = gesture_states.POINTING_RF_INIT;
            gesture_states.cyclesInState_pointing_rf_detecting = 0;
            gesture_states.cyclesInState_pointing_rf_lost = 0;
        }
        else
        {
            gesture_states.cyclesInState_pointing_rf_lost++;
        }
        break;
    }

    // POINTING TOP LEFT FORWARD POSE
    switch(gesture_states.pointing_tlf_gesture_state)
    {
    case gesture_states.POINTING_TLF_INIT:
        if( (RightX <= -50) &&
                (RightX >= -100 ) &&
                (RightY <= 80) &&
                (RightY >= 40) &&
                (RightZ <= 420) &&
                (RightZ >= 180) &&
                ((LeftY < -50) || (LeftY > 90)))
        {
            gesture_states.pointing_tlf_gesture_state = gesture_states.POINTING_TLF_DETECTING;
            cout << "DETECTING POINTING TLF..." << endl;

            gesture_states.cyclesInState_pointing_tlf_detecting = 0;
        }
        break;
    case gesture_states.POINTING_TLF_DETECTING:
        if( (RightX <= -50) &&
                (RightX >= -100 ) &&
                (RightY <= 80) &&
                (RightY >= 40) &&
                (RightZ <= 420) &&
                (RightZ >= 180) &&
                ((LeftY < -50) || (LeftY > 90)))
        {
            gesture_states.cyclesInState_pointing_tlf_detecting++;
            cout << "DETECTING POINTING TLF..." << gesture_states.cyclesInState_pointing_tlf_detecting << " CYCLES" << endl;

            if(gesture_states.cyclesInState_pointing_tlf_detecting >= STATIC_POSE_DETECTING_TIMEOUT)
            {
                cout << "POINTING TOP LEFT FORWARD POSE DETECTED!" << endl << endl;
                resetGestureStates(gesture_states);
                return GESTURE_POINTING_TLF;
            }
        }
        else
        {
            gesture_states.pointing_tlf_gesture_state = gesture_states.POINTING_TLF_LOST;
            cout << "DETECTING POINTING TOP LEFT FORWARD LOST" << endl;

            gesture_states.cyclesInState_pointing_tlf_lost = 0;
        }
        break;

    case gesture_states.POINTING_TLF_LOST:
        //cout << "LOST POINTING TLF..." << gesture_states.cyclesInState_pointing_tlf_lost << " CYCLES" << endl;
        if( (RightX <= -50) &&
                (RightX >= -100 ) &&
                (RightY <= 80) &&
                (RightY >= 40) &&
                (RightZ <= 420) &&
                (RightZ >= 180) &&
                ((LeftY < -50) || (LeftY > 90)))
        {
            gesture_states.pointing_tlf_gesture_state = gesture_states.POINTING_TLF_DETECTING;
            gesture_states.cyclesInState_pointing_tlf_lost = 0;
        }
        else if(gesture_states.cyclesInState_pointing_tlf_lost >= STATIC_POSE_LOST_TIMEOUT)
        {
            gesture_states.pointing_tlf_gesture_state = gesture_states.POINTING_TLF_INIT;
            gesture_states.cyclesInState_pointing_tlf_detecting = 0;
            gesture_states.cyclesInState_pointing_tlf_lost = 0;
        }
        else
        {
            gesture_states.cyclesInState_pointing_tlf_lost++;
        }
        break;
    }

    // POINTING LEFT FORWARD POSE
    switch(gesture_states.pointing_lf_gesture_state)
    {
    case gesture_states.POINTING_LF_INIT:
        if( (RightX <= -60) &&
                (RightX >= -120 ) &&
                (RightY <= 30) &&
                (RightY >= -20) &&
                (RightZ <= 550) &&
                (RightZ >= 70) &&
                ((LeftY < -50) || (LeftY > 90)))
        {
            gesture_states.pointing_lf_gesture_state = gesture_states.POINTING_LF_DETECTING;
            cout << "DETECTING POINTING LF..." << endl;

            gesture_states.cyclesInState_pointing_lf_detecting = 0;
        }
        break;
    case gesture_states.POINTING_LF_DETECTING:
        if( (RightX <= -60) &&
                (RightX >= -120 ) &&
                (RightY <= 30) &&
                (RightY >= -20) &&
                (RightZ <= 550) &&
                (RightZ >= 70) &&
                ((LeftY < -50) || (LeftY > 90)))
        {
            gesture_states.cyclesInState_pointing_lf_detecting++;
            cout << "DETECTING POINTING LF..." << gesture_states.cyclesInState_pointing_lf_detecting << " CYCLES" << endl;

            if(gesture_states.cyclesInState_pointing_lf_detecting >= STATIC_POSE_DETECTING_TIMEOUT)
            {
                cout << "POINTING LEFT FORWARD POSE DETECTED!" << endl << endl;
                resetGestureStates(gesture_states);
                return GESTURE_POINTING_LF;
            }
        }
        else
        {
            gesture_states.pointing_lf_gesture_state = gesture_states.POINTING_LF_LOST;
            cout << "DETECTING POINTING LEFT FORWARD LOST" << endl;

            gesture_states.cyclesInState_pointing_lf_lost = 0;
        }
        break;

    case gesture_states.POINTING_LF_LOST:
        //cout << "LOST POINTING LF..." << gesture_states.cyclesInState_pointing_lf_lost << " CYCLES" << endl;
        if( (RightX <= -60) &&
                (RightX >= -120 ) &&
                (RightY <= 30) &&
                (RightY >= -20) &&
                (RightZ <= 550) &&
                (RightZ >= 70) &&
                ((LeftY < -50) || (LeftY > 90)))
        {
            gesture_states.pointing_lf_gesture_state = gesture_states.POINTING_LF_DETECTING;
            gesture_states.cyclesInState_pointing_lf_lost = 0;
        }
        else if(gesture_states.cyclesInState_pointing_lf_lost >= STATIC_POSE_LOST_TIMEOUT)
        {
            gesture_states.pointing_lf_gesture_state = gesture_states.POINTING_LF_INIT;
            gesture_states.cyclesInState_pointing_lf_detecting = 0;
            gesture_states.cyclesInState_pointing_lf_lost = 0;
        }
        else
        {
            gesture_states.cyclesInState_pointing_lf_lost++;
        }
        break;
    }

    // POINTING TOP RIGHT POSE
    switch(gesture_states.pointing_tr_gesture_state)
    {
    case gesture_states.POINTING_TR_INIT:
        if( (LeftX <= 100) &&
                (LeftX >= 50 ) &&
                (LeftY <= 80) &&
                (LeftY >= 30) &&
                (LeftZ <= 20) &&
                (LeftZ >= -140) &&
                ((RightY < -40) || (RightY > 110)))
        {
            gesture_states.pointing_tr_gesture_state = gesture_states.POINTING_TR_DETECTING;
            cout << "DETECTING POINTING TR..." << endl;

            gesture_states.cyclesInState_pointing_tr_detecting = 0;
        }
        break;
    case gesture_states.POINTING_TR_DETECTING:
        if( (LeftX <= 100) &&
                (LeftX >= 50 ) &&
                (LeftY <= 80) &&
                (LeftY >= 30) &&
                (LeftZ <= 160) &&
                (LeftZ >= -140) &&
                ((RightY < -40) || (RightY > 110)))
        {
            gesture_states.cyclesInState_pointing_tr_detecting++;
            cout << "DETECTING POINTING TR..." << gesture_states.cyclesInState_pointing_tr_detecting << " CYCLES" << endl;

            if(gesture_states.cyclesInState_pointing_tr_detecting >= STATIC_POSE_DETECTING_TIMEOUT)
            {
                cout << "POINTING TOP RIGHT POSE DETECTED!" << endl << endl;
                resetGestureStates(gesture_states);
                return GESTURE_POINTING_TR;
            }
        }
        else
        {
            gesture_states.pointing_tr_gesture_state = gesture_states.POINTING_TR_LOST;
            cout << "DETECTING POINTING TOP RIGHT LOST" << endl;

            gesture_states.cyclesInState_pointing_tr_lost = 0;
        }
        break;

    case gesture_states.POINTING_TR_LOST:
        //cout << "LOST POINTING TR..." << gesture_states.cyclesInState_pointing_tr_lost << " CYCLES" << endl;
        if( (LeftX <= 100) &&
                (LeftX >= 50 ) &&
                (LeftY <= 80) &&
                (LeftY >= 30) &&
                (LeftZ <= 160) &&
                (LeftZ >= -140) &&
                ((RightY < -40) || (RightY > 110)))
        {
            gesture_states.pointing_tr_gesture_state = gesture_states.POINTING_TR_DETECTING;
            gesture_states.cyclesInState_pointing_tr_lost = 0;
        }
        else if(gesture_states.cyclesInState_pointing_tr_lost >= STATIC_POSE_LOST_TIMEOUT)
        {
            gesture_states.pointing_tr_gesture_state = gesture_states.POINTING_TR_INIT;
            gesture_states.cyclesInState_pointing_tr_detecting = 0;
            gesture_states.cyclesInState_pointing_tr_lost = 0;
        }
        else
        {
            gesture_states.cyclesInState_pointing_tr_lost++;
        }
        break;
    }

    // POINTING RIGHT POSE
    switch(gesture_states.pointing_r_gesture_state)
    {
    case gesture_states.POINTING_R_INIT:
        if( (LeftX <= 110) &&
                (LeftX >= 80 ) &&
                (LeftY <= 25) &&
                (LeftY >= -20) &&
                (LeftZ <= 20) &&
                (LeftZ >= -200) &&
                ((RightY < -40) || (RightY > 110)))
        {
            gesture_states.pointing_r_gesture_state = gesture_states.POINTING_R_DETECTING;
            cout << "DETECTING POINTING R..." << endl;

            gesture_states.cyclesInState_pointing_r_detecting = 0;
        }
        break;
    case gesture_states.POINTING_R_DETECTING:
        if( (LeftX <= 110) &&
                (LeftX >= 80 ) &&
                (LeftY <= 25) &&
                (LeftY >= -20) &&
                (LeftZ <= 20) &&
                (LeftZ >= -200) &&
                ((RightY < -40) || (RightY > 110)))
        {
            gesture_states.cyclesInState_pointing_r_detecting++;
            cout << "DETECTING POINTING R..." << gesture_states.cyclesInState_pointing_r_detecting << " CYCLES" << endl;

            if(gesture_states.cyclesInState_pointing_r_detecting >= STATIC_POSE_DETECTING_TIMEOUT)
            {
                cout << "POINTING RIGHT POSE DETECTED!" << endl << endl;
                resetGestureStates(gesture_states);
                return GESTURE_POINTING_R;
            }
        }
        else
        {
            gesture_states.pointing_r_gesture_state = gesture_states.POINTING_R_LOST;
            cout << "DETECTING POINTING RIGHT LOST" << endl;

            gesture_states.cyclesInState_pointing_r_lost = 0;
        }
        break;

    case gesture_states.POINTING_R_LOST:
        //cout << "LOST POINTING R..." << gesture_states.cyclesInState_pointing_r_lost << " CYCLES" << endl;
        if( (LeftX <= 110) &&
                (LeftX >= 80 ) &&
                (LeftY <= 25) &&
                (LeftY >= -20) &&
                (LeftZ <= 20) &&
                (LeftZ >= -200) &&
                ((RightY < -40) || (RightY > 110)))
        {
            gesture_states.pointing_r_gesture_state = gesture_states.POINTING_R_DETECTING;
            gesture_states.cyclesInState_pointing_r_lost = 0;
        }
        else if(gesture_states.cyclesInState_pointing_r_lost >= STATIC_POSE_LOST_TIMEOUT)
        {
            gesture_states.pointing_r_gesture_state = gesture_states.POINTING_R_INIT;
            gesture_states.cyclesInState_pointing_r_detecting = 0;
            gesture_states.cyclesInState_pointing_r_lost = 0;
        }
        else
        {
            gesture_states.cyclesInState_pointing_r_lost++;
        }
        break;
    }

    // POINTING TOP LEFT POSE
    switch(gesture_states.pointing_tl_gesture_state)
    {
    case gesture_states.POINTING_TL_INIT:
        if( (RightX <= -50) &&
                (RightX >= -80 ) &&
                (RightY <= 80) &&
                (RightY >= 40) &&
                (RightZ <= 110) &&
                (RightZ >= -120) &&
                ((LeftY < -50) || (LeftY > 90)))
        {
            gesture_states.pointing_tl_gesture_state = gesture_states.POINTING_TL_DETECTING;
            cout << "DETECTING POINTING TL..." << endl;

            gesture_states.cyclesInState_pointing_tl_detecting = 0;
        }
        break;
    case gesture_states.POINTING_TL_DETECTING:
        if( (RightX <= -50) &&
                (RightX >= -80 ) &&
                (RightY <= 80) &&
                (RightY >= 40) &&
                (RightZ <= 110) &&
                (RightZ >= -120) &&
                ((LeftY < -50) || (LeftY > 90)))
        {
            gesture_states.cyclesInState_pointing_tl_detecting++;
            cout << "DETECTING POINTING TL..." << gesture_states.cyclesInState_pointing_tl_detecting << " CYCLES" << endl;

            if(gesture_states.cyclesInState_pointing_tl_detecting >= STATIC_POSE_DETECTING_TIMEOUT)
            {
                cout << "POINTING TOP LEFT POSE DETECTED!" << endl << endl;
                resetGestureStates(gesture_states);
                return GESTURE_POINTING_TL;
            }
        }
        else
        {
            gesture_states.pointing_tl_gesture_state = gesture_states.POINTING_TL_LOST;
            cout << "DETECTING POINTING TOP LEFT LOST" << endl;

            gesture_states.cyclesInState_pointing_tl_lost = 0;
        }
        break;

    case gesture_states.POINTING_TL_LOST:
        //cout << "LOST POINTING TL..." << gesture_states.cyclesInState_pointing_tl_lost << " CYCLES" << endl;
        if( (RightX <= -50) &&
                (RightX >= -80 ) &&
                (RightY <= 80) &&
                (RightY >= 40) &&
                (RightZ <= 110) &&
                (RightZ >= -120) &&
                ((LeftY < -50) || (LeftY > 90)))
        {
            gesture_states.pointing_tl_gesture_state = gesture_states.POINTING_TL_DETECTING;
            gesture_states.cyclesInState_pointing_tl_lost = 0;
        }
        else if(gesture_states.cyclesInState_pointing_tl_lost >= STATIC_POSE_LOST_TIMEOUT)
        {
            gesture_states.pointing_tl_gesture_state = gesture_states.POINTING_TL_INIT;
            gesture_states.cyclesInState_pointing_tl_detecting = 0;
            gesture_states.cyclesInState_pointing_tl_lost = 0;
        }
        else
        {
            gesture_states.cyclesInState_pointing_tl_lost++;
        }
        break;
    }

    // POINTING LEFT POSE
    switch(gesture_states.pointing_l_gesture_state)
    {
    case gesture_states.POINTING_L_INIT:
        if( (RightX <= -70) &&
                (RightX >= -100 ) &&
                (RightY <= 20) &&
                (RightY >= -10) &&
                (RightZ <= 110) &&
                (RightZ >= -90) &&
                ((LeftY < -50) || (LeftY > 90)))
        {
            gesture_states.pointing_l_gesture_state = gesture_states.POINTING_L_DETECTING;
            cout << "DETECTING POINTING L..." << endl;

            gesture_states.cyclesInState_pointing_l_detecting = 0;
        }
        break;
    case gesture_states.POINTING_L_DETECTING:
        if( (RightX <= -70) &&
                (RightX >= -100 ) &&
                (RightY <= 20) &&
                (RightY >= -10) &&
                (RightZ <= 110) &&
                (RightZ >= -90) &&
                ((LeftY < -50) || (LeftY > 90)))
        {
            gesture_states.cyclesInState_pointing_l_detecting++;
            cout << "DETECTING POINTING L..." << gesture_states.cyclesInState_pointing_l_detecting << " CYCLES" << endl;

            if(gesture_states.cyclesInState_pointing_l_detecting >= STATIC_POSE_DETECTING_TIMEOUT)
            {
                cout << "POINTING LEFT POSE DETECTED!" << endl << endl;
                resetGestureStates(gesture_states);
                return GESTURE_POINTING_L;
            }
        }
        else
        {
            gesture_states.pointing_l_gesture_state = gesture_states.POINTING_L_LOST;
            cout << "DETECTING POINTING LEFT LOST" << endl;

            gesture_states.cyclesInState_pointing_l_lost = 0;
        }
        break;

    case gesture_states.POINTING_L_LOST:
        //cout << "LOST POINTING L..." << gesture_states.cyclesInState_pointing_l_lost << " CYCLES" << endl;
        if( (RightX <= -70) &&
                (RightX >= -100 ) &&
                (RightY <= 20) &&
                (RightY >= -10) &&
                (RightZ <= 110) &&
                (RightZ >= -90) &&
                ((LeftY < -50) || (LeftY > 90)))
        {
            gesture_states.pointing_l_gesture_state = gesture_states.POINTING_L_DETECTING;
            gesture_states.cyclesInState_pointing_l_lost = 0;
        }
        else if(gesture_states.cyclesInState_pointing_l_lost >= STATIC_POSE_LOST_TIMEOUT)
        {
            gesture_states.pointing_l_gesture_state = gesture_states.POINTING_L_INIT;
            gesture_states.cyclesInState_pointing_l_detecting = 0;
            gesture_states.cyclesInState_pointing_l_lost = 0;
        }
        else
        {
            gesture_states.cyclesInState_pointing_l_lost++;
        }
        break;
    }





    // FLYING GESTURE
    switch(gesture_states.flying_gesture_state)
    {
    case gesture_states.FLYING_INIT:
        if((LeftX >= 60) &&
                //                (LeftX <= 60) &&
                (LeftY >= -20) &&
                (LeftY <= 30) && //15
                //                (RightX >= -110) &&
                (RightX >= -100) &&
                (RightY >= -10) &&
                (RightY <= 40) &&
                (jointCoords.headz >= 1650) &&
                (jointCoords.headz <= 2200))
        {
            gesture_states.cyclesInState_flying = 0;
            gesture_states.flying_gesture_state = gesture_states.FLYING_MAX_1;
        }
        //cout << "FLYING_INIT..." << endl;
        break;

    case gesture_states.FLYING_MAX_1:
        if((LeftY >= -100) &&
                (LeftY <= 0) &&
                (RightY >= -90) &&
                (RightY <= -50))
        {
            gesture_states.cyclesInState_flying = 0;
            gesture_states.flying_gesture_state = gesture_states.FLYING_MIN_1;
        }
        else if(gesture_states.cyclesInState_flying >= FLYING_TIMEOUT){
            gesture_states.cyclesInState_flying = 0;
            gesture_states.flying_gesture_state = gesture_states.FLYING_INIT;
        }
        else
        {
            gesture_states.cyclesInState_flying++;
        }
        cout << "FLYING_MAX_1..." << endl;
        break;

    case gesture_states.FLYING_MIN_1:
        if((LeftY >= -20) &&
                (LeftY <= 20) && //15
                (RightY >= -10) &&
                (RightY <= 30))
        {
            gesture_states.cyclesInState_flying = 0;
            gesture_states.flying_gesture_state = gesture_states.FLYING_MAX_2;
        }
        else if(gesture_states.cyclesInState_flying >= FLYING_TIMEOUT){
            gesture_states.cyclesInState_flying = 0;
            gesture_states.flying_gesture_state = gesture_states.FLYING_INIT;
        }
        else
        {
            gesture_states.cyclesInState_flying++;
        }
        cout << "FLYING_MIN_1..." << endl;
        break;

    case gesture_states.FLYING_MAX_2:
        if((LeftY >= -100) &&
                (LeftY <= 0) &&
                (RightY >= -90) &&
                (RightY <= -50))
        {
            cout << "FLYING GESTURE DETECTED!" << endl;
            gesture_states.flying_gesture_state = gesture_states.FLYING_INIT;
            return GESTURE_FLYING;
        }
        else if(gesture_states.cyclesInState_flying >= FLYING_TIMEOUT){
            gesture_states.cyclesInState_flying = 0;
            gesture_states.flying_gesture_state = gesture_states.FLYING_INIT;
        }
        else
        {
            gesture_states.cyclesInState_flying++;
        }
        //        cout << "FLYING_MAX_2..." << endl;
        break;

    default:
        break;
    }

    // RIGHT WAVING
    switch(gesture_states.waving_r_gesture_state)
    {
    case gesture_states.WAVING_R_INIT:
        //        if((LeftX >= 40) &&
        //                (LeftX <= 90) &&
        //                (LeftY >= 10) &&
        //                (LeftY <= 40) &&
        //                ((RightX > 0) || (RightX < -50)) &&
        //                ((RightY > 50) || (RightY < 20)))
        //        {
        if((LeftX >= 40) &&
                (LeftX <= 90) &&
                (LeftY >= -20) && // 10
                (LeftY <= 40) &&
                ((RightX > 0) || (RightX < -50)) &&
                ((RightY > 50) || (RightY < 20)))
        {
            gesture_states.cyclesInState_waving_r = 0;
            gesture_states.waving_r_gesture_state = gesture_states.WAVING_R_MAX_1;
        }
        //        cout << "WAVING_R_INIT..." << endl;
        break;

    case gesture_states.WAVING_R_MAX_1:
        //        if((LeftX >= -10) &&
        //                (LeftX <= 40) &&
        //                (LeftY >= 0) &&
        //                (LeftY <= 40) &&
        //                ((RightX > 0) || (RightX < -50)) &&
        //                ((RightY > 50) || (RightY < 20)))
        //        {
        if((LeftX >= -20) &&
                (LeftX <= 40) &&
                (LeftY >= -20) && // 10
                (LeftY <= 40) &&
                ((RightX > 0) || (RightX < -50)) &&
                ((RightY > 50) || (RightY < 20)))
        {
            gesture_states.cyclesInState_waving_r = 0;
            gesture_states.waving_r_gesture_state = gesture_states.WAVING_R_MIN_1;
        }
        else if(gesture_states.cyclesInState_waving_r >= WAVING_TIMEOUT){
            gesture_states.cyclesInState_waving_r = 0;
            gesture_states.waving_r_gesture_state = gesture_states.WAVING_R_INIT;
        }
        else
        {
            gesture_states.cyclesInState_waving_r++;
        }
        //        cout << "WAVING_R_MAX_1..." << endl;
        break;

    case gesture_states.WAVING_R_MIN_1:
        if((LeftX >= 40) &&
                (LeftX <= 90) &&
                (LeftY >= -20) && // 10
                (LeftY <= 40) &&
                ((RightX > 0) || (RightX < -50)) &&
                ((RightY > 50) || (RightY < 20)))
        {
            gesture_states.cyclesInState_waving_r = 0;
            gesture_states.waving_r_gesture_state = gesture_states.WAVING_R_MAX_2;
        }
        else if(gesture_states.cyclesInState_waving_r >= WAVING_TIMEOUT){
            gesture_states.cyclesInState_waving_r = 0;
            gesture_states.waving_r_gesture_state = gesture_states.WAVING_R_INIT;
        }
        else
        {
            gesture_states.cyclesInState_waving_r++;
        }
        //        cout << "WAVING_R_MIN_1..." << endl;
        break;

    case gesture_states.WAVING_R_MAX_2:
        if((LeftX >= -20) &&
                (LeftX <= 40) &&
                (LeftY >= -20) && // 10
                (LeftY <= 40) &&
                ((RightX > 0) || (RightX < -50)) &&
                ((RightY > 50) || (RightY < 20)))
        {
            cout << "RIGHT WAVING GESTURE DETECTED!" << endl;
            gesture_states.waving_r_gesture_state = gesture_states.WAVING_R_INIT;
            return GESTURE_WAVING_R;
        }
        else if(gesture_states.cyclesInState_waving_r >= WAVING_TIMEOUT) {
            gesture_states.cyclesInState_waving_r = 0;
            gesture_states.waving_r_gesture_state = gesture_states.WAVING_R_INIT;
        }
        else
        {
            gesture_states.cyclesInState_waving_r++;
        }
        //cout << "WAVING_R_MAX_2..." << endl;
        break;

    default:
        break;
    }

    // LEFT WAVING

    switch(gesture_states.waving_l_gesture_state)
    {
    case gesture_states.WAVING_L_INIT:
        //        if((RightX >= -70) &&
        //                (RightX <= -30) &&
        //                (RightY >= -10) &&
        //                (RightY <= 70))
        //        {
        if((RightX >= -110) &&
                (RightX <= -50) &&
                (RightY >= -50) &&
                (RightY <= 40) &&
                ((LeftX > 70) || (LeftX < 0)) &&
                ((LeftX > 50) || (LeftY < 0)))
        {
            gesture_states.cyclesInState_waving_l = 0;
            gesture_states.waving_l_gesture_state = gesture_states.WAVING_L_MAX_1;
        }
        //        cout << "WAVING_L_INIT..." << endl;
        break;

    case gesture_states.WAVING_L_MAX_1:
        //        if((RightX >= -30) &&
        //                (RightX <= 10) &&
        //                (RightY >= -10) &&
        //                (RightY <= 70))
        //        {
        if((RightX >= -60) &&
                (RightX <= 0) &&
                (RightY >= -20) &&
                (RightY <= 50) &&
                ((LeftX > 70) || (LeftX < 0)) &&
                ((LeftX > 50) || (LeftY < 0)))
        {
            gesture_states.cyclesInState_waving_l = 0;
            gesture_states.waving_l_gesture_state = gesture_states.WAVING_L_MIN_1;
        }
        else if(gesture_states.cyclesInState_waving_l >= WAVING_TIMEOUT){
            gesture_states.cyclesInState_waving_l = 0;
            gesture_states.waving_l_gesture_state = gesture_states.WAVING_L_INIT;
        }
        else
        {
            gesture_states.cyclesInState_waving_l++;
        }
        //        cout << "WAVING_L_MAX_1..." << endl;
        break;

    case gesture_states.WAVING_L_MIN_1:
        if((RightX >= -100) &&
                (RightX <= -50) &&
                (RightY >= -20) &&
                (RightY <= 40) &&
                ((LeftX > 70) || (LeftX < 0)) &&
                ((LeftX > 50) || (LeftY < 0)))
        {
            gesture_states.cyclesInState_waving_l = 0;
            gesture_states.waving_l_gesture_state = gesture_states.WAVING_L_MAX_2;
        }
        else if(gesture_states.cyclesInState_waving_l >= WAVING_TIMEOUT){
            gesture_states.cyclesInState_waving_l = 0;
            gesture_states.waving_l_gesture_state = gesture_states.WAVING_L_INIT;
        }
        else
        {
            gesture_states.cyclesInState_waving_l++;
        }
        //        cout << "WAVING_L_MIN_1..." << endl;
        break;

    case gesture_states.WAVING_L_MAX_2:
        if((RightX >= -40) &&
                (RightX <= 0) &&
                (RightY >= -20) &&
                (RightY <= 40) &&
                ((LeftX > 70) || (LeftX < 0)) &&
                ((LeftX > 50) || (LeftY < 0)))
        {
            cout << "LEFT WAVING GESTURE DETECTED!" << endl;
            gesture_states.waving_l_gesture_state = gesture_states.WAVING_L_INIT;
            return GESTURE_WAVING_L;
        }
        else if(gesture_states.cyclesInState_waving_l >= WAVING_TIMEOUT){
            gesture_states.cyclesInState_waving_l = 0;
            gesture_states.waving_l_gesture_state = gesture_states.WAVING_L_INIT;
        }
        else
        {
            gesture_states.cyclesInState_waving_l++;
        }
        //cout << "WAVING_L_MAX_2..." << endl;
        break;

    default:
        break;
    }


//    // RUNNING
//    switch(gesture_states.running_gesture_state)
//    {
//    case gesture_states.RUNNING_INIT:
//        if(/*(RightX >= -40) &&
//                                (RightX <= -10) &&*/
//                (RightX >= -110) &&
//                (RightY >= -110) &&
//                (RightY <= -80) &&
//                (LeftX >= 0) &&
//                (LeftX <= 50) &&
//                (LeftY >= 0) &&
//                (LeftY <= 40))
//        {
//            gesture_states.cyclesInState_running = 0;
//            gesture_states.running_gesture_state = gesture_states.RUNNING_MAX_1;
//        }
//        //        cout << "RUNNING_INIT..." << endl;
//        break;

//    case gesture_states.RUNNING_MAX_1:
//        if((RightX >= -40) &&
//                (RightX <= 10) &&
//                (RightY >= -10) &&
//                (RightY <= 50) &&
//                (LeftY < 100)/*  // this is here to prevent detection when user crouches (due to LH giving (0,0,0))
//                                (LeftX >= 10) &&
//                                (LeftX <= 40) &&
//                                (LeftY >= -100) &&
//                                (LeftY <= -70)*/)
//        {
//            gesture_states.cyclesInState_running = 0;
//            gesture_states.running_gesture_state = gesture_states.RUNNING_MIN_1;
//        }
//        else if(gesture_states.cyclesInState_running >= RUNNING_TIMEOUT){
//            gesture_states.cyclesInState_running = 0;
//            gesture_states.running_gesture_state = gesture_states.RUNNING_INIT;
//        }
//        else
//        {
//            gesture_states.cyclesInState_running++;
//        }
//        cout << "RUNNING_MAX_1..." << endl;
//        break;

//    case gesture_states.RUNNING_MIN_1:
//        if(/*(RightX >= -40) &&
//                                        (RightX <= -10) &&*/
//                (RightX >= -110) &&
//                (RightY >= -110) &&
//                (RightY <= -80) &&
//                (LeftX >= 0) &&
//                (LeftX <= 50) &&
//                (LeftY >= 0) &&
//                (LeftY <= 40))
//        {
//            gesture_states.cyclesInState_running = 0;
//            gesture_states.running_gesture_state = gesture_states.RUNNING_MAX_2;
//        }
//        else if(gesture_states.cyclesInState_running >= RUNNING_TIMEOUT){
//            gesture_states.cyclesInState_running = 0;
//            gesture_states.running_gesture_state = gesture_states.RUNNING_INIT;
//        }
//        else
//        {
//            gesture_states.cyclesInState_running++;
//        }
//        cout << "RUNNING_MIN_1..." << endl;
//        break;

//    case gesture_states.RUNNING_MAX_2:
//        if((RightX >= -40) &&
//                (RightX <= 10) &&
//                (RightY >= -10) &&
//                (RightY <= 50) &&
//                (LeftY < 100)/*  // this is here to prevent detection when user crouches (due to LH giving (0,0,0))
//                                (LeftX >= 10) &&
//                                (LeftX <= 40) &&
//                                (LeftY >= -100) &&
//                                (LeftY <= -70)*/)
//        {
//            cout << "RUNNING GESTURE DETECTED!" << endl;
//            gesture_states.running_gesture_state = gesture_states.RUNNING_INIT;
//            return GESTURE_RUNNING;
//        }
//        else if(gesture_states.cyclesInState_running >= RUNNING_TIMEOUT){
//            gesture_states.cyclesInState_running= 0;
//            gesture_states.running_gesture_state = gesture_states.RUNNING_INIT;
//        }
//        else
//        {
//            gesture_states.cyclesInState_running++;
//        }
//        //        cout << "RUNNING_MAX_2..." << endl;
//        break;

//    default:
//        break;
//    }

//    // JUMPING GESTURE

//    switch(gesture_states.jumping_gesture_state)
//    {
//    case gesture_states.JUMPING_INIT:
//        for(int i = 0; i < jumpVector.size(); i++ ) {
//            //            cout << "in INIT "<<jumpVector[i] << ", ";
//        }
//        cout << endl;

//        if(jumpCount<15){
//            jumpVector.push_back(jointCoords.heady); // do this 7 times
//            cout<<jumpCount<<endl<<endl;
//            jumpCount++;//maybe stop after 7 times
//        }
//        else{
//            jumpVector.erase(jumpVector.begin());
//            jumpVector.push_back(jointCoords.heady);
//            for(int i=1; i<15; i++){
//                if((jumpVector[i] < jumpVector[0] - 30) &&
//                        (jointCoords.headz >= 1650) &&
//                        (jointCoords.headz <= 2200)){
//                    jumpStartValue = jumpVector[0];
//                    cout << "START VALUE: " << jumpStartValue << endl;
//                    gesture_states.jumping_gesture_state = gesture_states.JUMPING_MAX;
//                    //            cout << "ENTER JUMPMAX..." << endl;
//                    break;
//                }
//            }
//        }

//        //                cout << "JUMPING_INIT..." << endl;
//        break;
//    case gesture_states.JUMPING_MAX:
//        for(int i = 0; i < jumpVector.size(); i++ ) {
//            //            cout << "in MAX "<<jumpVector[i] << ", ";
//        }
//        cout << endl;
//        if(gesture_states.cyclesInState_jumping >= JUMPING_TIMEOUT){
//            gesture_states.cyclesInState_jumping = 0;
//            gesture_states.jumping_gesture_state = gesture_states.JUMPING_INIT;
//            jumpVector.clear();
//            jumpCount=0;
//        } else {

//            if(/*abs(jointCoords.heady - jumpStartValue) <= 15*/ (jointCoords.heady > jumpStartValue - 15) &&
//                    (jointCoords.headz >= 1650) &&
//                    (jointCoords.headz <= 2200)) {
//                gesture_states.jumping_gesture_state = gesture_states.JUMPING_MIN;
//                break;
//            }
//            gesture_states.cyclesInState_jumping++;
//            cout << "JUMPING_MAX..." << endl;
//        }
//        break;

//    case gesture_states.JUMPING_MIN:
//        cout << "JUMPING GESTURE DETECTED!" << endl<<endl<<endl;
//        jumpVector.clear();
//        jumpCount=0;
//        gesture_states.cyclesInState_jumping = 0;
//        gesture_states.jumping_gesture_state = gesture_states.JUMPING_INIT;

//        return GESTURE_JUMPING;
//        break;

//    default:
//        break;
//    }

    //    switch(gesture_states.jumping_gesture_state)
    //    {
    //    case gesture_states.JUMPING_INIT:
    //        //        cout << "HEAD PREV: " << jumpHeadPreX << ", " << jumpHeadPreY << ", " << jumpHeadPreZ << " | CURR: " << jumpHeadCurrX << ", " << jumpHeadCurrY << ", " << jumpHeadCurrZ << endl;
    //        if((jumpHeadCurrY < jumpHeadPreY - 20) &&
    //                (jumpHeadCurrY != 0) && (jumpHeadPreY != 0) &&
    //                (jumpHeadCurrZ >= 1650) &&
    //                (jumpHeadCurrZ <= 2200) &&
    //                ((LeftY < 45) || (LeftY > 100)) &&
    //                ((RightY < 50) || RightY > 90))
    //        {
    //            //            cout << "ENTER JUMPMAX..." << endl;
    //            gesture_states.jumping_gesture_state = gesture_states.JUMPING_MAX;
    //        }
    //        else {
    //            jumpHeadPreX = jumpHeadCurrX;
    //            jumpHeadPreY = jumpHeadCurrY;
    //            jumpHeadPreZ = jumpHeadCurrZ;
    //            //            cout << "REPLACE OLD VALUES..." << endl;
    //        }
    //        //        cout << "JUMPING_INIT..." << endl;
    //        break;
    //    case gesture_states.JUMPING_MAX:
    //        if((jumpHeadCurrY >= jumpHeadPreY - 5) && //+10 OG
    //                (jumpHeadCurrY != 0) && (jumpHeadPreY!=0) &&
    //                (jumpHeadCurrZ >= 1650) &&
    //                (jumpHeadCurrY <= 2200) &&
    //                ((LeftY < 45) || (LeftY > 100)) &&
    //                ((RightY < 50) || RightY > 90))
    //        {
    //            gesture_states.cyclesInState_jumping = 0;
    //            gesture_states.jumping_gesture_state = gesture_states.JUMPING_MIN;
    //        }
    //        else if(gesture_states.cyclesInState_jumping >= JUMPING_TIMEOUT){
    //            gesture_states.cyclesInState_jumping = 0;
    //            jumpHeadPreX = jumpHeadCurrX;
    //            jumpHeadPreY = jumpHeadCurrY;
    //            jumpHeadPreZ = jumpHeadCurrZ;
    //            gesture_states.jumping_gesture_state = gesture_states.JUMPING_INIT;
    //        }
    //        else
    //        {
    //            gesture_states.cyclesInState_jumping++;
    //        }
    //        cout << "JUMPING_MAX: HEAD PREV: " << jumpHeadPreX << ", " << jumpHeadPreY << ", " << jumpHeadPreZ << " | CURR: " << jumpHeadCurrX << ", " << jumpHeadCurrY << ", " << jumpHeadCurrZ << endl;
    //        break;

    //    case gesture_states.JUMPING_MIN:
    //        cout << "JUMPING GESTURE DETECTED!" << endl<<endl<<endl;
    //        jumpHeadPreX = jumpHeadCurrX;
    //        jumpHeadPreY = jumpHeadCurrY;
    //        jumpHeadPreZ = jumpHeadCurrZ;
    //        gesture_states.jumping_gesture_state = gesture_states.JUMPING_INIT;

    //        return GESTURE_JUMPING;
    //        break;

    //    default:
    //        break;
    //    }

            printJointCoords(jointCoords);

    return GESTURE_UNDEFINED;
}

void printJointCoords(jointCoords_t& jc)
{
    // 6: LH, 7: RH, 10: H, 19: S, 16: LS, 17: RS
    cout << "LH: " << jc.Lhandx << ", " << jc.Lhandy
         << ", " << jc.Lhandz
         << "|RH: " << jc.Rhandx << ", " << jc.Rhandy
         << ", " << jc.Rhandz
         << "|LS: " << jc.Lshoulderx << ", " << jc.Lshouldery
         << ", " << jc.Lshoulderz
         << "|RS: " << jc.Rshoulderx << ", " << jc.Rshouldery
         << ", " << jc.Rshoulderz
//         << "|H: " << jc.headx << ", " << jc.heady
//         << ", " << jc.headz
            //         << "|S: " << jc.Spinex << ", " << jc.Spiney
            //         << ", " << jc.Spinez
         << endl;
}

void playContent(gestures_e gesture, bool quit)
{

    // Play the specified video in fullscreen mode and close vlc when finished
    // (We should use this in our production code)
    //    system("cvlc -f --play-and-exit file:///home/zac/electricTree/videos/test.mov");

    // Play the specified video on a loop (useful for testing cancel gesture)
    //system("cvlc -R file:///home/zac/electricTree/videos/test.mov");
    int rand_idx;

    // uncomment this one to get fullscreen:
//    string vlc_cmd ("cvlc -f --one-instance --no-video-title-show ");
    string vlc_cmd ("cvlc -f --one-instance --no-video-title-show ");


//    string vlc_cmd ("cvlc --one-instance --no-video-title-show ");
    string base_path ("file:///home/zac/electricTree/videos/");

    string title;
    string idx (to_string(rand_idx));
    string ext (".mp4");
    string ext2 (".mov");

    string full_cmd;

    switch (gesture)
    {
    case GESTURE_VICTORY:
        //system("cvlc -f --play-and-exit --no-video-title-show file:///home/zac/electricTree/videos/victory.mov");
        title.assign("victory");
        full_cmd.assign(vlc_cmd + base_path + title + ext);
        victory_count++;
        break;
    case GESTURE_USAIN:
        //system("cvlc -f --play-and-exit --no-video-title-show file:///home/zac/electricTree/videos/bolt.mov");
        title.assign("bolt");
        full_cmd.assign(vlc_cmd + base_path + title + ext2);
        usain_count++;
        break;
    case GESTURE_T:
        //system("cvlc -f --play-and-exit --no-video-title-show file:///home/zac/electricTree/videos/bolt.mov");
        title.assign("tpose");
        full_cmd.assign(vlc_cmd + base_path + title + ext2);
        t_count++;
        break;
    case GESTURE_POWERPOSE:
        title.assign("flexing");
        full_cmd.assign(vlc_cmd + base_path + title + ext);
        powerpose_count++;
        cout << "POWER POSE COUNT: " << powerpose_count << endl;
        break;
    case GESTURE_STOP:
        //system("cvlc -f --play-and-exit --no-video-title-show file:///home/zac/electricTree/videos/fly.mov");
        title.assign("fly");
        full_cmd.assign(vlc_cmd + base_path + title + ext);
        stop_count++;
        break;
    case GESTURE_POINTING_TRF:
        //system("cvlc -f --play-and-exit --no-video-title-show file:///home/zac/electricTree/videos/toprightforward.mp4");
        title.assign("toprightforward");
        full_cmd.assign(vlc_cmd + base_path + title + ext);
        pointing_trf_count++;
        break;
    case GESTURE_POINTING_RF:
        //system("cvlc -f --play-and-exit --no-video-title-show file:///home/zac/electricTree/videos/rightforward.mp4");
        title.assign("rightforward");
        full_cmd.assign(vlc_cmd + base_path + title + ext);
        pointing_rf_count++;
        break;
    case GESTURE_POINTING_TLF:
        //system("cvlc -f --play-and-exit --no-video-title-show file:///home/zac/electricTree/videos/topleftforward.mp4");
        title.assign("topleftforward");
        full_cmd.assign(vlc_cmd + base_path + title + ext);
        pointing_tlf_count++;
        break;
    case GESTURE_POINTING_LF:
        //system("cvlc -f --play-and-exit --no-video-title-show file:///home/zac/electricTree/videos/leftforward.mp4");
        title.assign("leftforward");
        full_cmd.assign(vlc_cmd + base_path + title + ext);
        pointing_lf_count++;
        break;
    case GESTURE_POINTING_TR:
        //system("cvlc -f --play-and-exit --no-video-title-show file:///home/zac/electricTree/videos/topright.mp4");
        title.assign("topright");
        full_cmd.assign(vlc_cmd + base_path + title + ext);
        pointing_tr_count++;
        break;
    case GESTURE_POINTING_R:
        //system("cvlc -f --play-and-exit --no-video-title-show file:///home/zac/electricTree/videos/right.mp4");
        title.assign("right");
        full_cmd.assign(vlc_cmd + base_path + title + ext);
        pointing_r_count++;
        break;
    case GESTURE_POINTING_TL:
        //system("cvlc -f --play-and-exit --no-video-title-show file:///home/zac/electricTree/videos/topleft.mp4");
        title.assign("topleft");
        full_cmd.assign(vlc_cmd + base_path + title + ext);
        pointing_tl_count++;
        break;
    case GESTURE_POINTING_L:
        //system("cvlc -f --play-and-exit --no-video-title-show file:///home/zac/electricTree/videos/left.mp4");
        title.assign("left");
        full_cmd.assign(vlc_cmd + base_path + title + ext);
        pointing_l_count++;
        break;
    case GESTURE_FLYING:
        //system("cvlc -f --play-and-exit --no-video-title-show file:///home/zac/electricTree/videos/fly.mov");
        title.assign("fly");
        full_cmd.assign(vlc_cmd + base_path + title + ext);
        flying_count++;
        break;
    case GESTURE_WAVING_L:
        //system("cvlc -f --play-and-exit --no-video-title-show file:///home/zac/electricTree/videos/fly.mov");
        title.assign("fly");
        full_cmd.assign(vlc_cmd + base_path + title + ext);
        waving_l_count++;
        break;

    case GESTURE_IDLE:
        rand_idx = rand() % 3;
        idx.assign(to_string(rand_idx));
        //cout << rand_idx << endl;

        if(rand_idx > 0)
        {
            title.assign("idle-");
            full_cmd.assign(vlc_cmd + base_path + title + idx + ext);
        }
        else
        {
            title.assign("idle");
            full_cmd.assign(vlc_cmd + base_path + title + ext);
        }

//        if(!quit) {
//            system(full_cmd.c_str());
//        }
        break;
    case GESTURE_READY:
        title.assign("ready");
        full_cmd.assign(vlc_cmd + base_path + title + ext2);
        cout << "play video in ready" <<endl<<endl;
        break;
    default:
        system("cvlc -f --play-and-exit --no-video-title-show file:///home/zac/electricTree/videos/test.mov");
        title.assign("test");
        full_cmd.assign(vlc_cmd + base_path + title + ext2);
        break;

    }

    if(!quit) {
        system(full_cmd.c_str());
    }

    // Set finished to true. This is a reference so its value will be seen in the main loop.
    if(gesture == GESTURE_READY)
    {
        cout << "ReAdY" << endl;
    }
    else if(gesture == GESTURE_IDLE)
    {
        cout << "Idle" << endl;
    }
    else
    {
        cout << "General" << endl;
    }

}

gestures_e currentVideoType()
{
    FILE *pPipe;
    pPipe = popen("lsof -wc vlc | awk '$4~\"[0-9]r\" && $5==\"REG\"' | grep -o '[^/]*$'", "r");

    char buf[20];

    int i=0;
    if(fgets(buf, 20, pPipe) != NULL)
    {
        puts(buf);
    }

    pclose(pPipe);

    if(strstr(buf, "ready") != NULL)
    {
        return GESTURE_READY;
    }
    else if(strstr(buf, "idle") != NULL)
    {
        return GESTURE_IDLE;
    }
    else if(strlen(buf) == 0)
    {
        return GESTURE_UNDEFINED;
    }
    else
    {
        return GESTURE_USAIN;
    }
}

void updateAnalytics(bool update, gesture_states_t &gesture_states) {
    // Update Analytics
    if(update == true) {
        bool error_detected_in = false;
        bool error_detected_out = false;

        ifstream inFile;
        inFile.open(filename);
        if(!inFile.is_open()) {
            cout << "Unable to open read file" << endl;
            error_detected_in = true;
        }

        ofstream outFile;
        outFile.open(temp_filename);
        if(!outFile.is_open()) {
            cout << "Unable to open output file." << endl;
            error_detected_out = true;
        }

        if(error_detected_in == false && error_detected_out == true) inFile.close();
        if(error_detected_in == true && error_detected_out == false) outFile.close();

        if(error_detected_in == false && error_detected_out == false) {
            cout << endl << "***************************" << endl << endl;

            inFile >> format;
            cout << format << endl;
            outFile << format << endl;

            inFile >> format;
            cout << format << endl;
            outFile << format << endl;


            while(inFile >> gesture >> gestureCount >> total >> totalCount) {
                if(gesture == "USAIN:"){
                    totalCount += usain_count;
                    gestureCount = usain_count;
                }
                else if (gesture == "POWERPOSE:") {
                    totalCount += powerpose_count;
                    gestureCount = powerpose_count;
                }
                else if (gesture == "VICTORY:") {
                    totalCount += victory_count;
                    gestureCount = victory_count;
                }
                else if (gesture == "TPOSE:") {
                    totalCount += t_count;
                    gestureCount = t_count;
                }
                else if (gesture == "STOP:") {
                    totalCount += stop_count;
                    gestureCount = stop_count;
                }
                else if (gesture == "POINTING_TRF:") {
                    totalCount += pointing_trf_count;
                    gestureCount = pointing_trf_count;
                }
                else if (gesture == "POINTING_RF:") {
                    totalCount += pointing_rf_count;
                    gestureCount = pointing_rf_count;
                }
                else if (gesture == "POINTING_TLF:") {
                    totalCount += pointing_tlf_count;
                    gestureCount = pointing_tlf_count;
                }
                else if (gesture == "POINTING_LF:") {
                    totalCount += pointing_lf_count;
                    gestureCount = pointing_lf_count;
                }
                else if (gesture == "POINTING_TR:") {
                    totalCount += pointing_tr_count;
                    gestureCount = pointing_tr_count;
                }
                else if (gesture == "POINTING_R:") {
                    totalCount += pointing_r_count;
                    gestureCount = pointing_r_count;
                }
                else if (gesture == "POINTING_TL:") {
                    totalCount += pointing_tl_count;
                    gestureCount = pointing_tl_count;
                }
                else if (gesture == "POINTING_L:") {
                    totalCount += pointing_l_count;
                    gestureCount = pointing_l_count;
                }
                else if (gesture == "FLYING:") {
                    totalCount += flying_count;
                    gestureCount = flying_count;
                }
                else if (gesture == "WAVING_R:") {
                    totalCount += waving_r_count;
                    gestureCount = waving_r_count;
                }
                else if (gesture == "WAVING_L:") {
                    totalCount += waving_l_count;
                    gestureCount = waving_l_count;
                }

                // Present day gestures count
                cout << gesture << " " << gestureCount << " ";
                outFile << gesture << " " << gestureCount << " ";


                // Total Count of Gestures to date
                cout << total << " " << totalCount << endl;
                outFile << total << " " << totalCount << endl;
            }

            inFile.close();
            outFile.close();

            cout << endl;

            result = remove(filename);
            if(result != 0)
                perror("Error deleting file");
            else
                puts("File successfully deleted");

            result = rename(temp_filename, filename);
            if(result != 0)
                perror("Error renaming file");
            else
                puts("File successfully renamed");
        }

        cout << endl << "UPDATED ANALYTICS!" << endl;
        cout << endl << "***************************" << endl;
    }
}

void resetGestureStates(gesture_states_t &gesture_states)
{
    gesture_states.usain_gesture_state = gesture_states.USAIN_INIT;
    gesture_states.tpose_gesture_state = gesture_states.TPOSE_INIT;
    gesture_states.victory_gesture_state = gesture_states.VICTORY_INIT;
    gesture_states.powerpose_gesture_state = gesture_states.POWERPOSE_INIT;
    gesture_states.stop_gesture_state = gesture_states.STOP_INIT;
    gesture_states.pointing_trf_gesture_state = gesture_states.POINTING_TRF_INIT;
    gesture_states.pointing_rf_gesture_state = gesture_states.POINTING_RF_INIT;
    gesture_states.pointing_tlf_gesture_state = gesture_states.POINTING_TLF_INIT;
    gesture_states.pointing_lf_gesture_state = gesture_states.POINTING_LF_INIT;
    gesture_states.pointing_tr_gesture_state = gesture_states.POINTING_TR_INIT;
    gesture_states.pointing_r_gesture_state = gesture_states.POINTING_R_INIT;
    gesture_states.pointing_tl_gesture_state = gesture_states.POINTING_TL_INIT;
    gesture_states.pointing_l_gesture_state = gesture_states.POINTING_L_INIT;
    gesture_states.flying_gesture_state = gesture_states.FLYING_INIT;
    gesture_states.waving_r_gesture_state = gesture_states.WAVING_R_INIT;
    gesture_states.waving_l_gesture_state = gesture_states.WAVING_L_INIT;
    gesture_states.jumping_gesture_state = gesture_states.JUMPING_INIT;
    gesture_states.running_gesture_state = gesture_states.RUNNING_INIT;
}


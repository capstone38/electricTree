// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

// C++ std libraries
#include <iostream>
#include <signal.h>
#include <thread>
#include <cstdlib>
#include <ctime>

// Boost Libraries-
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


bool analyticsOn = false;
int result; // file renaming and removing error checking
char filename[] = "/home/capstone38/Desktop/electricTree/analytics.txt";
char temp_filename[] = "/home/capstone38/Desktop/electricTree/temp_analytics.txt";

int numVideos[GESTURE_UNDEFINED];

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



    rs_set_device_option(camera, RS_OPTION_COLOR_ENABLE_AUTO_EXPOSURE, 1, 0);

    rs_set_device_option(camera, RS_OPTION_R200_AUTO_EXPOSURE_MEAN_INTENSITY_SET_POINT, 1806, 0); //400


    rs_set_device_option(camera, RS_OPTION_R200_LR_GAIN  , 400, 0); //400
    rs_set_device_option(camera, RS_OPTION_R200_LR_EXPOSURE , 164, 0); //164


    rs_option options[11];

    options[0] = RS_OPTION_R200_AUTO_EXPOSURE_MEAN_INTENSITY_SET_POINT;
    options[1] = RS_OPTION_R200_DEPTH_CONTROL_ESTIMATE_MEDIAN_DECREMENT;
    options[2] = RS_OPTION_R200_DEPTH_CONTROL_ESTIMATE_MEDIAN_INCREMENT;
    options[3] = RS_OPTION_R200_DEPTH_CONTROL_MEDIAN_THRESHOLD;
    options[4] = RS_OPTION_R200_DEPTH_CONTROL_SCORE_MINIMUM_THRESHOLD;
    options[5] = RS_OPTION_R200_DEPTH_CONTROL_SCORE_MAXIMUM_THRESHOLD;
    options[6] = RS_OPTION_R200_DEPTH_CONTROL_TEXTURE_COUNT_THRESHOLD;
    options[7] = RS_OPTION_R200_DEPTH_CONTROL_TEXTURE_DIFFERENCE_THRESHOLD;
    options[8] = RS_OPTION_R200_DEPTH_CONTROL_SECOND_PEAK_THRESHOLD;
    options[9] = RS_OPTION_R200_DEPTH_CONTROL_NEIGHBOR_THRESHOLD;
    options[10] = RS_OPTION_R200_DEPTH_CONTROL_LR_THRESHOLD;


    rs_reset_device_options_to_default(camera, options, 11, 0);



    vector<Gesture> staticgesturelist = defineStaticGestures();
    vector<DynamicGesture> dynamicgesturelist = defineDynamicGestures();

    updateNumVideos(numVideos);

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

        // USE FOR DEBUG:
        // Display color image
        //        auto colorImage = sampleSet[rs::core::stream_type::color];
        //        console_view->render_color_frames(colorImage);

        //        // Release color and depth image
        //        sampleSet.images[static_cast<uint8_t>(rs::core::stream_type::color)]->release();
        //        sampleSet.images[static_cast<uint8_t>(rs::core::stream_type::depth)]->release();


        trackingData = ptModule->QueryOutput();

        auto ids_in_frame = console_view->get_person_ids(ptModule->QueryOutput());

        pid_in_center_prev = pid_in_center;

        for(auto iter = ids_in_frame->begin(); iter != ids_in_frame->end(); ++iter){
            int id = *iter;

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
                {
                    thread video(playContent, GESTURE_READY, shouldQuit);
                    video.detach();
                }

                state = STATE_READY;
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

            if(currentVideoType() == GESTURE_UNDEFINED)
            {
                {
                    thread video(playContent, GESTURE_READY, shouldQuit);
                    video.detach();
                }
            }



            if(pid_in_center != INVALID_PERSONID)
            {
                personJoints = personData->QuerySkeletonJoints();
                std::vector<Intel::RealSense::PersonTracking::PersonTrackingData::PersonJoints::SkeletonPoint> skeletonPoints(personJoints->QueryNumJoints());
                personJoints->QueryJoints(skeletonPoints.data());

                gestureDetected = detectGestures(personJoints, staticgesturelist, dynamicgesturelist, gesture_states);

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

                gestureDetected = detectGestures(personJoints, staticgesturelist, dynamicgesturelist, gesture_states);

                // Implement cancel gesture.
                if(gestureDetected == GESTURE_CANCEL)
                {
                    shouldCancel = true;
                    //system("killall vlc");

                }
            }


            if(currentVideoType() == GESTURE_UNDEFINED || shouldCancel)
            {
                state = STATE_READY;

                {
                    thread video(playContent, GESTURE_READY, shouldQuit);
                    video.detach();
                }


            }
            break;


        case STATE_IDLEVIDEO_START:
            // Issue system call to playback idle video content in a detached thread

            //do
        {
            thread idlevideo(playContent, GESTURE_IDLE, shouldQuit);
            idlevideo.detach();
        } //while(waitUntilContentStart(GESTURE_IDLE));

            cyclesSpentDetected = 0;

            state = STATE_IDLEVIDEO_UNDERWAY;
            break;

        case STATE_IDLEVIDEO_UNDERWAY:
            if(pid_in_center != INVALID_PERSONID)
            {
                if(cyclesSpentDetected >= SEC_TO_CYCLES(0.5)) {

                    state = STATE_IDLE;
                    cyclesSpentIdle = 0;
                    break;
                } else {
                    cyclesSpentDetected++;
                }
            }
            else
            {
                cyclesSpentDetected = 0;
            }

            if(currentVideoType() == GESTURE_UNDEFINED)
            {
                state = STATE_IDLEVIDEO_START;
            }

            break;
        }

    }

    system("killall vlc");


    pt_utils.stop_camera();
    actualModuleConfig.projection->release();
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
        return true;
    }

    return false;
}

gestures_e detectGestures(Intel::RealSense::PersonTracking::PersonTrackingData::PersonJoints *personJoints, vector<Gesture>& staticgesturelist, vector<DynamicGesture>& dynamicgesturelist, gesture_states_t& gesture_states)
{
    int numDetectedJoints = personJoints->QueryNumJoints();
    std::vector<Intel::RealSense::PersonTracking::PersonTrackingData::PersonJoints::SkeletonPoint> skeletonPoints(personJoints->QueryNumJoints());


    personJoints->QueryJoints(skeletonPoints.data());

    jointCoords_t jointCoords;

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

    gestures_e detectedGesture = GESTURE_UNDEFINED;

    for(Gesture &g : staticgesturelist)
    {
        if(g.detect(jointCoords))
        {
            detectedGesture = g.id;
        }
    }

    for(DynamicGesture &g : dynamicgesturelist)
    {
        if(g.detect(jointCoords))
        {
            detectedGesture = g.id;
        }
    }

    if(detectedGesture != GESTURE_UNDEFINED)
    {
        for(Gesture &g : staticgesturelist)
        {
            g.resetGestureState();
        }

        for(DynamicGesture &g : dynamicgesturelist)
        {
            g.resetStates();
        }
    }



    int LeftX = jointCoords.Lshoulderx - jointCoords.Lhandx;
    int LeftY = jointCoords.Lshouldery - jointCoords.Lhandy;
    int LeftZ = jointCoords.Lshoulderz - jointCoords.Lhandz;
    int RightX = jointCoords.Rshoulderx - jointCoords.Rhandx;
    int RightY = jointCoords.Rshouldery - jointCoords.Rhandy;
    int RightZ = jointCoords.Rshoulderz - jointCoords.Rhandz;

/*
    // RIGHT WAVING
    switch(gesture_states.waving_r_gesture_state)
    {
    case gesture_states.WAVING_R_INIT:
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
        break;

    case gesture_states.WAVING_R_MAX_1:
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
        break;

    case gesture_states.WAVING_R_MAX_2:
        if((LeftX >= -20) &&
                (LeftX <= 40) &&
                (LeftY >= -20) && // 10
                (LeftY <= 40) &&
                ((RightX > 0) || (RightX < -50)) &&
                ((RightY > 50) || (RightY < 20)))
        {
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
        break;

    default:
        break;
    }

*/
    return detectedGesture;

    /*

    // FLYING GESTURE
    switch(gesture_states.flying_gesture_state)
    {
    case gesture_states.FLYING_INIT:
        if(
                (LeftY >= -30) &&
                (LeftY <= 40) && //15
                (RightY >= -10) &&
                (RightY <= 50))
        {
            gesture_states.cyclesInState_flying = 0;
            gesture_states.flying_gesture_state = gesture_states.FLYING_MAX_1;
        }
        break;

    case gesture_states.FLYING_MAX_1:
        if((LeftY >= -110) &&
                (LeftY <= -60) &&
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
        break;

    case gesture_states.FLYING_MIN_1:
        if(
                (LeftY >= -30) &&
                (LeftY <= 40) && //15
                (RightY >= -10) &&
                (RightY <= 50))
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
        break;

    case gesture_states.FLYING_MAX_2:
        if((LeftY >= -110) &&
                (LeftY <= -60) &&
                (RightY >= -90) &&
                (RightY <= -50))
        {
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
        break;

    default:
        break;
    }

    // RIGHT WAVING
    switch(gesture_states.waving_r_gesture_state)
    {
    case gesture_states.WAVING_R_INIT:
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
        break;

    case gesture_states.WAVING_R_MAX_1:
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
        break;

    case gesture_states.WAVING_R_MAX_2:
        if((LeftX >= -20) &&
                (LeftX <= 40) &&
                (LeftY >= -20) && // 10
                (LeftY <= 40) &&
                ((RightX > 0) || (RightX < -50)) &&
                ((RightY > 50) || (RightY < 20)))
        {
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
        if((RightX >= -70) &&
                (RightX <= -30) &&
                (RightY >= -20) &&
                (RightY <= 40) &&
                ((LeftX > 70) || (LeftX < 0)) &&
                ((LeftY > 50) || (LeftY < 0)))
        {
            gesture_states.cyclesInState_waving_l = 0;
            gesture_states.waving_l_gesture_state = gesture_states.WAVING_L_MAX_1;
        }
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
                ((LeftY > 50) || (LeftY < 0)))
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
        break;

    case gesture_states.WAVING_L_MIN_1:
        if((RightX >= -70) &&
                (RightX <= -30) &&
                (RightY >= -20) &&
                (RightY <= 40) &&
                ((LeftX > 70) || (LeftX < 0)) &&
                ((LeftY > 50) || (LeftY < 0)))
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
        break;

    case gesture_states.WAVING_L_MAX_2:
        if((RightX >= -60) &&
                (RightX <= 0) &&
                (RightY >= -20) &&
                (RightY <= 50) &&
                ((LeftX > 70) || (LeftX < 0)) &&
                ((LeftY > 50) || (LeftY < 0)))
        {
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
        break;

    default:
        break;
    }
*/
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
         << endl;
}

void playContent(gestures_e gesture, bool quit)
{
    int rand_idx;

    // uncomment this one to get fullscreen:
    string vlc_cmd ("cvlc -f --one-instance --no-video-title-show file:///");

    string base_path ("/home/capstone38/Desktop/electricTree/videos/");

    string default_video(DEFAULT_VIDEO);

    string title;
    string idx (to_string(rand_idx));
    string ext (".mp4");
    string ext2 (".mov");

    string full_cmd;
    string full_cmd2;

    if(numVideos[gesture] != 0)
    {
        rand_idx = rand() % (numVideos[gesture]);
    }
    else
    {
        rand_idx = 0;
    }


    switch (gesture)
    {
    case GESTURE_VICTORY:
        title.assign("victory");
        victory_count++;
        break;
    case GESTURE_USAIN:
        title.assign("bolt");
        usain_count++;
        break;
    case GESTURE_T:
        title.assign("tpose");
        t_count++;
        break;
    case GESTURE_FLEXING:
        title.assign("flexing");
        powerpose_count++;
        break;
    case GESTURE_STOP:
        title.assign("stop");
        stop_count++;
        break;
    case GESTURE_POINTING_TRF:
        title.assign("toprightforward");
        pointing_trf_count++;
        break;
    case GESTURE_POINTING_RF:
        title.assign("rightforward");
        pointing_rf_count++;
        break;
    case GESTURE_POINTING_TLF:
        title.assign("topleftforward");
        pointing_tlf_count++;
        break;
    case GESTURE_POINTING_LF:
        title.assign("leftforward");
        pointing_lf_count++;
        break;
    case GESTURE_POINTING_TR:
        title.assign("topright");
        pointing_tr_count++;
        break;
    case GESTURE_POINTING_R:
        title.assign("right");
        pointing_r_count++;
        break;
    case GESTURE_POINTING_TL:
        title.assign("topleft");
        pointing_tl_count++;
        break;
    case GESTURE_POINTING_L:
        title.assign("left");
        pointing_l_count++;
        break;
    case GESTURE_FLYING:
        title.assign("fly");
        flying_count++;
        break;
    case GESTURE_WAVING_L: //CHANGE TO LEFT WAVING AFTER THE PRESENTATION
        title.assign("leftwave");
        waving_l_count++;
        break;

    case GESTURE_IDLE:
        title.assign("idle");
        break;
    case GESTURE_READY:
        title.assign("ready");
        break;
    default:
        title.assign("test");
        full_cmd.assign(base_path + title + ext2);
        break;

    }

    // process strings to escape brackets properly
    idx.assign(to_string(rand_idx));
    if(rand_idx > 0)
    {
        ext.assign("\).mp4");
        full_cmd.assign(base_path + title + "\(" + idx + ext);
        full_cmd2.assign(base_path + title + "\\(" + idx + "\\" + ext);
    }
    else
    {
        ext.assign(".mp4");
        full_cmd.assign(base_path + title + ext);
        full_cmd2.assign(full_cmd);
    }


    bool good = false;
    if(FILE *file = fopen(full_cmd.c_str(), "r"))
    {
        fclose(file);
        good = true;
    }

    string valid_video_cmd(vlc_cmd+full_cmd2);
    string default_video_cmd(vlc_cmd+default_video);

    if(!quit) {
        if(good)
        {
            system(valid_video_cmd.c_str());
        }
        else
        {
            system(default_video_cmd.c_str());
        }
    }

}

gestures_e currentVideoType()
{
    char buf[20];

    do
    {
        FILE *pPipe;
        pPipe = popen("lsof -wc vlc | awk '$4~\"[0-9]r\" && $5==\"REG\"' | grep -o '[^/]*$'", "r");


        if(fgets(buf, 20, pPipe) != NULL)
        {
            puts(buf);
        }

        pclose(pPipe);
    }
    while(strstr(buf, "maps") != NULL);

    if(strstr(buf, "ready") != NULL)
    {
        return GESTURE_READY;
    }
    else if(strstr(buf, "idle") != NULL)
    {
        return GESTURE_IDLE;
    }
    //else if(strlen(buf) == 0)
    else if(strstr(buf, "mov") != NULL || strstr(buf, "mp4") != NULL)
    {
        return GESTURE_USAIN;
    }
    else
    {
        return GESTURE_UNDEFINED;
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
            error_detected_in = true;
        }

        ofstream outFile;
        outFile.open(temp_filename);
        if(!outFile.is_open()) {
            error_detected_out = true;
        }

        if(error_detected_in == false && error_detected_out == true) inFile.close();
        if(error_detected_in == true && error_detected_out == false) outFile.close();

        if(error_detected_in == false && error_detected_out == false) {

            inFile >> format;
            outFile << format << endl;

            inFile >> format;
            outFile << format << endl;


            while(inFile >> gesture >> gestureCount >> total >> totalCount) {
                if(gesture == "USAIN:"){
                    totalCount += usain_count;
                    gestureCount = usain_count;
                }
                else if (gesture == "FLEXING:") {
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
                outFile << gesture << " " << gestureCount << " ";


                // Total Count of Gestures to date
                outFile << total << " " << totalCount << endl;
            }

            inFile.close();
            outFile.close();

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
    }
}

int detectNumVideo(string name)
{
    string base_path("/home/capstone38/Desktop/electricTree/videos/");
    string title;
    string ext;
    string full_cmd;

    int idx = 0;
    bool good = false;
    do
    {
        string id(to_string(idx));

        if(idx > 0)
        {
            title.assign(name + "\(");
            ext.assign("\).mp4");
            full_cmd.assign(base_path + title + id + ext);
        }
        else
        {
            title.assign(name);
            ext.assign(".mp4");
            full_cmd.assign(base_path + title + ext);
        }



        if(FILE *file = fopen(full_cmd.c_str(), "r"))
        {
            fclose(file);
            good = true;
        }
        else
        {
            good = false;
        }


        idx++;
    } while(good == true);
    idx--;

    return idx;
}

void updateNumVideos(int *numVideos)
{
    vector<string> names;

    names.push_back("bolt");
    names.push_back("tpose");
    names.push_back("victory");
    names.push_back("flexing");
    names.push_back("stop");
    names.push_back("fly");
    names.push_back("rightwave");
    names.push_back("leftwave");
    names.push_back("jump");
    names.push_back("toprightforward");
    names.push_back("rightforward");
    names.push_back("topleftforward");
    names.push_back("leftforward");
    names.push_back("topright");
    names.push_back("right");
    names.push_back("topleft");
    names.push_back("left");
    names.push_back("running");
    names.push_back("idle");
    names.push_back("ready");
    names.push_back("undefined");


    for(int gesture = 0; gesture < GESTURE_UNDEFINED; gesture++)
    {
        numVideos[gesture] = detectNumVideo(names[gesture]);
    }
}

int waitUntilContentStart(gestures_e gesture)
{
    int retval = 1;
    int timeout = 100;
    int i=0;
    while((currentVideoType() != gesture) && i < timeout)
    {
        i++; // wait
        if(i==timeout)
        {
            retval = 0;
        }
    }

    return retval;
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

vector<Gesture> defineStaticGestures(void)
{
    vector<Gesture> out;

    Gesture usain(GESTURE_USAIN,
                 (int)0,           // LeftX_min
                 (int)80,          // LeftX_max
                 (int)-60,         // LeftY_min
                 (int)30,          // LeftY_max
                 (int)-100,        // RightX_min
                 (int)-50,         // RightX_max
                 (int)20,          // RightY_min
                 (int)70);         // RightY_max
    out.push_back(usain);

    Gesture tpose(GESTURE_T,
                 (int)45,          // LeftX_min
                 (int)MAXCOORD,    // LeftX_max
                 (int)-20,         // LeftY_min
                 (int)20,          // LeftY_max
                 (int)-MAXCOORD,   // RightX_min
                 (int)-45,         // RightX_max
                 (int)-20,         // RightY_min
                 (int)20);         // RightY_max
    out.push_back(tpose);

    Gesture stop(GESTURE_STOP,
                 (int)-20,         // LeftX_min
                 (int)40,          // LeftX_max
                 (int)0,           // LeftY_min
                 (int)60,          // LeftY_max
                 (int)0,           // LeftZ_min
                 (int)1550,        // LeftZ_max
                 (int)-MAXCOORD,   // RightX_min
                 (int)MAXCOORD,    // RightX_max
                 (int)-MAXCOORD,   // RightY_min
                 (int)MAXCOORD,    // RightY_max
                 (int)0,           // RightZ_min
                 (int)MAXCOORD);   // RightZ_max
    out.push_back(stop);

    Gesture point_trf(GESTURE_POINTING_TRF,
                 (int)50,          // LeftX_min
                 (int)90,          // LeftX_max
                 (int)50,          // LeftY_min
                 (int)90,          // LeftY_max
                 (int)300,         // LeftZ_min
                 (int)500,         // LeftZ_max
                 (int)-MAXCOORD,   // RightX_min
                 (int)MAXCOORD,    // RightX_max
                 (int)110,         // RightY_min
                 (int)-40,         // RightY_max
                 (int)-MAXCOORD,   // RightZ_min
                 (int)MAXCOORD);   // RightZ_max
    out.push_back(point_trf);

    Gesture point_rf(GESTURE_POINTING_RF,
                 (int)60,          // LeftX_min
                 (int)120,         // LeftX_max
                 (int)-30,         // LeftY_min
                 (int)30,          // LeftY_max
                 (int)230,         // LeftZ_min
                 (int)670,         // LeftZ_max
                 (int)-MAXCOORD,   // RightX_min
                 (int)MAXCOORD,    // RightX_max
                 (int)110,         // RightY_min
                 (int)-40,         // RightY_max
                 (int)-MAXCOORD,   // RightZ_min
                 (int)MAXCOORD);   // RightZ_max
    out.push_back(point_rf);

    Gesture point_tlf(GESTURE_POINTING_TLF,
                 (int)-MAXCOORD,   // LeftX_min
                 (int)MAXCOORD,    // LeftX_max
                 (int)90,          // LeftY_min
                 (int)-50,         // LeftY_max
                 (int)-MAXCOORD,   // LeftZ_min
                 (int)MAXCOORD,    // LeftZ_max
                 (int)-100,        // RightX_min
                 (int)-50,         // RightX_max
                 (int)40,          // RightY_min
                 (int)80,          // RightY_max
                 (int)180,         // RightZ_min
                 (int)420);        // RightZ_max
    out.push_back(point_tlf);

    Gesture point_lf(GESTURE_POINTING_LF,
                 (int)-MAXCOORD,   // LeftX_min
                 (int)MAXCOORD,    // LeftX_max
                 (int)90,          // LeftY_min
                 (int)-50,         // LeftY_max
                 (int)-MAXCOORD,   // LeftZ_min
                 (int)MAXCOORD,    // LeftZ_max
                 (int)-120,        // RightX_min
                 (int)-60,         // RightX_max
                 (int)-20,         // RightY_min
                 (int)30,          // RightY_max
                 (int)70,          // RightZ_min
                 (int)550);        // RightZ_max
    out.push_back(point_lf);


    Gesture point_tr(GESTURE_POINTING_TR,
                 (int)50,          // LeftX_min
                 (int)100,         // LeftX_max
                 (int)30,          // LeftY_min
                 (int)80,          // LeftY_max
                 (int)-140,        // LeftZ_min
                 (int)20,          // LeftZ_max
                 (int)-MAXCOORD,   // RightX_min
                 (int)MAXCOORD,    // RightX_max
                 (int)110,         // RightY_min
                 (int)-40,         // RightY_max
                 (int)-MAXCOORD,   // RightZ_min
                 (int)MAXCOORD);   // RightZ_max
    out.push_back(point_tr);

    Gesture point_r(GESTURE_POINTING_R,
                 (int)80,          // LeftX_min
                 (int)110,         // LeftX_max
                 (int)-20,         // LeftY_min
                 (int)20,          // LeftY_max
                 (int)-200,        // LeftZ_min
                 (int)20,          // LeftZ_max
                 (int)-MAXCOORD,   // RightX_min
                 (int)MAXCOORD,    // RightX_max
                 (int)110,         // RightY_min
                 (int)-40,         // RightY_max
                 (int)-MAXCOORD,   // RightZ_min
                 (int)MAXCOORD);   // RightZ_max
    out.push_back(point_r);

    Gesture point_tl(GESTURE_POINTING_TL,
                 (int)-MAXCOORD,   // LeftX_min
                 (int)MAXCOORD,    // LeftX_max
                 (int)90,          // LeftY_min
                 (int)-50,         // LeftY_max
                 (int)-MAXCOORD,   // LeftZ_min
                 (int)MAXCOORD,    // LeftZ_max
                 (int)-80,         // RightX_min
                 (int)-50,         // RightX_max
                 (int)40,          // RightY_min
                 (int)80,          // RightY_max
                 (int)-120,        // RightZ_min
                 (int)110);        // RightZ_max
    out.push_back(point_tl);

    Gesture point_l(GESTURE_POINTING_L,
                 (int)-MAXCOORD,   // LeftX_min
                 (int)MAXCOORD,    // LeftX_max
                 (int)90,          // LeftY_min
                 (int)-50,         // LeftY_max
                 (int)-MAXCOORD,   // LeftZ_min
                 (int)MAXCOORD,    // LeftZ_max
                 (int)-100,        // RightX_min
                 (int)-70,         // RightX_max
                 (int)-10,         // RightY_min
                 (int)20,          // RightY_max
                 (int)-90,         // RightZ_min
                 (int)110);        // RightZ_max
    out.push_back(point_l);

    Gesture victory(GESTURE_VICTORY,
                 (int)45,          // LeftX_min
                 (int)100,         // LeftX_max
                 (int)45,          // LeftY_min
                 (int)90,          // LeftY_max
                 (int)-80,         // RightX_min
                 (int)-30,         // RightX_max
                 (int)50,          // RightY_min
                 (int)100);        // RightY_max
    out.push_back(victory);

    Gesture flexing(GESTURE_FLEXING,
                 (int)0,           // LeftX_min
                 (int)70,          // LeftX_max
                 (int)0,           // LeftY_min
                 (int)50,          // LeftY_max
                 (int)-50,         // RightX_min
                 (int)0,           // RightX_max
                 (int)20,          // RightY_min
                 (int)50);         // RightY_max
    out.push_back(flexing);

    return out;
}

vector<DynamicGesture> defineDynamicGestures(void)
{
    vector<DynamicGesture> out;

    DynamicGesture waving_r(GESTURE_WAVING_R);

    Gesture waving_r_int_0(GESTURE_WAVING_R,
                 (int)40,           // LeftX_min
                 (int)90,          // LeftX_max
                 (int)-20,           // LeftY_min
                 (int)40,          // LeftY_max
                 (int)0,         // RightX_min
                 (int)-50,           // RightX_max
                 (int)50,          // RightY_min
                 (int)20);         // RightY_max
    waving_r.addIntermediateGesture(waving_r_int_0);

    Gesture waving_r_int_1(GESTURE_WAVING_R,
                 (int)-20,           // LeftX_min
                 (int)40,          // LeftX_max
                 (int)-20,           // LeftY_min
                 (int)40,          // LeftY_max
                 (int)0,         // RightX_min
                 (int)-50,           // RightX_max
                 (int)50,          // RightY_min
                 (int)20);         // RightY_max
    waving_r.addIntermediateGesture(waving_r_int_1);

    Gesture waving_r_int_2(GESTURE_WAVING_R,
                 (int)40,           // LeftX_min
                 (int)90,          // LeftX_max
                 (int)-20,           // LeftY_min
                 (int)40,          // LeftY_max
                 (int)0,         // RightX_min
                 (int)-50,           // RightX_max
                 (int)50,          // RightY_min
                 (int)20);         // RightY_max
    waving_r.addIntermediateGesture(waving_r_int_2);

    Gesture waving_r_int_3(GESTURE_WAVING_R,
                 (int)-20,           // LeftX_min
                 (int)40,          // LeftX_max
                 (int)-20,           // LeftY_min
                 (int)40,          // LeftY_max
                 (int)0,         // RightX_min
                 (int)-50,           // RightX_max
                 (int)50,          // RightY_min
                 (int)20);         // RightY_max
    waving_r.addIntermediateGesture(waving_r_int_3);

    out.push_back(waving_r);

    return out;
}


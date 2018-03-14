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

    // Init RNG
    srand(time(nullptr));

    state_e state = STATE_IDLE;
    int cyclesSpentIdle = 0;
    int numTracked;
    gestures_e gestureDetected = GESTURE_UNDEFINED;
    bool playbackFinished = false;
    gesture_states_t gesture_states;
    resetGestureStates(gesture_states);

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


        trackingData = ptModule->QueryOutput();

        numTracked = trackingData->QueryNumberOfPeople();
        cout << "numTracked: " << numTracked << endl <<endl;


        auto new_ids = console_view->get_person_ids(ptModule->QueryOutput());

        for(auto iter = new_ids->begin(); iter != new_ids->end(); ++iter)
        {
            cout << "Id in set: " << *iter << endl;
        }


        for(auto iter = new_ids->begin(); iter != new_ids->end(); ++iter){
            cout<<"testing"<<endl;
            int id = *iter;
            //auto personData = trackingData->QueryPersonData(Intel::RealSense::PersonTracking::PersonTrackingData::ACCESS_ORDER_BY_INDEX, id);

            auto personData = trackingData->QueryPersonDataById(id);

            if(personData){
                Intel::RealSense::PersonTracking::PersonTrackingData::PersonTracking* personTrackData = personData->QueryTracking();
                Intel::RealSense::PersonTracking::PersonTrackingData::PointCombined centerMass = personTrackData->QueryCenterMass();
                cout<<"Person ID: " << id <<" has centerMass x:" << centerMass.world.point.x <<endl;
                cout<<"Person ID: " << id <<" has centerMass y:" << centerMass.world.point.y <<endl;
                cout<<"Person ID: " << id <<" has centerMass z:" << centerMass.world.point.z <<endl<<endl;


                if(personIsInCenter(centerMass))
                {
                    cout<<"Person ID: " << id <<" is in center!" <<endl<<endl;

                    trackingData->StartTracking(id);

                    personJoints = personData->QuerySkeletonJoints();
                    std::vector<Intel::RealSense::PersonTracking::PersonTrackingData::PersonJoints::SkeletonPoint> skeletonPoints(personJoints->QueryNumJoints());
                    personJoints->QueryJoints(skeletonPoints.data());

                    gestureDetected = detectGestures(personJoints, gesture_states);

                }
                else
                {
                    trackingData->StopTracking(id);
                }
            }
        }




  /*      //Main program FSM implementation
                switch (state)
        {
        case STATE_IDLE:
            numTracked = trackingData->QueryNumberOfPeople();
            //cout << "numTracked: " << numTracked << endl;

            //            if(1)
            //            {
            //                auto config = ptModule->QueryConfiguration()->QueryRecognition();
            //                auto database = config->QueryDatabase();

            //                cout << "num registered users: " << database->GetNumberOfRegisteredUsers() << endl;
            //            }

            if(numTracked == 1)
            {
                // If we are tracking exactly one person, detect their gesture
                cout << "found someone!" << endl;
                int personId = trackingData->QueryPersonData(
                            Intel::RealSense::PersonTracking::PersonTrackingData::ACCESS_ORDER_BY_INDEX, 0)->QueryTracking()->QueryId();
                cout << "ID before clearing database: " << personId << endl;


                auto config = ptModule->QueryConfiguration()->QueryRecognition();
                auto database = config->QueryDatabase();

                database->Clear();

                int *ids = new int(100);
                int *numIds = new int;

                database->GetRegisteredUsersIds(ids, 100, numIds);

                cout << *numIds << " Ids registered in database" << endl;
                for(int i=0; i< *numIds; i++)
                {
                    cout << ids[i] << endl;
                }

                delete ids;
                delete numIds;



                                personId = trackingData->QueryPersonData(
                                            Intel::RealSense::PersonTracking::PersonTrackingData::ACCESS_ORDER_BY_INDEX, 0)->QueryTracking()->QueryId();
                                cout << "ID after clearing database: " << personId << endl;

                if(personId == 0)
                {
                    console_view->set_tracking(ptModule);
                    state = STATE_READY;
                    //gesture_states.flying_gesture_state = gesture_states.FLYING_MAX_1;
                }

            }

            else if(cyclesSpentIdle >= SEC_TO_CYCLES(10))
            {
                // We have idled for 10 seconds. Begin playback of idle video
                state = STATE_IDLEVIDEO_START;
            }

            cyclesSpentIdle++;
            break;

        case STATE_READY:
            // Track skeleton joints
            if(trackingData->QueryNumberOfPeople() != 1)
            {
                // If we no longer see a person, back to idle state
                state = STATE_IDLE;
                cyclesSpentIdle = 0;
                //trackingData->StopTracking(0);

            }
            else
            {
                // Start tracking the first person detected in the frame
                personJoints = console_view->on_person_skeleton(ptModule);
                gestureDetected = detectGestures(personJoints, gesture_states);

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
                gestureDetected = detectGestures(personJoints, gesture_states);

                // Implement cancel gesture.
                if(gestureDetected == GESTURE_CANCEL)
                {
                    system("killall vlc");
                }
            }

            if(playbackFinished)
            {
                cout << "playback completed or killed!" << endl;
                state = STATE_READY;

                // double-check there are no stray vlc processes running
                system("killall vlc");
            }
            break;


        case STATE_IDLEVIDEO_START:
            // Issue system call to playback idle video content in a detached thread
            playbackFinished = false;
        {
            thread idlevideo(playContent, GESTURE_IDLE, ref(playbackFinished));
            idlevideo.detach();
        }

            state = STATE_IDLEVIDEO_UNDERWAY;
            break;

        case STATE_IDLEVIDEO_UNDERWAY:
            numTracked = trackingData->QueryNumberOfPeople();

            if(numTracked == 1)
            {
                cout << "Person detected during idle content playback!" << endl;
                system("killall vlc");
                state = STATE_IDLE;
                cyclesSpentIdle = 0;
            }
            else if(playbackFinished)
            {
                cout << "Idle playback completed. Starting new idle video at random" << endl;
                state = STATE_IDLEVIDEO_START;

                // double-check there are no stray vlc processes running
                system("killall vlc");
            }

            break;
        } */

    }

    system("killall vlc");


    pt_utils.stop_camera();
    actualModuleConfig.projection->release();
    cout << "-------- Stopping --------" << endl;
    return 0;
}

bool personIsInCenter(Intel::RealSense::PersonTracking::PersonTrackingData::PointCombined centerMass)
{
    if(     centerMass.world.point.x >= -0.3 &&
            centerMass.world.point.z >= 1.8 &&
            centerMass.world.point.x <= 0.3 &&
            centerMass.world.point.z <= 2.4
            )
    {
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

    cout << "num detected joints" << numDetectedJoints << endl;

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

    // check for each pose sequentially.
    // some way to make this cleaner, perhaps a helper function with just jointCoords struct as only parameter?

    int LeftX = jointCoords.Lshoulderx - jointCoords.Lhandx;
    int LeftY = jointCoords.Lshouldery - jointCoords.Lhandy;
    int LeftZ = jointCoords.Lshoulderz - jointCoords.Lhandz;
    int RightX = jointCoords.Rshoulderx - jointCoords.Rhandx;
    int RightY = jointCoords.Rshouldery - jointCoords.Rhandy;
    int RightZ = jointCoords.Rshoulderz - jointCoords.Rhandz;

    /*
     * STATIC GESTURES BEGIN
     */

    // USAIN BOLT POSE
  /*  switch(gesture_states.usain_gesture_state)
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

    // JUMPING GESTURE
/*
    switch(gesture_states.jumping_gesture_state)
    {
    case gesture_states.JUMPING_INIT:

        if((jointCoords.heady < jumpHeadY - 20) &&
                (jointCoords.heady != 0) && (jumpHeadY!=0) &&
                (jointCoords.headz >= 1650) &&
                (jointCoords.headz <= 2200) &&
                ((LeftY < 45) || (LeftY > 100)) &&
                ((RightY < 50) || RightY > 90))
        {
            cout << "jumpMaxX "<<jointCoords.headx << endl;
            cout << "HeadX "<<jumpHeadX << endl;
            cout << "jumpMaxY "<<jointCoords.heady << endl;
            cout << "HeadY "<<jumpHeadY << endl;
            cout << "jumpMaxZ "<<jointCoords.headz << endl;
            cout << "HeadZ "<<jumpHeadZ << endl <<endl;

            gesture_states.jumping_gesture_state = gesture_states.JUMPING_MAX;
        }
        else if(jointCoords.heady != 0){
            jumpHeadY = jointCoords.heady;
            jumpHeadX = jointCoords.headx;
            jumpHeadZ = jointCoords.headz;
//            cout << "preHeadY "<<jumpHeadY << endl;
//            cout << "preHeadX "<<jumpHeadX << endl<<endl;
            gesture_states.cyclesInState_jumping = 0;
        }

//                cout << "JUMPING_INIT..." << endl;
        break;

    case gesture_states.JUMPING_MAX:
        if((jointCoords.heady >= jumpHeadY - 15) && //+10 OG
                (jointCoords.heady != 0) && (jumpHeadY!=0) &&
                (jointCoords.headz >= 1650) &&
                (jointCoords.headz <= 2200) &&
                ((LeftY < 45) || (LeftY > 100)) &&
                ((RightY < 50) || RightY > 90))
        {
//            cout << "jumpBackX "<<jointCoords.headx << endl;
            cout << "jumpBackY "<<jointCoords.heady << endl;
//            cout << "jumpBackZ "<<jointCoords.headz << endl<<endl;
            gesture_states.cyclesInState_jumping = 0;
            gesture_states.jumping_gesture_state = gesture_states.JUMPING_MIN;
        }
        else if(gesture_states.cyclesInState_jumping >= JUMPING_TIMEOUT){
            gesture_states.cyclesInState_jumping = 0;
            jumpHeadX = jointCoords.headx;
            jumpHeadY = jointCoords.heady;
            jumpHeadZ = jointCoords.headz;
            gesture_states.jumping_gesture_state = gesture_states.JUMPING_INIT;
        }
        else
        {
            gesture_states.cyclesInState_jumping++;
        }
//        cout << "DIFF: " << jointCoords.heady - jumpHeadY << endl;
        cout << "CURR: " << jointCoords.heady << endl;
        cout << "JUMPING_MAX..." << "X: " << jumpHeadX << ", Y: " << jumpHeadY <<  ", Z: " << jumpHeadZ << endl <<endl;
        break;

    case gesture_states.JUMPING_MIN:

        cout << "JUMPING GESTURE DETECTED!" << endl<<endl<<endl;

//        gesture_states.cyclesInState_jumping = 0;
//        jumpHeadX = jointCoords.headx;
//        jumpHeadY = jointCoords.heady;
//        jumpHeadZ = jointCoords.headz;

        gesture_states.jumping_gesture_state = gesture_states.JUMPING_INIT;

        return GESTURE_JUMPING;

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
                     << "|H: " << jc.headx << ", " << jc.heady
            //         << ", " << jc.headz
            //         << "|S: " << jc.Spinex << ", " << jc.Spiney
         << endl;
}

void playContent(gestures_e gesture, bool &finished)
{
    // Play the specified video in fullscreen mode and close vlc when finished
    // (We should use this in our production code)
    //    system("cvlc -f --play-and-exit file:///home/zac/electricTree/videos/test.mov");

    // Play the specified video on a loop (useful for testing cancel gesture)
    //system("cvlc -R file:///home/zac/electricTree/videos/test.mov");
    int rand_idx;

    string vlc_cmd ("cvlc -f --play-and-exit --no-video-title-show ");
    string base_path ("file:///home/zac/electricTree/videos/");

    string title;
    string idx (to_string(rand_idx));
    string ext (".mp4");

    string full_cmd;

    switch (gesture)
    {
    case GESTURE_VICTORY:
        system("cvlc -f --play-and-exit --no-video-title-show file:///home/zac/electricTree/videos/victory.mov");
        break;
    case GESTURE_USAIN:
        system("cvlc -f --play-and-exit --no-video-title-show file:///home/zac/electricTree/videos/bolt.mov");
        break;
    case GESTURE_T:
        system("cvlc -f --play-and-exit --no-video-title-show file:///home/zac/electricTree/videos/bolt.mov");
        break;
            case GESTURE_POINTING_TRF:
                system("cvlc -f --play-and-exit --no-video-title-show file:///home/zac/electricTree/videos/toprightforward.mp4");
                break;
            case GESTURE_POINTING_RF:
                system("cvlc -f --play-and-exit --no-video-title-show file:///home/zac/electricTree/videos/rightforward.mp4");
                break;
            case GESTURE_POINTING_TLF:
                system("cvlc -f --play-and-exit --no-video-title-show file:///home/zac/electricTree/videos/topleftforward.mp4");
                break;
            case GESTURE_POINTING_LF:
                system("cvlc -f --play-and-exit --no-video-title-show file:///home/zac/electricTree/videos/leftforward.mp4");
            case GESTURE_POINTING_TR:
                system("cvlc -f --play-and-exit --no-video-title-show file:///home/zac/electricTree/videos/topright.mp4");
                //system("cvlc -f --play-and-exit --start-time=100 --no-video-title-show file:///home/zac/electricTree/videos/bolt.mov");
                break;
            case GESTURE_POINTING_R:
                system("cvlc -f --play-and-exit --no-video-title-show file:///home/zac/electricTree/videos/right.mp4");
                break;
            case GESTURE_POINTING_TL:
                system("cvlc -f --play-and-exit --no-video-title-show file:///home/zac/electricTree/videos/topleft.mp4");
                break;
            case GESTURE_POINTING_L:
                system("cvlc -f --play-and-exit --no-video-title-show file:///home/zac/electricTree/videos/left.mp4");
                break;
    case GESTURE_FLYING:
        system("cvlc -f --play-and-exit --no-video-title-show file:///home/zac/electricTree/videos/fly.mov");
        break;
    case GESTURE_IDLE:
        rand_idx = rand() % 3;
        idx.assign(to_string(rand_idx));
        cout << rand_idx << endl;
        ext.assign(".mp4");

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

        system(full_cmd.c_str());
        break;
    default:
        system("cvlc -f --play-and-exit --no-video-title-show file:///home/zac/electricTree/videos/test.mov");
        break;

    }

    // Set finished to true. This is a reference so its value will be seen in the main loop.
    finished = true;
    cout << "playback completed! " << endl;
}

void resetGestureStates(gesture_states_t &gesture_states)
{
    gesture_states.usain_gesture_state = gesture_states.USAIN_INIT;
    gesture_states.tpose_gesture_state = gesture_states.TPOSE_INIT;
    gesture_states.victory_gesture_state = gesture_states.VICTORY_INIT;
    gesture_states.powerpose_gesture_state = gesture_states.POWERPOSE_INIT;
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

}


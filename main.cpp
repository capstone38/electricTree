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

int numVideos[GESTURE_UNDEFINED];

struct analytics_t analytics_counts = analytics_init();

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
    int cyclesSpentDetected = 0;

    int pid_in_center = INVALID_PERSONID;
    int pid_in_center_prev;

    bool shouldCancel = false;



    // Start main loop
    while(!pt_utils.user_request_exit())
    {
        // Check for cancel request from system
        bool shouldQuit = false;
        mq.try_receive(&shouldQuit, sizeof(shouldQuit), recvd_size, priority);
        if(shouldQuit) {
            updateAnalytics(analytics_counts);
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
                }
            }
            else
            {
                // If we no longer see a person, back to idle state
                pid_in_center = INVALID_PERSONID;
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

                gestureDetected = detectGestures(personJoints, staticgesturelist, dynamicgesturelist);

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

                gestureDetected = detectGestures(personJoints, staticgesturelist, dynamicgesturelist);

                // Implement cancel gesture.
                if(gestureDetected == GESTURE_CANCEL)
                {
                    shouldCancel = true;
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
        {
            thread idlevideo(playContent, GESTURE_IDLE, shouldQuit);
            idlevideo.detach();
        }

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

gestures_e detectGestures(Intel::RealSense::PersonTracking::PersonTrackingData::PersonJoints *personJoints, vector<Gesture>& staticgesturelist, vector<DynamicGesture>& dynamicgesturelist)
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

    // use for debug:
    //printJointCoords(jointCoords);

    return detectedGesture;
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
        analytics_counts.victory_count++;
        break;
    case GESTURE_USAIN:
        title.assign("bolt");
        analytics_counts.usain_count++;
        break;
    case GESTURE_T:
        title.assign("tpose");
        analytics_counts.t_count++;
        break;
    case GESTURE_FLEXING:
        title.assign("flexing");
        analytics_counts.powerpose_count++;
        break;
    case GESTURE_STOP:
        title.assign("stop");
        analytics_counts.stop_count++;
        break;
    case GESTURE_POINTING_TRF:
        title.assign("toprightforward");
        analytics_counts.pointing_trf_count++;
        break;
    case GESTURE_POINTING_RF:
        title.assign("rightforward");
        analytics_counts.pointing_rf_count++;
        break;
    case GESTURE_POINTING_TLF:
        title.assign("topleftforward");
        analytics_counts.pointing_tlf_count++;
        break;
    case GESTURE_POINTING_LF:
        title.assign("leftforward");
        analytics_counts.pointing_lf_count++;
        break;
    case GESTURE_POINTING_TR:
        title.assign("topright");
        analytics_counts.pointing_tr_count++;
        break;
    case GESTURE_POINTING_R:
        title.assign("right");
        analytics_counts.pointing_r_count++;
        break;
    case GESTURE_POINTING_TL:
        title.assign("topleft");
        analytics_counts.pointing_tl_count++;
        break;
    case GESTURE_POINTING_L:
        title.assign("left");
        analytics_counts.pointing_l_count++;
        break;
    case GESTURE_FLYING:
        title.assign("fly");
        analytics_counts.flying_count++;
        break;
    case GESTURE_WAVING_L:
        title.assign("leftwave");
        analytics_counts.waving_l_count++;
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
    else if(strstr(buf, "mov") != NULL || strstr(buf, "mp4") != NULL)
    {
        return GESTURE_USAIN;
    }
    else
    {
        return GESTURE_UNDEFINED;
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

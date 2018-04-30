#ifndef MAIN_H
#define MAIN_H

#include <string>

#include "gesture.h"
#include "dynamicgesture.h"
#include "analytics.h"

#define CYCLES_PER_SECOND 30
#define SEC_TO_CYCLES(a) a*CYCLES_PER_SECOND

#define DEBUG 0
using namespace std;

enum state_e
{
    STATE_INIT=0,
    STATE_IDLE,
    STATE_READY,
    STATE_PLAYBACK_START,
    STATE_PLAYBACK_UNDERWAY,
    STATE_IDLEVIDEO_START,
    STATE_IDLEVIDEO_UNDERWAY,
    STATE_UPDATE,
    STATE_UNDEFINED
};

#define INVALID_PERSONID -1





gestures_e detectGestures(Intel::RealSense::PersonTracking::PersonTrackingData::PersonJoints *personJoints, vector<Gesture> &staticgesturelist, vector<DynamicGesture>& dynamicgesturelist);
void printJointCoords(jointCoords_t &jc);
void playContent(gestures_e gesture, bool quit);
bool personIsInCenter(Intel::RealSense::PersonTracking::PersonTrackingData::PointCombined centerMass);
gestures_e currentVideoType();
void updateNumVideos(int *numVideos);
int waitUntilContentStart(gestures_e gesture);
int detectNumVideo(string name);



vector<Gesture> defineStaticGestures(void);
vector<DynamicGesture> defineDynamicGestures(void);

#define VLC_CMD std::string("cvlc -f --play-and-exit --no-video-title-show")
#define VIDEOS_PATH std::string("file:///home/zac/electricTree/videos/")
#define DEFAULT_VIDEO "/home/capstone38/Desktop/electricTree/default.mp4"

#endif // MAIN_H

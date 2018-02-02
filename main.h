#ifndef MAIN_H
#define MAIN_H

enum state_e
{
    STATE_INIT=0,
    STATE_IDLE,
    STATE_READY,
    STATE_PLAYBACK,
    STATE_UPDATE,
    STATE_UNDEFINED
};

enum gestures_e
{
    GESTURE_WAVE=0,
    GESTURE_POINT,
    GESTURE_FLEX,
    GESTURE_USAIN,
    GESTURE_T,
    GESTURE_UNDEFINED
};

struct gestureJoints
{
    int Lhandx;
    int Lhandy;
    int Lshoulderx;
    int Lshouldery;

    int Rhandx;
    int Rhandy;
    int Rshoulderx;
    int Rshouldery;
};

int detectGestures(Intel::RealSense::PersonTracking::PersonTrackingData::PersonJoints *personJoints);

#endif // MAIN_H

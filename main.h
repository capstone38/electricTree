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
    GESTURE_0,
    GESTURE_SKY,
    GESTURE_POWERPOSE,
    GESTURE_UNDEFINED
};

struct jointCoords_t
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

gestures_e detectGestures(Intel::RealSense::PersonTracking::PersonTrackingData::PersonJoints *personJoints);
void printJointCoords(jointCoords_t jc);

#endif // MAIN_H

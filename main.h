#ifndef MAIN_H
#define MAIN_H

#include <string>

#define CYCLES_PER_SECOND 30
#define SEC_TO_CYCLES(a) a*CYCLES_PER_SECOND

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

enum gestures_e
{
    GESTURE_WAVE=0,
    GESTURE_POINT,
    GESTURE_USAIN,
    GESTURE_T,
    GESTURE_VICTORY,
    GESTURE_POWERPOSE,
    GESTURE_FLYING,
    GESTURE_WAVING_R,
    GESTURE_WAVING_L,
    GESTURE_JUMPING,
    GESTURE_POINTING_TR,
    GESTURE_POINTING_R,
    GESTURE_POINTING_BR,
    GESTURE_IDLE,
    GESTURE_UNDEFINED
};

struct gesture_states_t
{
    // static gestures begin
    enum usain_gesture_e
    {
        USAIN_INIT,
        USAIN_DETECTING,
        USAIN_LOST
    } usain_gesture_state;
    int cyclesInState_usain_detecting;
    int cyclesInState_usain_lost;

    enum tpose_gesture_e
    {
        TPOSE_INIT,
        TPOSE_DETECTING,
        TPOSE_LOST
    } tpose_gesture_state;
    int cyclesInState_tpose_detecting;
    int cyclesInState_tpose_lost;


    enum o_gesture_e
    {
        O_INIT,
        O_DETECTING
    } o_gesture_state;
    int cyclesInState_o;

    enum victory_gesture_e
    {
        VICTORY_INIT,
        VICTORY_DETECTING,
        VICTORY_LOST
    } victory_gesture_state;
    int cyclesInState_victory_detecting;
    int cyclesInState_victory_lost;

    enum powerpose_gesture_e
    {
        POWERPOSE_INIT,
        POWERPOSE_DETECTING,
        POWERPOSE_LOST,
    } powerpose_gesture_state;
    int cyclesInState_powerpose_detecting;
    int cyclesInState_powerpose_lost;

    enum pointing_tr_gesture_e
    {
        POINTING_TR_INIT,
        POINTING_TR_DETECTING,
        POINTING_TR_LOST,
    } pointing_tr_gesture_state;
    int cyclesInState_pointing_tr_detecting;
    int cyclesInState_pointing_tr_lost;

    enum pointing_r_gesture_e
    {
        POINTING_R_INIT,
        POINTING_R_DETECTING,
        POINTING_R_LOST,
    } pointing_r_gesture_state;
    int cyclesInState_pointing_r_detecting;
    int cyclesInState_pointing_r_lost;

    enum pointing_br_gesture_e
    {
        POINTING_BR_INIT,
        POINTING_BR_DETECTING,
        POINTING_BR_LOST,
    } pointing_br_gesture_state;
    int cyclesInState_pointing_br_detecting;
    int cyclesInState_pointing_br_lost;

    // dynamic gestures begin

    enum flying_gesture_e
    {
        FLYING_INIT,
        FLYING_MAX_1,
        FLYING_MIN_1,
        FLYING_MAX_2,
        FLYING_MIN_2
    } flying_gesture_state;
    int cyclesInState_flying;

    enum waving_r_gesture_e
    {
        WAVING_R_INIT,
        WAVING_R_MAX_1,
        WAVING_R_MIN_1,
        WAVING_R_MAX_2,
        WAVING_R_MIN_2
    } waving_r_gesture_state;
    int cyclesInState_waving_r;

    enum waving_l_gesture_e
    {
        WAVING_L_INIT,
        WAVING_L_MAX_1,
        WAVING_L_MIN_1,
        WAVING_L_MAX_2,
        WAVING_L_MIN_2
    } waving_l_gesture_state;
    int cyclesInState_waving_l;

    enum jumping_gesture_e
    {
        JUMPING_INIT,
        JUMPING_MAX,
        JUMPING_MIN,
    } jumping_gesture_state;
    int cyclesInState_jumping;
};

// gesture detection timeouts (units are frames)
#define STATIC_POSE_DETECTING_TIMEOUT 10
#define STATIC_POSE_LOST_TIMEOUT 5
#define FLYING_TIMEOUT 20
#define WAVING_TIMEOUT 20
#define JUMPING_TIMEOUT 30


#define GESTURE_CANCEL GESTURE_WAVING_R // update this later if we want a different cancel gesture!!

int jumpHeadX;
int jumpHeadY;
int jumpHeadZ;

struct jointCoords_t
{
    int Lhandx;
    int Lhandy;
    int Lhandz;
    int Lshoulderx;
    int Lshouldery;
    int Lshoulderz;

    int Rhandx;
    int Rhandy;
    int Rhandz;
    int Rshoulderx;
    int Rshouldery;
    int Rshoulderz;

    int Spinex;
    int Spiney;
    int headx;
    int heady;
    int headz;
};

gestures_e detectGestures(Intel::RealSense::PersonTracking::PersonTrackingData::PersonJoints *personJoints, gesture_states_t &gesture_states);
void printJointCoords(jointCoords_t &jc);
void playContent(gestures_e gesture, bool &finished);
void resetGestureStates(gesture_states_t &gesture_states);

#define VLC_CMD std::string("cvlc -f --play-and-exit --no-video-title-show")
#define VIDEOS_PATH std::string("file:///home/zac/electricTree/videos/")

#endif // MAIN_H

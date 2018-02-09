#ifndef MAIN_H
#define MAIN_H



enum state_e
{
    STATE_INIT=0,
    STATE_IDLE,
    STATE_READY,
    STATE_PLAYBACK_START,
    STATE_PLAYBACK_UNDERWAY,
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
    GESTURE_O,
    GESTURE_VICTORY,
    GESTURE_POWERPOSE,
    GESTURE_FLYING,
    GESTURE_UNDEFINED
};

struct gesture_states_t
{
    // static gestures begin
    enum usain_gesture_e
    {
        USAIN_INIT,
        USAIN_DETECTING
    } usain_gesture_state;
    int cyclesInState_usain;

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
        VICTORY_DETECTING
    } victory_gesture_state;
    int cyclesInState_victory;

    enum powerpose_gesture_e
    {
        POWERPOSE_INIT,
        POWERPOSE_DETECTING
    } powerpose_gesture_state;
    int cyclesInState_powerpose;

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
};

// gesture detection timeouts (units are frames)
#define STATIC_POSE_DETECTING_TIMEOUT 20
#define STATIC_POSE_LOST_TIMEOUT 4
#define FLYING_TIMEOUT 15


#define GESTURE_CANCEL GESTURE_T // update this later if we want a different cancel gesture!!

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

    int Spinex;
    int Spiney;
    int headx;
    int heady;
};

gestures_e detectGestures(Intel::RealSense::PersonTracking::PersonTrackingData::PersonJoints *personJoints, gesture_states_t &gesture_states);
void printJointCoords(jointCoords_t &jc);
void playContent(gestures_e gesture, bool &finished);
void resetGestureStates(gesture_states_t &gesture_states);

#endif // MAIN_H

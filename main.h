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
    GESTURE_USAIN=0,
    GESTURE_T,
    GESTURE_VICTORY,
    GESTURE_POWERPOSE,
    GESTURE_STOP,
    GESTURE_FLYING,
    GESTURE_WAVING_R,
    GESTURE_WAVING_L,
    GESTURE_JUMPING,
    GESTURE_POINTING_TRF,
    GESTURE_POINTING_RF,
    GESTURE_POINTING_TLF,
    GESTURE_POINTING_LF,
    GESTURE_POINTING_TR,
    GESTURE_POINTING_R,
    GESTURE_POINTING_TL,
    GESTURE_POINTING_L,
    GESTURE_RUNNING,
    GESTURE_IDLE,
    GESTURE_READY,
    GESTURE_UNDEFINED
};

#define INVALID_PERSONID -1

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
    int powertest = 0;

    enum stop_gesture_e
    {
        STOP_INIT,
        STOP_DETECTING,
        STOP_LOST
    } stop_gesture_state;
    int cyclesInState_stop_detecting;
    int cyclesInState_stop_lost;

    enum pointing_trf_gesture_e
    {
        POINTING_TRF_INIT,
        POINTING_TRF_DETECTING,
        POINTING_TRF_LOST,
    } pointing_trf_gesture_state;
    int cyclesInState_pointing_trf_detecting;
    int cyclesInState_pointing_trf_lost;

    enum pointing_rf_gesture_e
    {
        POINTING_RF_INIT,
        POINTING_RF_DETECTING,
        POINTING_RF_LOST,
    } pointing_rf_gesture_state;
    int cyclesInState_pointing_rf_detecting;
    int cyclesInState_pointing_rf_lost;

    enum pointing_tlf_gesture_e
    {
        POINTING_TLF_INIT,
        POINTING_TLF_DETECTING,
        POINTING_TLF_LOST,
    } pointing_tlf_gesture_state;
    int cyclesInState_pointing_tlf_detecting;
    int cyclesInState_pointing_tlf_lost;

    enum pointing_lf_gesture_e
    {
        POINTING_LF_INIT,
        POINTING_LF_DETECTING,
        POINTING_LF_LOST,
    } pointing_lf_gesture_state;
    int cyclesInState_pointing_lf_detecting;
    int cyclesInState_pointing_lf_lost;

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

    enum pointing_tl_gesture_e
    {
        POINTING_TL_INIT,
        POINTING_TL_DETECTING,
        POINTING_TL_LOST,
    } pointing_tl_gesture_state;
    int cyclesInState_pointing_tl_detecting;
    int cyclesInState_pointing_tl_lost;

    enum pointing_l_gesture_e
    {
        POINTING_L_INIT,
        POINTING_L_DETECTING,
        POINTING_L_LOST,
    } pointing_l_gesture_state;
    int cyclesInState_pointing_l_detecting;
    int cyclesInState_pointing_l_lost;

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

    enum running_gesture_e
    {
        RUNNING_INIT,
        RUNNING_MAX_1,  // right hand max
        RUNNING_MIN_1,
        RUNNING_MAX_2,
        RUNNING_MIN_2
    } running_gesture_state;
    int cyclesInState_running;
};

// gesture detection timeouts (units are frames)
#define STATIC_POSE_DETECTING_TIMEOUT 10
#define STATIC_POSE_LOST_TIMEOUT 10
#define FLYING_TIMEOUT 20
#define WAVING_TIMEOUT 20
#define JUMPING_TIMEOUT 20
#define RUNNING_TIMEOUT 25

#define GESTURE_CANCEL GESTURE_WAVING_R // update this later if we want a different cancel gesture!!

// variables used for jumping detection
//int jumpHeadPreX;
//int jumpHeadPreY;
//int jumpHeadPreZ;
//int jumpHeadCurrX;
//int jumpHeadCurrY;
//int jumpHeadCurrZ;

// move declaration to struct for cleaniness
int powerpose_count;
int t_count;
int victory_count;
int usain_count;
int stop_count;
int flying_count;
int waving_r_count;
int waving_l_count;
int pointing_trf_count;
int pointing_rf_count;
int pointing_tlf_count;
int pointing_lf_count;
int pointing_tr_count;
int pointing_r_count;
int pointing_tl_count;
int pointing_l_count;

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
    int Spinez;
    int headx;
    int heady;
    int headz;
};

gestures_e detectGestures(Intel::RealSense::PersonTracking::PersonTrackingData::PersonJoints *personJoints, gesture_states_t &gesture_states);
void printJointCoords(jointCoords_t &jc);
void playContent(gestures_e gesture, bool quit);
void resetGestureStates(gesture_states_t &gesture_states);
bool personIsInCenter(Intel::RealSense::PersonTracking::PersonTrackingData::PointCombined centerMass);
gestures_e currentVideoType();
void updateNumVideos(int *numVideos);
int waitUntilContentStart(gestures_e gesture);
int detectNumVideo(string name);

void updateAnalytics(bool update, gesture_states_t &gesture_states);
string format;  // random strings for formating
string total;   // "TOTAL:","TODAY_TOTAL_COUNT:", "OVERALL_TOTAL_COUNT:"
int totalCount;  // "TOTAL: X","TODAY_TOTAL_COUNT: X", "OVERALL_TOTAL_COUNT: X"
string gesture; // "WAVING_R:", "USAIN:", ...
int gestureCount;   // "WAVING_R: X", "USAIN: X", ...

#define VLC_CMD std::string("cvlc -f --play-and-exit --no-video-title-show")
#define VIDEOS_PATH std::string("file:///home/zac/electricTree/videos/")
#define DEFAULT_VIDEO "/home/capstone38/Desktop/electricTree/default.mp4"

#endif // MAIN_H

#ifndef GESTURE_H
#define GESTURE_H


// gesture detection timeouts (units are frames)
#define STATIC_POSE_DETECTING_TIMEOUT 7
#define STATIC_POSE_LOST_TIMEOUT 10
#define FLYING_TIMEOUT 20
#define WAVING_TIMEOUT 20
#define JUMPING_TIMEOUT 20
#define RUNNING_TIMEOUT 25

#define MAXCOORD 10000

#define GESTURE_CANCEL GESTURE_WAVING_R // update this later if we want a different cancel gesture!!


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

enum gestures_e
{
    GESTURE_USAIN=0,
    GESTURE_T,
    GESTURE_VICTORY,
    GESTURE_FLEXING,
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

enum static_gesture_states_e
{
    STATIC_GESTURE_STATE_INIT,
    STATIC_GESTURE_STATE_DETECTING,
    STATIC_GESTURE_STATE_LOST
};

struct static_gesture_states_t
{
    static_gesture_states_e static_gesture_state;
    int cyclesInState_static_gesture_detecting;
    int cyclesInState_static_gesture_lost;
};

struct dynamic_gesture_states_t
{
    enum dynamic_gesture_states_e
    {
        DYNAMIC_GESTURE_STATE_INIT,
        DYNAMIC_GESTURE_STATE_MAX_1,
        DYNAMIC_GESTURE_STATE_MIN_1,
        DYNAMIC_GESTURE_STATE_MAX_2,
    } dynamic_gesture_state;
    int cyclesInState_dynamic_gesture_detecting;
    int cyclesInState_dynamic_gesture_lost;
};

class Gesture
{
private:
    struct static_gesture_states_t state;
    int ls_lh_x_min;
    int ls_lh_x_max;
    int ls_lh_y_min;
    int ls_lh_y_max;
    int ls_lh_z_min;
    int ls_lh_z_max;
    int rs_rh_x_min;
    int rs_rh_x_max;
    int rs_rh_y_min;
    int rs_rh_y_max;
    int rs_rh_z_min;
    int rs_rh_z_max;
    struct dynamic_gesture_states_t state_d;


public:
    Gesture(gestures_e id,
            int ls_lh_x_min,
            int ls_lh_x_max,
            int ls_lh_y_min,
            int ls_lh_y_max,
            int rs_rh_x_min,
            int rs_rh_x_max,
            int rs_rh_y_min,
            int rs_rh_y_max) :
                id(id),
                ls_lh_x_min(ls_lh_x_min),
                ls_lh_x_max(ls_lh_x_max),
                ls_lh_y_min(ls_lh_y_min),
                ls_lh_y_max(ls_lh_y_max),
                rs_rh_x_min(rs_rh_x_min),
                rs_rh_x_max(rs_rh_x_max),
                rs_rh_y_min(rs_rh_y_min),
                rs_rh_y_max(rs_rh_y_max)
    {
        rs_rh_z_min = -MAXCOORD;
        rs_rh_z_max = MAXCOORD;
        ls_lh_z_min = -MAXCOORD;
        ls_lh_z_max = MAXCOORD;
        resetGestureState();
    }

    Gesture(gestures_e id,
            int ls_lh_x_min,
            int ls_lh_x_max,
            int ls_lh_y_min,
            int ls_lh_y_max,
            int ls_lh_z_min,
            int ls_lh_z_max,
            int rs_rh_x_min,
            int rs_rh_x_max,
            int rs_rh_y_min,
            int rs_rh_y_max,
            int rs_rh_z_min,
            int rs_rh_z_max) :
                id(id),
                ls_lh_x_min(ls_lh_x_min),
                ls_lh_x_max(ls_lh_x_max),
                ls_lh_y_min(ls_lh_y_min),
                ls_lh_y_max(ls_lh_y_max),
                ls_lh_z_min(ls_lh_z_min),
                ls_lh_z_max(ls_lh_z_max),
                rs_rh_x_min(rs_rh_x_min),
                rs_rh_x_max(rs_rh_x_max),
                rs_rh_y_min(rs_rh_y_min),
                rs_rh_y_max(rs_rh_y_max),
                rs_rh_z_min(rs_rh_z_min),
                rs_rh_z_max(rs_rh_z_max)
    {
        resetGestureState();
    }

    void resetGestureState(void);
    bool detect(jointCoords_t jointCoords);
    bool detectDynamic(jointCoords_t jointCoords);
    bool isWithinThreshold(jointCoords_t jointCoords);

    static_gesture_states_e getState(void);

    gestures_e id;

};

#endif // GESTURE_H

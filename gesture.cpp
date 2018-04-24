#include "gesture.h"
#include <iostream>


/*Gesture::Gesture::Gesture::Gesture(gestures_e id,
                 int ls_lh_x_min,
                 int ls_lh_x_max,
                 int ls_lh_y_min,
                 int ls_lh_y_max,
                 int rs_rh_x_min,
                 int rs_rh_x_max,
                 int rs_rh_y_min,
                 int rs_rh_y_max)
*/
using namespace std;

void Gesture::resetGestureState(void)
{
    state.static_gesture_state = state.STATIC_GESTURE_STATE_INIT;
    state.cyclesInState_static_gesture_detecting = 0;
    state.cyclesInState_static_gesture_lost = 0;
}

bool Gesture::detect(jointCoords_t jointCoords)
{
    cout << "Gesture " << id << " state " << state.static_gesture_state << endl;

    switch(state.static_gesture_state)
    {
    case state.STATIC_GESTURE_STATE_INIT:
        if( isWithinThreshold(jointCoords) )
            {
                state.static_gesture_state = state.STATIC_GESTURE_STATE_DETECTING;
                state.cyclesInState_static_gesture_detecting = 0;
            }
        break;
    case state.STATIC_GESTURE_STATE_DETECTING:
        if( isWithinThreshold(jointCoords) )
        {
            state.cyclesInState_static_gesture_detecting++;

            if(state.cyclesInState_static_gesture_detecting >= STATIC_POSE_DETECTING_TIMEOUT)
            {
                resetGestureState();
                return true;
            }
        }
        else
        {
            state.static_gesture_state = state.STATIC_GESTURE_STATE_LOST;
            state.cyclesInState_static_gesture_lost = 0;
        }
        break;

    case state.STATIC_GESTURE_STATE_LOST:
        if( isWithinThreshold(jointCoords) )
        {
            state.static_gesture_state = state.STATIC_GESTURE_STATE_DETECTING;
            state.cyclesInState_static_gesture_lost = 0;
        }
        else if(state.cyclesInState_static_gesture_lost >= STATIC_POSE_LOST_TIMEOUT)
        {
            resetGestureState();
        }
        else
        {
            state.cyclesInState_static_gesture_lost++;
        }
        break;
    }

    return false;
}

bool Gesture::isWithinThreshold(jointCoords_t jointCoords)
{
    int LeftX = jointCoords.Lshoulderx - jointCoords.Lhandx;
    int LeftY = jointCoords.Lshouldery - jointCoords.Lhandy;
    int LeftZ = jointCoords.Lshoulderz - jointCoords.Lhandz;
    int RightX = jointCoords.Rshoulderx - jointCoords.Rhandx;
    int RightY = jointCoords.Rshouldery - jointCoords.Rhandy;
    int RightZ = jointCoords.Rshoulderz - jointCoords.Rhandz;

    if(id == GESTURE_POINTING_TRF ||
       id == GESTURE_POINTING_RF  ||
       id == GESTURE_POINTING_TR   ||
       id == GESTURE_POINTING_R)
    {
        return ( (RightX <= rs_rh_x_max) &&
                (RightX >= rs_rh_x_min ) &&
                (RightY <= rs_rh_y_max) &&
                (RightY >= rs_rh_y_min) &&
                (RightZ <= rs_rh_z_max) &&
                (RightZ >= rs_rh_z_min) &&
                ((LeftY < ls_lh_y_max) || (LeftY > ls_lh_y_min)));
    }
    else if(id == GESTURE_POINTING_TL ||
            id == GESTURE_POINTING_L ||
            id == GESTURE_POINTING_TLF ||
            id == GESTURE_POINTING_LF)
    {
        return ( (LeftX <= ls_lh_x_max) &&
                 (LeftX >= ls_lh_x_min ) &&
                 (LeftY <= ls_lh_y_max) &&
                 (LeftY >= ls_lh_y_min) &&
                 (LeftZ <= ls_lh_z_max) &&
                 (LeftZ >= ls_lh_z_min) &&
                 ((RightY < rs_rh_y_max) || (RightY > rs_rh_y_min)));
    }
    else if(id == GESTURE_STOP)
    {
        return ((LeftX >= ls_lh_x_min) &&
                (LeftX <= ls_lh_x_max) &&
                (LeftY >= ls_lh_y_min) &&
                (LeftY <= ls_lh_y_max) &&
                (jointCoords.Lhandz <= ls_lh_z_max));
    }
    else
    {
        return (     (LeftX <= ls_lh_x_max) &&
                     (LeftX >= ls_lh_x_min ) &&
                     (LeftY <= ls_lh_y_max) &&
                     (LeftY >= ls_lh_y_min) &&
                     (LeftZ <= ls_lh_z_max) &&
                     (LeftZ >= ls_lh_z_min) &&
                     (RightX <= rs_rh_x_max) &&
                     (RightX >= rs_rh_x_min ) &&
                     (RightY <= rs_rh_y_max) &&
                     (RightY >= rs_rh_y_min) &&
                     (RightZ <= rs_rh_z_max) &&
                     (RightZ >= rs_rh_z_min)
                     );
    }
}

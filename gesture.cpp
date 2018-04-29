#include "gesture.h"
#include <iostream>


using namespace std;

void Gesture::resetGestureState(void)
{
    state.static_gesture_state = STATIC_GESTURE_STATE_INIT;
    state.cyclesInState_static_gesture_detecting = 0;
    state.cyclesInState_static_gesture_lost = 0;
}

bool Gesture::detect(jointCoords_t jointCoords)
{
    cout << "Gesture " << id << " state " << state.static_gesture_state << endl;

    switch(state.static_gesture_state)
    {
    case STATIC_GESTURE_STATE_INIT:
        if( isWithinThreshold(jointCoords) )
            {
                state.static_gesture_state = STATIC_GESTURE_STATE_DETECTING;
                state.cyclesInState_static_gesture_detecting = 0;
            }
        break;
    case STATIC_GESTURE_STATE_DETECTING:
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
            state.static_gesture_state = STATIC_GESTURE_STATE_LOST;
            state.cyclesInState_static_gesture_lost = 0;
        }
        break;

    case STATIC_GESTURE_STATE_LOST:
        if( isWithinThreshold(jointCoords) )
        {
            state.static_gesture_state = STATIC_GESTURE_STATE_DETECTING;
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

bool Gesture::detectDynamic(jointCoords_t jointCoords) {
/*
    cout << "Gesture " << id << " state " << state_d.dynamic_gesture_state << endl;

    switch(state_d.dynamic_gesture_state)
    {
    case state_d.DYNAMIC_GESTURE_STATE_INIT:
        if( isWithinThreshold(jointCoords) )
        {
            state_d.dynamic_gesture_state = state_d.DYNAMIC_GESTURE_STATE_MAX_1;
            state_d.cyclesInState_dynamic_gesture_detecting = 0;
        }
        break;

    case state_d.DYNAMIC_GESTURE_STATE_MAX_1:
        if( isWithinThreshold(jointCoords) )
        {
            state_d.dynamic_gesture_state = state_d.DYNAMIC_GESTURE_STATE_MIN_1;
            state_d.cyclesInState_dynamic_gesture_detecting = 0;
        }
        else if(state_d.cyclesInState_dynamic_gesture_detecting >= WAVING_TIMEOUT){
            state_d.dynamic_gesture_state = state_d.DYNAMIC_GESTURE_STATE_INIT;
            state_d.cyclesInState_dynamic_gesture_detecting = 0;
        }
        else
        {
            state_d.cyclesInState_dynamic_gesture_detecting++;
        }
        break;

    case state_d.DYNAMIC_GESTURE_STATE_MIN_1:
        if( isWithinThreshold(jointCoords) )
        {
            state_d.dynamic_gesture_state = state_d.DYNAMIC_GESTURE_STATE_MAX_2;
            state_d.cyclesInState_dynamic_gesture_detecting = 0;
        }
        else if(state_d.cyclesInState_dynamic_gesture_detecting >= WAVING_TIMEOUT){
            state_d.dynamic_gesture_state = state_d.DYNAMIC_GESTURE_STATE_INIT;
            state_d.cyclesInState_dynamic_gesture_detecting = 0;
        }
        else
        {
            state_d.cyclesInState_dynamic_gesture_detecting++;
        }
        break;

    case state_d.DYNAMIC_GESTURE_STATE_MAX_2:
        if( isWithinThreshold(jointCoords) )
        {
            resetGestureState();
            return true;
        }
        else if(state_d.cyclesInState_dynamic_gesture_detecting >= WAVING_TIMEOUT){
            state_d.dynamic_gesture_state = state_d.DYNAMIC_GESTURE_STATE_INIT;
            state_d.cyclesInState_dynamic_gesture_detecting = 0;
        }
        else
        {
            state_d.cyclesInState_dynamic_gesture_detecting++;
        }
        break;
    }
*/
    return false;
}

static_gesture_states_e Gesture::getState(void)
{
    return state.static_gesture_state;
}

#include "dynamicgesture.h"

DynamicGesture::DynamicGesture()
{

}

void DynamicGesture::addIntermediateGesture(Gesture g)
{
    intermediate_gestures.push_back(g);
}

bool DynamicGesture:detect(void)
{
    /*
    switch(gesture_states.waving_r_gesture_state)
    {
    case gesture_states.WAVING_R_INIT:
        if((LeftX >= 40) &&
                (LeftX <= 90) &&
                (LeftY >= -20) && // 10
                (LeftY <= 40) &&
                ((RightX > 0) || (RightX < -50)) &&
                ((RightY > 50) || (RightY < 20)))
        {
            gesture_states.cyclesInState_waving_r = 0;
            gesture_states.waving_r_gesture_state = gesture_states.WAVING_R_MAX_1;
        }
        break;

    case gesture_states.WAVING_R_MAX_1:
        if((LeftX >= -20) &&
                (LeftX <= 40) &&
                (LeftY >= -20) && // 10
                (LeftY <= 40) &&
                ((RightX > 0) || (RightX < -50)) &&
                ((RightY > 50) || (RightY < 20)))
        {
            gesture_states.cyclesInState_waving_r = 0;
            gesture_states.waving_r_gesture_state = gesture_states.WAVING_R_MIN_1;
        }
        else if(gesture_states.cyclesInState_waving_r >= WAVING_TIMEOUT){
            gesture_states.cyclesInState_waving_r = 0;
            gesture_states.waving_r_gesture_state = gesture_states.WAVING_R_INIT;
        }
        else
        {
            gesture_states.cyclesInState_waving_r++;
        }
        break;

    case gesture_states.WAVING_R_MIN_1:
        if((LeftX >= 40) &&
                (LeftX <= 90) &&
                (LeftY >= -20) && // 10
                (LeftY <= 40) &&
                ((RightX > 0) || (RightX < -50)) &&
                ((RightY > 50) || (RightY < 20)))
        {
            gesture_states.cyclesInState_waving_r = 0;
            gesture_states.waving_r_gesture_state = gesture_states.WAVING_R_MAX_2;
        }
        else if(gesture_states.cyclesInState_waving_r >= WAVING_TIMEOUT){
            gesture_states.cyclesInState_waving_r = 0;
            gesture_states.waving_r_gesture_state = gesture_states.WAVING_R_INIT;
        }
        else
        {
            gesture_states.cyclesInState_waving_r++;
        }
        break;

    case gesture_states.WAVING_R_MAX_2:
        if((LeftX >= -20) &&
                (LeftX <= 40) &&
                (LeftY >= -20) && // 10
                (LeftY <= 40) &&
                ((RightX > 0) || (RightX < -50)) &&
                ((RightY > 50) || (RightY < 20)))
        {
            gesture_states.waving_r_gesture_state = gesture_states.WAVING_R_INIT;
            return GESTURE_WAVING_R;
        }
        else if(gesture_states.cyclesInState_waving_r >= WAVING_TIMEOUT) {
            gesture_states.cyclesInState_waving_r = 0;
            gesture_states.waving_r_gesture_state = gesture_states.WAVING_R_INIT;
        }
        else
        {
            gesture_states.cyclesInState_waving_r++;
        }
        break;

    default:
        break;
    }
*/

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
    return false;
}

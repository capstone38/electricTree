#include "dynamicgesture.h"

#include <iostream>

void DynamicGesture::addIntermediateGesture(Gesture g)
{
    intermediate_gestures.push_back(g);
}

bool DynamicGesture::detect(jointCoords_t jointCoords)
{
    cout << "Dynamic Gesture " << id << " state " << state << endl;

    if(intermediate_gestures.size() == 0)
    {
        return false;
    }

    intermediate_gestures[state].detect(jointCoords);

    static_gesture_states_e int_state = intermediate_gestures[state].getState();

    if(int_state == STATIC_GESTURE_STATE_DETECTING)
    {
        intermediate_gestures[state].resetGestureState();
        cyclesInState = 0;

        if(state == intermediate_gestures.size()-1)
        {
            resetStates();
            return true;
        }
        else
        {
            state++;
        }
    }
    else if(cyclesInState >= DYNAMIC_POSE_TIMEOUT)
    {
        resetStates();
    }
    else
    {
        cyclesInState++;
    }

    return false;
}

void DynamicGesture::resetStates(void)
{
    int n = intermediate_gestures.size();

    for(int i = 0; i < n; i++)
    {
        intermediate_gestures[i].resetGestureState();
    }

    state = 0;
    cyclesInState = 0;
}

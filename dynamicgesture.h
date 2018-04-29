#ifndef DYNAMICGESTURE_H
#define DYNAMICGESTURE_H

#include "gesture.h"
#include <vector>

#define DYNAMIC_POSE_TIMEOUT 20

using namespace std;

class DynamicGesture
{
private:
    vector<Gesture> intermediate_gestures;
    unsigned int state;
    unsigned int cyclesInState;

public:
    DynamicGesture(gestures_e id) : id(id)
    {
        resetStates();
    }

    void addIntermediateGesture(Gesture g);

    bool detect(jointCoords_t jointCoords);

    void resetStates(void);

    gestures_e id;
};

#endif // DYNAMICGESTURE_H

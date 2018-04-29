#ifndef DYNAMICGESTURE_H
#define DYNAMICGESTURE_H

#include "gesture.h"


class DynamicGesture
{
private:
    vector<Gesture> intermediate_gestures;

public:
    DynamicGesture();

    void addIntermediateGesture(Gesture g);
};

#endif // DYNAMICGESTURE_H

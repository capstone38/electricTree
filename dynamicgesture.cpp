#include "dynamicgesture.h"

DynamicGesture::DynamicGesture()
{

}

void DynamicGesture::addIntermediateGesture(Gesture g)
{
    intermediate_gestures.push_back(g);
}


// Realsense libraries
#include <librealsense/rs.hpp>
#include "rs_sdk.h"
#include "version.h"
#include "pt_utils.hpp"

#include <vector>
#include "gesture.h"
#include "dynamicgesture.h"
#include "main.h"

// This file contains the thresholds for detection of each gesture.

vector<Gesture> defineStaticGestures(void)
{
    vector<Gesture> out;

    Gesture usain(GESTURE_USAIN,
                 (int)0,           // LeftX_min
                 (int)80,          // LeftX_max
                 (int)-60,         // LeftY_min
                 (int)30,          // LeftY_max
                 (int)-100,        // RightX_min
                 (int)-50,         // RightX_max
                 (int)20,          // RightY_min
                 (int)70);         // RightY_max
    out.push_back(usain);

    Gesture tpose(GESTURE_T,
                 (int)45,          // LeftX_min
                 (int)MAXCOORD,    // LeftX_max
                 (int)-20,         // LeftY_min
                 (int)20,          // LeftY_max
                 (int)-MAXCOORD,   // RightX_min
                 (int)-45,         // RightX_max
                 (int)-20,         // RightY_min
                 (int)20);         // RightY_max
    out.push_back(tpose);

    Gesture stop(GESTURE_STOP,
                 (int)-20,         // LeftX_min
                 (int)40,          // LeftX_max
                 (int)0,           // LeftY_min
                 (int)60,          // LeftY_max
                 (int)0,           // LeftZ_min
                 (int)1550,        // LeftZ_max
                 (int)-MAXCOORD,   // RightX_min
                 (int)MAXCOORD,    // RightX_max
                 (int)-MAXCOORD,   // RightY_min
                 (int)MAXCOORD,    // RightY_max
                 (int)0,           // RightZ_min
                 (int)MAXCOORD);   // RightZ_max
    out.push_back(stop);

    Gesture point_trf(GESTURE_POINTING_TRF,
                 (int)50,          // LeftX_min
                 (int)90,          // LeftX_max
                 (int)50,          // LeftY_min
                 (int)90,          // LeftY_max
                 (int)300,         // LeftZ_min
                 (int)500,         // LeftZ_max
                 (int)-MAXCOORD,   // RightX_min
                 (int)MAXCOORD,    // RightX_max
                 (int)110,         // RightY_min
                 (int)-40,         // RightY_max
                 (int)-MAXCOORD,   // RightZ_min
                 (int)MAXCOORD);   // RightZ_max
    out.push_back(point_trf);

    Gesture point_rf(GESTURE_POINTING_RF,
                 (int)60,          // LeftX_min
                 (int)120,         // LeftX_max
                 (int)-30,         // LeftY_min
                 (int)30,          // LeftY_max
                 (int)230,         // LeftZ_min
                 (int)670,         // LeftZ_max
                 (int)-MAXCOORD,   // RightX_min
                 (int)MAXCOORD,    // RightX_max
                 (int)110,         // RightY_min
                 (int)-40,         // RightY_max
                 (int)-MAXCOORD,   // RightZ_min
                 (int)MAXCOORD);   // RightZ_max
    out.push_back(point_rf);

    Gesture point_tlf(GESTURE_POINTING_TLF,
                 (int)-MAXCOORD,   // LeftX_min
                 (int)MAXCOORD,    // LeftX_max
                 (int)90,          // LeftY_min
                 (int)-50,         // LeftY_max
                 (int)-MAXCOORD,   // LeftZ_min
                 (int)MAXCOORD,    // LeftZ_max
                 (int)-100,        // RightX_min
                 (int)-50,         // RightX_max
                 (int)40,          // RightY_min
                 (int)80,          // RightY_max
                 (int)180,         // RightZ_min
                 (int)420);        // RightZ_max
    out.push_back(point_tlf);

    Gesture point_lf(GESTURE_POINTING_LF,
                 (int)-MAXCOORD,   // LeftX_min
                 (int)MAXCOORD,    // LeftX_max
                 (int)90,          // LeftY_min
                 (int)-50,         // LeftY_max
                 (int)-MAXCOORD,   // LeftZ_min
                 (int)MAXCOORD,    // LeftZ_max
                 (int)-120,        // RightX_min
                 (int)-60,         // RightX_max
                 (int)-20,         // RightY_min
                 (int)30,          // RightY_max
                 (int)70,          // RightZ_min
                 (int)550);        // RightZ_max
    out.push_back(point_lf);


    Gesture point_tr(GESTURE_POINTING_TR,
                 (int)50,          // LeftX_min
                 (int)100,         // LeftX_max
                 (int)30,          // LeftY_min
                 (int)80,          // LeftY_max
                 (int)-140,        // LeftZ_min
                 (int)20,          // LeftZ_max
                 (int)-MAXCOORD,   // RightX_min
                 (int)MAXCOORD,    // RightX_max
                 (int)110,         // RightY_min
                 (int)-40,         // RightY_max
                 (int)-MAXCOORD,   // RightZ_min
                 (int)MAXCOORD);   // RightZ_max
    out.push_back(point_tr);

    Gesture point_r(GESTURE_POINTING_R,
                 (int)80,          // LeftX_min
                 (int)110,         // LeftX_max
                 (int)-20,         // LeftY_min
                 (int)20,          // LeftY_max
                 (int)-200,        // LeftZ_min
                 (int)20,          // LeftZ_max
                 (int)-MAXCOORD,   // RightX_min
                 (int)MAXCOORD,    // RightX_max
                 (int)110,         // RightY_min
                 (int)-40,         // RightY_max
                 (int)-MAXCOORD,   // RightZ_min
                 (int)MAXCOORD);   // RightZ_max
    out.push_back(point_r);

    Gesture point_tl(GESTURE_POINTING_TL,
                 (int)-MAXCOORD,   // LeftX_min
                 (int)MAXCOORD,    // LeftX_max
                 (int)90,          // LeftY_min
                 (int)-50,         // LeftY_max
                 (int)-MAXCOORD,   // LeftZ_min
                 (int)MAXCOORD,    // LeftZ_max
                 (int)-80,         // RightX_min
                 (int)-50,         // RightX_max
                 (int)40,          // RightY_min
                 (int)80,          // RightY_max
                 (int)-120,        // RightZ_min
                 (int)110);        // RightZ_max
    out.push_back(point_tl);

    Gesture point_l(GESTURE_POINTING_L,
                 (int)-MAXCOORD,   // LeftX_min
                 (int)MAXCOORD,    // LeftX_max
                 (int)90,          // LeftY_min
                 (int)-50,         // LeftY_max
                 (int)-MAXCOORD,   // LeftZ_min
                 (int)MAXCOORD,    // LeftZ_max
                 (int)-100,        // RightX_min
                 (int)-70,         // RightX_max
                 (int)-10,         // RightY_min
                 (int)20,          // RightY_max
                 (int)-90,         // RightZ_min
                 (int)110);        // RightZ_max
    out.push_back(point_l);

    Gesture victory(GESTURE_VICTORY,
                 (int)45,          // LeftX_min
                 (int)100,         // LeftX_max
                 (int)45,          // LeftY_min
                 (int)90,          // LeftY_max
                 (int)-80,         // RightX_min
                 (int)-30,         // RightX_max
                 (int)50,          // RightY_min
                 (int)100);        // RightY_max
    out.push_back(victory);

    Gesture flexing(GESTURE_FLEXING,
                 (int)0,           // LeftX_min
                 (int)70,          // LeftX_max
                 (int)0,           // LeftY_min
                 (int)50,          // LeftY_max
                 (int)-50,         // RightX_min
                 (int)0,           // RightX_max
                 (int)20,          // RightY_min
                 (int)50);         // RightY_max
    out.push_back(flexing);

    return out;
}

vector<DynamicGesture> defineDynamicGestures(void)
{
    vector<DynamicGesture> out;

    DynamicGesture waving_r(GESTURE_WAVING_R);

    Gesture waving_r_int_0(GESTURE_WAVING_R,
                 (int)40,           // LeftX_min
                 (int)90,          // LeftX_max
                 (int)-20,           // LeftY_min
                 (int)40,          // LeftY_max
                 (int)0,         // RightX_min
                 (int)-50,           // RightX_max
                 (int)50,          // RightY_min
                 (int)20);         // RightY_max
    waving_r.addIntermediateGesture(waving_r_int_0);

    Gesture waving_r_int_1(GESTURE_WAVING_R,
                 (int)-20,           // LeftX_min
                 (int)40,          // LeftX_max
                 (int)-20,           // LeftY_min
                 (int)40,          // LeftY_max
                 (int)0,         // RightX_min
                 (int)-50,           // RightX_max
                 (int)50,          // RightY_min
                 (int)20);         // RightY_max
    waving_r.addIntermediateGesture(waving_r_int_1);

    Gesture waving_r_int_2(GESTURE_WAVING_R,
                 (int)40,           // LeftX_min
                 (int)90,          // LeftX_max
                 (int)-20,           // LeftY_min
                 (int)40,          // LeftY_max
                 (int)0,         // RightX_min
                 (int)-50,           // RightX_max
                 (int)50,          // RightY_min
                 (int)20);         // RightY_max
    waving_r.addIntermediateGesture(waving_r_int_2);

    Gesture waving_r_int_3(GESTURE_WAVING_R,
                 (int)-20,           // LeftX_min
                 (int)40,          // LeftX_max
                 (int)-20,           // LeftY_min
                 (int)40,          // LeftY_max
                 (int)0,         // RightX_min
                 (int)-50,           // RightX_max
                 (int)50,          // RightY_min
                 (int)20);         // RightY_max
    waving_r.addIntermediateGesture(waving_r_int_3);

    out.push_back(waving_r);

    DynamicGesture flying(GESTURE_FLYING);

    Gesture flying_int_0(GESTURE_FLYING,
                 (int)60,           // LeftX_min
                 (int)MAXCOORD,          // LeftX_max
                 (int)-20,           // LeftY_min
                 (int)30,          // LeftY_max
                 (int)-100,         // RightX_min
                 (int)MAXCOORD,           // RightX_max
                 (int)-10,          // RightY_min
                 (int)40);         // RightY_max
    flying.addIntermediateGesture(flying_int_0);

    Gesture flying_int_1(GESTURE_FLYING,
                 (int)-MAXCOORD,           // LeftX_min
                 (int)MAXCOORD,          // LeftX_max
                 (int)-100,           // LeftY_min
                 (int)0,          // LeftY_max
                 (int)-MAXCOORD,         // RightX_min
                 (int)MAXCOORD,           // RightX_max
                 (int)-90,          // RightY_min
                 (int)-50);         // RightY_max
    flying.addIntermediateGesture(flying_int_1);

    Gesture flying_int_2(GESTURE_FLYING,
                 (int)-MAXCOORD,           // LeftX_min
                 (int)MAXCOORD,          // LeftX_max
                 (int)-20,           // LeftY_min
                 (int)30,          // LeftY_max
                 (int)-MAXCOORD,         // RightX_min
                 (int)MAXCOORD,           // RightX_max
                 (int)-10,          // RightY_min
                 (int)40);         // RightY_max
    flying.addIntermediateGesture(flying_int_2);

    Gesture flying_int_3(GESTURE_FLYING,
                 (int)-MAXCOORD,           // LeftX_min
                 (int)MAXCOORD,          // LeftX_max
                 (int)-100,           // LeftY_min
                 (int)0,          // LeftY_max
                 (int)-MAXCOORD,         // RightX_min
                 (int)MAXCOORD,           // RightX_max
                 (int)-90,          // RightY_min
                 (int)-50);         // RightY_max
    flying.addIntermediateGesture(flying_int_3);

    out.push_back(flying);

    DynamicGesture waving_l(GESTURE_WAVING_L);

    Gesture waving_l_int_0(GESTURE_WAVING_L,
                 (int)80,           // LeftX_min
                 (int)0,          // LeftX_max
                 (int)50,           // LeftY_min
                 (int)0,          // LeftY_max
                 (int)-110,         // RightX_min
                 (int)-50,           // RightX_max
                 (int)-50,          // RightY_min
                 (int)40);         // RightY_max
    waving_l.addIntermediateGesture(waving_l_int_0);

    Gesture waving_l_int_1(GESTURE_WAVING_L,
                 (int)70,           // LeftX_min
                 (int)0,          // LeftX_max
                 (int)50,           // LeftY_min
                 (int)0,          // LeftY_max
                 (int)-60,         // RightX_min
                 (int)0,           // RightX_max
                 (int)-20,          // RightY_min
                 (int)50);         // RightY_max
    waving_l.addIntermediateGesture(waving_l_int_1);

    Gesture waving_l_int_2(GESTURE_WAVING_L,
                 (int)70,           // LeftX_min
                 (int)0,          // LeftX_max
                 (int)50,           // LeftY_min
                 (int)0,          // LeftY_max
                 (int)-100,         // RightX_min
                 (int)-50,           // RightX_max
                 (int)-20,          // RightY_min
                 (int)40);         // RightY_max
    waving_l.addIntermediateGesture(waving_l_int_2);

    Gesture waving_l_int_3(GESTURE_WAVING_L,
                 (int)70,           // LeftX_min
                 (int)0,          // LeftX_max
                 (int)50,           // LeftY_min
                 (int)0,          // LeftY_max
                 (int)-40,         // RightX_min
                 (int)0,           // RightX_max
                 (int)-20,          // RightY_min
                 (int)40);         // RightY_max
    waving_l.addIntermediateGesture(waving_l_int_3);

    out.push_back(waving_l);

    return out;
}

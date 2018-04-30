#ifndef ANALYTICS_H
#define ANALYTICS_H

#include "gesture.h"
#include <iostream>
#include <string>
#include <fstream>

using namespace std;

void updateAnalytics(struct analytics_t counts);
static string format;  // random strings for formating
static string total;   // "TOTAL:","TODAY_TOTAL_COUNT:", "OVERALL_TOTAL_COUNT:"
static int totalCount;  // "TOTAL: X","TODAY_TOTAL_COUNT: X", "OVERALL_TOTAL_COUNT: X"
static string gesture; // "WAVING_R:", "USAIN:", ...
static int gestureCount;   // "WAVING_R: X", "USAIN: X", ...

#define ENABLE_ANALYTICS true

struct analytics_t
{
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
};

struct analytics_t analytics_init(void);

#endif // ANALYTICS_H


// Low tech solution to store multiple states
// By sticking to a single array, we're setting
// aside unfragmented memory & that should be
// faster to read.

#ifndef LedStateData_h
#define LedStateData_h
 
// four sets of color, one per state
const int STATESIZE = 12;
// three colors (RGB) in each substate
const int STATEWIDTH = 3;
const int LEDMODES = 3;
const int DEVICEMODES = 6;
const int FILTERMODES = 3;

const int ALLSTATES[] = {
// LEDMODE_OFF
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
// LEDMODE_TRAFFIC_LIGHT
    255, 0, 0,
    145, 105, 0,
    0, 255, 0,
    2, 2, 2,
// LEDMODE_IMPERIAL
    255, 0, 0,
    205, 205, 205,
    0, 0, 255,
    2, 2, 2,
// DEVICEMODE_DEFAULT_SLOW
    255, 3, 3,
    3, 3, 3,
    3, 3, 3,
    0, 0, 0,
// DEVICEMODE_DEFAULT_MEDIUM
    255, 3, 3,
    255, 3, 3,
    3, 3, 3,
    0, 0, 0,
// DEVICEMODE_DEFAULT_FAST
    255, 3, 3,
    255, 3, 3,
    255, 3, 3,
    0, 0, 0,
// DEVICEMODE_BOTH_SLOW
    3, 255, 3,
    3, 3, 3,
    3, 3, 3,
    0, 0, 0,
// DEVICEMODE_BOTH_MEDIUM
    3, 255, 3,
    3, 255, 3,
    3, 3, 3,
    0, 0, 0,
// DEVICEMODE_BOTH_FAST
    3, 255, 3,
    3, 255, 3,
    3, 255, 3,
    0, 0, 0,
// FILTERMODE_WEAK
    0, 0, 255,
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
// FILTERMODE_MEDIUM
    0, 0, 255,
    0, 0, 255,
    0, 0, 0,
    0, 0, 0,
// FILTERMODE_STRONG
    0, 0, 255,
    0, 0, 255,
    0, 0, 255,
    0, 0, 0,
};

#endif

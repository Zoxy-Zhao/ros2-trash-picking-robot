#ifndef ARM6_CONFIG_H
#define ARM6_CONFIG_H

/* Opt in only after measuring the six-axis hardware. Channel 5 is the claw.
 * These reference offsets/ranges are NOT calibration of the original robot.
 */
#ifndef ARM6_ENABLED
#define ARM6_ENABLED 0
#endif
static const unsigned char ARM6_CHANNELS[6] = {0, 1, 2, 3, 4, 6};
static const double ARM6_ZERO_DEG[6] = {135, 90, 90, 90, 90, 90};
static const double ARM6_DIRECTION[6] = {1, 1, 1, 1, 1, 1};
static const double ARM6_TRAVEL_DEG[6] = {270, 180, 180, 180, 180, 180};
static const double ARM6_HOME_DEG[6] = {0, -45, 90, 0, 45, 0};

#endif

/*
 * Minimal math stubs for bare-metal ARM (newlib-nano lacks these)
 * Used by: beads_dsp reverb (fmod), grain engine (cos), quality processor (sin)
 */

#include <cmath>

extern "C" double fmod(double x, double y) {
    if (y == 0.0) return 0.0;
    return x - (int)(x / y) * y;
}

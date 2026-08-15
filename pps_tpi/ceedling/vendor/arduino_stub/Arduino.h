#pragma once

// Minimal stand-in for the real Arduino.h, used only to compile the
// hardware-independent modules natively under Ceedling. Provides just
// the constants those modules pull in; nothing hardware-specific.

#include <cmath>
#include <cstddef>

using std::size_t;

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

#ifndef constrain
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif

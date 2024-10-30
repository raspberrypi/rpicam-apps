#include "fbc_common.h"

// This file simply allows us to verify that the write preprocessor definitions are setup since the defaults changed between v8_3 and v8_4_1

#if !defined(FBC_ENABLED)
#error FBC_ENABLED is not defined - Kakadu requires this to support HTJ2K
#endif
#if defined(FBC_NO_ACCELERATION)
#warning FBC_NO_ACCELERATION is defined - Kakadu will use non accelerated (slow) HTJ2K path
#endif
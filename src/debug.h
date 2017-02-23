/*
 * Debug printing
 *
 * This debug printing functionality is implemented so that the compiler always sees the debug printing code.
 *
 * 1. Compile with -DDEBUG to enable debug printing possibilities.
 * 2. set ifcdaqdrvDebug to a LEVEL to print messages on that level and below.
 */

#ifndef DEBUG_H
#define DEBUG_H

#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>

/*
 * Usage TRACE((LEVEL, fmt, ...))
 *
 * Typical levels are 5 (notice), 6 (info), 7 (debug).
 *
 * - LEVEL debug verbosity number
 * - fmt printf format
 * - ... printf arguments
 */

#define LEVEL_NOTICE 5
#define LEVEL_INFO 6
#define LEVEL_DEBUG 7

#ifdef DEBUG
#define TRACE(x)    do { TRACE_ARGS x; } while(0)
#define DEBUG 1
#else
#define TRACE(x)    do { if (0) TRACE_ARGS x; } while (0)
#define DEBUG 0
#endif /* DEBUG */

extern int32_t ifcdaqdrvDebug;

#define TRACE_ARGS(level, fmt, args...) \
    if(level <= ifcdaqdrvDebug) { \
      fprintf(stderr, "%s:%d:%s(): ", __FILE__, __LINE__, __func__); \
      fprintf(stderr, fmt, ## args); \
    }


#define UNUSED(x) (void)(x)

#endif /* DEBUG_H */

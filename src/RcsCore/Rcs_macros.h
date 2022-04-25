/*******************************************************************************

  Copyright (c) 2017, Honda Research Institute Europe GmbH

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:

  1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#ifndef RCS_MACROS_H
#define RCS_MACROS_H

/*!
 * \defgroup RcsMacros Macros for logging and memory allocation
 *
 *           Misc convenience macros. If the code is compiled with flag
 *           -D RCS_NO_DEBUG, most macros will not be expanded. This should
 *           should be done if code is sufficiently tested and needs to run
 *           most performant.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#if defined(_MSC_VER)
#include "Rcs_msvc.h"
#else
#define RCSCORE_API
#include <alloca.h>
#include <strings.h>
#endif

#if !defined (__FILENAME__)
#if defined (RCS_BASE_PATH_LENGTH)
#define __FILENAME__ (__FILE__ + RCS_BASE_PATH_LENGTH)
#else
#define __FILENAME__ (__FILE__)
#endif
#endif

#if !defined (__FUNCTION__)
#define __FUNCTION__ __func__
#endif

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

// Degrees to Radian
#define RCS_DEG2RAD(degrees) ((M_PI/180.0)*(degrees))

// Radians to Degrees
#define RCS_RAD2DEG(radians) ((180.0/M_PI)*(radians))

typedef void (*RLOG_CB_FUNC)(int log_level, const char* file,
                             const char* function, int line,
                             const char* msg, ...);

typedef void* (*RCS_MALLOC_FUNC)(size_t nmemb, size_t size);

#if defined (__cplusplus)
#include <iostream>

extern "C" {
#endif

/*! \ingroup RcsMacros
 *  \brief Obtain a backtrace and print it to stdout. The function is
 *         implemented in Rcs_stackTrace.cpp.
 */
void print_trace(void);

/*! \ingroup RcsMacros
 *  \brief Helper function for print_trace() also used by SegFaultHandler.h
 *         implemented in Rcs_stackTrace.cpp.
 */
int print_trace_entry(void* address, size_t stack_frame);

extern RCSCORE_API int RcsLogLevel;

extern RLOG_CB_FUNC RCS_GLOBAL_RLOG_CB;
extern RCSCORE_API RCS_MALLOC_FUNC Rcs_calloc;

#if defined (__cplusplus)
}
#endif // __cplusplus



// These macros are expanded even if RCS_NO_DEBUG is flagged

/*! \ingroup RcsMacros
 *  \brief For convenience. If RLOG(...) is changed to NLOG(...), nothing will
 *         be expanded, but the code can remain. So it is easier to keep
 *         debugging information without affecting the speed.
 */
#define NLOG(debugLevel, ...)
#define NLOGS(debugLevel, ...)

/*! \ingroup RcsMacros
 *  \brief For convenience. If severe NULL checking is required, this macro can
 *         be copied from the RCHECK macro. Otherwise it is left empty to
 *         produce no run-time overhead. The stack trace will in most cases be
 *         sufficient.
 */
#define RCHECK_PEDANTIC(cond)

/*! \ingroup RcsMacros
 *  \brief Triggers a stack trace and exits the application.
 */
#define RFATAL(...)                                \
  do                                               \
  {                                                \
    fprintf(stderr, "[%s: %s(%d)]: Fatal: -   ",   \
            __FILENAME__, __FUNCTION__, __LINE__); \
    fprintf(stderr, __VA_ARGS__);                  \
    fprintf(stderr, "\n");                         \
    print_trace();                                 \
    exit(EXIT_FAILURE);                            \
  } while (0)

/*! \ingroup RcsMacros
 *  \brief Triggers a stack trace.
 */
#define RWARNING(debugLevel, ...)                                         \
  do                                                                      \
  {                                                                       \
    if (debugLevel <= RcsLogLevel)                                        \
    {                                                                     \
      fprintf(stderr, ANSI_COLOR_CYAN "[%s: " ANSI_COLOR_GREEN "%s(%d)]:" \
              ANSI_COLOR_RED "WARNING: -   ",                             \
              __FILENAME__, __FUNCTION__, __LINE__);                      \
      fprintf(stderr, __VA_ARGS__);                                       \
      fprintf(stderr, "\n");                                              \
      print_trace(); }                                                    \
  } while (0)


/*! \ingroup RcsMacros
 *  \brief String compare macro. Returns true if the strings a and b are equal,
 *         false otherwise.
 */
#define STREQ(a,b) (strcmp((a),(b))==0)

/*! \ingroup RcsMacros
 *  \brief String compare macro for max. number of characters. Returns true if
 *         the strings a and b a re equal, false otherwise.
 */
#define STRNEQ(a,b,n) (strncmp((a),(b),(n))==0)
#define STRCASEEQ(a,b) (strcasecmp((a),(b))==0)



// Memory allocation macros

/*! \ingroup RcsMacros
 *  \brief Allocates heap memory for one variable of the given type. The memory
 *         is set to zero.
 */
#define RALLOC(type) (type*) Rcs_calloc(1, sizeof(type))

/*! \ingroup RcsMacros
 *  \brief Allocates heap memory for nMemb variables of the given type. The
 *         memory is set to zero.
 */
#define RNALLOC(nMemb, type) (type*) Rcs_calloc((nMemb), sizeof(type))

/*! \ingroup RcsMacros
 *  \brief Convenience macro around realloc.
 */
#define RREALLOC(ptr, nMemb, type) (type*) realloc(ptr, (nMemb)*sizeof(type));

/*! \ingroup RcsMacros
 *  \brief Frees the heap memory.
 */
#define RFREE(ptr) free((void*) (ptr))

/*! \ingroup RcsMacros
 *  \brief Gets the memory for one variable of the given type from the stack.
 *         It must not be freed, otherwise there will be undefined behavior.
 *         The memory is uninitialized.
 */
#define RSTALLOC(type) (type*) alloca(sizeof(type))

/*! \ingroup RcsMacros
 *  \brief Gets the memory for nMemb variables of the given type from the
 *         stack. It must not be freed, otherwise there will be undefined
 *         behavior. The memory is uninitialized. The alloca function is not
 *         part of the C standard, and deemed unsafe. However, it behaves the
 *         same as the variable sized arrays in C99, which we allow to use.
 *         Therefore using alloca is accepted if used with care (small memory
 *         allocations, no recursions, etc.).
 */
#define RNSTALLOC(nMemb, type) (type*) alloca((nMemb)*sizeof(type))


/*! \ingroup RcsMacros
 *  \brief Print macro.
 */
#define RMSG(...)                                                            \
  do                                                                         \
  {                                                                          \
    fprintf(stderr, "[%s: %s(%d)]: ", __FILENAME__, __FUNCTION__, __LINE__); \
    fprintf(stderr, __VA_ARGS__);                                            \
    fprintf(stderr, "\n");                                                   \
    fflush(stderr);                                                          \
  } while (0)

/*! \ingroup RcsMacros
 *  \brief Print macro (short form)
 */
#define RMSGS(...)                                         \
  do                                                       \
  {                                                        \
    fprintf(stderr, "[%s(%d)]: ", __FUNCTION__, __LINE__); \
    fprintf(stderr, __VA_ARGS__);                          \
    fprintf(stderr, "\n");                                 \
    fflush(stderr);                                        \
  } while (0)

/*! \ingroup RcsMacros
 * \brief Rcs stack trace function
 */
#define RTRACE() \
  do \
  { \
    print_trace(); \
  } while (0)

// These macros are empty if RCS_NO_DEBUG is flagged

#if !defined (RCS_NO_DEBUG)

/*! \ingroup RcsMacros
 *  \brief Pause macro.
 */
#define RPAUSE()                                           \
  do                                                       \
  {                                                        \
    fprintf(stderr, "[%s: %s(%d)]: Hit enter to continue", \
            __FILENAME__, __FUNCTION__, __LINE__);         \
    (void)getchar();                                       \
  } while (0)

/*! \ingroup RcsMacros
 *  \brief Pause macro.
 */
#define RPAUSE_DL(debugLevel)                                \
  do                                                         \
  {                                                          \
    if (debugLevel <= RcsLogLevel)                           \
    {                                                        \
      fprintf(stderr, "[%s: %s(%d)]: Hit enter to continue", \
              __FILENAME__, __FUNCTION__, __LINE__);         \
      (void)getchar();                                       \
    }                                                        \
  } while (0)

/*! \ingroup RcsMacros
 *  \brief Pause macro with message.
 */
#define RPAUSE_MSG(...)                            \
  do                                               \
  {                                                \
    fprintf(stderr, "[%s: %s(%d)]: ",              \
            __FILENAME__, __FUNCTION__, __LINE__); \
    fprintf(stderr, __VA_ARGS__);                  \
    (void)getchar();                               \
  } while (0)

/*! \ingroup RcsMacros
 *  \brief Pause macro with message.
 */
#define RPAUSE_MSG_DL(debugLevel, ...)                \
  do                                                  \
  {                                                   \
    if (debugLevel <= RcsLogLevel)                    \
    {                                                 \
      fprintf(stderr, "[%s: %s(%d)]: ",               \
              __FILENAME__, __FUNCTION__, __LINE__);  \
      fprintf(stderr, __VA_ARGS__);                   \
      (void)getchar();                                \
    }                                                 \
  } while (0)

/*! \ingroup RcsMacros
 *  \brief Check macro. Exits if condition is not met.
 */
#define RCHECK(cond)                                                  \
  do                                                                  \
  {                                                                   \
    if (!(cond))                                                      \
    {                                                                 \
      fprintf(stderr, "\n[%s: %s(%d)]: Check failed: %s - Exiting\n", \
              __FILENAME__, __FUNCTION__, __LINE__, #cond);           \
      print_trace();                                                  \
      abort(); }                                                      \
  } while (0)

/*! \ingroup RcsMacros
 *  \brief Check macro. Exits if condition is not met.
 */
#define RCHECK_EQ(var1, var2) \
  do                                                                      \
  {                                                                       \
    if (var1 != var2)                                                     \
    {                                                                     \
      fprintf(stderr, "\n[%s: %s(%d)]: Check failed: %s (%g) != %s (%g) " \
              "- Exiting\n", __FILENAME__, __FUNCTION__, __LINE__, #var1, \
              (double) var1, #var2, (double) var2);                       \
      print_trace();                                                      \
      exit(EXIT_FAILURE); }                                               \
  } while (0)

/*! \ingroup RcsMacros
 *  \brief Check macro. Exits with a message if condition is not met.
 */
#define RCHECK_MSG(cond, ...)                                    \
  do                                                             \
  {                                                              \
    if (!(cond))                                                 \
    {                                                            \
      fprintf(stderr, "\n[%s: %s(%d)]: Check failed: %s   -   ", \
              __FILENAME__, __FUNCTION__, __LINE__, #cond);      \
      fprintf(stderr, __VA_ARGS__);                              \
      fprintf(stderr, "\n");                                     \
      print_trace();                                             \
      exit(EXIT_FAILURE); }                                      \
  } while (0)


/*! \ingroup RcsMacros
 *  \brief Log macro with debug level. Optionally calls a registered callback
 *         method, which can be used to collect messages as for example in the
 *         LogCollector
 * \todo: callback mechanism not supported in Windows
 */
#if !defined(_MSC_VER)

#define RLOG(debugLevel, ...)                                        \
  do                                                                 \
  {                                                                  \
    if (RCS_GLOBAL_RLOG_CB)                                          \
    {                                                                \
      RCS_GLOBAL_RLOG_CB(debugLevel, __FILENAME__, __FUNCTION__,     \
                         __LINE__, __VA_ARGS__); }                   \
    if (debugLevel <= RcsLogLevel)                                   \
    {                                                                \
      fprintf(stderr, "[%s : %s(%d)]: ", __FILENAME__, __FUNCTION__, \
              __LINE__);                                             \
      fprintf(stderr, __VA_ARGS__);                                  \
      fprintf(stderr, "\n"); }                                       \
  } while (0)

#define RLOGS(debugLevel, ...)                                   \
  do                                                             \
  {                                                              \
    if (RCS_GLOBAL_RLOG_CB)                                      \
    {                                                            \
      RCS_GLOBAL_RLOG_CB(debugLevel, "", __FUNCTION__, __LINE__, \
                         __VA_ARGS__);                           \
    }                                                            \
    if (debugLevel <= RcsLogLevel)                               \
    {                                                            \
      fprintf(stderr, "[%s(%d)]: ", __FUNCTION__, __LINE__);     \
      fprintf(stderr, __VA_ARGS__);                              \
      fprintf(stderr, "\n"); }                                   \
  } while (0)

#else

#define RLOG(debugLevel, ...)                                \
  do                                                         \
  {                                                          \
    if (debugLevel <= RcsLogLevel)                           \
    {                                                        \
      fprintf(stderr, "[%s(%d)]: ", __FILENAME__, __LINE__); \
      fprintf(stderr, __VA_ARGS__);                          \
      fprintf(stderr, "\n"); }                               \
  } while (0)

#define RLOGS(debugLevel, ...)                               \
  do                                                         \
  {                                                          \
    if (debugLevel <= RcsLogLevel)                           \
    {                                                        \
      fprintf(stderr, "[%s(%d)]: ", __FILENAME__, __LINE__); \
      fprintf(stderr, __VA_ARGS__);                          \
      fprintf(stderr, "\n"); }                               \
  } while (0)

#endif

#if defined (__cplusplus)

#define RLOG_CPP(debugLevel, msg) \
  do                                                                  \
  {                                                                   \
    if (debugLevel <= RcsLogLevel)                                    \
    {                                                                 \
      std::cerr << "[" << __FILENAME__ << ": " << __FUNCTION__ << "(" \
                << __LINE__ << ")]: " << msg << std::endl; }          \
  } while (0)

#define RMSG_CPP(msg)                                               \
  do                                                                \
  {                                                                 \
    std::cerr << "[" << __FILENAME__ << ": " << __FUNCTION__ << "(" \
              << __LINE__ << ")]: " << msg << std::endl;            \
  } while (0)
#endif

/*! \ingroup RcsMacros
 *  \brief Execution macro with debug level.
 */
#define REXEC(debugLevel) \
  if (debugLevel<=RcsLogLevel)

#else   // RCS_NO_DEBUG

#define RPAUSE()
#define RPAUSE_DL(debugLevel)
#define RPAUSE_MSG(...)
#define RPAUSE_MSG_DL(debugLevel, ...)
#define RCHECK(cond)
#define RCHECK_EQ(var1, var2)
#define RCHECK_MSG(cond, ...)
#define RLOG(debugLevel, ...)
#define RLOG_CPP(debugLevel, ...)
#define RLOGS(debugLevel, ...)
#define REXEC(debugLevel) \
  if (false)
#define RTICK(s)
#define RTOCK(s)

#endif   // RCS_NO_DEBUG

#endif   // RCS_MACROS_H

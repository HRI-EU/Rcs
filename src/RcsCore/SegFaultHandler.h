/*******************************************************************************

  Modified version of work by Jaco Kroon <jaco@kroon.co.za>
  Original header:

  This source file is used to print out a stack-trace when your program
  segfaults. It is relatively reliable and spot-on accurate.

  This code is in the public domain. Use it as you see fit, some credit
  would be appreciated, but is not a prerequisite for usage. Feedback
  on it's use would encourage further development and maintenance.

  Due to a bug in gcc-4.x.x you currently have to compile as C++ if you want
  demangling to work.

  Please note that it's been ported into my ULS library, thus the check for
  HAS_ULSLIB and the use of the sigsegv_outp macro based on that define.

  Author: Jaco Kroon <jaco@kroon.co.za>

  Copyright (C) 2005 - 2009 Jaco Kroon

*******************************************************************************/

#ifndef SEGFAULTHANDLER_H
#define SEGFAULTHANDLER_H

#if !defined (_MSC_VER)

/* Bug in gcc prevents from using CPP_DEMANGLE in pure "C" */
#if !defined(__cplusplus) && !defined(NO_CPP_DEMANGLE)
#define NO_CPP_DEMANGLE
#endif

#include <Rcs_macros.h>
#include <memory.h>
#include <signal.h>
#include <sys/stat.h>
#include <dlfcn.h>
#include <unistd.h>
#include <ucontext.h>

// VARIABLE_IS_NOT_USED macro for avoiding warnings by GCC, because we use the
// signal_segv function only with the RCS_INSTALL_SEGFAULTHANDLER macro
#ifdef __GNUC__
#define VARIABLE_IS_NOT_USED __attribute__ ((unused))
#else
#define VARIABLE_IS_NOT_USED
#endif


#ifndef NO_CPP_DEMANGLE
#include <cxxabi.h>
#ifdef __cplusplus
using __cxxabiv1::__cxa_demangle;
#endif
#endif

#if defined(REG_RIP)
# define SIGSEGV_STACK_IA64
# define REGFORMAT "%016lx"
#elif defined(REG_EIP)
# define SIGSEGV_STACK_X86
# define REGFORMAT "%08x"
#else
# define SIGSEGV_STACK_GENERIC
# define REGFORMAT "%x"
#endif

#define RCS_ERRORCODE_SEGFAULT (10) ///< Error code returned in case of a segfault captured by the segfault handler
#define RCS_ERRORCODE_ABORT (11) ///< Error code returned in case of a sigabrt captured by the segfault handler



/*!
 * \brief Callback function that is registered using the
 *        \ref RCS_INSTALL_SEGFAULTHANDLER macro
 * \param[in] signum The signal number
 * \param[in] info Info structure holding the signal code and address
 * \param[in] ptr Pointer holding the context of the segfault (from this,
 *                things like the instruction pointer (ip) can be extracted)
 *
 * The method prints a stacktrace upon segfaults
 * \note There are various assumptions made in this method about the "calling"
 *       signal. Do not connect the callback method to other signals (e.g.,
 *       si_codes would need to be adapted).
 */
static void VARIABLE_IS_NOT_USED signal_segv(int signum, siginfo_t* info,
                                             void* ptr)
{
  static const char* si_codes[3] = {"SI_NOINFO", "SEGV_MAPERR", "SEGV_ACCERR"};

  int f = 0;
  ucontext_t* ucontext = (ucontext_t*)ptr;
  void** bp = 0;
  void* ip = 0;

  const char* userName = getenv("USER");
  fprintf(stderr, "\nSegmentation Fault! Hey %s, go FIX your code!!!\n",
          userName ? userName : "");
  fprintf(stderr, "info.si_signo = %d\n", signum);
  fprintf(stderr, "info.si_errno = %d\n", info->si_errno);
  fprintf(stderr, "info.si_code  = %d (%s)\n", info->si_code, si_codes[info->si_code]);
  fprintf(stderr, "info.si_addr  = %p\n", info->si_addr);

#ifndef SIGSEGV_NOSTACK
#if defined(SIGSEGV_STACK_IA64) || defined(SIGSEGV_STACK_X86)
#if defined(SIGSEGV_STACK_IA64)
  ip = (void*)ucontext->uc_mcontext.gregs[REG_RIP];
  bp = (void**)ucontext->uc_mcontext.gregs[REG_RBP];
#elif defined(SIGSEGV_STACK_X86)
  ip = (void*)ucontext->uc_mcontext.gregs[REG_EIP];
  bp = (void**)ucontext->uc_mcontext.gregs[REG_EBP];
#endif

  fprintf(stderr, "\n########## Stacktrace ##########\n");
  while (bp && ip)
  {
    if (!print_trace_entry(ip, f++))
    {
      break;
    }

    ip = bp[1];
    bp = (void**)bp[0];
  }
#else
  fprintf(stderr, "Stack trace (non-dedicated):\n");
  sz = backtrace(bt, 20);
  strings = backtrace_symbols(bt, sz);
  int i;
  for (i = 0; i < sz; ++i)
  {
    sigsegv_outp("%s", strings[i]);
  }
#endif
  fprintf(stderr, "################################\n\n");
#else
  fprintf(stderr, "Not printing stack strace.\n");
#endif
  _exit(RCS_ERRORCODE_SEGFAULT);
}

static void VARIABLE_IS_NOT_USED signal_abrt(int, siginfo_t*, void*)
{
  const char* userName = getenv("USER");
  fprintf(stderr, "\nSIGABORT! Hey %s, go FIX your code!!!\n",
          userName ? userName : "");

  // just print a stacktrace and exit
  RTRACE();
  _exit(RCS_ERRORCODE_ABORT);
}


/*!
 * \brief Include this macro in your executable cpp file to automatically
 *        register the Rcs segfault handler, which prints a stack trace upon
 *        segmentation faults.
 */
#define RCS_INSTALL_SEGFAULTHANDLER \
  static void __attribute__((constructor)) setup_sigsegv() \
  { \
    struct sigaction action; \
    memset(&action, 0, sizeof(action)); \
    action.sa_sigaction = signal_segv; \
    action.sa_flags = SA_SIGINFO; \
    if (sigaction(SIGSEGV, &action, NULL) < 0) \
      perror("sigaction"); \
  } \
  void (*setup_sigsegv_dummy)() = &setup_sigsegv; // use the function pointer to avoid unused method warnings


/*!
 * \brief Include this macro in your executable cpp file to automatically
 *        register the Rcs abort handler, which prints a stack trace upon
 *        sigaborts
 */
#define RCS_INSTALL_ABORTHANDLER \
  static void __attribute__((constructor)) setup_sigabrt() \
  { \
    struct sigaction action; \
    memset(&action, 0, sizeof(action)); \
    action.sa_sigaction = signal_abrt; \
    action.sa_flags = SA_SIGINFO; \
    if (sigaction(SIGABRT, &action, NULL) < 0) \
      perror("sigabrt"); \
  } \
  void (*setup_sigabrt_dummy)() = &setup_sigabrt; // use the function pointer to avoid unused method warnings

/*!
 * \brief Include this macro in your executable cpp file to automatically
 *        register the segfault handler and the sigabort handler, which both
 *        print stack traces
 */
#define RCS_INSTALL_ERRORHANDLERS \
  RCS_INSTALL_SEGFAULTHANDLER \
  RCS_INSTALL_ABORTHANDLER

#else    // _MSC_VER

#define RCS_INSTALL_SEGFAULTHANDLER
#define RCS_INSTALL_ERRORHANDLERS

#endif   // _MSC_VER

#endif   // SEGFAULTHANDLER_H

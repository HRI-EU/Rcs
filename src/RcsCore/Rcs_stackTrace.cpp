/*******************************************************************************

  Copyright (c) 2017, Honda Research Institute Europe GmbH.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

  3. All advertising materials mentioning features or use of this software
     must display the following acknowledgement: This product includes
     software developed by the Honda Research Institute Europe GmbH.

  4. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

/* Bug in gcc prevents from using CPP_DEMANGLE in pure "C" */
#if !defined(__cplusplus) && !defined(NO_CPP_DEMANGLE)
#define NO_CPP_DEMANGLE
#endif

#if !defined(NO_CPP_DEMANGLE) && !defined(_MSC_VER)
#include <cxxabi.h>
#ifdef __cplusplus
#include <string>
using __cxxabiv1::__cxa_demangle;
#endif
#endif

#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <string.h>
#include <string>

// Uncomment if memory addresses should be printed in the stack trace
//#define RCS_STACKTRACE_SHOW_MEMORY

// Comment the following line to NOT reduce the function signature to ()
#define RCS_STACKTRACE_REDUCE_FUNCTION_SIGNATURE

// Comment the following line to disable path stripping
#define RCS_STACKTRACE_STRIP_PATH

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

#if !defined (_MSC_VER)

#include <execinfo.h>
#include <dlfcn.h>

#ifdef __cplusplus
std::string symbolToFunction(const std::string& symbol)
{

  // search first '(' and last ')'
  size_t p1 = symbol.find('(');
  size_t p2 = symbol.rfind(')');

  if (p1 == std::string::npos || p2 == std::string::npos || p2 < p1)
  {
    return symbol;
  }

  std::string function = symbol.substr(0, p1+1) + symbol.substr(p2, std::string::npos);
  return function;
}
#endif

extern "C" {

  /****************************************************************************

    \brief Obtain a backtrace and print it to stdout.

  ****************************************************************************/

  int print_trace_entry(void* address, size_t stack_frame)
  {
    Dl_info info;
    if (dladdr(address, &info))
    {
      char symsfilename[1024];
      char filename[1024];

      // check for .syms file
      sprintf(symsfilename, "%s.syms", info.dli_fname);
      struct stat sts;
      if (stat(symsfilename, &sts) != -1)
      {
        sprintf(filename, "%s", symsfilename);
      }
      else
      {
        sprintf(filename, "%s", info.dli_fname);
      }

      // find file and line number for the given address
      char buff[1024];
      char* buff_ptr = buff;
      memset(buff, 0, 1024);
      char line_number[32];
      memset(line_number, 0, 32);

      // get debug information
      if (filename[0] != '\0')
      {
        char syscom[512];
        if (address < (void*)0x8200000 && address > (void*)0x8000000)   // TODO check memory ranges, these are only guesses
        {
          sprintf(syscom, "addr2line %p -C -e %s", address, filename);
        }
        else
        {
          sprintf(syscom, "addr2line %p -C -e %s", (void*)((unsigned long) address - (unsigned long) info.dli_fbase), filename);
        }

        // print filename and line number
        FILE* fp = popen(syscom, "r");
        if (fp)
        {
          while (fgets(buff, sizeof(buff), fp) != NULL);
          pclose(fp);
        }
        else
        {
          fprintf(stderr, "Couldn't open pipe \"%s\"", syscom);
        }

        int lastIdx = (strlen(buff)-1 > 0) ? strlen(buff)-1 : 0;
        buff[lastIdx] = '\0';

        // split buff into filename and line number
        char* s;
        s = strrchr(buff, ':');

        if (s != NULL)
        {
          sprintf(line_number, "%s", s);
          *s = '\0';
        }

#ifdef RCS_STACKTRACE_STRIP_PATH
        /// \todo: This leads to a memory leak, but who cares here?

        // find the second last '/' and move char* pointer to there
        // Thus the output will contain the last directory and source file name
        char* p = strchr(buff,'/');
        char* st = NULL;
        char* nd = NULL;
        while (p != NULL)
        {
          nd = st;
          st = p;
          p = strchr(st+1,'/');
        }

        if (nd != NULL)
        {
          buff_ptr = nd + 1;
        }
#endif
      }

      // print demangled function name
      const char* symname = info.dli_sname;
#ifndef NO_CPP_DEMANGLE
      int status;
      char* tmp = __cxa_demangle(symname, NULL, 0, &status);

      if (status == 0 && tmp)
      {
        symname = tmp;
      }

#ifdef RCS_STACKTRACE_REDUCE_FUNCTION_SIGNATURE
      std::string only_function_name = symbolToFunction(symname ? symname : "NULL");
      symname = only_function_name.c_str();
#endif

#endif

#ifdef RCS_STACKTRACE_SHOW_MEMORY
      fprintf(stderr, ANSI_COLOR_CYAN " %2u: " ANSI_COLOR_BLUE "%12p " ANSI_COLOR_RED "<%s+%lu>" ANSI_COLOR_GREEN " %s" ANSI_COLOR_RESET "%s\n",
              (unsigned int) stack_frame,
              array[i],
              symname,
              (unsigned long)address - (unsigned long)info.dli_saddr,
              buff_ptr,
              line_number);
#else
      fprintf(stderr, ANSI_COLOR_CYAN " %2u: " ANSI_COLOR_RED "<%s+%lu>" ANSI_COLOR_GREEN " %s" ANSI_COLOR_RESET "%s\n",
              (unsigned int) stack_frame,
              symname,
              (unsigned long)address - (unsigned long)info.dli_saddr,
              buff_ptr,
              line_number);
#endif

#ifndef NO_CPP_DEMANGLE
      if (tmp)
      {
        free(tmp);
      }
#endif

      if (info.dli_sname && !strcmp(info.dli_sname, "main"))
      {
        return false;
      }
    }
    else
    {
      return false;
    }

    return true;
  }



  void print_trace(bool with_memory_addresses)
  {
    void* array[100];
    size_t size;
    char** strings;
    size_t i;

    size = backtrace(array, 100);
    strings = backtrace_symbols(array, size);

    fprintf(stderr, "\n########## Stacktrace ##########\n");

    for (i = 1; i < size; i++) // start at 1 to skip this method
    {
      bool success = print_trace_entry(array[i], i-1);

      if (!success)
      {
        break;
      }
    }
    fprintf(stderr, "################################\n\n");
    fflush(stderr);

    free(strings);
  }

}  // extern "C"
#else

extern "C" {

  int print_trace_entry(void* address, size_t stack_frame)
  {
    return false;
  }
  void print_trace(void)
  {
    fprintf(stderr, "No stack trace under windows\n");
  }

}  // extern "C"
#endif






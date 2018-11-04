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

#include "Rcs_utils.h"
#include "Rcs_macros.h"
#include "Rcs_timer.h"

#include <locale.h>

#if !defined(_MSC_VER)
#include <unistd.h>
#include <termios.h>
#else
#include <conio.h>
#include <stdio.h>
#include <winsock2.h>
#include <windows.h>
#endif



/*******************************************************************************
 * Clones a character array with dynamic memory allocation.
 ******************************************************************************/
char* String_clone(const char* src)
{
  char* dst = NULL;

  if (src != NULL)
  {
    // strlen does not include the trailing zero
    size_t len = 1+strlen(src);
    dst = RNALLOC(len, char);
    RCHECK(dst);
    memcpy(dst, src, len);
  }

  return dst;
}

/*******************************************************************************
 * See header
 ******************************************************************************/
void String_copyOrRecreate(char** dst, const char* src)
{
  if (src==NULL)
  {
    if (*dst != NULL)
    {
      RFREE(*dst);
      *dst = NULL;
    }
    return;
  }

  if (*dst==NULL)
  {
    *dst = String_clone(src);
  }
  else if (strlen(*dst)<strlen(src))
  {
    // strlen does not include the trailing zero
    *dst = (char*) realloc(*dst, (1+strlen(src))*sizeof(char));
    RCHECK(*dst);
    strcpy(*dst, src);
  }
  else
  {
    strcpy(*dst, src);
  }
}

/*******************************************************************************
 * Prepends t into s. Assumes s has enough space allocated for the
 * combined string.
 ******************************************************************************/
void String_prepend(char* s, const char* t)
{
  size_t len = strlen(t);
  size_t i;

  memmove(s + len, s, strlen(s) + 1);

  for (i = 0; i < len; ++i)
  {
    s[i] = t[i];
  }
}

/*******************************************************************************
 * See header
 ******************************************************************************/
bool String_hasEnding(const char* str, const char* ending,
                      bool caseSensitive)
{
  if (str == NULL)
  {
    RLOG(4, "str is NULL");
    return false;
  }

  if (ending == NULL)
  {
    RLOG(4, "ending is NULL (compared against \"%s\")", str);
    return false;
  }

  if (strlen(ending) > strlen(str))
  {
    RLOG(4, "length of ending (%s) is larger than length of str (%s)",
         ending, str);
    return false;
  }

  const char* strPtr = &str[strlen(str)-strlen(ending)];
  bool isEqual;

  if (caseSensitive==true)
  {
    isEqual = STREQ(strPtr, ending);
  }
  else
  {
    isEqual = STRCASEEQ(strPtr, ending);
  }

  return isEqual;
}

/*******************************************************************************

  Trailing zero removal. See:
  http://stackoverflow.com/questions/277772/avoid-trailing-zeroes-in-printf

  The function nDecimals and morphNumericString have been adapted from here:
  http://stackoverflow.com/users/14860/paxdiablo

  It is licensed as

  All code I post on Stack Overflow is covered by the "Do whatever the heck
  you want with it" licence, the full text of which is:

  Do whatever the heck you want with it.

 ******************************************************************************/
static void nDecimals(char* s, double d, int n)
{
  // Allow for negative.
  double d2 = (d >= 0) ? d : -d;
  int sz = (d >= 0) ? 0 : 1;

  // Add one for each whole digit (0.xx special case).
  if (d2 < 1)
  {
    sz++;
  }
  while (d2 >= 1)
  {
    d2 /= 10.0;
    sz++;
  }

  // Adjust for decimal point and fractionals.
  sz += 1 + n;

  // Create format string then use it.
  sprintf(s, "%*.*f", sz, n, d);
}

static void morphNumericString(char* s, int n)
{
  char* p;
  int count;

  p = strchr(s,'.');        // Find decimal point, if any.

  if (p != NULL)
  {
    count = n;              // Adjust for more or less decimals.

    while (count >= 0)      // Maximum decimals allowed.
    {
      count--;

      if (*p == '\0')       // If there's less than desired.
      {
        break;
      }

      p++;                  // Next character.
    }

    *p-- = '\0';            // Truncate string.

    while (*p == '0')       // Remove trailing zeros.
    {
      *p-- = '\0';
    }

    if (*p == '.')          // If all decimals were zeros, remove ".".
    {
      *p = '\0';
    }
  }
}

char* String_fromDouble(char* str, double value, unsigned int maxDigits)
{
  nDecimals(str, value, maxDigits);
  morphNumericString(str, maxDigits);

  if (STREQ(str, "-0"))
  {
    strcpy(str, "0");
  }

  return str;
}

/*******************************************************************************
 * See header
 ******************************************************************************/
bool String_toBool(const char* str)
{
  if (STRCASEEQ(str, "true"))
  {
    return true;
  }
  else
  {
    return false;
  }

}

/*******************************************************************************
 * See header
 ******************************************************************************/
unsigned int String_countSubStrings(const char* str, const char* delim)
{
  char* lStr, *pch;
  unsigned int nSubStrings = 0;

  // Make a local copy of str, since strtok modifies it during processing
  lStr = String_clone(str);
  pch = strtok(lStr, delim);

  // Determine number of delim-separated sub-strings
  while (pch != NULL)
  {
    pch = strtok(NULL, delim);
    nSubStrings++;
  }

  RFREE(lStr);

  return nSubStrings;
}


/*******************************************************************************
 *
 ******************************************************************************/
static char* String_expandMacros_(char* str)
{
  const char* p = str;
  int count = 0;
  bool foundOne = false;
  int len = strlen(str);

  while (count<len && (foundOne==false))
  {

    if ((*p)=='$' && (*(p+1)=='{'))
    {
      const char* q = p+2;
      int macroStartIdx = count+2;
      int macroEndIdx = macroStartIdx;

      while ((*q) != '\0')
      {
        if ((*q)=='}')
        {
          int macroLen = macroEndIdx-macroStartIdx;
          char* macro = RNSTALLOC(macroLen+1, char);
          memcpy(macro, &str[macroStartIdx], macroLen);
          macro[macroLen] = '\0';
          char* envStr = getenv(macro);
          unsigned int macroStrLen = envStr ? strlen(envStr) : 0;
          char* before = String_clone(str);
          before[macroStartIdx-2] = '\0';

          const char* after = &str[macroEndIdx+1];
          foundOne = true;

          unsigned int newLen = strlen(before) + macroStrLen +
                                strlen(after) + 2;
          char* newStr = RNALLOC(newLen, char);
          if (envStr)
          {
            sprintf(newStr, "%s%s%s", before, envStr, after);
          }
          else
          {
            sprintf(newStr, "%s%s", before, after);
          }
          RFREE(before);
          RFREE(str);
          str = String_expandMacros_(newStr);

          break;
        }
        else if ((*q)=='$')
        {
          break;
        }

        macroEndIdx++;
        q++;
      }   // while ((*q) != '\0')
    }   // if ((*p)=='$' && (*(p+1)=='{'))

    count++;
    p++;
  }   // while ((*p) != '\0')

  return str;
}

char* String_expandEnvironmentVariables(const char* str)
{
  char* copyOfString = String_clone(str);
  return String_expandMacros_(copyOfString);
}

/*******************************************************************************
 * See header
 ******************************************************************************/
double String_toDouble_l(const char* str)
{
  locale_t tmpLocale = newlocale(LC_NUMERIC_MASK, "C", NULL);
  double val = strtod_l(str, NULL, tmpLocale);
  freelocale(tmpLocale);

  return val;
}

/*******************************************************************************
 * See header
 ******************************************************************************/
bool File_exists(const char* filename)
{
  FILE* file;

  if (filename == NULL)
  {
    return false;
  }

  file = fopen(filename, "r");

  if (file != NULL)
  {
    fclose(file);
    return true;
  }

  return false;
}

/*******************************************************************************
 * See header
 ******************************************************************************/
long File_getSize(const char* name)
{
  long eof_ftell;
  FILE* fd;

  fd = fopen(name, "rb");

  if (fd == NULL)
  {
    RLOG(4, "File \"%s\" could not be opened", name);
    return 0;
  }

  fseek(fd, 0, SEEK_END);
  eof_ftell = ftell(fd);
  fclose(fd);

  return eof_ftell;
}

/*******************************************************************************
 * See header
 ******************************************************************************/
long File_getLineCount(const char* fileName)
{
  FILE* fd;
  long nLines = 0;
  int ch = 0;

  fd = fopen(fileName, "r");

  if (fd == NULL)
  {
    RLOG(4, "File \"%s\" could not be opened", fileName);
    return 0;
  }

  while ((ch = fgetc(fd)) != EOF)
  {
    if (ch == '\n')
    {
      nLines++;
    }
  }

  fclose(fd);

  return nLines;
}

/*******************************************************************************
 * See header
 ******************************************************************************/
int Rcs_getch(void)
{
#if defined(_MSC_VER)
  RFATAL("getch undefined!");
  return 0;
#else
  static int ch = -1, fd = 0;
  struct termios neu, alt;
  fd = fileno(stdin);
  tcgetattr(fd, &alt);
  neu = alt;
  neu.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(fd, TCSANOW, &neu);
  ch = getchar();
  tcsetattr(fd, TCSANOW, &alt);
  return ch;
#endif
}

/*******************************************************************************
 * See header
 ******************************************************************************/
bool Rcs_kbhit(void)
{
#if defined(_MSC_VER)
  return (_kbhit() == 0) ? false : true;
#else
  struct termios term, oterm;
  int fd = 0;
  int c = 0;
  tcgetattr(fd, &oterm);
  memcpy(&term, &oterm, sizeof(term));
  term.c_lflag = term.c_lflag & (!ICANON);
  term.c_cc[VMIN] = 0;
  term.c_cc[VTIME] = 0;
  tcsetattr(fd, TCSANOW, &term);
  c = getchar();
  tcsetattr(fd, TCSANOW, &oterm);
  if (c != -1)
  {
    ungetc(c, stdin);
  }
  return ((c != -1) ? true : false);
#endif
}

/*******************************************************************************
 * See header
 ******************************************************************************/
#if defined (_MSC_VER)
void Rcs_setCursorPosition(int x, int y)
{
  HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
  COORD c;
  c.X = x;
  c.Y = y;
  SetConsoleCursorPosition(h, c);
  return;
}
#else
void Rcs_setCursorPosition(int x, int y)
{
}
#endif

/*******************************************************************************
 * Gradient test routine written by Marc Toussaint.
 * J is analytic solution, JJ is numerical estimate.
 *
 * I is dimension of vector wrt to which is differentiated
 * Finite difference is I*O, O is dimension of vector x
 ******************************************************************************/
static void printArr(double* x, int d0, int d1)
{
  int i, j;
  for (i = 0; i < d0; i++)
  {
    for (j = 0; j < d1; j++)
    {
      fprintf(stderr, " %+5.3f", x[i * d1 + j]);
    }
    fprintf(stderr, "\n");
  }
  fprintf(stderr, "\n");
}

static void printDeltaArr(double* x0, double* x1, int d0, int d1)
{
  int i, j;
  for (i = 0; i < d0; i++)
  {
    for (j = 0; j < d1; j++)
    {
      fprintf(stderr, " %+5.3f", x0[i * d1 + j] - x1[i * d1 + j]);
    }
    fprintf(stderr, "\n");
  }
  fprintf(stderr, "\n");
}

static bool Rcs_testGradient2(void (*f)(double*, const double*, void*),
                              void (*df)(double*, const double*, void*),
                              void* data,
                              const double* x,
                              int dimX,
                              int dimY,
                              double tolerance,
                              double* grad,
                              bool verbose)
{
  double eps = 1.0e-5, md = 0.0, mDenom=1.0, d;
  double* y  = RNALLOC(dimY, double);
  double* J  = RNALLOC(dimX * dimY, double);
  double* dx = RNALLOC(dimX, double);
  double* dy = RNALLOC(dimY, double);
  double* JJ = RNALLOC(dimX * dimY, double);
  int i, k, m = 0;
  bool failure = false;
  double dtNumeric, dtAnalytic;

  // df goes first since we randomly initialize some global variables that
  // are referenced in f.
  dtAnalytic = Timer_getTime();
  df(J, x, data);
  dtAnalytic = Timer_getTime() - dtAnalytic;

  // Find the max. absolut element of the analytic derivative. It is
  // used to compute the relative error of each finite difference
  // approximation. We add a tiny value to avoid divisions by zero
  // later.
  double maxAbsEleJ = fabs(J[0]);

  for (int i = 0; i < dimX*dimY; i++)
  {
    if (fabs(J[i]) > maxAbsEleJ)
    {
      maxAbsEleJ = fabs(J[i]);
    }
  }

  // From here, the gradient is approximated numerically.
  dtNumeric = Timer_getTime();
  f(y, x, data);

  // The memory alignment is:
  // df1/dx1   df1/dx2   df1/dx2 ... df1/dxI
  // df2/dx1   df2/dx2   df2/dx2 ... df2/dxI
  //                  ...
  // dfO/dx1   dfO/dx2   dfO/dx2 ... dfO/dxI
  for (i = 0; i < dimX; i++)
  {
    NLOG(0, "Finite difference pass %d from %d - computing %d values",
         i, dimX, dimY);
    for (k = 0; k < dimX; k++)
    {
      dx[k] = x[k];
    }
    dx[i] += eps;
    f(dy, dx, data);

    for (k = 0; k < dimY; k++)
    {
      JJ[k * dimX + i] = (dy[k] - y[k]) / eps;
    }
  }
  dtNumeric = Timer_getTime() - dtNumeric;

  if (grad != NULL)
  {
    memmove(grad, JJ, dimX*dimY*sizeof(double));
  }

  for (k = 0; k < dimX * dimY; k++)
  {
    // Calculate the approximation error. We refer to the max. of absolute
    // values of the analytic and numeric gradients. In order to avoid
    // numerical artefacts, we impose a minimum norm for the denominator.
    // This also avoids divisions by zero if the gradients are 0. The
    // denominator is always larger than epsDenom. If the largest element
    // of the gradient is larger than 1, the denominator is scaled with
    // it. This is an implicit normalization of the gradient and avoids
    // numerical effects if the gradient magnitude is very large.
    const double epsDenom = 1.0e-3;
    double denom = fabs(J[k]) > fabs(JJ[k]) ? fabs(J[k]) : fabs(JJ[k]);
    double scaleMag = maxAbsEleJ > 1.0 ? maxAbsEleJ : 1.0;

    if (denom < scaleMag*epsDenom)
    {
      denom = scaleMag*epsDenom;
    }

    d = fabs((J[k] - JJ[k])/denom);

    if (d > md)
    {
      md = d;
      m = k;
      mDenom = denom;
    }
  }

  if ((md > tolerance) || (RcsLogLevel > 5))
  {
    if (md > tolerance)
    {
      failure = true;
    }

    if (verbose==true)
    {
      REXEC(1)
      {
        fprintf(stderr, "max diff = %g\n analytic \t numeric\n", md);

        if (dimX * dimY < 1000)
        {
          printArr(J, dimY, dimX);
          printArr(JJ, dimY, dimX);
          printDeltaArr(J, JJ, dimY, dimX);
        }
        else
        {
          fprintf(stderr, "  <<too large to display>>\n");
        }

        if (md > tolerance)
        {
          fprintf(stderr, "gradient test FAILURE (max err=%g %%, "
                  "tolerance=%g, %g!=%g for element %d: row %.0f, "
                  "col %.0f denominator=%g)\n...",
                  100.0*md, tolerance, J[m], JJ[m], m,
                  floor((double)m/dimX)+1,  m-floor((double)m/dimX)*dimX+1,
                  mDenom);
          RPAUSE();
        }

      }

    }

    // Reset the internal state after key hit (display!)
    f(y, x, data);

    if (verbose==true)
    {
      REXEC(3)
      {
        fprintf(stderr, "analytic: %.3f msec numeric: %.3f msec",
                dtAnalytic * 1.0e3, dtNumeric * 1.0e3);
      }
    }

    RFREE(y);
    RFREE(J);
    RFREE(dx);
    RFREE(dy);
    RFREE(JJ);

    return !failure;
  }

  // Reset the internal state
  f(y, x, data);

  if (verbose==true)
  {
    fprintf(stderr, "gradient test SUCCESS! ");

    REXEC(4)
    {
      fprintf(stderr, "analytic: %.3f msec numeric: %.3f msec",
              dtAnalytic * 1.0e3, dtNumeric * 1.0e3);

      REXEC(5)
      {
        fprintf(stderr, " max ele: %.4f", maxAbsEleJ);
      }
    }

    fprintf(stderr, " max err=%g %%)\n", 100.0*md);
  }

  RFREE(y);
  RFREE(J);
  RFREE(dx);
  RFREE(dy);
  RFREE(JJ);

  return !failure;
}

bool Rcs_testGradient(void (*f)(double*, const double*, void*),
                      void (*df)(double*, const double*, void*),
                      void* data,
                      const double* x,
                      int dimX,
                      int dimY,
                      double tolerance,
                      bool verbose)
{
  return Rcs_testGradient2(f, df, data, x, dimX, dimY, tolerance, NULL,
                           verbose);
}

/*******************************************************************************
 * Prints some statistics of the computer.
 ******************************************************************************/
void Rcs_printComputerStats(FILE* out)
{
  pthread_attr_t attr;
  size_t stackSize;

  fprintf(out, "[%s: %s(%d)]:\n", __FILE__, __FUNCTION__, __LINE__);

#if !defined(_MSC_VER)
  // Get hostname
  char hostName[256];
  int success = gethostname(hostName, 256);

  if (success == -1)
  {
    fprintf(out, "Failed to get hostname\n");
  }
  else
  {
    fprintf(out, "Hostname is \"%s\"\n", hostName);
  }
#endif

#if !defined(_MSC_VER)
  fprintf(out, "Number of cores: %ld\n",  sysconf(_SC_NPROCESSORS_ONLN));
  fprintf(out, "Pagesize [bytes]: %ld\n", sysconf(_SC_PAGESIZE));
#else
  SYSTEM_INFO sysinfo;
  GetSystemInfo(&sysinfo);

  fprintf(out, "Hardware information: \n");
  fprintf(out, "  OEM ID: %u\n", sysinfo.dwOemId);
  fprintf(out, "  Number of processors: %u\n",
          sysinfo.dwNumberOfProcessors);
  fprintf(out, "  Page size: %u\n", sysinfo.dwPageSize);
  fprintf(out, "  Processor type: %u\n", sysinfo.dwProcessorType);
  fprintf(out, "  Minimum application address: %lx\n",
          (unsigned int)sysinfo.lpMinimumApplicationAddress);
  fprintf(out, "  Maximum application address: %lx\n",
          (unsigned int)sysinfo.lpMaximumApplicationAddress);
  fprintf(out, "  Active processor mask: %u\n",
          (unsigned int) sysinfo.dwActiveProcessorMask);
#endif

  // Stack size
  if (pthread_attr_init(&attr) != 0)
  {
    perror("error in pthread_attr_init");
    return;
  }

  if (pthread_attr_getstacksize(&attr, &stackSize) != 0)
  {
    perror("error in pthread_attr_getstackstate()");
    pthread_attr_destroy(&attr);
    return;
  }
  printf("Stack size [doubles]: %d\n", (int)(stackSize/sizeof(double)));

  if (pthread_attr_destroy(&attr) != 0)
  {
    perror("error in pthread_attr_destroy");
    return;
  }

}

/*******************************************************************************
 * Prints pthread settings to a file descriptor.
 ******************************************************************************/
void Rcs_printThreadInfo(FILE* out, const char* prefix)
{
#if !defined(_MSC_VER)
  pthread_t thisThread = pthread_self();
  pthread_attr_t thisThreadAttr;
  int s = pthread_getattr_np(thisThread, &thisThreadAttr);
  if (s != 0)
  {
    RLOG(1, "pthread_getattr_np: Error \"%s\"", strerror(s));
  }
  else
  {
    Rcs_printThreadAttributes(out, &thisThreadAttr, prefix);
  }
#else
  RLOG(1, "Not available on Windows");
#endif
}

/*******************************************************************************
 * Prints pthread settings to a file descriptor.
 ******************************************************************************/
void Rcs_printThreadAttributes(FILE* out, pthread_attr_t* attr,
                               const char* prefix)
{
  int s, i;
  struct sched_param sp;

  RCHECK(attr);

  s = pthread_attr_getdetachstate(attr, &i);
  if (s != 0)
  {
    RLOG(1, "pthread_attr_getdetachstate: Error \"%s\"", strerror(s));
  }
  else
  {
    fprintf(out, "%sDetach state        = %s\n", prefix,
            (i == PTHREAD_CREATE_DETACHED) ? "PTHREAD_CREATE_DETACHED" :
            (i == PTHREAD_CREATE_JOINABLE) ? "PTHREAD_CREATE_JOINABLE" :
            "???");
  }

  s = pthread_attr_getscope(attr, &i);
  if (s != 0)
  {
    RLOG(1, "pthread_attr_getscope: Error \"%s\"", strerror(s));

  }
  else
  {
    fprintf(out, "%sScope               = %s\n", prefix,
            (i == PTHREAD_SCOPE_SYSTEM)  ? "PTHREAD_SCOPE_SYSTEM" :
            (i == PTHREAD_SCOPE_PROCESS) ? "PTHREAD_SCOPE_PROCESS" :
            "???");
  }

  s = pthread_attr_getinheritsched(attr, &i);
  if (s != 0)
  {
    RLOG(1, "pthread_attr_getinheritsched: Error \"%s\"", strerror(s));
  }
  else
  {
    fprintf(out, "%sInherit scheduler   = %s\n", prefix,
            (i == PTHREAD_INHERIT_SCHED)  ? "PTHREAD_INHERIT_SCHED" :
            (i == PTHREAD_EXPLICIT_SCHED) ? "PTHREAD_EXPLICIT_SCHED" :
            "???");
  }

  s = pthread_attr_getschedpolicy(attr, &i);
  if (s != 0)
  {
    RLOG(1, "pthread_attr_getschedpolicy: Error \"%s\"", strerror(s));
  }
  else
  {
    fprintf(out, "%sScheduling policy   = %s\n", prefix,
            (i == SCHED_OTHER) ? "SCHED_OTHER" :
            (i == SCHED_FIFO)  ? "SCHED_FIFO" :
            (i == SCHED_RR)    ? "SCHED_RR" :
            "???");
  }

  s = pthread_attr_getschedparam(attr, &sp);
  if (s != 0)
  {
    RLOG(1, "pthread_attr_getschedparam: Error \"%s\"", strerror(s));
  }
  else
  {
    fprintf(out, "%sScheduling priority = %d\n", prefix, sp.sched_priority);
  }

#if !defined(_MSC_VER)
  size_t v;
  void* stkaddr;

  s = pthread_attr_getguardsize(attr, &v);
  if (s != 0)
  {
    RLOG(1, "pthread_attr_getguardsize: Error \"%s\"", strerror(s));
  }
  else
  {
    fprintf(out, "%sGuard size          = %lu bytes\n", prefix, v);
  }

  s = pthread_attr_getstack(attr, &stkaddr, &v);
  if (s != 0)
  {
    RLOG(1, "pthread_attr_getstack: Error \"%s\"", strerror(s));
  }
  else
  {
    fprintf(out, "%sStack address       = %p\n", prefix, stkaddr);
    fprintf(out, "%sStack size          = 0x%zx (%lu) bytes\n", prefix, v, v);
  }
#endif
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs_getRealTimeThreadAttribute(int schedulingPolicy, int prio,
                                    pthread_attr_t* attrRT)
{
  if ((schedulingPolicy!=SCHED_RR) && (schedulingPolicy!=SCHED_FIFO))
  {
    RLOG(1, "Scheduling policy must be SCHED_RR (%d) or SCHED_FIFO (%d), but "
         "is %d", SCHED_RR, SCHED_FIFO, schedulingPolicy);
    return false;
  }

  // Now we set the thread attributes to real-time
  pthread_attr_init(attrRT);
  int res = pthread_attr_setinheritsched(attrRT, PTHREAD_EXPLICIT_SCHED);
  if (res!=0)
  {
    RLOG(1, "pthread_attr_setinheritsched: %s", strerror(res));
    return false;
  }

  res = pthread_attr_setschedpolicy(attrRT, schedulingPolicy);
  if (res!=0)
  {
    RLOG(1, "pthread_attr_setschedpolicy: %s", strerror(res));
    return false;
  }


  int prioMin = sched_get_priority_min(schedulingPolicy);
  int prioMax = sched_get_priority_max(schedulingPolicy);

  struct sched_param param;
  param.sched_priority = prio;

  if (param.sched_priority < prioMin)
  {
    RLOG(1, "Prio min: %d   prio: %d   prio max: %d - clipping to prio min",
         prioMin, prio, prioMax);
    param.sched_priority = prioMin;
  }
  else if (param.sched_priority > prioMax)
  {
    RLOG(1, "Prio min: %d   prio: %d   prio max: %d - clipping to prio max",
         prioMin, prio, prioMax);
    param.sched_priority = prioMax;
  }

  res = pthread_attr_setschedparam(attrRT, &param);
  if (res!=0)
  {
    RLOG(1, "pthread_attr_setschedparam: %s", strerror(res));
    return false;
  }

  // It is not advisable to set the stack size according to www.
  // res = pthread_attr_setstacksize(attr, 4000000);  // 4MB
  res = pthread_attr_setdetachstate(attrRT, PTHREAD_CREATE_JOINABLE);
  if (res!=0)
  {
    RLOG(1, "pthread_attr_setdetachstate: %s", strerror(res));
    return false;
  }

  return true;
}

/*******************************************************************************
 * See header
 ******************************************************************************/
char* File_createUniqueName(char* fileName,
                            const char* pattern, const char* suffix)
{
#if !defined(_MSC_VER)
  int fd = -1;

  if (pattern==NULL)
  {
    strcpy(fileName, "tmpfile");
  }
  else
  {
    strcpy(fileName, pattern);
  }
  strcat(fileName, "XXXXXX");

  if (suffix != NULL)
  {
    strcat(fileName, ".");
    strcat(fileName, suffix);
    fd = mkstemps(fileName, 4);
  }
  else
  {
    fd = mkstemp(fileName);
  }

  if (fd == -1)
  {
    strcpy(fileName, "");
    return NULL;
  }

  close(fd);
  unlink(fileName);
#else
  //strcpy(fileName, "C:\\temp");   // Slash added by tmpnam
  strcpy(fileName, "C:/temp/");
  const char* tmp = tmpnam(NULL);
  strcat(fileName, &tmp[1]);   // Ignore back slash at index 0 added by tmpnam
  if (suffix != NULL)
  {
    strcat(fileName, suffix);
  }
  else
  {
    strcat(fileName, "tmp");
  }
  RLOG(5, "tmp-file is \"%s\"   tmpnam is \"%s\"", fileName, tmp);
#endif

  return fileName;
}

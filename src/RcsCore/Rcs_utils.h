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

#ifndef RCS_UTILS_H
#define RCS_UTILS_H

/*!
 *  \page RcsCore Basic utility functions
 *
 *  <h1> Core functions </h1>
 *
 *  \ref RcsMacros
 *
 *  \ref RcsParserFunctions
 *
 *  \ref RcsTimerFunctions
 *
 *  \ref RcsUtilsFunctions
 *
 *  \ref Rcs::CmdLineParser "Parser for command line arguments"
 *
 *  \ref ResourcePathFunctions
 */

#include "Rcs_bool.h"

#include <stdio.h>
#include <signal.h>
#include <pthread.h>


#ifdef __cplusplus
extern "C" {
#endif


/*!
 * \defgroup RcsUtilsFunctions Miscellaneous utility functions
 *
 */



/**
 * @name Convenience functions for handling character arrays
 */

///@{

/*! \ingroup RcsUtilsFunctions
 *  \brief Returns an (deep) copy of src from the heap. You have to take care
 *         that it gets deleted. If src is NULL, no memory will be allocated
 *         and NULL is returned. If the memory allocation fails, the function
 *         exits with a fatal error. That's the difference to strdup.
 */
char* String_clone(const char* src);

/*! \ingroup RcsUtilsFunctions
 *  \brief Copies the character array in src into the de-referenced dst.
 *         If that is NULL, it will be allocated and pointed to by dst.
 *         If it already exists but has not enough memory for src, it is
 *         re-alloced and pointed to. Otherwise, it is just copied.
 *         If src is NULL, the memory pointed to by dst will be freed
 *         (if it was allocated) and set to NULL.
 *
 *  \param[out] dst    Pointer to target character array. The modified array
 *                     may have a different memory adress after re-allocation,
 *                     therefore we need to pass a pointer here.
 *  \param[in]  src    Character array to be copied into dst.
 */
void String_copyOrRecreate(char** dst, const char* src);

/*! \ingroup RcsUtilsFunctions
 *  \brief Prepends t into s. Assumes s has enough space allocated for the
 *         combined string. Example: before s="abc", t="123" after: s="123abc"
 */
void String_prepend(char* s, const char* t);

/*! \ingroup RcsUtilsFunctions
 *  \brief Checks if str has a given ending.
 *
 *  \param[in] str   String to be checked. It must be terminated with a
 *                   trailing zero, or NULL. In the latter case, the
 *                   result is always false.
 *  \param[in] ending   Ending to be checked. Must be a valid string (with
 *                      trailing zero). If the length of ending is larger than
 *                      str, the function returns false. If ending is NULL,
 *                      also false is returned.
 *  \param[in] caseSensitive   If true, the case of the strings is considered.
 *  \return True if ending matches, false otherwise.
 */
bool String_hasEnding(const char* str, const char* ending, bool caseSensitive);

/*! \ingroup RcsUtilsFunctions
 *  \brief Converts a double value to a string with a maximum of maxDigits
 *         digits after the dot. Trailing zeroes are removed.
 *
 *  \param[out] str    Char pointer holding the value as a string. Must be long
 *                     enough.
 *  \param[in]  value  Value to be converted into string
 *  \param maxDigits   Max. number of digits after the point.
 *  \return Pointer to str
 */
char* String_fromDouble(char* str, double value, unsigned int maxDigits);

/*! \ingroup RcsUtilsFunctions
 * \brief Convenience function to convert a string to bool:
 *        - Any upper- or lower case combination of the letters of true
 *          (e.g. "true", "True", "TRUE", "TrUe") returns true
 *        - Everything else returns false.
 */
bool String_toBool(const char* str);

/*! \ingroup RcsUtilsFunctions
 * \brief Returns the number of strings in str that are separated by
 *        delim.
 */
unsigned int String_countSubStrings(const char* str, const char* delim);

/*! \ingroup RcsUtilsFunctions
 * \brief Searches through the string for any environment variables that are
 *        contained in the ${ and }. If found, they are expanded with the
 *        getenv() function. A string with the expanded environment variables
 *        is returned. It is allocated with malloc, and the caller is
 *        responsible to free its memory. If argument str is NULL, the function
 *        returns NULL.
 */
char* String_expandEnvironmentVariables(const char* str);

///@}



/**
 * @name Convenience functions for file I/O
 */

///@{

/*! \ingroup RcsUtilsFunctions
 *  \brief Creates an unique file name based on the character array template.
 *         Example: template is "/tmp/myFile" and suffix is "dat": result is
 *                  "tmp/myFile123456.dat" where 123456 makes the file name
 *                  unique.
 *
 *  \param[out] fileName   Target array holding the unique file name. If it is
 *                         NULL, "tmpfile" is taken as default. It must
 *                         contain enough memory (length of template + suffix
 *                         + 6 characters), see mkstemp for details. If no
 *                         unique file name can be found, the character array
 *                         will be empty, and false is returned.
 *  \param[in] pattern     Char pointer holding the file name template. Must
 *                         not be NULL. The template may contain the full path,
 *                         or a relative path.
 *  \param[in] suffix      If suffix is not NULL, it will be appended to the
 *                         filename. The dot must not be part of the suffix.
 *  \return                Pointer to fileName for success, NULL for failure.
 */
char* File_createUniqueName(char* fileName, const char* pattern,
                            const char* suffix);

/*! \ingroup RcsUtilsFunctions
 *  \brief Returns true if the given file exists (meaning being readable
 *         by the user), false otherwise. If filename is NULL, the function
 *         returns false.
 */
bool File_exists(const char* filename);

/*! \ingroup RcsUtilsFunctions
 *  \brief Returns the number of bytes of a file. If the file doesn't exist
 *         or cannot be opened, the function returns 0 and prints a warning
 *         on debug level 4.
 */
long File_getSize(const char* name);

/*! \ingroup RcsUtilsFunctions
 *  \brief Returns the number of lines of a file. If the file doesn't exist
 *         or cannot be opened, the function returns 0 and prints a warning
 *         on debug level 4.
 */
long File_getLineCount(const char* fileName);

///@}



/*! \ingroup RcsUtilsFunctions
 *  \brief Get the pressed key if kbhit() == true
 */
int Rcs_getch(void);

/*! \ingroup RcsUtilsFunctions
 *  \brief Non-blocking keypress checking
 */
bool Rcs_kbhit(void);

/*! \ingroup RcsUtilsFunctions
 *  \brief Sets the cursor to the x and y position in the console. (0,0) is
 *         top left. This only works for MSVC.
 */
void Rcs_setCursorPosition(int x, int y);

/*! \ingroup RcsUtilsFunctions
 *  \brief Finite difference test function. Returns true if the finite
 *         difference test failed due to a larger error than given in
 *         tolerance, false otherwise. The finite difference approximation
 *         of the gradient is of dimension dimY x dimX.
 *
 *  \param[in] f  Function pointer to cost function
 *  \param[in] df Function pointer to gradient function
 *  \param[in] param User pointer to underlying struct (NULL if none)
 *  \param[in] x State vector
 *  \param[in] dimX Dimension of vector x (Number of columns of the gradient
 *             matrix if x is a dimY x 1 vector)
 *  \param[in] dimY Dimension of vector wrt to which is differentiated (Number
 *             of rows of the gradient matrix if x is a dimY x 1 vector)
 *  \param[in] tolerance Threshold of largest error to pass test
 *  \param[in] verbose If true, debug information is printed to the console
 *  \return true for success, false otherwise
 */
bool Rcs_testGradient(void (*f)(double* f, const double* x, void* param),
                      void (*df)(double* f, const double* x, void* param),
                      void* param, const double* x, int dimX, int dimY,
                      double tolerance, bool verbose);

/*! \ingroup RcsUtilsFunctions
 *  \brief Prints out some system-specific specifics about the computer
 *         the function is called from to the given file descriptor. For
 *         Linux, it is
 *         - the hostName
 *         - the number of cores
 *         - the pagesize
 *         - the stacksize
 *         For windows, only a few of the above are available.
 */
void Rcs_printComputerStats(FILE* out);




/**
 * @name Convenience functions for threading with pthreads
 */

///@{

/*! \ingroup RcsUtilsFunctions
 *  \brief Prints the thread's information to the file descriptor. This works
 *         for Posix threads only.
 *         - Detach state
 *         - Scope
 *         - Inherit scheduler
 *         - Scheduling policy
 *         - Scheduling priority
 *         - Guard size
 *         - Stack address
 *         - Stack size
 */
void Rcs_printThreadInfo(FILE* out, const char* prefix);

/*! \ingroup RcsUtilsFunctions
 *  \brief Prints the attribute's information to the file descriptor. This
 *         works for Posix threads only.
 *         - Detach state
 *         - Scope
 *         - Inherit scheduler
 *         - Scheduling policy
 *         - Scheduling priority
 *         - Guard size
 *         - Stack address
 *         - Stack size
 */
void Rcs_printThreadAttributes(FILE* out, pthread_attr_t* attr,
                               const char* prefix);

/*! \ingroup RcsUtilsFunctions
 *  \brief Creates a thread attribute given a scheduling policy and a priority
 *         value.
 *
 *  \param[in] schedulingPolicy Must be SCHED_RR or SCHED_FIFO
 *  \param[in] priority Thread priority value, should be between the
 *                      scheduler's minumum and maximum priority. If
 *                      it is not within these values, it will be clipped,
 *                      and a warning message is issued on debug level 1.
 *  \param[out] attr Pointer to thread attribute to be modified. It must not
 *                   be NULL.
 *  \return True for success, false otherwise. In case of failure, the argument
 *          attr will be initialized with the default values, and a warning
 *          will be shown at debug level 1.
 */
bool Rcs_getRealTimeThreadAttribute(int schedulingPolicy, int priority,
                                    pthread_attr_t* attr);

///@}



#ifdef __cplusplus
}
#endif

#endif   // RCS_UTILS_H

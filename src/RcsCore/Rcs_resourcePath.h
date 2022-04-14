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

#ifndef RCS_RESOURCEPATH_H
#define RCS_RESOURCEPATH_H


/*!
 * \defgroup ResourcePathFunctions Functions for handling global resource paths
 *
 *           These functinos organizes a set of resource paths. All
 *           configuration files are searched within the set of resource paths,
 *           in the order of how they are added to the overal path variable.
 *           Always the first path to be searched is the current working
 *           directory. Functions to add paths to the overall resource path
 *           variable are <br>
 *
 *           - \ref Rcs_addResourcePath() to append a path
 *           - \ref Rcs_insertResourcePath() to prepend a path
 *
 *           The function \ref Rcs_getAbsoluteFileName() accepts a file name,
 *           searches through all resource paths in their order, and copies
 *           the first full path to the file that is found to the output
 *           argument.
 */


#ifdef __cplusplus
extern "C" {
#endif // __cplusplus




/*! \ingroup ResourcePathFunctions
 *  \brief Clears the resource path from all elements.
 */
void Rcs_clearResourcePath(void);

/*! \ingroup ResourcePathFunctions
 *  \brief Appends argument path to the last element of the resource path
 *         list.
 *  \return True for success, false otherwise:
 *          - The path has already been added
 *          - Argument path is NULL for some reason.
 */
bool Rcs_addResourcePath(const char* path);

/*! \ingroup ResourcePathFunctions
 *  \brief Inserts argument path to be the first element in the resource
 *         path list.
 */
bool Rcs_insertResourcePath(const char* path);

/*! \ingroup ResourcePathFunctions
 *  \brief Returns the pointer to the resource path at the given index.
 *         If the index is out of bounds, NULL is returned. Please note that
 *         when after obtaining the pointer, the resource paths are changed,
 *         the pointer is not valid any more.
 */
const char* Rcs_getResourcePath(unsigned int index);

/*! \ingroup ResourcePathFunctions
 *  \brief Removes the given path from the rsource paths. All occurances are
 *         removed. The function returns false if path is NULL, or it was
 *         not found in the resource path array.
 */
bool Rcs_removeResourcePath(const char* path);

/*! \ingroup ResourcePathFunctions
 *  \brief For a given file name, the absolute file name is searched in the
 *         resource path list. The first match is copied to absFileName,
 *         and "true" is returned. If it is not found, absFileName is not
 *         changed, and "false" is returned. The function searches through the
 *         resource paths in the order they have been added, and then in
 *         the current directory. In case absFileName is NULL, no copying takes
 *         place.
 *
 *  \return True if a path has been found, false if not, or if fileName is NULL.
 */
bool Rcs_getAbsoluteFileName(const char* fileName, char* absFileName);

/*! \ingroup ResourcePathFunctions
 *  \brief Returns true if the file is found in any of the resource paths or in
 *         the current directory, false otherwise.
 */
bool Rcs_fileInResourcePath(const char* fileName);

/*! \ingroup ResourcePathFunctions
 *  \brief Prints out the resource paths to stderr.
 */
void Rcs_printResourcePath(void);
/*! \ingroup ResourcePathFunctions
 *  \brief Returns the number of registered resource paths.
 */
unsigned int Rcs_numResourcePaths(void);



#ifdef __cplusplus
}
#endif // __cplusplus



#endif   // RCS_RESOURCEPATH_H

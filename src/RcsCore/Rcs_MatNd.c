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

#include "Rcs_MatNd.h"
#include "Rcs_Mat3d.h"
#include "Rcs_VecNd.h"
#include "Rcs_Vec3d.h"
#include "Rcs_basicMath.h"
#include "Rcs_macros.h"
#include "Rcs_timer.h"
#include "Rcs_utils.h"

#include <float.h>



#ifdef MATND_DEBUG_STACK
unsigned int MatNd_stackBytes = 0;
unsigned int MatNd_maxStackBytes = 0;
#endif



/*******************************************************************************
 *
 * \brief Inverse of a 2 x 2 matrix.
 *
 * | a11 a12 |-1             |  a22 -a12 |
 * | a21 a22 |    =  1/det * | -a21  a11 |
 *
 * with det  =  a11*a22 - a12*a21
 *
 ******************************************************************************/
double MatNd_inverse2D(MatNd* invA, const MatNd* A)
{
  RCHECK_MSG((A->m == 2) && (A->n == 2), "A->m=%d   A->n=%d", A->m, A->n);

  double a11 = A->ele[0];
  double a12 = A->ele[1];
  double a21 = A->ele[2];
  double a22 = A->ele[3];
  double det = a11 * a22 - a12 * a21;

  MatNd_reshape(invA, 2, 2);

  if (det != 0.0)
  {
    invA->ele[0] = a22 / det;
    invA->ele[1] = -a12 / det;
    invA->ele[2] = -a21 / det;
    invA->ele[3] = a11 / det;
  }
  else
  {
    MatNd_setZero(invA);
  }

  return det;
}

/*******************************************************************************
 * In-place inversion for convenience
 ******************************************************************************/
double MatNd_inverse2DSelf(MatNd* A)
{
  return MatNd_inverse2D(A, A);
}

/*******************************************************************************
 *
 ******************************************************************************/
MatNd* MatNd_create(unsigned int m, unsigned int n)
{
  MatNd* self = RALLOC(MatNd);
  RCHECK(self);
  self->size = m * n > 0 ? m * n : 1; // If size is 0, ele will be NULL
  self->m = m;
  self->n = n;
  self->stackMem = false;
  self->ele = RNALLOC(self->size, double);
  RCHECK_MSG(self->ele, "Failed to allocate %u double values", self->size);

  return self;
}

/*******************************************************************************
 *
 ******************************************************************************/
MatNd* MatNd_createLike(const MatNd* src)
{
  return MatNd_create(src->m, src->n);
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_createLike2(MatNd* dst, const MatNd* src)
{
  MatNd_create2(dst, src->m, src->n);
}

/*******************************************************************************
 *
 ******************************************************************************/
MatNd* MatNd_clone(const MatNd* src)
{
  if (src==NULL)
  {
    return NULL;
  }

  MatNd* self = RALLOC(MatNd);
  RCHECK(self);
  self->size = src->size;
  self->m = src->m;
  self->n = src->n;
  self->ele = RNALLOC(src->size, double);
  RCHECK_MSG(self->ele, "Failed to allocate %u double values", self->size);

  memcpy(&self->ele[0], &src->ele[0], self->size * sizeof(double));

  return self;
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_destroy(MatNd* self)
{
  if (self == NULL)
  {
    return;
  }

  if (self->stackMem == false)
  {
    RFREE(self->ele);
    RFREE(self);
  }

#ifdef MATND_DEBUG_STACK
  if (self->stackMem==true)
  {
    MatNd_stackBytes -= sizeof(MatNd) + self->size*sizeof(double);
  }
#endif
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_destroyN(unsigned int nToDestroy, ...)
{
  va_list argPtr;
  va_start(argPtr, nToDestroy);

  for (unsigned int i = 0; i < nToDestroy; ++i)
  {
    MatNd* toDestroy = va_arg(argPtr, MatNd*);
    MatNd_destroy(toDestroy);
  }

  va_end(argPtr);
}

/*******************************************************************************
 *
 ******************************************************************************/
MatNd* MatNd_realloc(MatNd* self, unsigned int m, unsigned int n)
{
  if (self == NULL)
  {
    return MatNd_create(m, n);
  }

  if (m * n > self->size) // realloc only if neccessary
  {
    RCHECK(self->stackMem == false);
    self->size = m * n;
    self->ele = (double*) realloc(self->ele, self->size * sizeof(double));
    RCHECK(self->ele);

    // set new elements to zero
    memset(self->ele + self->m * self->n, 0,
           (m * n - self->m * self->n) * sizeof(double));
  }

  self->m = m;
  self->n = n;

  return self;
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_print(const MatNd* M)
{
  unsigned int i, j;

  for (i = 0; i < M->m; i++)
  {
    for (j = 0; j < M->n; j++)
    {
      fprintf(stderr, "%g ", M->ele[M->n * i + j]);
    }
    fprintf(stderr, "\n");
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_printTranspose(const MatNd* M)
{
  unsigned int i, j;

  for (i = 0; i < M->n; i++)
  {
    for (j = 0; j < M->m; j++)
    {
      fprintf(stderr, "%5g ", M->ele[M->n * j + i]);
    }
    fprintf(stderr, "\n");
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_printDetail(const MatNd* M)
{
  unsigned int i, j;

  fprintf(stderr, "Array is %u x %u, size is %u (m*n = %u)\n",
          M->m, M->n, M->size, M->m * M->n);

  for (i = 0; i < M->m; i++)
  {
    for (j = 0; j < M->n; j++)
    {
      fprintf(stderr, "%+5.2f ", M->ele[M->n * i + j]);
    }
    fprintf(stderr, "\n");
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_printDigits(const MatNd* M, int digits)
{
  unsigned int i, j;
  char formatStr[16];

  snprintf(formatStr, 16, "%%+5.%df ", digits);

  for (i = 0; i < M->m; i++)
  {
    for (j = 0; j < M->n; j++)
    {
      fprintf(stderr, formatStr, M->ele[M->n * i + j]);
    }
    fprintf(stderr, "\n");
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_printComment(const char* comment, const MatNd* M)
{
  unsigned int i, j;

  if (comment != NULL)
  {
    fprintf(stderr, "%s\n", comment);
  }

  for (i = 0; i < M->m; i++)
  {
    for (j = 0; j < M->n; j++)
    {
      fprintf(stderr, "%g ", M->ele[M->n * i + j]);
    }
    fprintf(stderr, "\n");
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_printCommentDigits(const char* text,
                              const MatNd* M,
                              unsigned int digits)
{
  unsigned int i, j;
  char formatStr[16];

  if (text != NULL)
  {
    fprintf(stderr, "%s\n", text);
  }

  snprintf(formatStr, 16, "%%+5.%df ", digits);

  for (i = 0; i < M->m; i++)
  {
    for (j = 0; j < M->n; j++)
    {
      fprintf(stderr, formatStr, M->ele[M->n * i + j]);
    }
    fprintf(stderr, "\n");
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_printFormatted(const char* text, const char* format, const MatNd* M)
{
  unsigned int i, j;

  if (text != NULL)
  {
    fprintf(stderr, "%s\n", text);
  }

  for (i = 0; i < M->m; i++)
  {
    for (j = 0; j < M->n; j++)
    {
      fprintf(stderr, format, M->ele[M->n * i + j]);
    }
    fprintf(stderr, "\n");
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_printDims(const char* comment, const MatNd* M)
{
  fprintf(stderr, "%s is %u x %u (size %u)\n", comment, M->m, M->n, M->size);
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_printTwoArrays(const MatNd* v1, const MatNd* v2, int digits)
{
  unsigned int i, j;
  char formatStr[16];

  RCHECK_MSG(v1->m == v2->m, "Row numbers differ: %d != %d", v1->m, v2->m);
  snprintf(formatStr, 16, "%%+.%df ", digits);

  for (i = 0; i < v1->m; i++)
  {
    for (j = 0; j < v1->n; j++)
    {
      fprintf(stderr, formatStr, MatNd_get(v1, i, j));
    }

    fprintf(stderr, "   ");

    for (j = 0; j < v2->n; j++)
    {
      fprintf(stderr, formatStr, MatNd_get(v2, i, j));
    }

    fprintf(stderr, "\n");
  }
  fprintf(stderr, "\n");

}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_printThreeArrays(const MatNd* v1, const MatNd* v2, const MatNd* v3,
                            int digits)
{
  unsigned int i, j;
  char formatStr[16];

  RCHECK_MSG(v1->m == v2->m, "Row numbers differ: %d != %d", v1->m, v2->m);
  RCHECK_MSG(v1->m == v3->m, "Row numbers differ: %d != %d", v1->m, v3->m);
  snprintf(formatStr, 16, "%%+.%df ", digits);

  for (i = 0; i < v1->m; i++)
  {
    for (j = 0; j < v1->n; j++)
    {
      fprintf(stderr, formatStr, MatNd_get(v1, i, j));
    }

    fprintf(stderr, "   ");

    for (j = 0; j < v2->n; j++)
    {
      fprintf(stderr, formatStr, MatNd_get(v2, i, j));
    }

    fprintf(stderr, "   ");

    for (j = 0; j < v3->n; j++)
    {
      fprintf(stderr, formatStr, MatNd_get(v3, i, j));
    }

    fprintf(stderr, "\n");
  }
  fprintf(stderr, "\n");

}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_printTwoArraysDiff(const MatNd* v1, const MatNd* v2, int digits)
{
  unsigned int i, j;
  char formatStr[16];

  snprintf(formatStr, 16, "%%+.%df ", digits);

  RCHECK_MSG(v1->m == v2->m, "Row numbers differ: %d != %d", v1->m, v2->m);
  RCHECK_MSG(v1->n == v2->n, "Column numbers differ: %d != %d", v1->n, v2->n);

  for (i = 0; i < v1->m; i++)
  {
    for (j = 0; j < v1->n; j++)
    {
      fprintf(stderr, formatStr, MatNd_get(v1, i, j));
    }

    fprintf(stderr, "   ");

    for (j = 0; j < v2->n; j++)
    {
      fprintf(stderr, formatStr, MatNd_get(v2, i, j));
    }

    fprintf(stderr, "   ");

    for (j = 0; j < v2->n; j++)
    {
      fprintf(stderr, formatStr, MatNd_get(v1, i, j) - MatNd_get(v2, i, j));
    }

    fprintf(stderr, "\n");
  }
  fprintf(stderr, "\n");

}

/*******************************************************************************
 *
 ******************************************************************************/
bool MatNd_toFile(const MatNd* M, const char* fileName)
{
  FILE* fd = fopen(fileName, "w+");

  if (fd == NULL)
  {
    RLOG(1, "Error opening file \"%s\"", fileName);
    return false;
  }

  for (unsigned int i = 0; i < M->m; i++)
  {
    for (unsigned int j = 0; j < M->n - 1; j++)
    {
      fprintf(fd, "%.16f ", MatNd_get(M, i, j));
    }

    fprintf(fd, "%.16f\n", MatNd_get(M, i, M->n - 1));
  }

  fclose(fd);

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_appendToFile(const MatNd* M, const char* fileName)
{
  FILE* fd = fopen(fileName, "a");

  RCHECK_MSG(fd, "Error opening file \"%s\"", fileName);

  if ((M->m>0) && (M->n>0))
  {
    for (unsigned int i = 0; i < M->m; i++)
    {
      for (unsigned int j = 0; j < M->n - 1; j++)
      {
        fprintf(fd, "%.16f ", MatNd_get(M, i, j));
      }

      fprintf(fd, "%.16f\n", MatNd_get(M, i, M->n - 1));
    }
  }

  fclose(fd);
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_gnuplotPipes(const char* title, const MatNd* self)
{
  FILE* gnuplotPipe = NULL;

#if defined(_MSC_VER)
  gnuplotPipe = _popen("pgnuplot.exe -persist", "w");
#else
  gnuplotPipe = popen("gnuplot -persist", "w");
#endif

  // If gnuplot is found
  if (gnuplotPipe == NULL)
  {
    RLOGS(1, "Couldn't open pipe to gnuplot");
    return;
  }

  char fileName[256] = "";
  File_createUniqueName(fileName, "MatNd_", "dat");
  MatNd_toFile(self, fileName);

  // using fprintf on the gnuplotPipe, we can directly issue commands in gnuplot

  if (title != NULL)
  {
    fprintf(gnuplotPipe, "set grid\nset title \"%s\"\nplot ", title);
  }
  else
  {
    fprintf(gnuplotPipe, "set grid\n\nplot ");
  }

  for (unsigned int i=1; i<=self->n; i++)
  {
    fprintf(gnuplotPipe, "\"%s\" u ($%d) w steps title \"%d\"", fileName, i, i);

    if (i!=self->n)
    {
      fprintf(gnuplotPipe, ", ");
    }
  }
  fprintf(gnuplotPipe, "\n");

  fflush(gnuplotPipe);
  pclose(gnuplotPipe);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool MatNd_gnuplot2(const char* title, const MatNd* self)
{
  char fileName[256], gpFileName[256];

  File_createUniqueName(fileName, "MatNd_", "dat");
  File_createUniqueName(gpFileName, "MatNd_", "gnu");

  bool wroteFile = MatNd_toFile(self, fileName);

  // Warning is done in MatNd_toFile()
  if (wroteFile == false)
  {
    return false;
  }

  char gpCmd[4096], gpCmd_col[512];
  if (title != NULL)
  {
    sprintf(gpCmd, "set grid\nset title \"%s\"\nplot ", title);
  }
  else
  {
    strcpy(gpCmd, "set grid\n\nplot ");
  }

  for (unsigned int i=1; i<=self->n; i++)
  {
    RLOG(0, "row %d from %d", i, self->n);

    sprintf(gpCmd_col, "\"%s\" u ($%d) w steps title \"%d\"", fileName, i, i);
    strcat(gpCmd, gpCmd_col);

    if (i!=self->n)
    {
      strcat(gpCmd, ", ");
    }
  }
  strcat(gpCmd, "\n");


  FILE* outDat = fopen(gpFileName, "w+");
  RCHECK_MSG(outDat, "Couldn't open file \"%s\"", gpFileName);
  fprintf(outDat, "%s", gpCmd);
  fflush(outDat);
  fclose(outDat);

  char sysCallStr[300];

#if defined(_MSC_VER)
  sprintf(sysCallStr, "wgnuplot.exe -persist %s", gpFileName);
#else
  sprintf(sysCallStr, "/usr/bin/gnuplot -persist %s", gpFileName);
#endif
  int err = system(sysCallStr);

  if (err == -1)
  {
    RLOG(4, "Couldn't start gnuplot");
    return false;
  }
  else
  {
    RLOGS(5, "\n\n%s\n\n%s", gpCmd, sysCallStr);
  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool MatNd_gnuplot(const char* title, const MatNd* self)
{
  char fileName[256], gpFileName[256];

  File_createUniqueName(fileName, "MatNd_", "dat");
  File_createUniqueName(gpFileName, "MatNd_", "gnu");

  bool wroteFile = MatNd_toFile(self, fileName);

  // Warning is done in MatNd_toFile()
  if (wroteFile == false)
  {
    return false;
  }
  FILE* outDat = fopen(gpFileName, "w+");

  if (outDat == NULL)
  {
    RLOG(4, "Couldn't open file \"%s\"", gpFileName);
    return false;
  }

  char gpCmd_col[512];
  if (title != NULL)
  {
    fprintf(outDat, "set grid\nset title \"%s\"\nplot ", title);
  }
  else
  {
    fprintf(outDat, "set grid\n\nplot ");
  }
  for (unsigned int i = 1; i <= self->n; i++)
  {
    int len = snprintf(gpCmd_col, 512, "\"%s\" u ($%d) w steps title \"%d\"",
                       fileName, i, i);

    if (len >= 512)
    {
      RLOG(4, "Line buffer overflow when writing file \"%s\". Thecommand is %d"
           "characters long, but should be less than 512", gpFileName, len);
      fclose(outDat);
      return false;
    }

    fprintf(outDat, "%s", gpCmd_col);

    if (i != self->n)
    {
      fprintf(outDat, ", ");
    }
  }
  fprintf(outDat, "\n");

  fflush(outDat);
  fclose(outDat);

  char sysCallStr[300];

#if defined(_MSC_VER)
  sprintf(sysCallStr, "START \"\" wgnuplot.exe -persist %s &", gpFileName);
#else
  sprintf(sysCallStr, "/usr/bin/gnuplot -persist %s", gpFileName);
#endif
  int err = system(sysCallStr);

  if (err == -1)
  {
    RLOG(4, "Couldn't start gnuplot");
    return false;
  }

  return true;
}

/*******************************************************************************
 *
 * \brief Counts rows and columns of an array in a file. Returns true for
 *        success, false otherwise. False means that the data in the file
 *        is malformed (different numbers of columns). A header line is
 *        counted for each line that does not begin with a
 *        - digit
 *        - white space
 *        - plus sign
 *        - minus sign
 *        - dot
 *        - e
 *        The file descriptor is moved to the position after the header
 *        lines.
 *
 ******************************************************************************/
bool MatNd_arraySizeFromFile(FILE* fd, int* rows, int* cols, int* nHeaderLines)
{
  // Determine file size
  fseek(fd, 0, SEEK_END);
  long fileSize = ftell(fd) + 1;
  rewind(fd);

  NLOG(0, "File size is %d bytes", fileSize);

  char* line = RNALLOC(fileSize, char);
  RCHECK_MSG(line, "Couldn't allocate %lu bytes to read file", fileSize);
  int i, colPrev = -1;
  *rows = 0;
  *cols = 0;
  *nHeaderLines = 0;

  while (fgets(line, fileSize, fd))
  {
    // Check for header line
    bool isValueLine = true;

    NLOG(0, "line length = %d", strlen(line));

    for (i = 0; i < (int)(strlen(line) - 1); i++)
    {
      if ((line[i] > 47 && line[i] < 58) // digit
          || line[i] == 32           // white space
          || line[i] == 43           // +
          || line[i] == 45           // -
          || line[i] == 46           // .
          || line[i] == 101)         // e
      {

      }
      else
      {
        isValueLine = false;
        RLOG(0, "Detected no-number character for line[%d]: \"%s\"",
             i, line);
      }
    }

    if (!isValueLine)
    {
      (*nHeaderLines)++;
      RLOG(0, "Detected header line %d at row %d: \"%s\"",
           *nHeaderLines, *rows, line);
    }
    else
    {
      *cols = String_countSubStrings(line, " ");

      if (colPrev != -1)
      {
        if (colPrev != *cols)
        {
          RLOG(1, "The routine found two rows with different number "
               "of values: row %d has %d elements, row %d has %d "
               "elements. You might have some spaces after the last "
               "number of a line, or empty lines at the end of the "
               "file or so. This routine is not so great to figure "
               "this all out. Please check your data file",
               (*rows) - 1, colPrev, *rows, *cols);
          RFREE(line);
          return false;
        }
      }

      colPrev = *cols;
      (*rows)++;
      line[0] = '\0';
      NLOG(0, "row = %d   col = %d", *rows, *cols);
    }
  }

  // Move file descriptor forward for the number of header lines
  rewind(fd);

  for (i = 0; i < (*nHeaderLines); i++)
  {
    char* character = fgets(line, fileSize, fd);
    RCHECK(character != NULL);
  }

  RFREE(line);

  return true;
}

/*******************************************************************************
 *
 *  \brief Counts rows and columns of an array in a file. Returns true for
 *         success, false otherwise. False means that the data in the file
 *         is malformed (different numbers of columns). A header line is
 *         counted for each line that does not begin with a
 *         - digit
 *         - white space
 *         - plus sign
 *         - minus sign
 *         - dot
 *         - e
 *         This is a convenience function that takes a filename as an argument
 *         and can be used for counting only if the file descriptor is not
 *         needed outside.
 *
 ******************************************************************************/
bool MatNd_arraySizeFromFilename(const char* filename, int* rows, int* cols,
                                 int* nHeaderLines)
{
  FILE* fd = fopen(filename, "r");
  if (fd == NULL)
  {
    RLOG(1, "Error opening file \"%s\"", filename);
    return false;
  }

  bool ret = MatNd_arraySizeFromFile(fd, rows, cols, nHeaderLines);

  fclose(fd);

  return ret;
}

/*******************************************************************************
 * See header, also for code example.
 ******************************************************************************/
bool MatNd_fromFile(MatNd* M, const char* fileName)
{
  int i, m, n, nHeaderLines, nEle = 0, nItems;
  bool success;
  FILE* fd = fopen(fileName, "r");

  if (fd == NULL)
  {
    RLOG(1, "Error opening file \"%s\"", fileName);
    return false;
  }

  success = MatNd_arraySizeFromFile(fd, &m, &n, &nHeaderLines);

  if (success == false)
  {
    RLOG(1, "data file \"%s\" is malformed", fileName);
    fclose(fd);
    return false;
  }

  MatNd_reshape(M, m, n);
  char buf[256];

  for (i = 0; i < m * n; i++)
  {
    nItems = fscanf(fd, "%255s", buf);
    M->ele[nEle] = String_toDouble_l(buf);// locale-independent
    nEle++;

    RCHECK_MSG(nItems != EOF, "Read error!");
  }

  fclose(fd);

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
MatNd* MatNd_createFromFile(const char* fileName)
{
  int i, m, n, nEle = 0, nHeaderLines, nItems;
  bool success;
  FILE* fd;
  MatNd* self;

  fd = fopen(fileName, "r");

  if (fd == NULL)
  {
    RLOG(1, "Can't open file \"%s\" - returning NULL", fileName);
    return NULL ;
  }

  success = MatNd_arraySizeFromFile(fd, &m, &n, &nHeaderLines);

  if (success == false)
  {
    RLOG(1, "data file \"%s\" is malformed", fileName);
    RLOG(4, "File has %d rows, %d columns and %d header lines",
         m, n, nHeaderLines);
    fclose(fd);
    return NULL ;
  }

  self = MatNd_create(m, n);
  char buf[256];

  for (i = 0; i < m * n; i++)
  {
    nItems = fscanf(fd, "%255s", buf);
    self->ele[nEle] = String_toDouble_l(buf);// locale-independent
    nEle++;
    RCHECK_MSG(nItems != EOF, "Read error at element %d", nEle);
  }

  fclose(fd);

  return self;
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_setZero(MatNd* self)
{
  memset(self->ele, 0, self->m * self->n * sizeof(double));
}

/*******************************************************************************
 * See header. Here we can use memset since the row is memory-aligned.
 * It's more efficient than going through the indices.
 ******************************************************************************/
void MatNd_setRowZero(MatNd* self, unsigned int row)
{
  RCHECK_MSG(row < (unsigned int) self->m, "Index out of limits: row=%d   "
             "self->m=%d",
             row, self->m);
  memset(&self->ele[row * self->n], 0, self->n * sizeof(double));
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_setColumnZero(MatNd* self, unsigned int column)
{
  unsigned int i;

  RCHECK_MSG(column < self->n, "column = %d, should be <= %d", column, self->n);

  for (i = 0; i < self->m; i++)
  {
    self->ele[i * self->n + column] = 0.0;
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_setColumnToValue(MatNd* self, unsigned int column, double value)
{
  unsigned int i;

  RCHECK_MSG(column < self->n, "column = %d, should be <= %d", column, self->n);

  for (i = 0; i < self->m; i++)
  {
    MatNd_set2(self, i, column, value);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_setIdentity(MatNd* self)
{
  unsigned int i;

  RCHECK_MSG(self->m == self->n, "Matrix is not square (%d x %d)",
             self->m, self->n);

  MatNd_setZero(self);

  for (i = 0; i < self->m; i++)
  {
    self->ele[i * self->m + i] = 1.0;
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_addIdentity(MatNd* self)
{
  unsigned int i;

  RCHECK_MSG(self->m == self->n, "Matrix is not square (%d x %d)",
             self->m, self->n);

  for (i = 0; i < self->m; i++)
  {
    self->ele[i * self->m + i] += 1.0;
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_set(MatNd* self, int m, int n, double val)
{
  RCHECK_MSG(m < (int) self->m, "Out of limits: m=%d self->m=%d", m, self->m);
  RCHECK_MSG(n < (int) self->n, "Out of limits: n=%d self->n=%d", n, self->n);
  RCHECK_MSG(m >= 0, "m has to be positive: m=%d", m);
  RCHECK_MSG(n >= 0, "n has to be positive: n=%d", n);

  self->ele[m * self->n + n] = val;
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_addToEle(MatNd* self, int m, int n, double val)
{
  RCHECK_MSG(m < (int) self->m, "Out of limits: m=%d self->m=%d", m, self->m);
  RCHECK_MSG(n < (int) self->n, "Out of limits: n=%d self->n=%d", n, self->n);

  self->ele[m * self->n + n] += val;
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_addConst(MatNd* dst, double value)
{
  int i, nEle = dst->m * dst->n;
  for (i = 0; i < nEle; i++)
  {
    dst->ele[i] += value;
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_addConstToDiag(MatNd* dst, double value)
{
  MatNd diag = MatNd_fromPtr(1, 1, &value);
  MatNd_addDiag(dst, &diag);
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_addConstToColum(MatNd* dst, unsigned int column, double value)
{
  RCHECK(dst);
  RCHECK(column < dst->n);

  for (unsigned int i = 0; i < dst->m; i++)
  {
    MatNd_addToEle(dst, i, column, value);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_setDiag(MatNd* dst, const MatNd* diag)
{
  MatNd_setZero(dst);
  MatNd_addDiag(dst, diag);
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_overwriteDiag(MatNd* dst, const MatNd* diag)
{
  unsigned int i;

  RCHECK_MSG(dst->m == dst->n, "Target matrix is not square (%d x %d)",
             dst->m, dst->n);
  RCHECK_MSG(diag->n == 1,
             "Diagonal vector must have only one column, but has %d", diag->n);

  if (diag->m == dst->m)
  {
    for (i = 0; i < dst->m; i++)
    {
      dst->ele[i * dst->n + i] = diag->ele[i];
    }
  }
  else if (diag->m == 1)
  {
    for (i = 0; i < dst->m; i++)
    {
      dst->ele[i * dst->n + i] = diag->ele[0];
    }
  }
  else
  {
    RFATAL("Size mismatch: diagonal vector is %d x 1, but square matrix "
           "is %d x %d",
           diag->m, dst->m, dst->n);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_addDiag(MatNd* dst, const MatNd* diag)
{
  unsigned int i;

  RCHECK_MSG(dst->m == dst->n, "Target matrix is not square (%d x %d)",
             dst->m, dst->n);
  RCHECK_MSG(diag->n == 1, "Diagonal vector must have only one column, but "
             "has %d",
             diag->n);

  if (diag->m == dst->m)
  {
    for (i = 0; i < dst->m; i++)
    {
      dst->ele[i * dst->n + i] += diag->ele[i];
    }
  }
  else if (diag->m == 1)
  {
    for (i = 0; i < dst->m; i++)
    {
      dst->ele[i * dst->n + i] += diag->ele[0];
    }
  }
  else
  {
    RFATAL("Size mismatch: diagonal vector is %d x 1, but square matrix "
           "is %d x %d",
           diag->m, dst->m, dst->n);
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_setElementsTo(MatNd* self, double val)
{
  unsigned int i, nEle = self->m * self->n;
  for (i = 0; i < nEle; i++)
  {
    self->ele[i] = val;
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
double MatNd_get(const MatNd* self, int m, int n)
{
  RCHECK_MSG((m >= 0) &&
             (n >= 0) &&
             (m < (int) self->m) &&
             (n < (int) self->n),
             "Index out of limits: array is of dimension %d x %d, your "
             "indices are %d %d",
             self->m, self->n, m, n);

  return self->ele[m * self->n + n];
}

/*******************************************************************************
 *
 ******************************************************************************/
double* MatNd_getElePtr(const MatNd* self, int m, int n)
{
  RCHECK_MSG((m >= 0) &&
             (n >= 0) &&
             (m < (int) self->m) &&
             (n < (int) self->n),
             "Index out of limits: array is of dimension %d x %d, your "
             "indices are %d %d",
             self->m, self->n, m, n);

  return &self->ele[m * self->n + n];
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_swapElements(MatNd* A, int m1, int n1, int m2, int n2)
{
  double* value1 = MatNd_getElePtr(A, m1, n1);
  double* value2 = MatNd_getElePtr(A, m2, n2);
  double tmp = *value1;
  *value1 = *value2;
  *value2 = tmp;
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_reshape(MatNd* self, int m, int n)
{
  RCHECK_MSG((int) self->size >= m * n,
             "size=%d (%d x %d), m=%d, n=%d",
             self->size, self->m, self->n, m, n);

  self->m = m;
  self->n = n;
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_reshapeAndSetZero(MatNd* self, int m, int n)
{
  RCHECK_MSG((int) self->size >= m * n,
             "size=%d (%d x %d), m=%d, n=%d",
             self->size, self->m, self->n, m, n);

  self->m = m;
  self->n = n;
  memset(self->ele, 0, self->m * self->n * sizeof(double));
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_copy(MatNd* dst, const MatNd* src)
{
  RCHECK_MSG((dst->m == src->m) && (dst->n == src->n),
             "dst: [%d x %d]  src: [%d x %d]", dst->m, dst->n, src->m, src->n);

  memmove(dst->ele, src->ele, src->m * src->n * sizeof(double));
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_reshapeCopy(MatNd* dst, const MatNd* src)
{
  unsigned int srcEle = src->m*src->n;

  RCHECK_MSG(dst->size >= srcEle,
             "dst->size=%d (%d x %d), src->m=%d, src->n=%d",
             dst->size, dst->m, dst->n, src->m, src->n);

  dst->m = src->m;
  dst->n = src->n;
  memmove(dst->ele, src->ele, srcEle*sizeof(double));
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_resizeCopy(MatNd* dst, const MatNd* src)
{
  MatNd_realloc(dst, src->m, src->n);
  MatNd_copy(dst, src);
}

/*******************************************************************************
 * See header, also for code example.
 ******************************************************************************/
void MatNd_copyColumns(MatNd* dst, const MatNd* src, const int* cols)
{
  unsigned int i, j, nCols = 0, maxCol = 0;

  RCHECK(cols);

  // Count the number of columns and determine the maximum column
  while (cols[nCols] != -1)
  {
    if (cols[nCols] > (int) maxCol)
    {
      maxCol = cols[nCols];
    }
    nCols++;
  }
  RCHECK_MSG(maxCol < src->n, "The last column index %d must be within the "
             "column range of src, wich is [%d x %d]",
             maxCol, src->m, src->n);
  MatNd_reshape(dst, src->m, nCols);

  // Read through file and copy respective elements
  for (i = 0; i < src->m; i++)
  {
    double* srcRow = MatNd_getRowPtr(src, i);
    double* dstRow = MatNd_getRowPtr(dst, i);

    // Copy row elements to target array in desired order
    for (j = 0; j < nCols; j++)
    {
      dstRow[j] = srcRow[cols[j]];
    }
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_copyColumn(MatNd* dst,
                      unsigned int dstColumn,
                      const MatNd* src,
                      unsigned int srcColumn)
{
  RCHECK_MSG(dst->m == src->m, "dst->m = %d != src->m = %d", dst->m, src->m);
  RCHECK_MSG(dst->n > dstColumn, "dst->n=%d dstColumn=%d", dst->n, dstColumn);
  RCHECK_MSG(src->n > srcColumn, "src->n=%d srcColumn=%d", src->n, srcColumn);

  for (unsigned int i = 0; i < src->m; i++)
  {
    double* srcRow = MatNd_getRowPtr(src, i);
    double* dstRow = MatNd_getRowPtr(dst, i);

    dstRow[dstColumn] = srcRow[srcColumn];
  }
}

/*******************************************************************************
 * dst = m1 - m2.
 ******************************************************************************/
void MatNd_sub(MatNd* dst, const MatNd* m1, const MatNd* m2)
{
  int i, nEle = dst->m * dst->n;

  RCHECK_MSG((dst->m == m2->m) &&
             (m2->m == m1->m) &&
             (dst->n == m2->n) &&
             (m2->n == m1->n),
             "dst: [%d x %d]  m1: [%d x %d]   m2: [%d x %d]",
             dst->m, dst->n, m1->m, m1->n, m2->m, m2->n);

  for (i = 0; i < nEle; i++)
  {
    dst->ele[i] = m1->ele[i] - m2->ele[i];
  }
}

/*******************************************************************************
 * dst = m1 + m2.
 ******************************************************************************/
void MatNd_add(MatNd* dst, const MatNd* m1, const MatNd* m2)
{
  int i, nEle = dst->m * dst->n;

  RCHECK_MSG((dst->m==m2->m) && (m2->m==m1->m) && (dst->n==m2->n) &&
             (m2->n==m1->n), "dst: [%d x %d]  m1: [%d x %d]   m2: [%d x %d]",
             dst->m, dst->n, m1->m, m1->n, m2->m, m2->n);

  for (i = 0; i < nEle; i++)
  {
    dst->ele[i] = m1->ele[i] + m2->ele[i];
  }
}

/*******************************************************************************
 * dst = dst - m1.
 ******************************************************************************/
void MatNd_subSelf(MatNd* dst, const MatNd* m1)
{
  int i, nEle = dst->m * dst->n;

  RCHECK_MSG((dst->m == m1->m) && (dst->n == m1->n),
             "dst: [%d x %d]  m1: [%d x %d]", dst->m, dst->m, m1->m, m1->n);

  for (i = 0; i < nEle; i++)
  {
    dst->ele[i] -= m1->ele[i];
  }
}

/*******************************************************************************
 * dst = dst + m1.
 ******************************************************************************/

void MatNd_addSelf(MatNd* dst, const MatNd* m1)
{
  int i, nEle = dst->m * dst->n;

  RCHECK_MSG((dst->m == m1->m) && (dst->n == m1->n),
             "dst: [%d x %d]  m1: [%d x %d]", dst->m, dst->n, m1->m, m1->n);

  for (i = 0; i < nEle; i++)
  {
    dst->ele[i] += m1->ele[i];
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_appendRows(MatNd* dst, const MatNd* src)
{
  MatNd_insertRows(dst, dst->m - 1, src, 0, src->m);
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_prependRows(MatNd* dst, const MatNd* src)
{
  MatNd_insertRows(dst, -1, src, 0, src->m);
}

/*******************************************************************************
 *
 * \brief inserts nCols starting with index colSrc of array src after column
 *       colDst of array dst.
 *
 *       Examples:
 *       - MatNd_insertRows(dst, 2, src, 4, 2):
 *         row 4 and 5 of src will be inserted behind the 3rd row of self
 *       - MatNd_insertRows(dst, -1, src, 4, 2):
 *         row 4 and 5 of src will be inserted before the first row of self
 *
 ******************************************************************************/
void MatNd_insertRows(MatNd* self, int rowDst, const MatNd* from, int rowSrc,
                      unsigned int nRows)
{
  int nEle = self->n, originalRows = self->m;

  RCHECK_MSG(self->n == from->n, "Arrays have different number of columns: "
             "%d != %d",
             self->n, from->n);

  RCHECK_MSG(rowSrc + nRows <= (int) from->m, "You are trying to copy row %d "
             "to %d, but the array only has %d rows",
             rowSrc, rowSrc + nRows, from->m);

  self = MatNd_realloc(self, self->m + nRows, self->n);

  if (rowDst == -1)
  {
    MatNd* buf;
    MatNd_clone2(buf, self);
    RCHECK(buf);

    // Prepend from, and then append self
    memmove(buf->ele, &from->ele[rowSrc * nEle],
            nRows * nEle * sizeof(double));
    memmove(&buf->ele[nRows * nEle], self->ele,
            originalRows * nEle * sizeof(double));

    MatNd_copy(self, buf);
    MatNd_destroy(buf);
  }
  else if (rowDst == originalRows - 1)
  {
    // Append from to self
    memmove(&self->ele[originalRows * nEle], &from->ele[rowSrc * nEle],
            nRows * nEle * sizeof(double));
  }
  else if ((rowDst >= originalRows) || (rowDst < -1))
  {
    RFATAL("Target row %d out of range: Array has only %d rows - the index"
           " must be within [0...%d]",
           rowDst, originalRows, originalRows - 1);
  }
  else
  {
    MatNd* buf;
    MatNd_clone2(buf, self);
    RCHECK(buf);

    // The first (rowDst+1) rows have already been cloned
    int offset = (rowDst + 1) * nEle;
    memmove(&buf->ele[offset], &from->ele[rowSrc * nEle],
            nRows * nEle * sizeof(double));
    offset += nRows * nEle;
    memmove(&buf->ele[offset], &self->ele[(rowDst + 1) * nEle],
            (originalRows - rowDst - 1) * nEle * sizeof(double));

    MatNd_copy(self, buf);
    MatNd_destroy(buf);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_appendColumns(MatNd* dst, const MatNd* src)
{
  MatNd_insertColumns(dst, dst->n - 1, src, 0, src->n);
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_insertColumns(MatNd* self,
                         int colDst,
                         const MatNd* from,
                         int colSrc, int nCols)
{
  MatNd* fromTp = NULL;

  MatNd_create2(fromTp, from->n, from->m);
  MatNd_transpose(fromTp, from);
  MatNd_transposeSelf(self);
  MatNd_insertRows(self, colDst, fromTp, colSrc, nCols);
  MatNd_transposeSelf(self);
  MatNd_destroy(fromTp);
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_condenseRows(MatNd* dst, const MatNd* src, double ratio)
{
  unsigned int i;
  unsigned int keepLine = lround(1.0 / ratio);

  if ((ratio < 0.0) || (ratio > 1.0))
  {
    RLOG(1, "Ratio out of range: %f != [0...1] - doing nothing", ratio);
    return;
  }

  MatNd_reshape(dst, 0, src->n);

  for (i = 0; i < src->m; i++)
  {
    if (i % keepLine == 0)
    {
      MatNd row = MatNd_fromPtr(1, src->n, MatNd_getRowPtr(src, i));
      MatNd_appendRows(dst, &row);
    }
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
double MatNd_mean(const MatNd* self)
{
  int i, nEle = self->m * self->n;
  double mean = 0.0;

  if (nEle == 0)
  {
    return 0.0;  // Otherwise division by zero
  }

  for (i = 0; i < nEle; i++)
  {
    mean += self->ele[i];
  }

  return mean / nEle;
}

/*******************************************************************************
 *
 ******************************************************************************/
double MatNd_getNorm(const MatNd* self)
{
  double norm = 0.0;
  int i, nEle = self->m * self->n;

  for (i = 0; i < nEle; i++)
  {
    norm += self->ele[i] * self->ele[i];
  }

  return sqrt(norm);
}

/*******************************************************************************
 *
 ******************************************************************************/
double MatNd_getNormL1(const MatNd* self)
{
  unsigned int i, nEle = self->m * self->n;
  double norm = 0.0;

  for (i = 0; i < nEle; i++)
  {
    norm += fabs(self->ele[i]);
  }

  return norm;
}

/*******************************************************************************
 *
 ******************************************************************************/
double MatNd_sqrDistance(const MatNd* A, const MatNd* B)
{
  int i, nEle = A->m * A->n;
  double sqrDist = 0.0, diff;

  RCHECK_MSG((A->m == B->m) && (A->n == B->n), "A: [%d x %d]   B: [%d x %d]",
             A->m, A->n, B->m, B->n);

  for (i = 0; i < nEle; i++)
  {
    diff = A->ele[i] - B->ele[i];
    sqrDist += diff * diff;
  }

  return sqrDist;
}

/*******************************************************************************
 *
 ******************************************************************************/
double MatNd_msqError(const MatNd* a1, const MatNd* a2)
{
  if (a1->m*a1->n==0)
  {
    return 0.0;
  }

  return MatNd_sqrDistance(a1, a2) / (a1->m * a1->n);
}

/*******************************************************************************
 *
 ******************************************************************************/
double MatNd_rmsqError(const MatNd* a1, const MatNd* a2)
{
  return sqrt(MatNd_msqError(a1, a2));
}

/*******************************************************************************
 * \brief C = A * B.
 *
 * Matrix multiplication: C(m x l) = A(m x n) * B(n x l)
 * m: rows of matrix C
 * n: columns of matrix A
 * l: columns of matrix C.
 *
 ******************************************************************************/
void MatNd_mul(MatNd* C, const MatNd* A, const MatNd* B)
{
  const int m = C->m, n = A->n, l = C->n;
  int i, j, k, i_times_n, i_times_l;
  double v;

  RCHECK_MSG((C->m == A->m) && (C->n == B->n) && (A->n == B->m),
             "C: [%d x %d]   A: [%d x %d]   B: [%d x %d]", C->m,
             C->n, A->m, A->n, B->m, B->n);

  for (i = 0; i < m; i++)
  {
    i_times_n = i * n;
    i_times_l = i * l;

    for (j = 0; j < l; j++)
    {
      v = 0.0;

      for (k = 0; k < n; k++)
      {
        v += A->ele[i_times_n + k] * B->ele[k * l + j];
      }

      C->ele[i_times_l + j] = v;
    }

  }

}

/*******************************************************************************
 * \brief C = C + A * B
 *
 * void MatNd_mulAndAddSelf(MatNd* C, const MatNd* A, const MatNd* B)
 * {
 * MatNd* AxB = NULL;
 * MatNd_create2(AxB, A->m, B->n);
 * MatNd_mul(AxB, A, B);
 * MatNd_addSelf(C, AxB);
 * MatNd_destroy(AxB);
 * }
 *
 ******************************************************************************/
void MatNd_mulAndAddSelf(MatNd* C, const MatNd* A, const MatNd* B)
{
  const int m = C->m, n = A->n, l = C->n;
  int i, j, k, i_times_n, i_times_l;
  double v;

  RCHECK_MSG((C->m == A->m) && (C->n == B->n) && (A->n == B->m),
             "C: [%d x %d]   A: [%d x %d]   B: [%d x %d]",
             C->m, C->n, A->m, A->n, B->m, B->n);

  for (i = 0; i < m; i++)
  {
    i_times_n = i * n;
    i_times_l = i * l;

    for (j = 0; j < l; j++)
    {
      v = 0.0;

      for (k = 0; k < n; k++)
      {
        v += A->ele[i_times_n + k] * B->ele[k * l + j];
      }

      C->ele[i_times_l + j] += v;
    }

  }

}

/*******************************************************************************
 * A = B * A
 ******************************************************************************/

void MatNd_preMulSelf(MatNd* A, const MatNd* B)
{
  MatNd* buf = NULL;

  MatNd_clone2(buf, A);
  MatNd_reshape(A, B->m, buf->n);
  MatNd_mul(A, B, buf);
  MatNd_destroy(buf);
}

/*******************************************************************************
 * A = A*B
 ******************************************************************************/

void MatNd_postMulSelf(MatNd* A, const MatNd* B)
{
  MatNd* buf = NULL;

  MatNd_clone2(buf, A);
  MatNd_reshape(A, buf->m, B->n);
  MatNd_mul(A, buf, B);
  MatNd_destroy(buf);
}

/*******************************************************************************
 *
 * Generalized pseudo-inverse with weighting matrices for both dimensions. This
 * function decides for the type of inverse by the number of rows and columns
 * of argument J. The more efficient inverse is utilized.
 *
 * Right inverse: J1# = Wq J^T (J Wq J^T + invWx)^-1     -> MatNd_rwPinv()
 * Left inverse:  J2# = (J^T Wx J + invWq)^-1 J^T Wx     -> MatNd_rwPinv2()
 *
 ******************************************************************************/
double MatNd_generalizedInverse(MatNd* pinvJ, const MatNd* J, const MatNd* Wx,
                                const MatNd* invWq)
{
  double det = 0.0;

  // More rows than columns: compute left inverse
  if (J->m>=J->n)
  {
    det = MatNd_rwPinv2(pinvJ, J, Wx, invWq);
  }
  else
  {
    MatNd* Wq = NULL;
    MatNd* invWx = NULL;

    if (invWq != NULL)
    {
      MatNd_create2(Wq, invWq->m, invWq->n);
      MatNd_inverseDiag(Wq, invWq);
    }

    if (Wx != NULL)
    {
      MatNd_create2(invWx, Wx->m, Wx->n);
      MatNd_inverseDiag(invWx, Wx);
    }

    det = MatNd_rwPinv(pinvJ, J, Wq, invWx);

    MatNd_destroy(Wq);
    MatNd_destroy(invWx);
  }

  return det;
}

/*******************************************************************************
 *
 * \brief Regularized weighted Pseudo-Inverse:  Wq J^T (J Wq J^T + lambda)^-1
 *
 * Arrays W and lambda are the diagonal vectors of the corresponding
 * (symmetric) matrices. The function returns the determinant of the
 * matrix part that is inverted: det(J*Wq*Jt). We use the Cholesky
 * decomposition. Please note that it is only applicable to symmetric
 * matrices.
 *
 ******************************************************************************/
double MatNd_rwPinv(MatNd* J_pinv, const MatNd* J, const MatNd* invW,
                    const MatNd* lambda)
{
  int m = J->m, n = J->n;
  double det;
  MatNd* WJt, *JWJt, *invJWJt;

  // Early exit if J is ill-defined. In this case, we set the determinant to 1.
  if ((m==0) || (n==0))
  {
    MatNd_reshape(J_pinv, J->n, J->m);
    return 1.0;
  }

  // WJt = W*J^T
  MatNd_create2(WJt, n, m);

  MatNd_transpose(WJt, J);

  if (invW != NULL)
  {
    if (invW->n == 1)
    {
      MatNd_preMulDiagSelf(WJt, invW);
    }
    else
    {
      MatNd_preMulSelf(WJt, invW);
    }
  }

  // JWJt = J*W*J^T
  MatNd_create2(JWJt, m, m);

  MatNd_mul(JWJt, J, WJt);

  // Adding regularization values depending on shape of lambda
  if (lambda != NULL)
  {
    // lambda is m x 1 or 1 x 1
    if (lambda->n == 1)
    {
      MatNd_addDiag(JWJt, lambda);
    }
    else
    {
      // checking for positive definiteness is expensive
      REXEC(4)
      {
        MatNd* L;
        MatNd_create2(L, m, m);
        det = MatNd_choleskyDecomposition(L, lambda);
        MatNd_destroy(L);

        RCHECK_MSG(det > 0.0, "lambda is not positive definite");
      }
      // this ensures the correct size internally
      MatNd_addSelf(JWJt, lambda);
    }
  }

  // Inverse (J W J^T)^-1
  invJWJt = JWJt; // Inversion can be done in place
  det = MatNd_choleskyInverse(invJWJt, JWJt);

  // Compute pseudoinverse
  MatNd_reshape(J_pinv, n, m);

  if (det > 0.0)
  {
    MatNd_mul(J_pinv, WJt, invJWJt);
  }
  else
  {
    MatNd_setZero(J_pinv);
  }

  // Clean up
  MatNd_destroy(WJt);
  MatNd_destroy(JWJt);

  return det;
}

/*******************************************************************************
 * Regularized weighted Pseudo-Inverse: (J^T Wx J + lambda)^-1 * J^T Wx
 ******************************************************************************/
double MatNd_rwPinv2(MatNd* J_pinv,
                     const MatNd* J,
                     const MatNd* Wx,
                     const MatNd* lambda)
{
  int m = J->m, n = J->n;

  MatNd* buf = NULL;
  MatNd_create2(buf, m * n + n * n, 1);

  double det = MatNd_rwPinv2_(J_pinv, J, Wx, lambda, buf);

  MatNd_destroy(buf);

  return det;
}

/*******************************************************************************
 * Regularized weighted Pseudo-Inverse: (J^T Wx J + lambda)^-1 * J^T Wx
 ******************************************************************************/
double MatNd_rwPinv2_(MatNd* J_pinv,
                      const MatNd* J,
                      const MatNd* Wx,
                      const MatNd* lambda, MatNd* buf)
{
  unsigned int m = J->m, n = J->n;


  // Early exit if J is ill-defined. In this case, we set the determinant to 1.
  if ((m==0) || (n==0))
  {
    MatNd_reshape(J_pinv, n, m);
    return 1.0;
  }

  RCHECK_MSG(buf->size >= m*n+n*n, "buf is not larg e enough (%d): Should be "
             "m*n + n*n=%d with m=%d and n=%d",
             buf->size, m * n + n * n, m, n);

  double det;
  MatNd JtW = MatNd_fromPtr(n, m, &buf->ele[0]);
  MatNd JtWJ = MatNd_fromPtr(n, n, &buf->ele[m * n]);

  MatNd_transpose(&JtW, J);

  if (Wx != NULL)
  {
    if (Wx->n == 1)
    {
      MatNd_postMulDiagSelf(&JtW, Wx);
    }
    else
    {
      MatNd_postMulSelf(&JtW, Wx);
    }
  }

  // Compute decomposition
  MatNd_mul(&JtWJ, &JtW, J);

  // Regularization
  if (lambda != NULL)
  {
    // lambda is n x 1 or 1 x 1
    if (lambda->n == 1)
    {
      MatNd_addDiag(&JtWJ, lambda);
    }
    else
    {
      // checking for positive definiteness is expensive
      REXEC(4)
      {
        MatNd* L;
        MatNd_create2(L, n, n);
        det = MatNd_choleskyDecomposition(L, lambda);
        MatNd_destroy(L);

        RCHECK_MSG(det > 0.0, "lambda is not positive definite");
      }
      // this ensures the correct size internally
      MatNd_addSelf(&JtWJ, lambda);
    }
  }
  // Cholesky inverse: Inversion can be done in place
  det = MatNd_choleskyInverse(&JtWJ, &JtWJ);

  // Post-multiplication part: inv(J^T*W*J) * (J^T*W)
  MatNd_reshape(J_pinv, n, m);

  if (det > 0.0)
  {
    MatNd_mul(J_pinv, &JtWJ, &JtW);
  }
  else
  {
    MatNd_setZero(J_pinv);
  }

  return det;
}

/*******************************************************************************
 *
 * \brief Miller matrix inversion for a sum of symmetric matrices.
 *
 *        See: Kenneth S. Miller: On the Inverse of the Sum of Matrices,
 *             Mathematics Magazine, Vol. 54, No. 2 (Mar., 1981), pp. 67-72
 *
 *        nq  = 7
 *        nx  = 3
 *        lambda = diag(1.0e-8*ones(nq,1));
 *        A = diag(lambda0*ones(nq,1));
 *        J = rand(nx,nq);
 *        Wx = diag(rand(nx,1))
 *
 *        % Initialization
 *        invC = inv(A);
 *
 *        % Miller algorithm
 *        for i = 1:nx
 *          Ji = J(i,:);
 *          Bi = transpose(Ji)*Wx(i,i)*Ji;
 *          gi = 1.0/(1.0+trace(invC*Bi));
 *          invC = invC - gi*invC*Bi*invC
 *        endfor
 *
 *        The result is in invC
 *
 ******************************************************************************/
void MatNd_MillerPinv(MatNd* pinvJ,
                      const MatNd* J,
                      const MatNd* Wx,
                      const MatNd* lambda)
{
  unsigned int m = J->m, n = J->n;

  // Initialize first inverse
  MatNd* invC = NULL;
  MatNd_create2(invC, n, n);

  if ((lambda->m==1) && (lambda->n==1))
  {
    MatNd_addConstToDiag(invC, 1.0/lambda->ele[0]);
  }
  else if ((lambda->m==n) && (lambda->n==1))
  {
    for (unsigned int i=0; i<n; i++)
    {
      MatNd_set(invC, i, i, 1.0/lambda->ele[i]);
    }
  }
  else
  {
    RFATAL("Wrong dimensions of lambda: %d x %d, but J is %d x %d",
           lambda->m, lambda->n, m, n);
  }

  MatNd* Bi = NULL;
  MatNd_create2(Bi, n, n);

  MatNd* invCBi = NULL;
  MatNd_create2(invCBi, n, n);

  for (unsigned int i=0; i<J->m; i++)
  {
    // Ji
    MatNd Ji = MatNd_getRowView(J, i);

    // Bi = Ji^T Wx J
    MatNd_sqrMulAtBA(Bi, &Ji, NULL);

    if (Wx != NULL)
    {
      if (Wx->n == 1)
      {
        MatNd_constMulSelf(Bi, MatNd_get(Wx, i, 0));
      }
      else if (Wx->n == Wx->m)
      {
        MatNd_constMulSelf(Bi, MatNd_get(Wx, i, i));
      }
      else
      {
        RFATAL("Wrong dimensions of Wx: %d x %d, but J is %d x %d",
               Wx->m, Wx->n, m, n);
      }
    }

    // gi = 1.0/(1.0+trace(invC*Bi));
    MatNd_mul(invCBi, invC, Bi);
    double gi = 1.0/(1.0+MatNd_trace(invCBi));

    // invC = invC - gi*invC*Bi*invC
    MatNd_postMulSelf(invCBi, invC);
    MatNd_constMulSelf(invCBi, gi);
    MatNd_subSelf(invC, invCBi);
  }


  // J^T Wx
  MatNd* JtW = NULL;
  MatNd_create2(JtW, n, m);
  MatNd_transpose(JtW, J);

  if (Wx != NULL)
  {
    if (Wx->n == 1)
    {
      MatNd_postMulDiagSelf(JtW, Wx);
    }
    else
    {
      MatNd_postMulSelf(JtW, Wx);
    }
  }

  // Overall result
  MatNd_reshape(pinvJ, n, m);
  MatNd_mul(pinvJ, invC, JtW);

  MatNd_destroy(Bi);
  MatNd_destroy(invC);
  MatNd_destroy(invCBi);
  MatNd_destroy(JtW);
}



/*******************************************************************************
 * Sets the given double array to the n-th column of the MatNd.
 ******************************************************************************/
void MatNd_setColumn(MatNd* A, int column, const double* p, int n)
{
  int i;

  RCHECK_MSG(column < (int) A->n, "column = %d, must be < %d", column, A->n);
  RCHECK_MSG(n <= (int) A->m, "n = %d, should be <= %d", n, A->m);

  for (i = 0; i < n; i++)
  {
    A->ele[i * A->n + column] = p[i];
  }
}



/*******************************************************************************
 * Adds the given double array to the n-th column of the MatNd.
 ******************************************************************************/
void MatNd_addToColumn(MatNd* A, int column, const double* p, int n)
{
  int i;

  RCHECK_MSG(column < (int) A->n, "column = %d, must be < %d", column, A->n);
  RCHECK_MSG(n <= (int) A->m, "n = %d, should be <= %d", n, A->m);

  for (i = 0; i < n; i++)
  {
    A->ele[i * A->n + column] += p[i];
  }
}

/*******************************************************************************
 * Copies the c-th column of A into B.
 ******************************************************************************/
void MatNd_getColumn(MatNd* B, int c, const MatNd* A)
{
  unsigned int i;

  RCHECK_MSG(B->size >= A->m, "Array size (%d) to small to hold %d elements",
             B->size, A->m);
  RCHECK_MSG((c >= 0) && (c < (int) A->n), "column = %d, should be [0 ... %d]",
             c, A->n - 1);

  MatNd_reshape(B, A->m, 1);

  for (i = 0; i < A->m; i++)
  {
    B->ele[i] = A->ele[i * A->n + c];
  }
}

/*******************************************************************************
 *
 * Some test code:
 *
 * MatNd *seppl = MatNd_create(6,6);
 * MatNd_setRandom(seppl, 0.0, 1.0);
 * MatNd_printCommentDigits("1", seppl, 2);
 * MatNd_deleteColumn(seppl, 0);
 * MatNd_printCommentDigits("2", seppl, 2);
 * MatNd_deleteColumn(seppl, 2);
 * MatNd_printCommentDigits("3", seppl, 2);
 *
 ******************************************************************************/
void MatNd_deleteColumn(MatNd* self, int index)
{
  MatNd* tmp = NULL;
  MatNd_create2(tmp, self->m, self->n - 1);
  unsigned int i, j, k = 0;

  RCHECK_MSG(index >= 0 && index < (int) self->n, "n = %d, index = %d",
             self->n, index);

  for (i = 0; i < self->m; i++)
  {
    int rowOffset = i * self->n;

    for (j = 0; j < self->n; j++)
    {
      if (j != index)
      {
        tmp->ele[k++] = self->ele[rowOffset + j];
      }
    }
  }

  MatNd_reshape(self, self->m, self->n - 1);
  MatNd_copy(self, tmp);
  MatNd_destroy(tmp);
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_deleteColumns(MatNd* self, int first, int last)
{
  int nCols = last - first;

  RCHECK_MSG(nCols >= 0, "First: %d   last: %d", first, last);

  for (int i = 0; i <= nCols; i++)
  {
    MatNd_deleteColumn(self, first);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_deleteRow(MatNd* self, int index)
{
  RCHECK_MSG(index >= 0 && index < (int) self->m, "m = %d, index = %d",
             self->m, index);

  if (index < (int)(self->m - 1))
  {
    memmove(&self->ele[index * self->n], &self->ele[(index + 1) * self->n],
            (self->m - 1 - index) * self->n * sizeof(double));
  }

  MatNd_reshape(self, self->m - 1, self->n);
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_deleteRows(MatNd* self, int first, int last)
{
  int nRows = last - first;

  RCHECK_MSG(nRows >= 0, "First: %d   last: %d", first, last);

  for (int i = 0; i <= nRows; i++)
  {
    MatNd_deleteRow(self, first);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_transpose(MatNd* dst, const MatNd* src)
{
  unsigned int i, j, i_n;

  RCHECK_MSG(dst->m == src->n && dst->n == src->m, "Size mismatch: dst->m=%d "
             "src->n=%d dst->n=%d src->m=%d",
             dst->m, src->n, dst->n, src->m);

  for (i = 0; i < src->m; i++)
  {
    i_n = i * src->n;

    for (j = 0; j < src->n; j++)
    {
      dst->ele[j * src->m + i] = src->ele[i_n + j];
    }
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_transposeSelf(MatNd* dst)
{
  // Fast solution for vectors only.
  if (dst->m==1)
  {
    dst->m = dst->n;
    dst->n = 1;
    return;
  }

  if (dst->n==1)
  {
    dst->n = dst->m;
    dst->m = 1;
    return;
  }

  // Both m and n are larger than 1
  MatNd* tmp = NULL;

  MatNd_create2(tmp, dst->n, dst->m);
  MatNd_transpose(tmp, dst);

  dst->m = tmp->m;
  dst->n = tmp->n;

  // Faster to use memcopy here since MatNd_copy() uses memmove
  memcpy(dst->ele, tmp->ele, tmp->m * tmp->n * sizeof(double));

  MatNd_destroy(tmp);
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_constMul(MatNd* dst, const MatNd* src, double c)
{
  int i, nEle = dst->m * dst->n;

  RCHECK_MSG((dst->m == src->m) && (dst->n == src->n),
             "dst: [%d x %d]  v1: [%d x %d]",
             dst->m, dst->n, src->m, src->n);

  for (i = 0; i < nEle; i++)
  {
    dst->ele[i] = c * src->ele[i];
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_constMulAndAdd(MatNd* dst,
                          const MatNd* v1,
                          const MatNd* v2,
                          double c)
{
  int i, nEle = dst->m * dst->n;

  RCHECK_MSG((dst->m == v1->m) &&
             (dst->n == v1->n) &&
             (dst->m == v2->m) &&
             (dst->n == v2->n),
             "dst: [%d x %d]  v1: [%d x %d]  v2: [%d x %d]",
             dst->m, dst->n, v1->m, v1->n, v2->m, v2->n);

  for (i = 0; i < nEle; i++)
  {
    dst->ele[i] = v1->ele[i] + c * v2->ele[i];
  }
}

/*******************************************************************************
 * dst += v1 * c
 ******************************************************************************/
void MatNd_constMulAndAddSelf(MatNd* dst, const MatNd* v1, double c)
{
  int i, nEle = dst->m * dst->n;

  RCHECK_MSG((dst->m == v1->m) && (dst->n == v1->n),
             "dst: [%d x %d]  v1: [%d x %d]", dst->m, dst->n, v1->m, v1->n);

  for (i = 0; i < nEle; i++)
  {
    dst->ele[i] += c * v1->ele[i];
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_constMulSelf(MatNd* self, double c)
{
  int i, nEle = self->m * self->n;

  for (i = 0; i < nEle; i++)
  {
    self->ele[i] *= c;
  }
}

/*******************************************************************************
 * Copies n elements of an array into the MatNd data structure.
 ******************************************************************************/
void MatNd_fromArray(MatNd* self, const double* p, int n)
{
  RCHECK_MSG((int) self->size >= n, "self->size=%d   n=%d", self->size, n);

  memmove(&self->ele[0], &p[0], n * sizeof(double));
}

/*******************************************************************************
 * Returns the sum of all values of the array.
 ******************************************************************************/
double MatNd_sumEle(const MatNd* self)
{
  double res = 0.0;
  int i, nEle = self->m * self->n;

  for (i = 0; i < nEle; i++)
  {
    res += self->ele[i];
  }

  return res;
}

/*******************************************************************************
 *
 ******************************************************************************/
double MatNd_columnSum(const MatNd* self, unsigned int column)
{
  double colSum = 0.0;
  RCHECK(column < self->n);

  for (unsigned int i = 0; i < self->m; i++)
  {
    colSum += MatNd_get2(self, i, column);
  }

  return colSum;
}

/*******************************************************************************
 * Returns the lowest absolut (=fabs()) value of the array.
 ******************************************************************************/
double MatNd_minAbsEle(const MatNd* self)
{
  double res = fabs(self->ele[0]);
  int i, nEle = self->m * self->n;

  for (i = 1; i < nEle; i++)
  {
    if (self->ele[i] < res)
    {
      res = fabs(self->ele[i]);
    }
  }

  return res;
}

/*******************************************************************************
 * Returns the highest absolut (=fabs()) value of the array.
 ******************************************************************************/
double MatNd_maxAbsEle(const MatNd* self)
{
  double res = fabs(self->ele[0]);
  int i, nEle = self->m * self->n;

  for (i = 0; i < nEle; i++)
  {
    if (fabs(self->ele[i]) > res)
    {
      res = fabs(self->ele[i]);
    }
  }

  return res;
}

/*******************************************************************************
 *
 ******************************************************************************/
double MatNd_minEle(const MatNd* self)
{
  double res = self->ele[0];
  int i, nEle = self->m * self->n;

  for (i = 1; i < nEle; i++)
  {
    if (self->ele[i] < res)
    {
      res = self->ele[i];
    }
  }

  return res;
}

/*******************************************************************************
 *
 ******************************************************************************/
double MatNd_maxEle(const MatNd* self)
{
  double res = self->ele[0];
  int i, nEle = self->m * self->n;

  for (i = 1; i < nEle; i++)
  {
    if (self->ele[i] > res)
    {
      res = self->ele[i];
    }
  }

  return res;
}

/*******************************************************************************
 *
 ******************************************************************************/
unsigned int MatNd_minAbsEleIndex(const MatNd* self)
{
  unsigned int minIndex = 0;
  int i, nEle = self->m * self->n;
  double val = fabs(self->ele[0]);

  for (i = 0; i < nEle; i++)
  {
    if (fabs(self->ele[i]) < val)
    {
      minIndex = i;
      val = fabs(self->ele[i]);
    }
  }

  return minIndex;
}

/*******************************************************************************
 *
 ******************************************************************************/
unsigned int MatNd_maxEleIndex(const MatNd* self)
{
  unsigned int maxIndex = 0;
  int i, nEle = self->m * self->n;
  double val = self->ele[0];

  for (i = 0; i < nEle; i++)
  {
    if (self->ele[i] > val)
    {
      maxIndex = i;
      val = self->ele[i];
    }
  }

  return maxIndex;
}

/*******************************************************************************
 *
 ******************************************************************************/
unsigned int MatNd_maxAbsEleIndex(const MatNd* self)
{
  unsigned int maxIndex = 0;
  int i, nEle = self->m * self->n;
  double val = fabs(self->ele[0]);

  for (i = 0; i < nEle; i++)
  {
    if (fabs(self->ele[i]) > val)
    {
      maxIndex = i;
      val = fabs(self->ele[i]);
    }
  }

  return maxIndex;
}

/*******************************************************************************
 * Scales the array such that the maximum absolut value doesn't exceed value s.
 * If maxEle is 0.0, the whole array is zero and doesn't need to be modified.
 * The scaling factor is returned.
 ******************************************************************************/
double MatNd_scaleSelfToScalar(MatNd* self, double s)
{
  double scale = 1.0;
  double maxEle = MatNd_maxAbsEle(self);

  if (maxEle == 0.0)
  {
    return 1.0;
  }

  if (s<0.0)
  {
    s = 0.0;
  }

  scale = s / maxEle;
  MatNd_constMulSelf(self, scale);

  return scale;
}

/*******************************************************************************
 * Scales the array such that the maximum absolut value of each element doesn't
 * exceed the corresponding value given in limit. The scaling factor is
 * returned. Limit is assumed to have only positive values.This is not checked.
 ******************************************************************************/
double MatNd_scaleSelf(MatNd* self, const MatNd* limit)
{
  // Early exit for scaling to scalar
  if ((limit->m == 1) && (limit->n == 1))
  {
    return MatNd_scaleSelfToScalar(self, limit->ele[0]);
  }

  // self and limit must have same shape
  RCHECK_EQ(self->m, limit->m);
  RCHECK_EQ(self->n, limit->n);

  double scale = 1.0, absEle, scale_i;
  unsigned int i, mn = self->m * self->n;

  for (i = 0; i < mn; i++)
  {
    absEle = fabs(self->ele[i]);

    if (absEle > limit->ele[i])
    {
      scale_i = limit->ele[i] / absEle;

      if (scale_i < scale)
      {
        scale = scale_i;
      }
    }
  }

  MatNd_constMulSelf(self, scale);

  return scale;
}

/*******************************************************************************
 * Replaces self with element-wise maximum of self and other
 ******************************************************************************/
void MatNd_maxSelf(MatNd* self, const MatNd* other)
{
  RCHECK(self->m == other->m && self->n == other->n);

  for (unsigned int i = 0; i < self->m; i++)
  {
    for (unsigned int j = 0; j < self->n; j++)
    {
      double s = MatNd_get(self, i, j);
      double o = MatNd_get(other, i, j);
      if (o > s)
      {
        MatNd_set(self, i, j, o);
      }
    }
  }
}

/*******************************************************************************
 * Replaces self with element-wise minimum of self and other
 ******************************************************************************/
void MatNd_minSelf(MatNd* self, const MatNd* other)
{
  RCHECK(self->m == other->m && self->n == other->n);

  for (unsigned int i = 0; i < self->m; i++)
  {
    for (unsigned int j = 0; j < self->n; j++)
    {
      double s = MatNd_get(self, i, j);
      double o = MatNd_get(other, i, j);
      if (o < s)
      {
        MatNd_set(self, i, j, o);
      }
    }
  }
}

/*******************************************************************************
 * Saturates self element-wise to +-limit.
 * Limit has to be either a 1x1 matrix or have the same size as self
 ******************************************************************************/
void MatNd_saturateSelf(MatNd* self, const MatNd* limit)
{
  unsigned int i, mn = self->m * self->n;
  double absLimit;

  if ((limit->m == 1) && (limit->n == 1))
  {
    absLimit = fabs(limit->ele[0]);

    for (i = 0; i < mn; i++)
    {
      self->ele[i] = Math_clip(self->ele[i], -absLimit, absLimit);
    }
  }

  else
  {
    RCHECK_MSG((self->m == limit->m) && (self->n == limit->n),
               "self->m=%d limit->m=%d self->n=%d limit->n=%d",
               self->m, limit->m, self->n, limit->n);

    for (i = 0; i < mn; i++)
    {
      absLimit = fabs(limit->ele[i]);
      self->ele[i] = Math_clip(self->ele[i], -absLimit, absLimit);
    }
  }
}

/*******************************************************************************
 * Sets the given double array to the n-th row of the MatNd.
 ******************************************************************************/
void MatNd_setRow(MatNd* A, int row, const double* p, int n)
{
  RCHECK_MSG(row + (n / A->n) <= A->m, "row = %d, should be < %d", row, A->m);
  RCHECK_MSG((n % A->n) == 0, "n = %d, needs to be a multiple of %d", n, A->n);

  memmove(&A->ele[row * A->n], p, n * sizeof(double)); // copy one or more rows
}

/*******************************************************************************
 *
 ******************************************************************************/
double* MatNd_getRowPtr(const MatNd* self, int row)
{
  RCHECK_MSG(row < (int) self->m, "row = %d, should be < %d", row, self->m);
  return &self->ele[row * self->n];
}

/*******************************************************************************
 * Copies the row-th row of A into B.
 ******************************************************************************/
void MatNd_getRow(MatNd* B, int row, const MatNd* A)
{
  MatNd_reshape(B, 1, A->n);
  const double* src = MatNd_getRowPtr(A, row);
  memmove(B->ele, src, A->n*sizeof(double));
}

/*******************************************************************************
 *
 ******************************************************************************/
MatNd MatNd_getRowView(const MatNd* B, int row)
{
  return MatNd_fromPtr(1, B->n, MatNd_getRowPtr(B, row));
}

/*******************************************************************************
 *
 ******************************************************************************/
MatNd MatNd_getRowViewTranspose(const MatNd* B, int row)
{
  return MatNd_fromPtr(B->n, 1, MatNd_getRowPtr(B, row));
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_rotateSelf(MatNd* self, double A_KI[3][3])
{
  // with heap memory
  MatNd A = MatNd_fromPtr(3, 3, &A_KI[0][0]);
  MatNd_preMulSelf(self, &A);

  // without heap memory but less efficient
#if 0
  RCHECK_MSG(self->m == 3, "Array must have 3 rows, but has %d", self->m);

  double col[3], dst[3];

  for (int i = 0; i < self->n; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      col[j] = MatNd_get(self, j, i);
    }
    for (int j = 0; j < 3; j++)
    {
      dst[j] = Vec3_innerProduct(&A_KI[j][0], col);
    }
    MatNd_setColumn(self, i, dst, 3);
  }
#endif
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_invRotateSelf(MatNd* self, double A_KI[3][3])
{
  double A_IK[3][3];
  Mat3d_transpose(A_IK, A_KI);
  MatNd_rotateSelf(self, A_IK);
}

/*******************************************************************************
 * dst = vec x self
 * This function allows in-place operation.
 ******************************************************************************/
void MatNd_columnCrossProduct(MatNd* dst, const MatNd* src, const double vec[3])
{
  double tmp[3];
  unsigned int i0, i1, i2;

  RCHECK_MSG(src->m == 3, "Array must have 3 rows, but has %d", src->m);
  RCHECK_MSG(dst->m == 3, "Array must have 3 rows, but has %d", dst->m);
  RCHECK_MSG(src->n == dst->n, "Arrays must have same column number (src->n=%d "
             "dst->n=%d)",
             src->n, dst->n);

  for (i0 = 0; i0 < src->n; i0++)
  {
    i1 = src->n + i0;
    i2 = 2 * src->n + i0;
    tmp[0] = vec[1] * src->ele[i2] - vec[2] * src->ele[i1];
    tmp[1] = -vec[0] * src->ele[i2] + vec[2] * src->ele[i0];
    tmp[2] = vec[0] * src->ele[i1] - vec[1] * src->ele[i0];
    dst->ele[i0] = tmp[0];
    dst->ele[i1] = tmp[1];
    dst->ele[i2] = tmp[2];
  }

}

/*******************************************************************************
* self = vec x self
 ******************************************************************************/
void MatNd_columnCrossProductSelf(MatNd* self, const double vec[3])
{
  MatNd_columnCrossProduct(self, self, vec);
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_setRandom(MatNd* self, double lower, double upper)
{
  VecNd_setRandom(self->ele, lower, upper, self->m * self->n);
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_eleMul(MatNd* dst, const MatNd* m1, const MatNd* m2)
{
  int i, nEle = dst->m * dst->n;

  RCHECK_MSG((dst->m == m1->m) &&
             (dst->n == m1->n) &&
             (m1->m == m2->m) &&
             (m1->n == m2->n),
             "dst: [%d x %d]  m1: [%d x %d]   m2: [%d x %d]",
             dst->m, dst->n, m1->m, m1->n, m2->m, m2->n);

  for (i = 0; i < nEle; i++)
  {
    dst->ele[i] = m1->ele[i] * m2->ele[i];
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_eleMulSelf(MatNd* dst, const MatNd* m)
{
  int i, nEle = dst->m * dst->n;

  RCHECK_MSG((dst->m == m->m) && (dst->n == m->n),
             "dst: [%d x %d]  m: [%d x %d]", dst->m, dst->n, m->m, m->n);

  for (i = 0; i < nEle; i++)
  {
    dst->ele[i] *= m->ele[i];
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
double MatNd_variance(const MatNd* self)
{
  int i, nEle = self->m * self->n;
  double mean = MatNd_mean(self);
  double var = 0.0;

  if (nEle == 0)
  {
    return 0.0;  // Otherwise division by zero
  }

  for (i = 0; i < nEle; i++)
  {
    var += pow(self->ele[i] - mean, 2);
  }

  return var / nEle;
}

/*******************************************************************************
 * dst = src^T * src
 ******************************************************************************/
void MatNd_dyadicProduct(MatNd* dst, const MatNd* src)
{
  MatNd* src_tp = NULL;

  RCHECK_MSG(dst->m == dst->n, "Target matrix is not square");
  RCHECK_MSG(dst->n == src->n, "dst->n=%d src->n=%d", dst->n, src->n);

  MatNd_create2(src_tp, src->n, src->m);
  MatNd_transpose(src_tp, src);
  MatNd_mul(dst, src_tp, src);
  MatNd_destroy(src_tp);
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_fromString(MatNd* J, const char* str)
{
  // Make a local copy of str, since strtok modifies it during processing
  // trailing '\0' not considered in strlen()
  char* lStr, *ele, eleStr[64];
  int numEle = 0, nRows = 0, nCols = 0, nColsPrev = -1;
  char* saveptr;

  lStr = RNALLOC(strlen(str) + 1, char);
  RCHECK(lStr);
  strcpy(lStr, str);
  ele = String_safeStrtok(lStr, " ", &saveptr);

  while (ele != NULL)
  {
    sscanf(ele, "%63s", eleStr);
    NLOG(0, "Element %d %d is \"%s\"", nRows, nCols, eleStr);

    if (STREQ(eleStr, ","))
    {
      if (nColsPrev != -1)
        RCHECK_MSG(nColsPrev == nCols, "Row %d has %d elements, row %d "
                   "has %d",
                   nRows, nCols, nRows - 1, nColsPrev);
      nColsPrev = nCols;
      nCols = 0;
      nRows++;
    }
    else
    {
      J->ele[numEle++] = atof(eleStr);
      nCols++;
      RCHECK_MSG(numEle <= (int) J->size, "Array too small: found %d "
                 "elements, but have only size of %d",
                 numEle, J->size);
    }

    // Switch to next row for each comma
    ele = String_safeStrtok(NULL, " ", &saveptr);
  }

  RFREE(lStr);
  MatNd_reshape(J, nRows + 1, nCols);
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_toString(const MatNd* M, char* str)
{
  unsigned int i, j;
  char* ptr = str;

  for (i = 0; i < M->m; i++)
  {
    for (j = 0; j < M->n; j++)
    {
      ptr += sprintf(ptr, "%g ", M->ele[M->n * i + j]);
    }
    if (i < M->m - 1)
    {
      ptr += sprintf(ptr, ", ");
    }
  }

  //remove trailing space
  *(--ptr) = '\0';
}

/*******************************************************************************
 *
 ******************************************************************************/
MatNd* MatNd_createFromString(const char* str)
{
  if (str==NULL)
  {
    return NULL;
  }

  int lengthCount = 0;
  const char* sPtr = str;

  while (*sPtr != '\0')
  {
    if (*sPtr == ' ')
    {
      lengthCount++;
    }
    else if (*sPtr == ',')
    {
      lengthCount--; //remove extra ' ' for each ','
    }
    sPtr++;
  }

  lengthCount++; //last entry

  MatNd* self = MatNd_create(1, lengthCount);
  MatNd_fromString(self, str);

  return self;
}

/*******************************************************************************
 *
 * \brief Mirrors upper to lower triangle.
 *
 * Here's a test function:
 *
 * MatNd *A = MatNd_create(i,i);
 * MatNd_setRandom(A, -1.0, 1.0);
 * RMSG("Original"); MatNd_print(A);
 *
 * // Walk through lower triangle and set to 0.0
 * for (int row=0;row<i;row++)
 * for (int col=0;col<row;col++)
 * A->ele[row*i+col] = 0.0;
 *
 *  RMSG("Lower triangle resetted"); MatNd_print(A);
 *
 * MatNd_mirrorUppeHTriangle(A);
 *
 * RMSG("Upper triangle mirrored to lower"); MatNd_print(A);
 * MatNd_destroy(A);
 *
 ******************************************************************************/
void MatNd_mirrorUppeHTriangle(MatNd* A)
{
  RCHECK_MSG(A->m == A->n, "Matrix is not square (%d x %d)", A->m, A->n);

  unsigned int n = A->m;

  // Walk through upper triangle of result matrix and mirror to lower triangle
  for (unsigned int row = 0; row < n; ++row)
  {
    unsigned int row_n = row*n;

    for (unsigned int col = row; col < n; ++col)
    {
      A->ele[col*n + row] = A->ele[row_n + col];
    }
  }
}

/******************************************************************************
 * Copies the lower triangle of src to dst. This does not include the
 * main diagonal. The matrices must be square.
 ******************************************************************************/
void MatNd_copyLowerTriangle(MatNd* dst, const MatNd* src)
{
  int row, col, n = src->m;

  RCHECK_MSG(src->m == src->n, "Matrix not square (%d x %d)", src->m, src->n);
  RCHECK_MSG(dst->m == dst->n, "Matrix not square (%d x %d)", dst->m, dst->n);
  RCHECK_MSG(src->m == dst->m, "Dimension mismatch: %d != %d", src->m, dst->m);

  // Walk through upper triangle of result matrix and mirror to lower triangle
  for (row = 0; row < n; row++)
    for (col = 0; col < row; col++)
    {
      dst->ele[row * n + col] = src->ele[row * n + col];
    }
}

/*******************************************************************************
 * Copies the main diagonal of src to dst. The matrices must be square.
 ******************************************************************************/
void MatNd_copyMainDiagonal(MatNd* dst, const MatNd* src)
{
  int i, n = src->m;

  RCHECK_MSG(src->m == src->n, "Matrix not square (%d x %d)", src->m, src->n);
  RCHECK_MSG(dst->m == dst->n, "Matrix not square (%d x %d)", dst->m, dst->n);
  RCHECK_MSG(src->m == dst->m, "Dimension mismatch: %d != %d", src->m, dst->m);

  for (i = 0; i < n; i++)
  {
    dst->ele[i * n + i] = src->ele[i * n + i];
  }
}

/*******************************************************************************
 * Walk through upper triangle of result matrix and mirror to lower triangle.
 ******************************************************************************/
bool MatNd_isSymmetric(const MatNd* A, double eps)
{
  int row, col, n = A->m;

  RCHECK_MSG(A->m == A->n, "Matrix is not square (%d x %d)", A->m, A->n);

  for (row = 0; row < n; row++)
    for (col = row; col < n; col++)
    {
      double delta = A->ele[col * n + row] - A->ele[row * n + col];
      if (fabs(delta) > eps)
      {
        return false;
      }
    }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool MatNd_isEqual(const MatNd* m1, const MatNd* m2, double eps)
{
  if (m1->m != m2->m)
  {
    return false;
  }

  if (m1->n != m2->n)
  {
    return false;
  }

  unsigned int mn = m1->m * m1->n;

  for (unsigned int i = 0; i < mn; i++)
  {
    if (fabs(m1->ele[i] - m2->ele[i]) > eps)
    {
      return false;
    }
  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool MatNd_isIdentity(const MatNd* self, double eps)
{
  if (self->m != self->n)
  {
    return false;
  }

  for (unsigned int i = 0; i < self->m; i++)
  {

    for (unsigned int j = 0; j < self->n; j++)
    {

      double dstEle = (i==j) ? 1.0 : 0.0;

      if (fabs(MatNd_get2(self, i, j) - dstEle) > eps)
      {
        return false;
      }

    }

  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool MatNd_isDiagonal(const MatNd* A, double eps)
{
  int row, col, n = A->m;

  RCHECK_MSG(A->m == A->n, "Matrix is not square (%d x %d)", A->m, A->n);

  for (row = 0; row < n; row++)
    for (col = 0; col < n; col++)
    {
      if ((row != col) && fabs(MatNd_get(A, row, col)) > eps)
      {
        NLOG(4, "A(%d,%d) = %g", row, col, MatNd_get(A, row, col));
        return false;
      }
    }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
double MatNd_lineSearch(MatNd* q, const MatNd* q0, const MatNd* grad,
                        double (*func)(double[], void*), void* data,
                        int* nEval, double stpmax, bool* converged)
{
  MatNd_copy(q, q0);
  return MatNd_lineSearchArmijo(q, grad, func, data, &stpmax);
}

/*******************************************************************************
 * See header, also for an example.
 ******************************************************************************/
double MatNd_lineSearchSelf(MatNd* x,
                            const MatNd* dfdx,
                            double (*func)(double[], void*),
                            void* data,
                            int* nEval,
                            double stpmax,
                            bool* converged)
{
  return MatNd_lineSearchArmijo(x, dfdx, func, data, &stpmax);
}

/*******************************************************************************
 *
 ******************************************************************************/
double MatNd_lineSearchArmijo(MatNd* x,
                              const MatNd* gradient,
                              double (*costFunction)(double[], void*),
                              void* params,
                              double* alpha_)
{
  const double c1 = 1.0e-4;
  const double rho = 0.9;
  double alpha = (alpha_==NULL) ? 0.9 : (*alpha_);

  MatNd* x_next = NULL;
  MatNd_create2(x_next, x->m, x->n);
  MatNd_constMulAndAdd(x_next, x, gradient, -alpha);

  double f = costFunction(x_next->ele, params);
  const double f0 = costFunction(x->ele, params);

  while (f > f0 + alpha*c1)
  {
    alpha *= rho;
    MatNd_constMulAndAdd(x_next, x, gradient, -alpha);
    f = costFunction(x_next->ele, params);

    // Stuck
    if (alpha < 1.0e-6)
    {
      MatNd_destroy(x_next);
      f = costFunction(x->ele, params);
      RCHECK_EQ(f, f0);
      return f;
    }

  }

  MatNd_copy(x, x_next);
  f = costFunction(x->ele, params);// Why?

  if (alpha_ != NULL)
  {
    *alpha_ = alpha;
  }

  MatNd_destroy(x_next);

  return f;
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_copyBlock(MatNd* dst, const MatNd* src, unsigned int m0,
                     unsigned int n0, unsigned int m1, unsigned int n1)
{
  RCHECK_MSG(m1 >= m0, "m1=%d   m0=%d", m1, m0);
  RCHECK_MSG(n1 >= n0, "n1=%d   n0=%d", n1, n0);
  RCHECK_MSG(dst->m - 1 == m1 - m0, "dst->m=%d m1=%d m0=%d", dst->m, m1, m0);
  RCHECK_MSG(dst->n - 1 == n1 - n0, "dst->n=%d n1=%d n0=%d", dst->n, n1, n0);
  RCHECK_MSG(src->m > m1, "src->m=%d m1=%d", src->m, m1);
  RCHECK_MSG(src->n > n1, "src->n=%d n1=%d", src->n, n1);

  for (unsigned int row = 0; row <= m1 - m0; row++)
  {
    for (unsigned int col = 0; col <= n1 - n0; col++)
    {
      MatNd_set2(dst, row, col, MatNd_get2(src, row + m0, col + n0));
    }
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_copyRows(MatNd* dst,
                    unsigned int dst_start_row,
                    const MatNd* src,
                    unsigned int src_start_row,
                    unsigned int number_of_rows)
{
  if (number_of_rows == 0)
  {
    return;
  }

  RCHECK_MSG(dst->n == src->n, "dst->n = %u  -  src->n = %u", dst->n, src->n);
  RCHECK_MSG(dst->m >= dst_start_row + number_of_rows,
             "dst->m = %u  -  dst_start_row = %u  -  number_of_rows = %u",
             dst->m, dst_start_row, number_of_rows);
  RCHECK_MSG(src->m >= src_start_row + number_of_rows,
             "src->m = %u  -  src_start_row = %u  -  number_of_rows = %u",
             src->m, src_start_row, number_of_rows);

  for (unsigned int row = 0; row < number_of_rows; row++)
  {
    MatNd_copyRow(dst, dst_start_row + row, src, src_start_row + row);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_copyRow(MatNd* dst, unsigned int dst_idx, const MatNd* src,
                   unsigned int src_idx)
{
  // Check for identical number of columns
  RCHECK_MSG(dst->n == src->n, "dst->n=%d   src->n=%d", dst->n, src->n);

  // These functions check for row index out of range
  double* dstPtr = MatNd_getRowPtr(dst, dst_idx);
  const double* srcPtr = MatNd_getRowPtr(src, src_idx);

  // memmove is fine with overlapping memory areas
  memmove(dstPtr, srcPtr, src->n * sizeof(double));
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_setBlockZero(MatNd* dst,
                        unsigned int m0,
                        unsigned int n0,
                        unsigned int m1, unsigned int n1)
{
  unsigned int row, col;

  RCHECK_MSG(m1 >= m0, "m1=%d m0=%d", m1, m0);
  RCHECK_MSG(n1 >= n0, "n1=%d n0=%d", n1, n0);
  RCHECK_MSG(dst->m > m1, "Out of limits dst->m=%d m1=%d", dst->m, m1);
  RCHECK_MSG(dst->n > n1, "Out of limits dst->n=%d n1=%d", dst->n, n1);

  for (row = m0; row <= m1; row++)
  {
    for (col = n0; col <= n1; col++)
    {
      MatNd_set(dst, row, col, 0.0);
    }
  }
}

/*******************************************************************************
 * ABAt = A * B * A^T
 ******************************************************************************/
void MatNd_sqrMulABAt(MatNd* ABAt, const MatNd* A, const MatNd* B)
{

  if (B==NULL)
  {
    const double* aPtr, *atPtr;
    unsigned int i, j, k, i_m, i_n;
    double sum;

    for (i=0; i<A->m ; ++i)
    {
      i_m = i*A->m;
      i_n = i*A->n;

      for (j=i; j<A->m ; ++j)
      {
        sum = 0.0;
        aPtr = &A->ele[i_n];
        atPtr = &A->ele[j*A->n];

        for (k=0; k<A->n; ++k)
        {
          sum += *(aPtr++) * *(atPtr++);
        }

        ABAt->ele[i_m+j] = sum;
        ABAt->ele[j*A->m+i] = sum;
      }
    }
  }
  // Diagonal vector
  else if (B->n==1)
  {
    if (B->m==A->n)
    {
      const double* aPtr, *atPtr, *bPtr;
      unsigned int i, j, k, i_m, i_n;
      double sum;

      for (i=0; i<A->m ; ++i)
      {
        i_m = i*A->m;
        i_n = i*A->n;

        for (j=i; j<A->m ; ++j)
        {
          sum = 0.0;
          aPtr  = &A->ele[i_n];
          atPtr = &A->ele[j*A->n];
          bPtr  = B->ele;

          for (k=0; k<A->n; ++k)
          {
            sum += *(aPtr++) * *(atPtr++) * *(bPtr++);
          }

          ABAt->ele[i_m+j] = sum;
          ABAt->ele[j*A->m+i] = sum;
        }
      }
    }
    else if (B->m==1)
    {
      MatNd_sqrMulABAt(ABAt, A, NULL);
      MatNd_constMulSelf(ABAt, B->ele[0]);
    }
    else
    {
      RFATAL("Dimension mismatch: B is %d x %d, A is %d x %d",
             B->m, B->n, A->m, A->n);
    }

  }
  // Square weighting matrix
  else if (B->n == B->m)
  {
    MatNd* At = NULL, *BAt = NULL;
    MatNd_create2(At, A->n, A->m);
    MatNd_create2(BAt, B->m, At->n);
    MatNd_transpose(At, A);    // A^T
    MatNd_mul(BAt, B, At);     // B * A^T
    MatNd_mul(ABAt, A, BAt);   // A * B * A^T
    MatNd_destroy(BAt);
    MatNd_destroy(At);
  }
  else
  {
    RFATAL("Dimension mismatch: B is %d x %d, A is %d x %d",
           B->m, B->n, A->m, A->n);
  }

}

/*******************************************************************************
 * AtBA = A^T * B * A
 ******************************************************************************/
void MatNd_sqrMulAtBA(MatNd* AtBA, const MatNd* A, const MatNd* B)
{
  // No weighting matrix
  if (B == NULL)
  {
    MatNd_dyadicProduct(AtBA, A);
  }
  // Square weighting matrix
  else if (B->n == B->m)
  {
    MatNd* BA = NULL, *At = NULL;
    MatNd_create2(BA, B->m, A->n);
    MatNd_create2(At, A->n, A->m);
    MatNd_transpose(At, A);    // A^T
    MatNd_mul(BA, B, A);       // B * A
    MatNd_mul(AtBA, At, BA);   // A^T * B * A
    MatNd_destroy(BA);
    MatNd_destroy(At);
  }
  // m x 1 weighting vector
  else if (B->n == 1)
  {
    MatNd* AtB = NULL;
    MatNd_create2(AtB, A->n, A->m);
    MatNd_transpose(AtB, A);
    MatNd_postMulDiagSelf(AtB, B);
    MatNd_mul(AtBA, AtB, A);
    MatNd_destroy(AtB);
  }

}

/*******************************************************************************
 * dst += A^T * B * A
 ******************************************************************************/
void MatNd_sqrMulAndAddAtBA(MatNd* dst, const MatNd* A, const MatNd* B)
{
  MatNd* AtBA = NULL;

  MatNd_create2(AtBA, A->n, A->n);
  MatNd_sqrMulAtBA(AtBA, A, B);
  MatNd_addSelf(dst, AtBA);
  MatNd_destroy(AtBA);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool MatNd_isNAN(const MatNd* self)
{
  int i, nEle = self->m * self->n;

  for (i = 0; i < nEle; i++)
  {
    if (Math_isNAN(self->ele[i]))
    {
      return true;
    }
  }

  return false;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool MatNd_isINF(const MatNd* self)
{
  int i, nEle = self->m * self->n;

  for (i = 0; i < nEle; i++)
  {
    if (Math_isFinite(self->ele[i]) == false)
    {
      return true;
    }
  }

  return false;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool MatNd_isFinite(const MatNd* self)
{
  int i, nEle = self->m * self->n;

  for (i = 0; i < nEle; i++)
  {
    if (Math_isFinite(self->ele[i]) == false)
    {
      return false;
    }
  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
MatNd MatNd_fromPtr(int m, int n, double* ptr)
{
  MatNd self;

  self.m = m;
  self.n = n;
  self.size = m * n;
  self.ele = ptr;
  self.stackMem = true; // Somebody else deletes ptr

  return self;
}

/*******************************************************************************
 *
 ******************************************************************************/
MatNd MatNd_fromMat3d(double mat[3][3])
{
  MatNd self;

  self.m = 3;
  self.n = 3;
  self.size = 9;
  self.ele = &mat[0][0];
  self.stackMem = true; // Somebody else deletes ptr

  return self;
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_preMulDiagSelf(MatNd* dst, const MatNd* vec)
{
  unsigned int i, j;

  RCHECK_MSG((dst->m == vec->m) || (vec->m == 1),
             "dst: [%d x %d]   vec: [%d x %d]",
             dst->m, dst->n, vec->m, vec->n);
  RCHECK(vec->n == 1);

  if (vec->m == 1)
  {
    MatNd_constMulSelf(dst, vec->ele[0]);
    return;
  }

  // Loop over all rows
  for (i = 0; i < dst->m; i++)
  {
    double* row = MatNd_getRowPtr(dst, i);

    // Each row is multiplied with the corresponding value of vec
    for (j = 0; j < dst->n; j++)
    {
      row[j] *= vec->ele[i];
    }
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_postMulDiagSelf(MatNd* dst, const MatNd* vec)
{
  unsigned int i, j;

  RCHECK_MSG((dst->n == vec->m) || (vec->m == 1), "dst->n=%d   vec->n=%d",
             dst->n, vec->m);
  RCHECK(vec->n == 1);

  if (vec->m == 1)
  {
    MatNd_constMulSelf(dst, vec->ele[0]);
    return;
  }

  // Loop over all rows
  for (i = 0; i < dst->m; i++)
  {
    double* row = MatNd_getRowPtr(dst, i);

    // Each row of dst is element-wise multiplied with vec
    for (j = 0; j < dst->n; j++)
    {
      row[j] *= vec->ele[j];
    }
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_columnDerivative(MatNd* dst, const MatNd* src)
{
  unsigned int i, j;
  double currEle, nextEle, *lastRow, *secondLastRow;
  RCHECK_MSG(src->m > 0, "Matrix to differentiate must have at least one row, "
             "but has %d rows",
             src->m);

  MatNd_reshape(dst, src->m, src->n);

  for (i = 0; i < dst->m - 1; i++)
  {
    for (j = 0; j < dst->n; j++)
    {
      currEle = MatNd_get(src, i, j);
      nextEle = MatNd_get(src, i + 1, j);

      MatNd_set(dst, i, j, nextEle - currEle);
    }
  }

  // Copy the last row from the second last one. We assume a constant
  // derivative here.
  lastRow = MatNd_getRowPtr(dst, dst->m - 1);
  secondLastRow = MatNd_getRowPtr(dst, dst->m - 2);
  memcpy(lastRow, secondLastRow, dst->n * sizeof(double));
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_columnIntegral(MatNd* x_int, const MatNd* x, const MatNd* x0)
{
  MatNd_reshape(x_int, x->m, x->n);
  MatNd_setZero(x_int);

  if (x0 != NULL)
  {
    RCHECK(x0->n == x->n);
    RCHECK(x0->m == 1);

    // If x0 is given, we copy it to the first row, and add the first row
    // of x to it
    VecNd_add(x_int->ele, x0->ele, x->ele, x->n);
  }
  else
  {
    VecNd_copy(x_int->ele, x->ele, x->n);
  }

  for (unsigned int i = 1; i < x->m; i++)
  {
    const double* xi = MatNd_getRowPtr(x, i);
    const double* x_int_i_prev = MatNd_getRowPtr(x_int, i - 1);
    double* x_int_i = MatNd_getRowPtr(x_int, i);
    VecNd_add(x_int_i, x_int_i_prev, xi, x->n);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_columnLength(MatNd* x_length, const MatNd* x)
{
  RCHECK_EQ(x_length->m, x->n);
  RCHECK_EQ(x_length->n, 1);

  MatNd_setZero(x_length);

  for (unsigned int i = 1; i < x->m; i++)
  {
    const double* xi = MatNd_getRowPtr(x, i);
    const double* xi_prev = MatNd_getRowPtr(x, i - 1);

    for (unsigned int j = 0; j < x->n; j++)
    {
      x_length->ele[j] += fabs(xi[j] - xi_prev[j]);
    }
  }
}


/*******************************************************************************
 * N = I - pinv(J) J
 ******************************************************************************/
void MatNd_nullspace(MatNd* N, const MatNd* J_pinv, const MatNd* J)
{
  unsigned int i, j, k;

  MatNd_mul(N, J_pinv, J);

  for (i = 0; i < J->n; i++)
    for (j = 0; j < J->n; j++)
    {
      k = i * J->n + j;
      N->ele[k] = (i == j) ? (1.0 - N->ele[k]) : (-N->ele[k]);
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_calcMeanAndCovariance(const MatNd* A, double* mu, MatNd* sigma)
{
  RCHECK(A->n == sigma->m && A->n == sigma->n);

  MatNd* X = NULL;
  MatNd_create2(X, A->m, 1);
  MatNd* Y = NULL;
  MatNd_create2(Y, A->m, 1);

  for (unsigned int row = 0; row < A->n; row++)
  {
    MatNd_getColumn(X, row, A);
    mu[row] = MatNd_mean(X);

    // Covariance matrix is symmetric, therefore only one half is calculated
    for (unsigned int col = row; col < A->n; col++)
    {
      MatNd_getColumn(Y, col, A);

      double covariance = VecNd_covariance(X->ele, Y->ele, A->m);

      MatNd_set(sigma, row, col, covariance);
    }

    for (unsigned int col = 0; col < row; col++)
    {
      MatNd_set(sigma, row, col, MatNd_get(sigma, col, row));
    }
  }

  MatNd_destroy(X);
  MatNd_destroy(Y);
}

/*******************************************************************************
 *
 * \brief Computes the partial derivative of the pseudo inverse.
 *
 * del_q(J#) = -J# dq(J) J# + N invW dq(J^T) (J invW J^T)^-1
 *
 * There's a paper by Golub et al.:
 *
 * The Differentiation of Pseudo-Inverses and Nonlinear Least Squares
 * Problems Whose Variables Separate. Author(s): G. H. Golub and
 * V. Pereyra. Source: SIAM Journal on Numerical Analysis, Vol. 10,
 * No. 2 (Apr., 1973), pp. 413-432
 *
 * del_q(J#) = -J# dq(J) J# + J# J#^T dq(J^T) (I - J J#)
 * + (I - J# J) dq(J^T) J#^T J#
 *
 * It's stated just for reference, however it is possible to get it into
 * our form: with J J# = I, the second term vanishes and we get
 *
 * del_q(J#) = -J# dq(J) J# + (I - J# J) dq(J^T) J#^T J#
 *
 * The term J#^T J# written out is: (J J^T)^-1 J J#
 *
 * where again J J# = I. With N = (I - J# J) we get
 *
 * del_q(J#) = -J# dq(J) J# + N dq(J^T) (J J^T)^-1
 *
 ******************************************************************************/
void MatNd_PinvHessian(MatNd* dqJpinv, const MatNd* J, const MatNd* dqJ,
                       const MatNd* invW, const MatNd* invJWJt,
                       const MatNd* Jpinv, bool transposed)
{
  int nx = J->m, nq = J->n;

  MatNd_reshape(dqJpinv, nx * nq, nq);

  // WJt = W*J^T
  MatNd* WJt = NULL;
  MatNd_create2(WJt, nq, nx);
  MatNd_transpose(WJt, J);

  if (invW != NULL)
  {
    MatNd_preMulDiagSelf(WJt, invW);
  }

  // invJWJt = (J invW J^T)^-1
  bool invJWJtAlloc = false;

  if (invJWJt == NULL)
  {
    invJWJt = MatNd_create(nx, nx);
    MatNd_mul((MatNd*) invJWJt, J, WJt);
    MatNd_choleskyInverse((MatNd*) invJWJt, invJWJt);
    invJWJtAlloc = true;
  }

  // Pseudoinverse
  bool JpinvAlloc = false;

  if (Jpinv == NULL)
  {
    Jpinv = MatNd_create(nq, nx);
    MatNd_mul((MatNd*) Jpinv, WJt, invJWJt);
    JpinvAlloc = true;
  }

  // -J# del_q(J) J# = (nq x nx)*(nx x nq x nq)*(nq x nx)
  //                 = (nq x nx)*(nx x nq x nx) = (nq x nq x nx)
  //                 = (nq x nq x nq)*(nq x nx) = (nq x nq x nx)

  // del_q(J) J# = (nx x nq x nq)*(nq x nx) = (nx x nq x nx)
  MatNd* term1 = NULL;
  MatNd_create2(term1, nx * nq, nx);
  MatNd_mul(term1, dqJ, Jpinv); // (nx*nq x nx) = (nx*nq x nq)*(nq x nx)

  // J# term1 = (nq x nx) (nx x nq x nx) = (nq x nq x nx)
  MatNd* term2 = NULL;
  MatNd_create2(term2, nq, nq * nx);
  MatNd_reshape(term1, nx, nx * nq);
  MatNd_mul(term2, Jpinv, term1); // (nq*nq x nx) = (nq x nx)*(nx x nx*nq)

  // N = (I - J# J) Wq^-1
  MatNd* NW = NULL;
  MatNd_create2(NW, nq, nq);
  MatNd_nullspace(NW, Jpinv, J);

  if (invW != NULL)
  {
    MatNd_postMulDiagSelf(NW, invW);  // N invW
  }

  // NW*H^T*iJ3
  MatNd* dqJtp = NULL;
  MatNd_create2(dqJtp, nq * nq, nx);
  int d0 = nq, d1 = nq, d2 = nx;

  for (int i = 0; i < d0; i++)
    for (int j = 0; j < d1; j++)
      for (int k = 0; k < d2; k++)
      {
        dqJtp->ele[(i*d1+j)*d2+k] = dqJ->ele[(k*d1+j)*d0+i];  // H^T
      }

  MatNd* term3 = NULL;
  MatNd_create2(term3, nq * nq, nx); // term3 = H^T*iJ3
  MatNd_mul(term3, dqJtp, invJWJt);
  MatNd* dJpinv = NULL;
  MatNd_create2(dJpinv, nq, nq * nx);
  MatNd_reshape(term3, nq, nx * nq);
  MatNd_mul(dJpinv, NW, term3); // dJpinv = NW*term3
  MatNd_subSelf(dJpinv, term2); // dJpinv = ...      - term2

  // Change memory alignment to first dimension along q
  if (transposed == false)
  {
    d0 = nx, d1 = nq, d2 = nq;
    for (int i = 0; i < d0; i++)
      for (int j = 0; j < d1; j++)
        for (int k = 0; k < d2; k++)
        {
          dqJpinv->ele[(i*d1+j)*d2+k] = dJpinv->ele[(k*d1+j)*d0+i];
        }

    MatNd fA = MatNd_fromPtr(nx * nq, nq, dqJpinv->ele);
    MatNd_transposeSelf(&fA);
  }
  else
  {
    memcpy(dqJpinv->ele, dJpinv->ele, nx * nq * nq * sizeof(double));
  }

  MatNd_destroy(WJt);
  MatNd_destroy(NW);
  MatNd_destroy(dqJtp);
  MatNd_destroy(term1);
  MatNd_destroy(term2);
  MatNd_destroy(term3);
  MatNd_destroy(dJpinv);

  if (invJWJtAlloc == true)
  {
    MatNd_destroy((MatNd*) invJWJt);
  }

  if (JpinvAlloc == true)
  {
    MatNd_destroy((MatNd*) Jpinv);
  }
}

/*******************************************************************************
 * Computes the partial derivative of the pseudo inverse:
 * del_q(J#) = -J# dq(J) J# + inv(JtWJ) dqJ^T Wx (I - J J#)
 ******************************************************************************/
void MatNd_PinvHessian2(MatNd* dqJpinv, const MatNd* J, const MatNd* dqJ,
                        const MatNd* invWx, const MatNd* invJtWJ,
                        const MatNd* Jpinv, bool transposed)
{
  int nx = Jpinv->n, nq = Jpinv->m;

  MatNd_reshape(dqJpinv, nx * nq, nq);

  // -J# del_q(J) J#

  // term1 = del_q(J) J#
  MatNd* term1 = NULL;
  MatNd_create2(term1, nx * nq, nx);
  MatNd_mul(term1, dqJ, Jpinv);

  // dJpinv = J# del_q(J) J#
  MatNd* dJpinv = NULL;
  MatNd_create2(dJpinv, nq, nq * nx);
  MatNd_reshape(term1, nx, nx * nq);
  MatNd_mul(dJpinv, Jpinv, term1);
  MatNd_constMulSelf(dJpinv, -1.0);

  // inv(JtWJ) dqJ^T Wx (I - J J#)

  // term2 = dqJ^T Wx
  MatNd* term2 = NULL;
  MatNd_create2(term2, nq * nq, nx);
  int d0 = nq, d1 = nq, d2 = nx;
  for (int i = 0; i < d0; i++)
    for (int j = 0; j < d1; j++)
      for (int k = 0; k < d2; k++)
      {
        term2->ele[(i * d1 + j) * d2 + k] = dqJ->ele[(k * d1 + j) * d0 + i];
      }
  if (invWx)
  {
    MatNd_postMulDiagSelf(term2, invWx);
  }

  // term3 = (I - J J#)
  MatNd* term3 = NULL;
  MatNd_create2(term3, nx, nx);
  MatNd_mul(term3, J, Jpinv);
  MatNd_constMulSelf(term3, -1.0);
  MatNd_addConstToDiag(term3, 1.0);

  // term4 = dqJ^T Wx (I - J J#) = term2*term3
  MatNd* term4 = NULL;
  MatNd_create2(term4, nq * nq, nx);
  MatNd_mul(term4, term2, term3);

  // term5 = inv(JtWJ) dqJ^T Wx (I - J J#) = invJtWJ*term4
  MatNd* term5 = NULL;
  MatNd_create2(term5, nq, nx * nq);
  MatNd_reshape(term4, nq, nx * nq);
  MatNd_mul(term5, invJtWJ, term4);

  // dJpinv += inv(JtWJ) dqJ^T Wx (I - J J#)
  MatNd_reshape(term5, nq, nq * nx);
  MatNd_addSelf(dJpinv, term5);

  // Change memory alignment to first dimension along q
  if (transposed == false)
  {
    int d0 = nx, d1 = nq, d2 = nq;
    for (int i = 0; i < d0; i++)
      for (int j = 0; j < d1; j++)
        for (int k = 0; k < d2; k++)
        {
          dqJpinv->ele[(i * d1 + j) * d2 + k] = dJpinv->ele[(k*d1+j)*d0+i];
        }

    MatNd fA = MatNd_fromPtr(nx * nq, nq, dqJpinv->ele);
    MatNd_transposeSelf(&fA);
  }
  else
  {
    memcpy(dqJpinv->ele, dJpinv->ele, nx * nq * nq * sizeof(double));
  }

  MatNd_destroy(term1);
  MatNd_destroy(term2);
  MatNd_destroy(term3);
  MatNd_destroy(term4);
  MatNd_destroy(term5);
  MatNd_destroy(dJpinv);
}

/*******************************************************************************
 * Data must be a n x 2 vector. Each column holds the data for one dimension:
 *
 *  data =
 * | x_0    y_0   |
 * | x_1    y_1   |
 * |      .       |
 * |      .       |
 * |      .       |
 * | x_n-1  y_n-1 |
 *
 ******************************************************************************/
static bool MatNd_lineFit2D_(double* A, double* B, const MatNd* data)
{
  unsigned int i;
  double x = 0.0, xx = 0.0, y = 0.0, xy = 0.0, xi, yi, det, invMat[2][2];
  MatNd invMatNd;

  if (data->m < 2)
  {
    RLOG(4, "Can't perform line fit for less than 2 points! "
         "You gave me only %d", data->m);
    return false;
  }

  RCHECK(data->n == 2);

  for (i = 0; i < data->m; i++) // number of samples
  {
    xi = MatNd_get2(data, i, 0);
    yi = MatNd_get2(data, i, 1);

    x += xi;
    xx += xi * xi;
    y += yi;
    xy += xi * yi;
  }

  invMat[0][0] = xx;
  invMat[0][1] = x;
  invMat[1][0] = x;
  invMat[1][1] = data->m;

  invMatNd = MatNd_fromPtr(2, 2, (double*) invMat);
  det = MatNd_inverse2DSelf(&invMatNd);

  if (det == 0.0)
  {
    RLOG(4, "Line fit failed - can't invert covariance matrix!");
    return false;
  }

  const double paramA = invMat[0][0] * xy + invMat[0][1] * y;
  const double paramB = invMat[1][0] * xy + invMat[1][1] * y;

  *A = paramA;
  *B = paramB;

  return true;
}

bool MatNd_lineFit2D(double* A, double* B, const MatNd* data)
{
  MatNd* dataTp = MatNd_clone(data);
  MatNd_transposeSelf(dataTp);

  const double xMin = VecNd_minEle(&dataTp->ele[0], dataTp->n);
  const double xMax = VecNd_maxEle(&dataTp->ele[0], dataTp->n);
  const double yMin = VecNd_minEle(&dataTp->ele[dataTp->n], dataTp->n);
  const double yMax = VecNd_maxEle(&dataTp->ele[dataTp->n], dataTp->n);

  const double xRange = xMax - xMin;
  const double yRange = yMax - yMin;

  if (xRange >= yRange)
  {
    MatNd_destroy(dataTp);
    return MatNd_lineFit2D_(A, B, data);
  }

  // x = Ay +B => y = (x - B) / A = (1/A)*x -B/A
  for (unsigned int i=0; i<dataTp->n; ++i)
  {
    MatNd_swapElements(dataTp, 0, i, 1, i);
  }
  MatNd_transposeSelf(dataTp);

  double A2, B2;
  bool success = MatNd_lineFit2D_(&A2, &B2, dataTp);

  if (A2==0.0)
  {
    RLOG(1, "Found vertical data points - setting line inclination to 1.0e8");
    A2 = 1.0e-8;
  }

  *A = 1.0/A2;
  *B = -B2/A2;

  MatNd_destroy(dataTp);

  return success;
}

/*******************************************************************************
 * a dot b = Sum_i(a_i, b_i)
 ******************************************************************************/
double MatNd_innerProduct(const MatNd* a, const MatNd* b)
{
  unsigned int i;
  double a_dot_b = 0.0;

  RCHECK(a->n == 1);
  RCHECK(b->n == 1);
  RCHECK(a->m == b->m);

  for (i = 0; i < a->m; i++)
  {
    a_dot_b += a->ele[i] * b->ele[i];
  }

  return a_dot_b;
}

/*******************************************************************************
 * cos(phi) = (a dot b) / ( |a| * |b|)
 ******************************************************************************/
double MatNd_diffAngle(const MatNd* a, const MatNd* b)
{
  unsigned int i;
  double a_dot_b = 0.0, norm_a = 0.0, norm_b = 0.0, cosPhi;

  RCHECK(a->n == 1);
  RCHECK(b->n == 1);
  RCHECK(a->m == b->m);

  for (i = 0; i < a->m; i++)
  {
    a_dot_b += a->ele[i] * b->ele[i];
    norm_a += a->ele[i] * a->ele[i];
    norm_b += b->ele[i] * b->ele[i];
  }

  norm_a = sqrt(norm_a);
  norm_b = sqrt(norm_b);

  if ((norm_a > 0.0) && (norm_b > 0.0))
  {
    cosPhi = a_dot_b / (norm_a * norm_b);
  }
  else
  {
    cosPhi = M_PI_2;
  }

  return Math_acos(cosPhi);
}

/*******************************************************************************
 *
 * \brief c is the projection of b on a
 * Vectors are interpreted as columns
 *
 * c = [ (a dot b) / (b dot b) ] * b
 *
 * The same, but less efficient (because of sqrt and vector by scalar
 * division), is:
 *
 * c = [ (a dot b) / |b| ] * [ b / |b| ]
 *
 * Vector projection
 *
 * The vector projection of a on b is a vector c which is either null or
 * parallel to b. More exactly,
 *
 * c = 0 if phi = 90 degrees
 * c and b have the same direction if 0 < phi < 90 degrees
 * c and b have opposite directions if 90 < phi < 180 degrees
 *
 * Explanations from: http://en.wikipedia.org/wiki/Vector_projection
 *
 ******************************************************************************/
void MatNd_vectorProjection(MatNd* c, const MatNd* b, const MatNd* a)
{
  unsigned int row;
  double a_dot_b, b_dot_b, scale;

  // Arrays must have the same number of rows
  RCHECK(a->m == b->m);
  RCHECK(a->m == c->m);

  // A vector is projected on one vector
  if ((a->n == 1) && (b->n == 1))
  {
    RCHECK(c->n == 1);

    a_dot_b = 0.0;
    b_dot_b = 0.0;

    for (row = 0; row < a->m; row++)
    {
      a_dot_b += a->ele[row] * b->ele[row];
      b_dot_b += b->ele[row] * b->ele[row];
    }

    scale = (b_dot_b > 0.0) ? a_dot_b / b_dot_b : 0.0;

    MatNd_constMul(c, b, scale);
  }
  // Other cases still to be implemented
  else
  {
    RFATAL("a->n=%d   b->n=%d   -   not implemented!", a->n, b->n);
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
static inline double minDouble(double a, double b)
{
  return a < b ? a : b;
}

/*******************************************************************************
 *
 ******************************************************************************/
double MatNd_DTW(MatNd* warped, const MatNd* dst, const MatNd* src,
                 const MatNd* weight)
{
  if (warped != NULL)
  {
    MatNd_reshapeAndSetZero(warped, dst->m, dst->n);
  }

  RCHECK_MSG(src->n == dst->n, "Dimension mismatch: src->n=%d dst->n=%d. Please"
             " make sure that the arrays src and dst have the same number of "
             "columns",
             src->n, dst->n);

  if (weight != NULL)
  {
    RCHECK_MSG(weight->m == dst->n, "%d != %d", weight->m, dst->n);
    RCHECK_MSG(weight->n == 1, "weight->n=%d", weight->n);
  }

  unsigned int N = dst->m, M = src->m, n, m;
  double val, result;

  MatNd* d = NULL;
  MatNd_create2(d, N, M);
  MatNd* v1 = NULL;
  MatNd_create2(v1, dst->n, 1);
  MatNd* v2 = NULL;
  MatNd_create2(v2, src->n, 1);

  // Calculate distance surface of each possible data pair
  for (n = 0; n < N; n++)
  {
    MatNd_getRow(v1, n, dst);

    for (m = 0; m < M; m++)
    {
      MatNd_getRow(v2, m, src);
      MatNd_subSelf(v2, v1);

      if (weight != NULL)
      {
        // masking unimportant columns by multiplying with mask
        MatNd_transposeSelf(v2);
        MatNd_eleMulSelf(v2, weight);
        MatNd_transposeSelf(v2);
      }

      val = MatNd_getNorm(v2); // euclidian norm
      MatNd_set(d, n, m, val);
    }
  }

  MatNd_destroy(v1);
  MatNd_destroy(v2);

  // Find the cheapest path through the distance surface
  MatNd* D = NULL;
  MatNd_create2(D, N, M);

  // First element
  MatNd_set(D, 0, 0, MatNd_get(d, 0, 0));

  // First column
  for (n = 1; n < N; n++)
  {
    val = MatNd_get(d, n, 0) + MatNd_get(D, n - 1, 0);
    MatNd_set(D, n, 0, val);
  }

  // First row
  for (m = 1; m < M; m++)
  {
    val = MatNd_get(d, 0, m) + MatNd_get(D, 0, m - 1);
    MatNd_set(D, 0, m, val);
  }

  // The rest of the matrix
  for (n = 1; n < N; n++)
  {
    for (m = 1; m < M; m++)
    {
      val = MatNd_get(d, n, m)
            + minDouble(MatNd_get(D, n - 1, m), minDouble(MatNd_get(D, n - 1, m - 1), MatNd_get(D, n, m - 1)));
      MatNd_set(D, n, m, val);
    }
  }
  result = MatNd_get(D, N - 1, M - 1);

  // Morph source data to the length of the target data
  // Maps every m to an n
  n = N - 1;
  m = M - 1;

  if (warped != NULL)
  {
    MatNd row = MatNd_getRowView((MatNd*) src, m);
    MatNd_setRow(warped, n, row.ele, row.n);
  }

  while (n + m > 0)
  {
    if (n == 0)
    {
      // We are in row 0 now get to (0, 0)
      m--;
    }
    else if (m == 0)
    {
      // We are in column 0 now get to (0, 0)
      n--;
    }
    else
    {
      // We are within the matrix, follow the cheapest path
      if (MatNd_get(D, n - 1, m) < MatNd_get(D, n, m - 1))
      {
        n--;
        if (MatNd_get(D, n, m - 1) <= MatNd_get(D, n, m))
        {
          m--;
        }
      }
      else
      {
        m--;
        if (MatNd_get(D, n - 1, m) <= MatNd_get(D, n, m))
        {
          n--;
        }
      }
    }

    if (warped != NULL)
    {
      MatNd row = MatNd_getRowView((MatNd*) src, m);
      MatNd_setRow(warped, n, row.ele, row.n);
    }
  }

  MatNd_destroy(D);
  MatNd_destroy(d);

  return result;
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_applyFctEle(MatNd* dst, double (*func)(double))
{
  unsigned int i, nEle = dst->m * dst->n;

  for (i = 0; i < nEle; i++)
  {
    dst->ele[i] = (*func)(dst->ele[i]);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_fabsEleSelf(MatNd* self)
{
  MatNd_applyFctEle(self, fabs);
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_powEle(MatNd* dst, const MatNd* src, double exponent)
{
  RCHECK((dst->m == src->m) && (dst->n == src->n));

  unsigned int i, nEle = dst->m * dst->n;

  for (i = 0; i < nEle; i++)
  {
    dst->ele[i] = pow(src->ele[i], exponent);
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_powEleSelf(MatNd* self, double exponent)
{
  unsigned int i, nEle = self->m * self->n;

  for (i = 0; i < nEle; i++)
  {
    self->ele[i] = pow(self->ele[i], exponent);
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_sigmoidExponentialEle(MatNd* sigX, const MatNd* x, double beta)
{
  RCHECK((sigX->m == x->m) && (sigX->n == x->n));

  unsigned int i, nEle = x->m * x->n;

  for (i = 0; i < nEle; i++)
  {
    sigX->ele[i] = 1.0 / (1.0 + exp(-beta * x->ele[i]));
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_interpolateRows(MatNd* dst, const MatNd* src)
{
  RCHECK(dst);
  RCHECK(src);
  RCHECK(dst->n == src->n);
  RCHECK(dst->m > 1);
  RCHECK(src->m > 1);

  if (dst->m == src->m)
  {
    MatNd_copy(dst, src);
  }
  else
  {
    unsigned int cols = dst->n;
    unsigned int rows = dst->m;

    double step = (double)(src->m - 1) / (double)(dst->m - 1);

    MatNd* col_data = NULL;
    MatNd_create2(col_data, src->m, 1);
    for (unsigned int col = 0; col < cols; col++)
    {
      MatNd_getColumn(col_data, col, src);

      for (unsigned int row = 0; row < rows; row++)
      {
        MatNd_set(dst, row, col,
                  Math_interpolateLinear(row*step, col_data->ele, col_data->m));
      }
    }
    MatNd_destroy(col_data);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_interpolateRowsEuler(MatNd* dst, const MatNd* src)
{
  RCHECK(dst);
  RCHECK(src);
  RCHECK(dst->n == 3);
  RCHECK(src->n == 3);
  RCHECK(dst->m > 1);
  RCHECK(src->m > 1);

  double step = (double)(src->m - 1) / (double)(dst->m - 1);

  MatNd* col_data = NULL;
  MatNd_create2(col_data, src->m, 1);

  for (unsigned int row = 0; row < dst->m; row++)
  {
    double dst_row[3];

    double x = row * step;

    if (x <= 0.0)
    {
      Vec3d_copy(dst_row, MatNd_getRowPtr(src, 0));
    }
    else if (x >= src->m - 1)
    {
      Vec3d_copy(dst_row, MatNd_getRowPtr(src, src->m - 1));
    }
    else
    {
      unsigned int j = (unsigned int) x;

      Vec3d_slerp(dst_row, MatNd_getRowPtr(src, j),
                  MatNd_getRowPtr(src, j + 1), x - j);
    }

    MatNd_setRow(dst, row, dst_row, 3);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_reverseSelf(MatNd* mat)
{
  MatNd* row = NULL;
  MatNd_create2(row, 1, mat->n);

  for (unsigned int i = 0; i < mat->m / 2; ++i)
  {
    MatNd_getRow(row, i, mat);
    MatNd_setRow(mat, i, MatNd_getRowPtr(mat, mat->m - i - 1), mat->n);
    MatNd_setRow(mat, mat->m - i - 1, row->ele, mat->n);
  }

  MatNd_destroy(row);
}

/*******************************************************************************
 *
 ******************************************************************************/
double MatNd_trace(const MatNd* self)
{
  RCHECK(self->m == self->n);

  double trace = 0.0;
  for (unsigned int i = 0; i < self->m; i++)
  {
    trace += MatNd_get2(self, i, i);
  }

  return trace;
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_rowLerp(MatNd* dst, const MatNd* src, const double s)
{
  unsigned int cols = src->n;
  unsigned int rows = src->m;

  RCHECK_MSG(rows>0, "Can't interpolate between 0 rows - exiting");

  MatNd_reshape(dst, 1, cols);

  if (rows==1)
  {
    MatNd_copy(dst, src);
    return;
  }

  if (cols==0)
  {
    return;
  }

  if (s <= 0.0)
  {
    MatNd_fromArray(dst, MatNd_getRowPtr(src, 0), cols);
    return;
  }
  else if (s >= 1.0)
  {
    MatNd_fromArray(dst, MatNd_getRowPtr(src, rows - 1), cols);
    return;
  }

  double step = s * (rows - 1.0);
  unsigned int lowIdx = (unsigned int) floor(step);
  unsigned int highIdx = (unsigned int) ceil(step);
  double ds = step - lowIdx;

  MatNd row0 = MatNd_getRowView(src, lowIdx);
  MatNd row1 = MatNd_getRowView(src, highIdx);

  MatNd_sub(dst, &row1, &row0);
  MatNd_constMulSelf(dst, ds);
  MatNd_addSelf(dst, &row0);
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_interpolateLinear(MatNd* dst, const MatNd* src,
                             unsigned int subSteps)
{
  unsigned int rows = src->m, cols = src->n, i, j, k;
  unsigned int transitions = rows - 1;

  MatNd_reshapeAndSetZero(dst, subSteps * transitions + 1, cols);

  for (i = 0; i < transitions; i++)
  {
    const double* currRow = MatNd_getRowPtr(src, i);
    const double* nextRow = MatNd_getRowPtr(src, i + 1);

    for (j = 0; j <= subSteps; j++)
    {
      double kappa = (double) j / subSteps;

      for (k = 0; k < cols; k++)
      {
        MatNd_set(dst, i * subSteps + j, k,
                  currRow[k] + kappa * (nextRow[k] - currRow[k]));
      }

    }
  }

}

/*******************************************************************************
 * Computes 5th order polynomial. Could be more efficient:
 *
 * - pre-compute x_dot and x_ddot before loop
 * - pre-compute ti and its exponents
 *
 ******************************************************************************/
void MatNd_interpolateFifthOrder(MatNd* dst, const MatNd* src,
                                 unsigned int subSteps)
{
  RCHECK_MSG(subSteps > 0, "Subdivisions must be >=1, but is %d", subSteps);

  MatNd_reshape(dst, (src->m - 1) * subSteps + 1, src->n);

  // Check if interpolation is needed, if not we just copy the matrices and
  // return
  if (subSteps == 1)
  {
    MatNd_copy(dst, src);
    return;
  }

  MatNd* srcExt = MatNd_create(src->m + 4, src->n);
  MatNd srcExtSub = MatNd_fromPtr(src->m, src->n, MatNd_getRowPtr(srcExt, 2));
  MatNd_copy(&srcExtSub, src);

#if 0
  MatNd_copyRow(srcExt, 1, src, 1);
  MatNd_copyRow(srcExt, 0, src, 2);
  MatNd_copyRow(srcExt, src->m + 2, src, src->m - 2);
  MatNd_copyRow(srcExt, src->m + 3, src, src->m - 3);
#else
  // Initial boundary condition on velocity level:
  // dx = 0.5 * (x(t+1) - x(t-1))
  // => x(t-1) = x(t+1)
  MatNd_copyRow(srcExt, 1, src, 1);

  // Initial boundary condition on acceleration level:
  // ddx = 0.25 * (x(t+2) - 2.0 * x(t) + x(t-2))
  // => x(t-2) = 2.0 * x(t) - x(t+2)
  double* rowTm2 = MatNd_getRowPtr(srcExt, 0);
  double* srcT   = MatNd_getRowPtr(src, 0);
  double* srcTp2 = MatNd_getRowPtr(src, 2);

  for (unsigned int i=0; i<dst->n; i++)
  {
    rowTm2[i] = 2.0*srcT[i] - srcTp2[i];
  }

  // Final boundary condition on velocity level:
  // dx = 0.5 * (x(t+1) - x(t-1))
  // => x(t+1) = x(t-1)
  MatNd_copyRow(srcExt, src->m + 2, src, src->m - 2);


  // Final boundary condition on acceleration level:
  // ddx = 0.25 * (x(t+2) - 2.0 * x(t) + x(t-2))
  // => x(t+2) = 2.0 * x(t) - x(t-2)
  double* rowTp2 = MatNd_getRowPtr(srcExt, src->m + 3);
  srcT           = MatNd_getRowPtr(src, src->m-1);
  double* srcTm2 = MatNd_getRowPtr(src, src->m-3);

  for (unsigned int i=0; i<dst->n; i++)
  {
    rowTp2[i] = 2.0*srcT[i] - srcTm2[i];
  }
#endif

  // Compute coefficient matrix
  double coeffMat[3][3];

  coeffMat[0][0] = 0.5;
  coeffMat[0][1] = -3.0;
  coeffMat[0][2] = 6.0;

  coeffMat[1][0] = -1.0;
  coeffMat[1][1] = 7.0;
  coeffMat[1][2] = -15.0;

  coeffMat[2][0] = 0.5;
  coeffMat[2][1] = -4.0;
  coeffMat[2][2] = 10.0;

  // Compute interpolation
  for (unsigned int column = 0; column < src->n; column++)
  {
    for (unsigned int row = 2; row < srcExt->m - 3; row++)
    {
      // Assemble right hand side
      double xtm2 = MatNd_get2(srcExt, row - 2, column);
      double xtm1 = MatNd_get2(srcExt, row - 1, column);
      double xt   = MatNd_get2(srcExt, row, column);
      double xtp1 = MatNd_get2(srcExt, row + 1, column);
      double xtp2 = MatNd_get2(srcExt, row + 2, column);
      double xtp3 = MatNd_get2(srcExt, row + 3, column);

      // Velocities
      double x_dot0 = 0.5 * (xtp1 - xtm1);
      double x_dot1 = 0.5 * (xtp2 - xt);

      // Accelerations
      double x_ddot0 = 0.25 * (xtp2 - 2 * xt + xtm2);
      double x_ddot1 = 0.25 * (xtp3 - 2 * xtp1 + xtm1);

      // Right hand side
      double rhs[3];
      rhs[0] = x_ddot1 - x_ddot0;
      rhs[1] = x_dot1 - x_dot0 - x_ddot0;
      rhs[2] = xtp1 - xt - 0.5*x_ddot0 - x_dot0;

      // Calculate 5th order coefficients
      double coeff[6];
      Vec3d_rotate(coeff, coeffMat, rhs);
      coeff[3] = 0.5*x_ddot0;
      coeff[4] = x_dot0;
      coeff[5] = xt;

      // Interpolation

      // Keep initial values for each segment
      MatNd_set2(dst, (row - 2) * subSteps, column,
                 MatNd_get2(src, row-2, column));

      // Go through sub steps
      for (unsigned int i = 1; i < subSteps; i++)
      {
        double ti = (double) i / subSteps;
        double ti2 = ti * ti;
        double ti3 = ti2 * ti;
        double ti4 = ti2 * ti2;
        double ti5 = ti4 * ti;
        double xi = coeff[0] * ti5 + coeff[1] * ti4 + coeff[2] * ti3 +
                    coeff[3] * ti2 + coeff[4] * ti + coeff[5];
        MatNd_set2(dst, (row - 2) * subSteps + i, column, xi);
      }

    }

  }   // for(unsigned int column=0;column<src->n;column++)

  // Copy last state
  MatNd_copyRow(dst, dst->m - 1, src, src->m - 1);

  // Clean up
  MatNd_destroy(srcExt);
}

/*******************************************************************************
 * Rolling average filter.
 ******************************************************************************/
void MatNd_filterMovingMean(MatNd* dst, const MatNd* src, unsigned int window)
{
  RCHECK((src->m == dst->m) && (src->n == dst->n));

  unsigned int dataPoints = src->m;
  unsigned int filterSteps = dataPoints + window - 1;

  MatNd* filteredData = MatNd_create(filterSteps, src->n);

  // Initial vector
  MatNd_copyRow(filteredData, 0, src, 0);
  VecNd_constMulSelf(filteredData->ele, window, filteredData->n);

  // Fill initial rows
  for (unsigned int i = 1; i < window; i++)
  {
    // Copy the last filtered mean
    MatNd_copyRow(filteredData, i, filteredData, i - 1);

    // Add the latest data
    MatNd currRow = MatNd_getRowView(src, i);
    MatNd filtRow = MatNd_getRowView(filteredData, i);
    MatNd_addSelf(&filtRow, &currRow);

    // Remove the oldest
    MatNd oldRow = MatNd_getRowView(src, 0);
    MatNd_subSelf(&filtRow, &oldRow);
  }

  // Walk through ring buffer
  for (unsigned int i = window; i < dataPoints; i++)
  {
    // Copy the last filtered mean
    MatNd_copyRow(filteredData, i, filteredData, i - 1);

    // Add the latest data
    MatNd currRow = MatNd_getRowView(src, i);
    MatNd filtRow = MatNd_getRowView(filteredData, i);
    MatNd_addSelf(&filtRow, &currRow);

    // Remove the oldest
    MatNd oldRow = MatNd_getRowView(src, i - window);
    MatNd_subSelf(&filtRow, &oldRow);
  }

  // Fade out
  for (unsigned int i = dataPoints; i < filterSteps; i++)
  {
    // Copy the last filtered mean
    MatNd_copyRow(filteredData, i, filteredData, i - 1);

    // Add the latest data
    MatNd currRow = MatNd_getRowView(src, dataPoints - 1);
    MatNd filtRow = MatNd_getRowView(filteredData, i);
    MatNd_addSelf(&filtRow, &currRow);

    // Remove the oldest
    MatNd oldRow = MatNd_getRowView(src, i - window);
    MatNd_subSelf(&filtRow, &oldRow);
  }

  // Calculate average
  MatNd_constMulSelf(filteredData, 1.0 / window);

  // Resize to original dimensions
  MatNd_interpolateRows(dst, filteredData);

  MatNd_destroy(filteredData);
}

/*******************************************************************************
 * Rolling average filter.
 ******************************************************************************/
void MatNd_filterMovingMeanSelf(MatNd* self, unsigned int window)
{
  MatNd* src = MatNd_clone(self);
  MatNd_filterMovingMean(self, src, window);
  MatNd_destroy(src);
}

/*******************************************************************************
 * First order lag filter
 ******************************************************************************/
void MatNd_filterFirstOrderLag(MatNd* dst, MatNd* src, double tmc)
{
  RCHECK_EQ(dst->m, src->m);
  RCHECK_EQ(dst->n, src->n);

  MatNd* filt = MatNd_create(src->n, 1);

  for (unsigned int i = 0; i < src->m; i++)
  {
    if (i == 0)
    {
      VecNd_copy(filt->ele, MatNd_getRowPtr(src, i), src->n);
    }
    else
    {
      MatNd_constMulSelf(filt, 1.0 - tmc);
      VecNd_constMulAndAddSelf(filt->ele, MatNd_getRowPtr(src, i), tmc, src->n);
    }

    MatNd_setRow(dst, i, filt->ele, dst->n);
  }

  MatNd_destroy(filt);
}

/*******************************************************************************
 * First order lag filter
 ******************************************************************************/
void MatNd_filterFirstOrderLagSelf(MatNd* self, double tmc)
{
  MatNd* src = MatNd_clone(self);
  MatNd_filterFirstOrderLag(self, src, tmc);
  MatNd_destroy(src);
}

/*******************************************************************************
 * Gets the interpolated row along the arc coordinate.
 ******************************************************************************/
double MatNd_interpolateArcLength(MatNd* res, const MatNd* s_, const MatNd* x,
                                  double s_des)
{
  RCHECK(x->n > 0);
  RCHECK(res->m == 1);
  RCHECK(res->n == x->n);

  if (s_ != NULL)
  {
    RCHECK_MSG(s_->m == x->m, "s->m=%d   x->m=%d", s_->m, x->m);
    RCHECK_MSG(s_->n == 1, "s->n=%d", s_->n);
  }

  const unsigned int cols = x->n;
  const unsigned int tLast = x->m;
  unsigned int idx = tLast - 1;

  if (s_des <= 0.0)
  {
    VecNd_copy(res->ele, MatNd_getRowPtr(x, 0), cols);
    return 0.0;
  }
  else if (s_des >= 1.0)
  {
    VecNd_copy(res->ele, MatNd_getRowPtr(x, tLast - 1), cols);
    return 1.0;
  }

  MatNd* s = (MatNd*) s_;

  // If no arc length array is given, we compute it from the array x
  if (s == NULL)
  {
    MatNd_create2(s, tLast, 1);

    for (unsigned int i=1; i<tLast; i++)
    {
      const double* dx_tm1 = MatNd_getRowPtr(x, i-1);
      const double* dx_t   = MatNd_getRowPtr(x, i);

      double dsi = 0.0;

      for (unsigned int j=0; j<cols; j++)
      {
        dsi += (dx_t[j]-dx_tm1[j])*(dx_t[j]-dx_tm1[j]);
      }

      s->ele[i] = s->ele[i-1] + sqrt(dsi);
    }

    // Scale columns so that last entry is 1, but avoid division by zero.
    if (s->ele[tLast-1]>0.0)
    {
      MatNd_constMulSelf(s, 1.0/s->ele[tLast-1]);
    }

  }

  for (unsigned int i = 1; i < tLast; i++)
  {
    RCHECK_MSG(s->ele[i] >= s->ele[i-1],
               "s->ele[%d] = %f   s->ele[%d] = %f",
               i, s->ele[i], i-1, s->ele[i - 1]);

    if ((s->ele[i-1] <= s_des) && (s->ele[i] > s_des))
    {
      idx = i-1;
      break;
    }
  }

  if (idx == tLast - 1)
  {
    VecNd_copy(res->ele, MatNd_getRowPtr(x, tLast - 1), cols);

    if (s_ == NULL)
    {
      MatNd_destroy(s);
    }
    return 1.0;
  }

  const double* x0 = MatNd_getRowPtr(x, idx);
  const double* x1 = MatNd_getRowPtr(x, idx + 1);
  const double s0 = s->ele[idx];
  const double s1 = s->ele[idx + 1];
  double t_s = (double) idx;

  VecNd_copy(res->ele, x0, cols);

  if (s1 - s0 > 0.0)
  {
    double* dx = RNALLOC(cols, double);
    VecNd_sub(dx, x1, x0, cols);
    double ratio = (s_des - s0) / (s1 - s0);
    VecNd_constMulAndAddSelf(res->ele, dx, ratio, cols);
    t_s += ratio;
    RFREE(dx);
  }

  // Clean up
  if (s_ == NULL)
  {
    MatNd_destroy(s);
  }

  NLOGS(0, "s_des = %.2f: Found point between idx %d and %d",
        s_des, idx, idx + 1);

  return (double) t_s/tLast;
}

/*******************************************************************************
 * Gets the interpolated row along the arc coordinate.
 ******************************************************************************/
void MatNd_interpolateArcLengthEuler(MatNd* res, const MatNd* s,
                                     const MatNd* x, double s_des)
{
  RCHECK(x->n == 3);
  RCHECK(s->m == x->m);
  RCHECK(s->n == 1);
  RCHECK(res->m == 1);
  RCHECK(res->n == x->n);
  RCHECK(s->n == 1);

  const unsigned int cols = x->n;
  const unsigned int tLast = s->m;
  unsigned int idx = tLast - 1;

  if (s_des <= 0.0)
  {
    VecNd_copy(res->ele, MatNd_getRowPtr(x, 0), cols);
    return;
  }
  else if (s_des >= 1.0)
  {
    VecNd_copy(res->ele, MatNd_getRowPtr(x, tLast - 1), cols);
    return;
  }

  for (unsigned int i = 1; i < tLast; i++)
  {
    RCHECK_MSG(s->ele[i] >= s->ele[i - 1],
               "s->ele[%d] = %f   s->ele[%d] = %f",
               i, s->ele[i], i, s->ele[i - 1]);

    if ((s->ele[i - 1] <= s_des) && (s->ele[i] > s_des))
    {
      idx = i - 1;
      break;
    }
  }

  if (idx == tLast - 1)
  {
    VecNd_copy(res->ele, MatNd_getRowPtr(x, tLast - 1), cols);
    return;
  }

  const double* x0 = MatNd_getRowPtr(x, idx);
  const double* x1 = MatNd_getRowPtr(x, idx + 1);
  const double s0 = s->ele[idx];
  const double s1 = s->ele[idx + 1];

  if (s1 - s0 > 0.0)
  {
    Vec3d_slerp(res->ele, x0, x1, (s_des - s0) / (s1 - s0));
  }

}


/*******************************************************************************
 * Get diagonal from square matrix.
 ******************************************************************************/
void MatNd_getDiag(MatNd* diag, const MatNd* src)
{
  unsigned int i;

  RCHECK_MSG(src->m == src->n, "Source matrix is not square (%d x %d)",
             src->m, src->n);
  RCHECK_MSG(diag->n == 1, "Diagonal vector must have only one column, but has"
             " %d", diag->n);
  RCHECK_MSG(diag->m == src->m, "Diagonal vector must same number of rows as "
             "source: source %d vector %d", src->m, diag->m);

  for (i = 0; i < diag->m; i++)
  {
    diag->ele[i] = src->ele[i * diag->m + i];
  }
}

/*******************************************************************************
 * Invert diagonal of a matrix.
 ******************************************************************************/
double MatNd_inverseDiag(MatNd* dst, const MatNd* src)
{
  RCHECK_MSG((src->n == 1) || (src->m == src->n),
             "Source matrix is not square nor n x 1 (%d x %d)", src->m, src->n);
  RCHECK_MSG((src->m == dst->m) && (src->n == dst->n),
             "Source (%d x %d) and destination (%d x %d) do not have the same"
             " size", src->m, src->n, dst->m, dst->n);

  unsigned int i;

  double det = 1.0;

  if (src->n == 1)
  {
    for (i = 0; i < src->m; i++)
    {
      const double src_i = src->ele[i];
      dst->ele[i] = (src_i == 0.0) ? DBL_MAX : 1.0/src_i;
      det *= src_i;
    }
  }
  else
  {
    for (i = 0; i < src->m * src->n; i++)
    {
      if (i % (src->m + 1) == 0)
      {
        const double src_i = src->ele[i];
        dst->ele[i] = (src_i == 0.0) ? DBL_MAX : 1.0/src_i;
        det *= src_i;
      }
      else
      {
        dst->ele[i] = 0.0;
      }
    }
  }

  return det;
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_computeMinimumJerkTrajectory(MatNd* s_, MatNd* s_dot_)
{
  MatNd* s = s_;
  MatNd* s_dot = s_dot_;

  // If s_ is NULL, we create an array for s internally
  if (s_ == NULL)
  {
    RCHECK_MSG(s_dot, "Both s and s_dot are NULL! That doesn't make sense.");
    MatNd_create2(s, s_dot->m, 1);
  }
  else
  {
    RCHECK_EQ(s->n, 1);
    MatNd_setZero(s);
  }

  // If s_dot_ is NULL, we create an array for s internally
  if (s_dot == NULL)
  {
    RCHECK_MSG(s, "Both s and s_dot are NULL! That doesn't make sense.");
    MatNd_create2(s_dot, s->m, 1);
  }
  else
  {
    RCHECK_EQ(s_dot->n, 1);
    MatNd_setZero(s_dot);
  }

  // Array dimensions must match
  RCHECK_EQ(s_dot->m, s->m);

  // Compute parameters of minimum jerk model
  // Compute denominater d = Sum_0^{T-1} (t-p2)^2 (t-p3)^2
  unsigned int T = s->m;
  double len = 0.0;
  const double p2 =  1.0;
  const double p3 = 0.0;

  for (unsigned int i=0; i<T; i++)
  {
    double t = (double)(i+1)/T;
    double jerk_i = (t-p2)*(t-p2)*(t-p3)*(t-p3);
    len += jerk_i;
    s_dot->ele[i] = jerk_i;
  }

  // This makes the integral of s_dot become 1 at the end of the sequence
  if (len>0.0)
  {
    MatNd_constMulSelf(s_dot, 1.0/len);
  }



  // Integrate s
  for (unsigned int i=1; i<T; i++)
  {
    s->ele[i] = s->ele[i-1] + s_dot->ele[i-1];
  }


  // Clean up
  if (s_ == NULL)
  {
    MatNd_destroy(s);
  }

  if (s_dot_ == NULL)
  {
    MatNd_destroy(s_dot);
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
unsigned int minJerkTrajectoryLengthFromMaxVelocity(double max_velocity)
{
  const double p2 =  1.0;
  const double p3 = 0.0;

  double int_0_1 = 1.0/5.0 - 1.0/2.0*p3 + 1.0/3.0*p3*p3 - 1.0/2.0*p2 + 4.0/3.0*p2*p3 -
                   p2*p3*p3 + 1.0/3.0*p2*p2 - p2*p2*p3;

  unsigned int T = lround(ceil((pow((0.5-p2), 2) * pow((0.5-p3), 2)) /
                               (max_velocity * int_0_1)));

  return T;
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_getSubset(MatNd* subset, const MatNd* full, const MatNd* logical)
{
  RCHECK_MSG(full->m * full->n <= subset->size,
             "full: [%d x %d] <= subset->size: %d",
             full->m, full->n, subset->size);
  RCHECK_MSG((logical->m == full->m) && (logical->n == full->n),
             "logical: [%d x %d] == full: [%d x %d]", logical->m,
             logical->n, full->m, full->n);

  subset->m = 0;
  subset->n = 1;
  for (unsigned int i = 0; i < logical->m * logical->n; i++)
  {
    if (logical->ele[i] >= .5)
    {
      subset->ele[subset->m] = full->ele[i];
      subset->m++;
    }
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_setSubset(MatNd* full, const MatNd* subset, const MatNd* logical)
{
  RCHECK_MSG(full->m * full->n >= subset->m*subset->n,
             "full: [%d x %d] >= subset: [%d x %d]",
             full->m, full->n, subset->m, subset->n);
  RCHECK_MSG((logical->m == full->m) && (logical->n == full->n),
             "logical: [%d x %d] == full: [%d x %d]", logical->m,
             logical->n, full->m, full->n);

  unsigned int counter = 0;
  for (unsigned int i = 0; i < logical->m * logical->n; i++)
  {
    if (logical->ele[i] >= .5)
    {
      RCHECK_MSG(counter < subset->m*subset->n,
                 "out of bounds entry %d of subset [%d x %d]",
                 counter, subset->m, subset->n);
      full->ele[i] = subset->ele[counter];
      counter++;
    }
  }
}



/*******************************************************************************
 * Returns the normalized time at which the arc length coordinate s_des
 * is reached, given the current arc length trajectory s_curr
 ******************************************************************************/
double MatNd_timeFromArcLength(const MatNd* x, double s_des,
                               const MatNd* s_curr_)
{
  if (s_des <= 0.0)
  {
    return 0.0;
  }
  else if (s_des >= 1.0)
  {
    return 1.0;
  }

  double normalizedTimePoint = 0.0;
  unsigned int idx = x->m - 1;
  const unsigned int cols = x->n;


  MatNd* s_internal = NULL;

  // Compute arc length array from the array x, if it is not given by s_curr_
  if (s_curr_ == NULL)
  {
    MatNd_create2(s_internal, x->m, 1);

    for (unsigned int i=1; i<x->m; i++)
    {
      const double* dx_tm1 = MatNd_getRowPtr(x, i-1);
      const double* dx_t   = MatNd_getRowPtr(x, i);

      double dsi = 0.0;

      for (unsigned int j=0; j<cols; j++)
      {
        dsi += (dx_t[j]-dx_tm1[j])*(dx_t[j]-dx_tm1[j]);
      }

      s_internal->ele[i] = s_internal->ele[i-1] + sqrt(dsi);
    }

    // Scale columns so that last entry is 1, but avoid division by zero.
    if (s_internal->ele[s_internal->m-1]>0.0)
    {
      MatNd_constMulSelf(s_internal, 1.0/s_internal->ele[s_internal->m-1]);
    }
  }

  // s has to be const otherwise it would discard the const qualifier of s_curr_
  const MatNd* s = s_curr_ ? s_curr_ : s_internal;

  // Query for interval. That's pretty inefficient, we just walk through the
  // array and search for the interval.
  for (unsigned int i = 1; i < s->m; i++)
  {
    if ((s->ele[i-1] <= s_des) && (s->ele[i] > s_des))
    {
      idx = i-1;
      break;
    }
  }

  // The normalized time is 1 if we end up after the last interval
  if (idx == s->m - 1)
  {
    normalizedTimePoint = 1.0;
  }
  else
  {
    const double s0 = s->ele[idx];
    const double s1 = s->ele[idx + 1];
    normalizedTimePoint = (double) idx/(s->m-1);

    if (s1 - s0 > 0.0)
    {
      double ratio = (s_des - s0) / (s1 - s0);
      normalizedTimePoint += ratio/(s->m-1);
    }
  }

  // Clean up
  if (s_curr_ == NULL)
  {
    MatNd_destroy(s_internal);
  }

  return normalizedTimePoint;
}

/******************************************************************************
 * Manipulability index according to Yoshikawa: w = sqrt(det(J*W*J^T))
 *****************************************************************************/
double MatNd_computeManipulabilityIndex(const MatNd* J, const MatNd* W)
{
  if (J->m==0)
  {
    return 0.0;
  }

  const double lambda = 1.0e-8;
  MatNd* L = NULL, *A = NULL;
  MatNd_create2(A, J->m, J->m);
  MatNd_create2(L, J->m, J->m);

  MatNd_sqrMulABAt(A, J, W);   // J*J^T
  MatNd_addConstToDiag(A, lambda);

  double det = MatNd_choleskyDecomposition(L, A);

  MatNd_destroy(L);
  MatNd_destroy(A);

  return sqrt(det);
}

/*******************************************************************************
 *
 * Manipulability index according to Yoshikawa: w = sqrt(det(J*W*J^T))
 *
 * Without weighting: the gradient is:
 *
 * dw/dqi = 1/(2*w)*d/dqi(det(J*J^T))
 *        = 1/(2*w)*det(J*J^T)*Tr{(J*J^T)^-1 d/dqi(J*J^T)}   // matrixcookbook
 *        = 0.5*sqrt(det(J*J^T))*Tr{(J*J^T)^-1 d/dqi(J*J^T)}
 *
 * where d/dqi(J*J^T) = dJ/dqi*J^T + J*(dJ/dqi)^T
 *                    = dJ/dqi*J^T + (dJ/dqi*J^T)^T
 *
 * which leads to
 *
 * dw/dqi = 0.5*sqrt(det(J*J^T))*Tr{(J*J^T)^-1*(dJ/dqi*J^T + (dJ/dqi*J^T)^T)}
 *        = 0.5*sqrt(det(J*J^T))*Tr{(J*J^T)^-1*2*(dJ/dqi*J^T)}
 *        = sqrt(det(J*J^T))*Tr{(J*J^T)^-1*(dJ/dqi*J^T)}
 *
 * With weighting: the gradient is:
 *
 * dw/dqi = 1/(2*w)*d/dqi(det(J*W*J^T))
 *        = 1/(2*w)*det(J*W*J^T)*Tr{(J*W*J^T)^-1 d/dqi(J*W*J^T)}
 *        = 0.5*sqrt(det(J*W*J^T))*Tr{(J*W*J^T)^-1 d/dqi(J*W*J^T)}
 *
 * where d/dqi(J*W*J^T) = d(J*W)/dqi*J^T + J*W*(dJ/dqi)^T
 *                      = dJ/dqi*W*J^T + (dJ/dqi*W*J^T)^T
 *
 * which leads to
 *
 * dw/dqi =
 *   0.5*sqrt(det(J*W*J^T))*Tr{(J*W*J^T)^-1*(dJ/dqi**WJ^T + (dJ/dqi*W*J^T)^T)}
 * = 0.5*sqrt(det(J*W*J^T))*Tr{(J*W*J^T)^-1*2*(dJ/dqi*W*J^T)}
 * = sqrt(det(J*W*J^T))*Tr{(J*W*J^T)^-1*(dJ/dqi*W*J^T)}
 *
 ******************************************************************************/
double MatNd_computeManipulabilityIndexGradient(MatNd* grad,
                                                const MatNd* J,
                                                const MatNd* H,
                                                const MatNd* W)
{
  const double lambda = 1.0e-8;
  unsigned int nq = J->n, nx = J->m;

  MatNd_reshapeAndSetZero(grad, 1, nq);

  if (J->m==0)
  {
    return 0.0;
  }

  MatNd* A = NULL, *invA = NULL;
  MatNd_create2(A, nx, nx);
  MatNd_create2(invA, nx, nx);
  MatNd_sqrMulABAt(A, J, W);   // J*W*J^T
  MatNd_addConstToDiag(A, lambda);

  double det = MatNd_choleskyInverse(invA, A);
  double sqrtDet = sqrt(det);

  // dJ/dqi is in a different storage order than the Hessian convention. We
  // therefore re-order it here so that it is ordered with the partial
  // derivatives for each joint. For instance, the incoming ordering is
  //
  // J = J00 J01 J02
  //     J10 J11 J12
  //
  // H = dJ00/dq0 dJ00/dq1 dJ00/dq2
  //     dJ01/dq0 dJ01/dq1 dJ01/dq2
  //     dJ02/dq0 dJ02/dq1 dJ02/dq2
  //     dJ10/dq0 dJ10/dq1 dJ10/dq2
  //     dJ11/dq0 dJ11/dq1 dJ11/dq2
  //     dJ12/dq0 dJ12/dq1 dJ12/dq2
  RCHECK_MSG((H->m==nx*nq) && (H->n==nq),
             "Hessian is wrongly shaped: H->m=%d H->n=%d nx=%d nq=%d",
             H->m, H->n, nx, nq);

  MatNd* JT = NULL;
  MatNd_create2(JT, nq, nx);
  MatNd_transpose(JT, J);

  if (W != NULL)
  {
    MatNd_preMulDiagSelf(JT, W);
  }

  MatNd* dJdqi = NULL, *dJdqiJT = NULL, *inneHTrace = NULL;
  MatNd_create2(dJdqi, J->m*J->n, 1);
  MatNd_create2(dJdqiJT, J->m, J->m);
  MatNd_create2(inneHTrace, J->m, J->m);


  for (size_t column = 0; column<nq; column++)
  {
    MatNd_reshape(dJdqi, nx*nq, 1);
    MatNd_getColumn(dJdqi, column, H);
    MatNd_reshape(dJdqi, nx, nq);

    MatNd_mul(dJdqiJT, dJdqi, JT);
    MatNd_mul(inneHTrace, invA, dJdqiJT);

    grad->ele[column] = sqrtDet*MatNd_trace(inneHTrace);
  }


  // Clean up
  MatNd_destroy(A);
  MatNd_destroy(invA);
  MatNd_destroy(JT);
  MatNd_destroy(dJdqi);
  MatNd_destroy(dJdqiJT);
  MatNd_destroy(inneHTrace);

  return sqrtDet;
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_softMax(MatNd* dst, const MatNd* src, double beta)
{
  RCHECK_MSG((dst->m == src->m) && (dst->n == src->n),
             "dst: [%d x %d] src: [%d x %d]", dst->m, dst->n, src->m, src->n);
  RCHECK_MSG(beta > 0, "beta: %f", beta);

  for (unsigned int j = 0; j < src->n; j++)
  {
    double dnom = 0.;
    for (unsigned int i = 0; i < src->m; i++)
    {
      dnom += exp(beta*MatNd_get2(src, i, j));
    }

    for (unsigned int i = 0; i < src->m; i++)
    {
      MatNd_set2(dst, i, j, exp(beta*MatNd_get2(src, i, j))/dnom);
    }
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNd_binarizeSelf(MatNd* self, double zeroThreshold)
{
  for (unsigned int i=0; i<self->m*self->n; ++i)
  {
    if (fabs(self->ele[i])<=zeroThreshold)
    {
      self->ele[i] = 0.0;
    }
    else
    {
      self->ele[i] = 1.0;
    }
  }
}

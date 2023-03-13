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

#ifndef MATNDWIDGET_H
#define MATNDWIDGET_H

#include "AsyncWidget.h"
#include "LcdSlider.h"

#include <Rcs_MatNd.h>


#include <QScrollArea>

#include <vector>
#include <string>
#include <pthread.h>

namespace Rcs
{

class MatNdGui : public Rcs::AsyncWidget
{
public:
  MatNdGui(MatNd* mat, const char* title=NULL, pthread_mutex_t* mutex=NULL);
  MatNdGui(MatNd* mat, double lower, double upper, const char* title=NULL,
           pthread_mutex_t* mutex=NULL);
  MatNdGui(MatNd* mat, const MatNd* dispMat, double lower, double upper,
           const char* title=NULL, pthread_mutex_t* mutex=NULL);

  void construct();
  void setLabels(std::vector<std::string>& labels);
  void reset(const MatNd* values);

protected:
  MatNd* mat;
  const MatNd* dispMat;
  pthread_mutex_t* mtx;
  std::string title;
  double lower;
  double upper;
};

class MatNdWidget : public QScrollArea
{
  Q_OBJECT

public:

  static MatNdWidget* create(MatNd* mat, const char* title=NULL,
                             pthread_mutex_t* mutex = NULL);

  static MatNdWidget* create(MatNd* mat, double lower, double upper,
                             const char* title=NULL,
                             pthread_mutex_t* mutex=NULL);

  static MatNdWidget* create(MatNd* mat, const MatNd* dispMat,
                             double lower, double upper,
                             const char* title=NULL,
                             pthread_mutex_t* mutex=NULL);

  MatNdWidget(MatNd* mat, const MatNd* dispMat, double lower, double upper,
              const char* title, pthread_mutex_t* mutex);

  /*! \brief Virtual destructor to allow inheriting from this nice class.
   */
  virtual ~MatNdWidget();

  /*! \brief Sets all slider lower bounds to the values given in limit. Array
   *         limit must have exactly as many rows as there are sliders. If this
   *         is not the case, the function complains on debug level 1 and does
   *         nothing. It is fine to set the lower bound larger than the upper
   *         bound.
   */
  void setLowerBound(const MatNd* limit);

  /*! \brief Sets all slider lower bounds to the values given in limit. Array
   *         limit must have exactly as many rows as there are sliders. If this
   *         is not the case, the function complains on debug level 1 and does
   *         nothing. It is fine to set the lower bound larger than the upper
   *         bound.
   */
  void setUpperBound(const MatNd* limit);

  /*! \brief Sets all slider lower bounds to the given value. See
   *         \ref setLowerBound(const MatNd* limit)
   */
  void setLowerBound(double limit);

  /*! \brief Sets all slider upper bounds to the given value. See
   *         \ref setUpperBound(const MatNd* limit)
   */
  void setUpperBound(double limit);

  void setLabels(std::vector<std::string>& labels);
  void setMutex(pthread_mutex_t* mutex);
  void setUpdateEnabled(bool enabled);
  bool isUpdateEnabled();
  void reset(const MatNd* values);

protected:
  void init(MatNd* mat, const MatNd* dispMat, double lower, double upper,
            const char* title);
  void lock();
  void unlock();

  MatNd* mat;
  const MatNd* dispMat;
  pthread_mutex_t* mutex;
  std::vector<LcdSlider*> slider;
  std::vector<std::string> labels;
  bool updateEnabled;

private slots:
  void displayAct();
  void setCommand();
};

}   // namespace Rcs

#endif   // MATNDWIDGET_H

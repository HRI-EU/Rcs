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

#ifndef MATNDWIDGET_H
#define MATNDWIDGET_H

#include <Rcs_MatNd.h>

#include <LcdSlider.h>

#include <QScrollArea>

#include <vector>
#include <pthread.h>



class MatNdWidget: public QScrollArea
{
  Q_OBJECT

public:
  static void test();
  static MatNdWidget* create(MatNd* mat, const char* title=NULL,
                             pthread_mutex_t* mutex=NULL);
  static MatNdWidget* create(MatNd* mat, double lower, double upper,
                             const char* title=NULL,
                             pthread_mutex_t* mutex=NULL);
  static MatNdWidget* create(MatNd* mat, const MatNd* dispMat,
                             double lower, double upper,
                             const char* title=NULL,
                             pthread_mutex_t* mutex=NULL);
  MatNdWidget(MatNd* mat, const MatNd* dispMat, double lower, double upper,
              const char* title, pthread_mutex_t* mutex);
  virtual ~MatNdWidget();

  void setLowerLimit(const MatNd* limit);
  void setUpperLimit(const MatNd* limit);
  void setLowerLimit(double limit);
  void setUpperLimit(double limit);
  void setLabels(std::vector<std::string>& labels);
  void setMutex(pthread_mutex_t* mutex);
  void setUpdateEnabled(bool enabled);
  bool isUpdateEnabled();

protected:
  void init(MatNd* mat, const MatNd* dispMat, double lower, double upper,
            const char* title);
  void lock();
  void unlock();

  MatNd* mat;
  MatNd* lowerLimit;
  MatNd* upperLimit;
  const MatNd* dispMat;
  pthread_mutex_t* mutex;
  std::vector<LcdSlider*> slider;
  std::vector<std::string> labels;
  bool _update_enabled;

private slots:
  void displayAct();
  void setCommand();
};

#endif   // MATNDWIDGET_H

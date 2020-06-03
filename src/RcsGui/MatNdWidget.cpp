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

#include "MatNdWidget.h"
#include "Rcs_guiFactory.h"

#include <Rcs_macros.h>

#include <QTimer>
#include <QLayout>

namespace Rcs
{

/*******************************************************************************
 *
 ******************************************************************************/
typedef struct
{
  void* ptr[10];
} VoidPointerList;


static void* matndGui(void* arg)
{
  VoidPointerList* p = (VoidPointerList*) arg;
  RCHECK(arg);
  MatNd* mat             = (MatNd*) p->ptr[0];
  const char* title      = (const char*) p->ptr[1];
  pthread_mutex_t* mutex = (pthread_mutex_t*) p->ptr[2];
  double* lb             = (double*) p->ptr[3];
  double* ub             = (double*) p->ptr[4];
  const MatNd* dispMat   = (const MatNd*) p->ptr[5];

  double lowerBound = (lb != NULL) ? *lb : -1.0;
  double upperBound = (ub != NULL) ? *ub :  1.0;
  MatNdWidget* w = new MatNdWidget(mat, dispMat, lowerBound, upperBound,
                                   title, mutex);
  w->show();

  if (lb != NULL)
  {
    delete lb;
  }

  if (ub != NULL)
  {
    delete ub;
  }

  delete p;

  return w;
}

/*******************************************************************************
 *
 ******************************************************************************/
MatNdWidget* MatNdWidget::create(MatNd* mat, const char* title,
                                 pthread_mutex_t* mutex)
{
  VoidPointerList* p = new VoidPointerList;
  p->ptr[0] = (void*) mat;
  p->ptr[1] = (void*) title;
  p->ptr[2] = (void*) mutex;
  p->ptr[3] = (void*) NULL;
  p->ptr[4] = (void*) NULL;
  p->ptr[5] = (void*) NULL;

  int handle = RcsGuiFactory_requestGUI(matndGui, p);

  return (MatNdWidget*) RcsGuiFactory_getPointer(handle);
}

/*******************************************************************************
 *
 ******************************************************************************/
MatNdWidget* MatNdWidget::create(MatNd* mat, double lower, double upper,
                                 const char* title, pthread_mutex_t* mutex)
{
  VoidPointerList* p = new VoidPointerList;
  p->ptr[0] = (void*) mat;
  p->ptr[1] = (void*) title;
  p->ptr[2] = (void*) mutex;

  double* _lower = new double;
  *_lower = lower;
  double* _upper    = new double;
  *_upper    = upper;

  p->ptr[3] = (void*) _lower;
  p->ptr[4] = (void*) _upper;
  p->ptr[5] = (void*) NULL;

  int handle = RcsGuiFactory_requestGUI(matndGui, p);

  return (MatNdWidget*) RcsGuiFactory_getPointer(handle);
}

/*******************************************************************************
 *
 ******************************************************************************/
MatNdWidget* MatNdWidget::create(MatNd* mat, const MatNd* dispMat,
                                 double lower, double upper,
                                 const char* title, pthread_mutex_t* mutex)
{
  VoidPointerList* p = new VoidPointerList;
  p->ptr[0] = (void*) mat;
  p->ptr[1] = (void*) title;
  p->ptr[2] = (void*) mutex;

  double* _lower = new double;
  *_lower = lower;
  double* _upper    = new double;
  *_upper    = upper;

  p->ptr[3] = (void*) _lower;
  p->ptr[4] = (void*) _upper;
  p->ptr[5] = (void*) dispMat;

  int handle = RcsGuiFactory_requestGUI(matndGui, p);

  return (MatNdWidget*) RcsGuiFactory_getPointer(handle);
}

/*******************************************************************************
 *
 ******************************************************************************/
MatNdWidget::MatNdWidget(MatNd* mat_, const MatNd* dispMat_,
                         double lower, double upper,
                         const char* title, pthread_mutex_t* mutex_) :
  QScrollArea(),
  mat(mat_),
  mutex(mutex_),
  updateEnabled(true)
{
  init(mat_, dispMat_, lower, upper, title);
}

/*******************************************************************************
 *
 ******************************************************************************/
MatNdWidget::~MatNdWidget()
{
  RLOG(5, "Destroying MatNdWidget");
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNdWidget::setLowerBound(const MatNd* limit)
{
  if (limit->m != this->slider.size())
  {
    RLOG(1, "The number of bounds does not match the number of sliders: "
         "%u != %zu", limit->m, this->slider.size());
    return;
  }

  lock();
  for (size_t i = 0; i < slider.size(); ++i)
  {
    slider[i]->setLowerBound(MatNd_get(limit, i, 0));
  }
  unlock();
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNdWidget::setUpperBound(const MatNd* limit)
{
  if (limit->m != this->slider.size())
  {
    RLOG(1, "The number of bounds does not match the number of sliders: "
         "%u != %zu", limit->m, this->slider.size());
    return;
  }

  lock();
  for (size_t i = 0; i < slider.size(); ++i)
  {
    slider[i]->setUpperBound(MatNd_get(limit, i, 0));
  }
  unlock();
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNdWidget::setLowerBound(double value)
{
  MatNd* bound = MatNd_create(slider.size(), 1);
  MatNd_setElementsTo(bound, value);
  setLowerBound(bound);
  MatNd_destroy(bound);
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNdWidget::setUpperBound(double value)
{
  MatNd* bound = MatNd_create(slider.size(), 1);
  MatNd_setElementsTo(bound, value);
  setUpperBound(bound);
  MatNd_destroy(bound);
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNdWidget::setLabels(std::vector<std::string>& labels)
{
  if (labels.size() != this->slider.size())
  {
    RLOG(1, "The number of labels does not match the number of vector "
         "elements: %zu != %zu", labels.size(), this->slider.size());
    return;
  }

  this->labels = labels;

  for (unsigned int i= 0; i< this->slider.size(); i++)
  {
    this->slider[i]->setLabel(labels[i].c_str());
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNdWidget::setMutex(pthread_mutex_t* mutex)
{
  this->mutex = mutex;
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNdWidget::setUpdateEnabled(bool enabled)
{
  lock();
  updateEnabled = enabled;
  unlock();
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNdWidget::init(MatNd* mat_, const MatNd* dispMat_,
                       double lower, double upper,
                       const char* title)
{
  RCHECK(this->mat);

  this->dispMat = dispMat_ ? dispMat_ : this->mat;

  char windowTitle[256];
  sprintf(windowTitle, "%d x %d matrix", this->mat->m, this->mat->n);
  setWindowTitle(title ? title : windowTitle);

  // The layout for the overall task widget
  QGridLayout* matLayout = new QGridLayout();

  // Add a slider instance for each array element
  for (unsigned int i = 0; i < this->mat->m; i++)
  {
    for (unsigned int j = 0; j < this->mat->n; j++)
    {
      char indexStr[32];
      sprintf(indexStr, "%u", i);
      LcdSlider* sl =
        new LcdSlider(lower, this->mat->ele[i*this->mat->n+j], upper,
                      1.0, 0.0001, indexStr);
      sl->updateLcd1FromSlider();
      matLayout->addWidget(sl, i, j, Qt::AlignLeft);
      this->slider.push_back(sl);

      connect(sl, SIGNAL(valueChanged(double)), SLOT(setCommand()));
    }
  }

  QWidget* scrollWidget = new QWidget(this);
  scrollWidget->setLayout(matLayout);
  this->setWidget(scrollWidget);
  this->setWidgetResizable(true);

  QTimer* timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), SLOT(displayAct()));

  setCommand();

  timer->start(40);
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNdWidget::displayAct()
{
  if (this->dispMat == NULL)
  {
    return;
  }

  lock();

  for (size_t i=0; i< this->slider.size(); i++)
  {
    slider[i]->setValueLcd2(this->dispMat->ele[i]);
  }

  unlock();
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNdWidget::setCommand()
{
  lock();

  if (updateEnabled)
  {
    unsigned int i = 0;
    std::vector<LcdSlider*>::iterator it;

    for (it = this->slider.begin(); it != this->slider.end(); ++it)
    {
      this->mat->ele[i] = (*it)->getSliderValue();
      i++;
    }
  }

  unlock();
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNdWidget::lock()
{
  if (this->mutex != NULL)
  {
    pthread_mutex_lock(this->mutex);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNdWidget::unlock()
{
  if (this->mutex != NULL)
  {
    pthread_mutex_unlock(this->mutex);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
bool MatNdWidget::isUpdateEnabled()
{
  lock();
  bool isUpdated = updateEnabled;
  unlock();

  return isUpdated;
}

/*******************************************************************************
 *
 ******************************************************************************/
void MatNdWidget::reset(const MatNd* values)
{
  if ((this->mat->m!=values->m) || (this->mat->n!=values->n))
  {
    RLOG(1, "Mismatch in reset: mat is %u x %u, desired matrix is %u x %u",
         mat->m, mat->n, values->m, values->n);
    return;
  }

  MatNd_copy(this->mat, values);

  for (unsigned int i=0; i<values->m; ++i)
  {
    slider[i]->setSliderValue(values->ele[i]);
  }
}

}   // namespace Rcs



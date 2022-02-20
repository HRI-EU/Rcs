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

#include "DynamicDataPlot.h"

#include <Rcs_guiFactory.h>
#include <Rcs_macros.h>

#include <QtGlobal>
#include <QBoxLayout>
#include <QPen>

#include <qwt_plot_canvas.h>
#include <qwt_thermo.h>
#include <qwt_knob.h>
#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_grid.h>

#include <cmath>


typedef struct
{
  void* ptr[10];
} VoidPointerList;

namespace Rcs
{

/*******************************************************************************
 * Thread function.
 ******************************************************************************/
void* DynamicDataPlot::threadFunc(void* arg)
{
  VoidPointerList* p = (VoidPointerList*)arg;
  RCHECK(arg);

  MatNd* mat = (MatNd*) p->ptr[0];
  const char* title = (const char*)p->ptr[1];
  pthread_mutex_t* mutex = (pthread_mutex_t*)p->ptr[2];

  delete p;

  DynamicDataPlot* widget = new DynamicDataPlot(mat, title, mutex);

  widget->show();

  return widget;
}

int DynamicDataPlot::create(MatNd* mat, const char* title,
                            pthread_mutex_t* mutex)
{
  VoidPointerList* p = new VoidPointerList;
  p->ptr[0] = (void*)mat;
  p->ptr[1] = (void*)title;
  p->ptr[2] = (void*)mutex;

  int handle = RcsGuiFactory_requestGUI(DynamicDataPlot::threadFunc, (void*)p);

  return handle;
}

bool DynamicDataPlot::destroy(int handle)
{
  return RcsGuiFactory_destroyGUI(handle);
}

DynamicDataPlot::DynamicDataPlot(MatNd* mat_, const char* title,
                                 pthread_mutex_t* mutex) : mat(mat_)
{
  QwtPlotCanvas* canvas = new QwtPlotCanvas();
  canvas->setLineWidth(1);
  canvas->setFrameStyle(QFrame::Box | QFrame::Plain);

  QPalette canvasPalette(Qt::white);
#if QT_VERSION < QT_VERSION_CHECK(5, 13, 0)
  canvasPalette.setColor(QPalette::Foreground, QColor(131, 131, 131));
#else
  canvasPalette.setColor(QPalette::WindowText, QColor(131, 131, 131));
#endif
  canvas->setPalette(canvasPalette);

  this->plot = new QwtPlot;
  plot->setTitle(title);
  plot->setCanvasBackground(Qt::white);
  plot->setCanvas(canvas);

  this->curve = new QwtPlotCurve;
  curve->attach(plot);
  curve->setTitle(title);
  curve->setPen(Qt::blue, 2), curve->setRenderHint(QwtPlotItem::RenderAntialiased, true);

  this->grid = new QwtPlotGrid();
  grid->setMajorPen(QPen(Qt::DotLine));
  grid->attach(plot);

  plot->show();

  QHBoxLayout* hLayout = new QHBoxLayout;
  hLayout->addWidget(plot);
  setLayout(hLayout);

  startTimer(40);
}

DynamicDataPlot::~DynamicDataPlot()
{
  delete this->curve;
  delete this->grid;
}

void DynamicDataPlot::timerEvent(QTimerEvent*)
{
  if (mat->m == 1)
  {
    double* xData = new double[mat->n];

    for (unsigned int i = 0; i < mat->n; ++i)
    {
      xData[i] = i;
    }

    curve->setSamples(xData, mat->ele, mat->n);
    delete [] xData;
  }
  else if (mat->m == 2)
  {
    curve->setSamples(mat->ele, &mat->ele[mat->n], mat->n);
  }

  plot->replot();
}

}   // namespace Rcs

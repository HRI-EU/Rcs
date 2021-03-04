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

#ifndef RCS_DYNAMICDATAPLOT_H
#define RCS_DYNAMICDATAPLOT_H

#include <Rcs_MatNd.h>

#include <QWidget>

#include <pthread.h>

class QwtPlot;
class QwtPlotCurve;

namespace Rcs
{

class DynamicDataPlot : public QWidget
{
  Q_OBJECT

public:

  static int create(MatNd* mat, const char* title = NULL,
                    pthread_mutex_t* mutex = NULL);

  static bool destroy(int handle);

  DynamicDataPlot(MatNd* mat, const char* title, pthread_mutex_t* mutex);

  virtual ~DynamicDataPlot();

  void timerEvent(QTimerEvent*);

private:

  static void* threadFunc(void* arg);

  QwtPlot*      plot;
  QwtPlotCurve* curve;

  const MatNd* mat;
};

}   // namespace Rcs

#endif // RCS_DYNAMICDATAPLOT_H

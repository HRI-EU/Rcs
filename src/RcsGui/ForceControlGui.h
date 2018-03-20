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

#ifndef RCS_FORCECONTROLGUI_H
#define RCS_FORCECONTROLGUI_H

#include <ControllerBase.h>
#include <Rcs_MatNd.h>

#include <QtGui/QScrollArea>

#include <pthread.h>

namespace Rcs
{

class ForceControlGui: public QScrollArea
{
  Q_OBJECT

public:
  struct Entry
  {
    Entry(double* _q_des, double* _q_curr, double* _active,
          const char* _name = 0, double _lowerBound = -1.0,
          double _zeroPos = 0.0, double _upperBound = 1.0,
          double _ticSize = 1.0):
      q_des(_q_des), q_curr(_q_curr), active(_active), name(_name),
      lowerBound(_lowerBound), zeroPos(_zeroPos), upperBound(_upperBound),
      ticSize(_ticSize)
    {
    }

    double* q_des;
    double* q_curr;
    double* active;
    const char* name;
    double lowerBound;
    double zeroPos;
    double upperBound;
    double ticSize;
  };

public:
  ForceControlGui(std::vector<Entry>* entries);
  virtual ~ForceControlGui();
  static void populateFTguiList(std::vector<ForceControlGui::Entry>& entries,
                                const ControllerBase* controller,
                                MatNd* ft_des,
                                MatNd* ft_curr,
                                MatNd* s);
  int getTaskEntryByName(const ControllerBase* controller, const char* name);
  static void* threadFunction(void* arg);

  // TODO: Mutex locking does not yet work
  static ForceControlGui* create(const ControllerBase* controller,
                                 MatNd* ft_des,
                                 MatNd* s_des,
                                 MatNd* ft_curr,
                                 const char* title=NULL,
                                 pthread_mutex_t* mutex=NULL);
};

}

#endif // RCS_FORCECONTROLGUI_H

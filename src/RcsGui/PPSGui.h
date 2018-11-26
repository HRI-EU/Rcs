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

#ifndef PPSGUI_H
#define PPSGUI_H

#include <QScrollArea>

#include <vector>
#include <string>
#include <pthread.h>


namespace Rcs
{


#ifdef __cplusplus
extern "C" {
#endif

void* ppsGui(void* arg);

#ifdef __cplusplus
}
#endif


class PPSGui: public QScrollArea
{
Q_OBJECT

public:
struct Entry
{
  Entry(const std::string& _name, const size_t _width, const size_t _height, const double* _data, double _scaling=1.0, double _offset=0.0, bool _palm=false):
    name(_name), width(_width), height(_height), data(_data), scaling(_scaling), offset(_offset), palm(_palm) {}
  std::string name;
  size_t width;
  size_t height;
  const double* data;
  double scaling;
  double offset;
  bool palm;
};

public:
static PPSGui* create(std::vector<Rcs::PPSGui::Entry> ppsEntries,
                      pthread_mutex_t* mutex=NULL);
PPSGui(std::vector<Entry>* entries, pthread_mutex_t* mutex);
virtual ~PPSGui();

};

}

#endif // PPSGUI_H

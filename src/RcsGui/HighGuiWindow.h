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

#ifndef HIGHGUIWINDOW_H
#define HIGHGUIWINDOW_H

#include "HighGuiWidget.h"

#include <QFrame>

#if defined (WITH_X11_SUPPORT)
#include <X11/X.h>
#endif

class QVBoxLayout;
class QGridLayout;

namespace Rcs
{

class HighGuiWindow : public HighGuiWidget
{
  Q_OBJECT

public:
  HighGuiWindow(const std::string& name);
  ~HighGuiWindow();


  static HighGuiWindow* create(const std::string& name);

  virtual void showSafe();

  void addWidget(HighGuiWidget* widget);

  void setWindowTitleSafe(const std::string& window_name);

#if defined (WITH_X11_SUPPORT)
  void attachToWindow(Window window);
#endif

private Q_SLOTS:
  virtual void update();
  virtual void updateAttachment();

private:
  std::list<HighGuiWidget*> _new_widgets;
  std::list<HighGuiWidget*> _widgets;
  QVBoxLayout* _main_layout;
  QGridLayout* _button_area;
  QVBoxLayout* _misc_area;
  std::string _window_name;
  bool _new_window_name;
#if defined (WITH_X11_SUPPORT)
  Window _window;
  Display* _display;
#endif
  bool _show;
};

}
#endif   // HIGHGUIWINDOW_H

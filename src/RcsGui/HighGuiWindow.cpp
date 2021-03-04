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

#include "HighGuiWindow.h"
#include "HighGuiButton.h"
#include "HighGuiLabel.h"
#include "Rcs_guiFactory.h"

#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#if defined (WITH_X11_SUPPORT)
#include <X11/Xlib.h>
#endif

namespace Rcs
{

typedef struct
{
  void* ptr[10];
} VoidPointerList;


static void* createHighGuiWindow(void* arg)
{
  VoidPointerList* p = (VoidPointerList*) arg;
  std::string name = *((std::string*) p->ptr[0]);

  HighGuiWindow* w = new HighGuiWindow(name);
  w->show();

  return w;
}

HighGuiWindow* HighGuiWindow::create(const std::string& name)
{
  VoidPointerList p;
  p.ptr[0] = (void*) &name;

  int handle = RcsGuiFactory_requestGUI(createHighGuiWindow, &p);
  return (HighGuiWindow*) RcsGuiFactory_getPointer(handle);
}



/******************************************************************************/




HighGuiWindow::HighGuiWindow(const std::string& name) :
  HighGuiWidget(name),
  _new_window_name(false),
#if defined (WITH_X11_SUPPORT)
  _window(0),
  _display(NULL),
#endif
  _show(false)
{

#if defined (WITH_X11_SUPPORT)
  _display = XOpenDisplay(NULL);
#endif

  _main_layout = new QVBoxLayout(this);
  _main_layout->setContentsMargins(0, 0, 0, 0);

  _button_area = new QGridLayout();
  _main_layout->addLayout(_button_area);

  _misc_area = new QVBoxLayout();
  _main_layout->addLayout(_misc_area);

  // 1 Hz timer callback for updating attachment
  QTimer* timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), SLOT(updateAttachment()));
  timer->start(1000);
}

HighGuiWindow::~HighGuiWindow()
{
#if defined (WITH_X11_SUPPORT)
  XCloseDisplay(_display);
#endif
}

void HighGuiWindow::showSafe()
{
  lock();
  _show = true;
  unlock();
}

void HighGuiWindow::addWidget(HighGuiWidget* widget)
{
  lock();
  _new_widgets.push_back(widget);
  unlock();
}

void HighGuiWindow::setWindowTitleSafe(const std::string& window_name)
{
  lock();
  _window_name = window_name;
  _new_window_name = true;
  unlock();
}

#if defined (WITH_X11_SUPPORT)
void HighGuiWindow::attachToWindow(Window window)
{
  _window = window;
}
#endif

void HighGuiWindow::update()
{
  HighGuiWidget::update();

  lock();
  if (!_new_widgets.empty())
  {
    HighGuiWidget* w = _new_widgets.front();
    _new_widgets.pop_front();
    w->setParent(this);

    // decide to which layout we add the widget
    if (dynamic_cast<HighGuiButton*>(w))
    {
      unsigned int s = _button_area->count();
      _button_area->addWidget(w, s / 3, s % 3);
    }
    else
    {
      _misc_area->addWidget(w);
    }

    _widgets.push_back(w);
  }

  if (_new_window_name)
  {
    setWindowTitle(QString::fromStdString(_window_name));
    _new_window_name = false;
  }

  if (_show)
  {
    show();
    _show = false;
  }
  unlock();
}

void HighGuiWindow::updateAttachment()
{
#if defined (WITH_X11_SUPPORT)
  if (_display && _window != 0)
  {
    setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
    show();

    // get attach position which is the bottom left of the window
    Window root;
    Window child;
    int x, y;
    unsigned int width, height, border_width, depth;
    int attach_pos[2];

    Status st = XGetGeometry(_display, _window, &root, &x, &y, &width, &height,
                             &border_width, &depth);

    if (st == 0)
    {
      // window does not exist, hide this window, and never do anything again
      hide();
      _window = 0;
    }

    XTranslateCoordinates(_display, _window, root, 0, height, &attach_pos[0],
                          &attach_pos[1], &child);
    resize(width, this->height());
    move(attach_pos[0], attach_pos[1]);
  }
#endif
}

}



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

#include "HighGui.h"
#include "HighGuiLabel.h"
#include "HighGuiWindow.h"

#include <Rcs_macros.h>
#include <Rcs_utilsCPP.h>

namespace Rcs
{
unsigned int HighGui::namedWindow(const std::string& title, int window)
{
  if (window < 0)
  {
    // create a new window with the first availbale id
    window = instance().getAvailableWindowID();
  }

  // show or create window
  instance().showWindow(window);

  instance().lock();
  // create if widget with the label name does not exist
  HighGuiWindow* w = instance().getWidget<HighGuiWindow>(computeID(window));

  // set window name
  if (w)
  {
    w->setWindowTitleSafe(title);
  }
  else
  {
    RLOGS(0, "Something went wrong with creating window %u", window);
  }

  instance().unlock();

  return window;
}

bool HighGui::checkButton(const std::string& name, unsigned int window,
                          bool reset)
{
  // show or create window
  instance().showWindow(window);

  bool ret_val = false;
  instance().lock();

  std::string id = computeID(window, name);

  // first check if button exists, if not we create it
  HighGuiButton* w = instance().getWidget<HighGuiButton>(id);

  if (!w)
  {
    w = HighGuiButton::create(name);
    instance()._widgets[id] = w;

    instance().addWidgetToWindow(window, w);
  }

  if (w)
  {
    ret_val = w->pressed(reset);
  }
  else
  {
    RLOGS(0, "Something went wrong with creating button %s", name.c_str());
  }

  instance().unlock();

  return ret_val;
}

void HighGui::registerButtonCallback(const std::string& name, unsigned int window,
                                     HighGuiButton::CallbackType cb_function)
{
  instance().lock();
  // get slider
  HighGuiButton* w = instance().getWidget<HighGuiButton>(computeID(window, name));
  if (w)
  {
    w->registerCallback(cb_function);
  }

  instance().unlock();
}

void HighGui::showLabel(const std::string& name, unsigned int window,
                        const char* str, ...)
{
  // show or create window
  instance().showWindow(window);

  instance().lock();

  std::string id = computeID(window, name);

  // create if widget with the label name does not exist
  HighGuiLabel* w = instance().getWidget<HighGuiLabel>(id);

  if (!w)
  {
    w = HighGuiLabel::create(name);
    instance()._widgets[id] = w;
    instance().addWidgetToWindow(window, w);
  }

  // set label text
  if (w)
  {
    va_list ap;
    va_start(ap, str);
    std::string label_string = formatStdString(str, ap);
    va_end(ap);

    w->setLabel(label_string);
  }
  else
  {
    RLOGS(0, "Error setting label, '%s' ('%d') is not of type HighGuiLabel",
          name.c_str(), window);
  }

  instance().unlock();
}

void HighGui::showSlider(const std::string& name, unsigned int window,
                         double min, double max,
                         double step_size, Atomic<double>* feedback)
{
  // show or create window
  instance().showWindow(window);

  instance().lock();

  std::string id = computeID(window, name);

  // create if widget with the label name does not exist
  HighGuiSlider* w = instance().getWidget<HighGuiSlider>(id);

  if (!w)
  {
    w = HighGuiSlider::create(name, min, max, step_size, feedback);
    instance()._widgets[id] = w;

    instance().addWidgetToWindow(window, w);
  }

  instance().unlock();
}

double HighGui::getSlider(const std::string& name, unsigned int window)
{
  double ret_val = 0.0;

  instance().lock();
  // get slider
  HighGuiSlider* w = instance().getWidget<HighGuiSlider>(computeID(window, name));
  if (w)
  {
    ret_val = w->getValue();
  }

  instance().unlock();

  return ret_val;
}

void HighGui::registerSliderCallback(const std::string& name, unsigned int window,
                                     HighGuiSlider::CallbackType cb_function)
{
  instance().lock();
  // get slider
  HighGuiSlider* w = instance().getWidget<HighGuiSlider>(computeID(window, name));
  if (w)
  {
    w->registerCallback(cb_function);
  }

  instance().unlock();
}

void HighGui::showPlot(const std::string& name, unsigned int window,
                       double value, unsigned int value_id)
{
  // show or create window
  instance().showWindow(window);

  instance().lock();

  std::string id = computeID(window, name);

  // create if widget with the name does not exist
  HighGuiPlot* w = instance().getWidget<HighGuiPlot>(id);
  if (!w)
  {
    w = HighGuiPlot::create(name);
    instance()._widgets[id] = w;
    instance().addWidgetToWindow(window, w);
  }

  // set plot value
  if (w)
  {
    w->addValue(value, value_id);
  }
  else
  {
    RLOGS(0, "Error setting plot value, '%s' ('%d') is not of type HighGuiPlot",
          name.c_str(), window);
  }

  instance().unlock();
}

void HighGui::showPlot(const std::string& name, unsigned int window,
                       const double* value, unsigned int n)
{
  // show or create window
  instance().showWindow(window);

  instance().lock();

  std::string id = computeID(window, name);

  // create if widget with the name does not exist
  HighGuiPlot* w = instance().getWidget<HighGuiPlot>(id);

  if (!w)
  {
    w = HighGuiPlot::create(name);
    instance()._widgets[id] = w;
    instance().addWidgetToWindow(window, w);
  }

  // set plot value
  if (w)
  {
    w->addValues(value, n);
  }
  else
  {
    RLOGS(0, "Error setting plot value, '%s' ('%d') is not of type HighGuiPlot",
          name.c_str(), window);
  }

  instance().unlock();
}

void HighGui::showPlot(const std::string& name, unsigned int window,
                       const std::vector<double> value)
{
  showPlot(name, window, value.data(), value.size());
}

void HighGui::configurePlot(const std::string& name, unsigned int window,
                            unsigned int capacity, double min, double max)
{
  // show or create window
  instance().showWindow(window);

  instance().lock();

  std::string id = computeID(window, name);

  // create if widget with the name does not exist
  HighGuiPlot* w = instance().getWidget<HighGuiPlot>(id);

  if (!w)
  {
    w = HighGuiPlot::create(name);
    instance()._widgets[id] = w;
    instance().addWidgetToWindow(window, w);
  }

  if (w)
  {
    w->setCapacity(capacity);
    w->setLimits(min, max);
  }

  instance().unlock();
}

#ifdef USE_OPENCV

void HighGui::showImage(const std::string& name, unsigned int window,
                        const cv::Mat& image)
{
  // show or create window
  instance().showWindow(window);

  instance().lock();

  std::string id = computeID(window, name);

  // create if widget with the name does not exist
  HighGuiImage* w = instance().getWidget<HighGuiImage>(id);

  if (!w)
  {
    w = HighGuiImage::create(name);
    instance()._widgets[id] = w;
    instance().addWidgetToWindow(window, w);
  }

  // set plot value
  if (w)
  {
    w->setImage(image);
  }
  else
  {
    RLOGS(0, "Error setting image, '%s' ('%d') is not of type HighGuiImage",
          name.c_str(), window);
  }

  instance().unlock();
}

#endif

#if defined (WITH_X11_SUPPORT)
void HighGui::attachWindowToWindow(unsigned int window, Window x11window)
{
  instance().lock();
  // create if widget with the label name does not exist
  HighGuiWindow* w = instance().getWidget<HighGuiWindow>(computeID(window));

  // set window name
  if (w)
  {
    w->attachToWindow(x11window);
  }
  else
  {
    RLOGS(0, "Window %u does not exist", window);
  }

  instance().unlock();
}
#endif

HighGui::~HighGui()
{
  pthread_mutex_destroy(&_mutex);
}

void HighGui::showWindow(unsigned int window)
{
  lock();

  std::string id = computeID(window);

  // check if window exists
  HighGuiWindow* w = getWidget<HighGuiWindow>(id);

  if (!w)
  {
    // create window
    w = HighGuiWindow::create(formatStdString("%u", window));
    _widgets[id] = w;
  }

  // bring window to foreground
  w->showSafe();

  unlock();
}

void HighGui::addWidgetToWindow(unsigned int window, HighGuiWidget* widget)
{
  // check if window exists
  HighGuiWindow* w = getWidget<HighGuiWindow>(computeID(window));

  if (!w)
  {
    RLOGS(0, "Window %d does not exist", window);
    return;
  }

  // add widget to window
  w->addWidget(widget);
}

template<class T>
T* HighGui::getWidget(const std::string& id)
{
  WidgetMapType::iterator it;
  it = _widgets.find(id);

  if (it != _widgets.end())
  {
    return dynamic_cast<T*>(it->second);
  }
  else
  {
    return NULL;
  }
}

std::string HighGui::computeID(unsigned int window, const std::string& name)
{
  return formatStdString("%u#%s", window, name.c_str());
}


unsigned int HighGui::getAvailableWindowID() const
{
  std::list<unsigned int> window_ids;
  for (WidgetMapType::const_iterator it = _widgets.begin(); it != _widgets.end(); ++it)
  {
    HighGuiWindow* w = dynamic_cast<HighGuiWindow*>(it->second);
    if (w)
    {
      std::string name = w->getName();
      unsigned int n = static_cast<unsigned int>(atoi(name.c_str()));
      window_ids.push_back(n);
    }
  }

  unsigned int id = 0;
  for (std::list<unsigned int>::iterator it = window_ids.begin();
       it != window_ids.end(); ++it, id++)
  {
    if (*it != id)
    {
      break;
    }
  }

  return id;
}


HighGui::HighGui()
{
  pthread_mutex_init(&_mutex, NULL);
}

void HighGui::lock()
{
  pthread_mutex_lock(&_mutex);
}

void HighGui::unlock()
{
  pthread_mutex_unlock(&_mutex);
}

HighGui& HighGui::instance()
{
  static HighGui inst;
  return inst;
}



}  //namespace Rcs

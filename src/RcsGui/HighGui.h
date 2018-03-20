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

#ifndef HIGHGUI_H
#define HIGHGUI_H

#include "Atomic.hpp"
#include "HighGuiButton.h"
#include "HighGuiImage.h"
#include "HighGuiPlot.h"
#include "HighGuiSlider.h"

#include <map>
#include <string>
#include <limits>
#include <pthread.h>

/*!
 *  \defgroup RcsHighGui RcsHighGui
 *
 * This is the RCS High-level GUI. The idea is to provide a set of simple
 * static functions that allow to create a GUI for prototyping.
 */

// forward declarations
typedef unsigned long Window;
class QWidget;
namespace Rcs
{
class HighGuiWidget;
}


namespace Rcs
{

/*!
 * \ingroup RcsHighGui
 * \brief The main HighGui class providing all the static functions for creating
 *        the GUI
 *
 * This class is implemented as a singleton, so access is only possible via
 * static functions. Example usage can be found in ExampleHighGui.cpp.
 *
 * The HighGui is completely thread-safe, so you can call any combination of
 * methods in any thread. Note however, that all callback functions are executed
 * in the context of the GUI threads.
 */
class HighGui
{
public:

  /*!
   * \brief Create a new window and set its title
   *
   * If the window already exists, it will brought to foreground and
   * its title will be updated.
   *
   * \param[in] title New window title
   * \param[in] window ID of the window, if set to -1, a new window will be
   *                   created with the first available ID
   * \return ID of the newly created window (useful if window is set to -1)
   */
  static unsigned int namedWindow(const std::string& title, int window = 0);

  /*!
   * \brief Check the state of a button and optionally resets it. If the button
   *        does not exist yet, it will be created.
   * \param[in] name Button label (also used as button id for referencing
   *                    with checkButton()
   * \param[in] window The window id in which the button should be shown
   * \param[in] reset Reset the state of the button to unchecked. If set to
   *                  false, the button behaves as a toggle button.
   * \return True if button state is pressed (checked)
   */
  static bool checkButton(const std::string& name,
                          unsigned int window = 0, bool reset = true);

  /*!
   * \brief Register a callback function that is executed upon button press
   *
   * If a callback function is registered, the button will always be reset
   * automatically. Thus it will always behave as a normal button and cannot
   * be used as a toggle button.
   *
   * \param[in] name Button name
   * \param[in] window Button window
   * \param[in] cb_function Callback function to be called upon button press
   */
  static void registerButtonCallback(const std::string& name,
                                     unsigned int window,
                                     HighGuiButton::CallbackType cb_function);

  /*!
   * \brief Displays a simple text label
   *
   * \param[in] name Label name
   * \param[in] window Label window
   * \param[in] str String in typical printf format
   * \param[in] ... Arguments for str
   *
   */
  static void showLabel(const std::string& name, unsigned int window = 0,
                        const char* str = NULL, ...);

  /*!
   * \brief Shows a slider
   *
   * The feedback value is optional and it will be used as in and output during
   * operation. This means, sliders will update themselves from the value, but
   * also write into the value, if the user moves the slider. Using
   * Atomic<double> is inherently thread-safe.
   *
   * \param[in] name Slider name
   * \param[in] window Slider window
   * \param[in] min Minimum slider value
   * \param[in] max Maximum slider value
   * \param[in] step_size Slider step size
   * \param[in] feedback Pointer to value that will be updated by the slider or
   *                     NULL if not needed
   */
  static void showSlider(const std::string& name, unsigned int window = 0,
                         double min = 0.0, double max = 100.0,
                         double step_size = 1.0,
                         Atomic<double>* feedback = NULL);

  /*!
   * \brief Returns the current value of the slider
   *
   * \param[in] name Slider name
   * \param[in] window Slider window
   * \return Current slider value
   */
  static double getSlider(const std::string& name, unsigned int window = 0);

  /*!
   * \brief Register a callback function that is executed upon a value change
   *
   * \param[in] name Slider name
   * \param[in] window Slider window
   * \param[in] cb_function Callback function to be called upon value change
   */
  static void registerSliderCallback(const std::string& name,
                                     unsigned int window,
                                     HighGuiSlider::CallbackType cb_function);

  /*!
   * \brief Show a single curve plot or update it with the given value
   *
   * The initial call to this method will create the plot and any subsequent
   * call with the same parameters will add another data sample. Specifying
   * the value_id allows to have multiple curves in one plot. In this case
   * it is probably best to always update all values in your loop, otherwise
   * they will get out of sync.
   *
   * \param[in] name Plot name
   * \param[in] window Plot window
   * \param[in] value A new value that will be added as the latest data sample
   * \param[in] value_id ID of the curve starting with 0
   */
  static void showPlot(const std::string& name, unsigned int window,
                       double value, unsigned int value_id = 0);

  /*!
   * \brief Shows a multi curve plot or updates all values of an existing multi
   *        curve plot
   *
   * This is a convenience method to plot values from a vector over time.
   * The same thing could be achieved by iterating over the vector and calling
   * the single curve version of showPlot().
   * Using this method, you have no control over the value_id. Instead,
   * internally value_ids larger than MaxValueID will be generated and updated.
   * This allows also to mix single curve plots and multi curve plots within one
   * plot.
   *
   * \param[in] name Plot name
   * \param[in] window Plot window
   * \param[in] value Vector representing the new data sample
   * \param[in] n Number of elements in the vector
   */
  static void showPlot(const std::string& name, unsigned int window,
                       const double* value, unsigned int n);

  static void showPlot(const std::string& name, unsigned int window,
                       const std::vector<double> value);

  /*!
   * \brief Sets various properties of a HighGuiPlot
   *
   * \note Some of these properties can also be set via the context menu.
   *
   * \param[in] name Plot name
   * \param[in] window Plot window
   * \param[in] capacity Number of shown data samples
   * \param[in] min Minimum value of the left y axis. Setting min or max to nan,
   *                enables auto scaling.
   * \param[in] max Maximum value of the left y axis. Setting min or max to nan,
   *                enables auto scaling.
   */
  static void configurePlot(const std::string& name, unsigned int window = 0,
                            unsigned int capacity = HighGuiPlot::DefaultCapacity,
                            double min = std::numeric_limits<double>::quiet_NaN(),
                            double max = std::numeric_limits<double>::quiet_NaN());

#ifdef USE_OPENCV
  /*!
   * \brief Shows an OpenCV image in a HighGui window
   *
   * This is method basically replaces OpenCV imshow(), because that one
   * does not work together with Qt applications.
   */
  static void showImage(const std::string& name, unsigned int window,
                        const cv::Mat& image);
#endif

#if !defined (_MSC_VER)
  /*!
   * \brief Attaches a HighGuiWindow to any X11 window
   *
   * Attaching means the the HighGuiWindow checks once a second if the given
   * X11 window has changed. If yes the title bar of the HighGuiWindow will be
   * hidden, the width will be resized to match the other window and it will
   * be moved below the other window.
   *
   * A use case for this function is to create a config window for a RcsViewer.
   *
   * \param[in] window HighGui window ID
   * \param[in] x11window X11 window handle of the other window
   */
  static void attachWindowToWindow(unsigned int window, Window x11window);
#endif

  ~HighGui();

private:
  /*!
   * \brief Private constructor
   *
   * Class can only be instantiated via getInstance().
   */
  HighGui();

  /*!
   * \brief Create window or bring window to foreground
   *
   * \param[in] window Window ID
   */
  void showWindow(unsigned int window);

  /*!
   * \brief Adds a new HighGuiWidget to a window
   *
   * \param[in] window Window ID
   * \param[in] widget HighGuiWidget to be added
   */
  void addWidgetToWindow(unsigned int window, HighGuiWidget* widget);

  /*!
   * \brief Convenience funcation that searches for a widget and performs a
   *        dynamic_cast to type T
   *
   * \param id Internal widget name including the window id
   */
  template<class T>
  T* getWidget(const std::string& id);

  /*!
   * \brief Creates the ID from window number and name
   */
  static std::string computeID(unsigned int window,
                               const std::string& name = std::string());

  /*!
   * \brief Find the first available window ID
   *
   * \return Unused window ID
   */
  unsigned int getAvailableWindowID() const;

  /*!
   * \brief Locks the internal mutex
   */
  void lock();

  /*!
   * \brief Unlocks the internal mutex
   */
  void unlock();

  /*!
   * \brief Instance method holding the HighGui instance
   *
   * Better than member variable because of automatic destruction
   */
  static HighGui& instance();

private:
  typedef std::map<std::string, HighGuiWidget*> WidgetMapType; //!< Widget map type
  WidgetMapType _widgets; //!< All registered widgets
  pthread_mutex_t _mutex; //!< Internal mutex
};


} //namespace Rcs

#endif // HIGHGUI_H

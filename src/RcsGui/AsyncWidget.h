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

#ifndef RCS_ASYNCWIDGET_H
#define RCS_ASYNCWIDGET_H

#include <QWidget>
#include <QEvent>
#include <QMutex>



namespace Rcs
{

// In order to make this class launch the GuiFactory, it cannot inherit
// from QObject. The Qt documentation states that all calls to anything
// like QObject must be done after the QAppliucation has been launched in
// the Gui thread. If this inherits from QObject, the constructor calls a
// QObject method from the wrong thread, which leads to a warning.
class AsyncWidget
{
  friend class WidgetLauncher;

public:
  AsyncWidget();

  virtual ~AsyncWidget();


  const QWidget* getWidget() const;

  QWidget* getWidget();

  /*! \brief Emits an event that calls construct() from the Gui thread. The
   *         function returns after the widget has been created, it is
   *         ensured that getWidget() returns a valid pointer.
   */
  void launch();

  /*! \brief Emits an event that calls destroy() from the Gui thread. This will
   *         delete the QWidget member w and set it to NULL. This is done
   *         either directly or through the Qt event loop in case the unlaunch
   *         function was called from a different thread. The function only
   *         returns after the widget has been deleted and is NULL.
   */
  void unlaunch();

  // Blocks until widget has been deleted.
  void waitUntilWidgetDeleted();

  void setWidget(QWidget* widget);

protected:

  void setLaunched(bool launched);
  bool isLaunched() const;

  virtual void construct() = 0;
  virtual void destroy();

private:
  bool launched;   // Fully mutex protected
  QWidget* w;
  static int refCount;
  mutable QMutex launchMtx;
};




}   // namespace Rcs

#endif   // RCS_ASYNCWIDGET_H

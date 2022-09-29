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

#ifndef RCS_TEXTEDITGUI_H
#define RCS_TEXTEDITGUI_H

#include "AsyncWidget.h"

#include <QScrollArea>
#include <QLineEdit>
#include <QMutex>

#include <string>



namespace Rcs
{


class TextEditWidget : public QScrollArea
{
  Q_OBJECT

public:

  class TextChangeCallback
  {
  public:
    virtual void callback(std::string text) = 0;
  };
  TextEditWidget();

  virtual ~TextEditWidget();
  // Takes ownership of the callback
  void registerCallback(TextChangeCallback* callback);

  std::string getAndResetText();

private slots:
  void handleText();

private:
  std::string text;
  QLineEdit* lineEdit;
  QMutex mtx;
  std::vector<TextChangeCallback*> callback;
};

class TextEditGui : public Rcs::AsyncWidget
{
public:
  TextEditGui();
  ~TextEditGui();
  void construct();
  std::string getAndResetText();

  // Takes ownership of the callback
  void registerCallback(TextEditWidget::TextChangeCallback* callback);

private:
  std::vector<TextEditWidget::TextChangeCallback*> callback;
};

}   // namespace Rcs

#endif   // RCS_TEXTEDITGUI_H

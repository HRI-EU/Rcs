/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH

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

#include "STTGui.h"

#include <Rcs_utilsCPP.h>
#include <Rcs_macros.h>

#include <QLayout>
#include <QTimer>



namespace Rcs
{

/*******************************************************************************
 *
 ******************************************************************************/
STTGui::STTGui()
{
  launch();
}

void STTGui::construct()
{
  setWidget(new Rcs::STTWidget());
}

std::string STTGui::getAndResetText()
{
  STTWidget* tew = dynamic_cast<STTWidget*>(getWidget());
  return tew->getAndResetText();
}

/*******************************************************************************
 *
 ******************************************************************************/
STTWidget::STTWidget() : QScrollArea(), active(false)
{
  QHBoxLayout* gridLayout = new QHBoxLayout;
  gridLayout->setContentsMargins(0, 0, 0, 0);

  this->lineEdit = new QLineEdit(this);
  connect(lineEdit, SIGNAL(returnPressed()), this, SLOT(handleText()));
  gridLayout->addWidget(lineEdit);

  timerId = startTimer(40);
}

STTWidget::~STTWidget()
{
  killTimer(timerId);
}

std::string STTWidget::getAndResetText()
{
  mtx.lock();
  std::string tmp = text;
  text.clear();
  mtx.unlock();

  return tmp;
}

std::string STTWidget::getDisplayedText()
{
  mtx.lock();
  std::string tmp = lineEdit->displayText().toStdString();
  mtx.unlock();

  return tmp;
}

void STTWidget::handleText()
{
  mtx.lock();
  text = lineEdit->text().toStdString();
  mtx.unlock();
  lineEdit->clear();
}

void STTWidget::timerEvent(QTimerEvent* event)
{
  std::string spokenText = getDisplayedText();
  std::string wakeUpWord = "Robot ";

  //RLOG(0, "Text: \"%s\"", spokenText.c_str());

  if (!active && spokenText.compare(0, 6, "Robot ")==0)
  {
    active = true;
    spokenText.erase(spokenText.begin(), spokenText.begin() + wakeUpWord.length());
    lineEdit->clear();
    RLOG(0, "ACTIVE");
  }

  if (active && String_endsWith(spokenText, ". "))
  {
    active = false;
    RLOG(0, "INACTIVE");
    spokenText.pop_back();
    spokenText.pop_back();

    RLOG(0, "\"%s\"", spokenText.c_str());
    setText(spokenText);
  }

}

void STTWidget::setText(const std::string& newText)
{
  mtx.lock();
  text = newText;
  mtx.unlock();
}

}   // namespace Rcs

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

#include "TextEditGui.h"

#include <Rcs_macros.h>

#include <QLayout>



namespace Rcs
{

/*******************************************************************************
 *
 ******************************************************************************/
TextEditGui::TextEditGui()
{
  launch();
}

void TextEditGui::construct()
{
  setWidget(new Rcs::TextEditWidget());
}

std::string TextEditGui::getAndResetText()
{
  TextEditWidget* tew = dynamic_cast<TextEditWidget*>(getWidget());
  return tew->getAndResetText();
}

/*******************************************************************************
 *
 ******************************************************************************/
TextEditWidget::TextEditWidget() : QScrollArea()
{
  QHBoxLayout* gridLayout = new QHBoxLayout;
  gridLayout->setContentsMargins(0, 0, 0, 0);

  this->lineEdit = new QLineEdit(this);
  connect(lineEdit, SIGNAL(returnPressed()), this, SLOT(handleText()));
  gridLayout->addWidget(lineEdit);
}

TextEditWidget::~TextEditWidget()
{
}

std::string TextEditWidget::getAndResetText()
{
  mtx.lock();
  std::string tmp = text;
  text.clear();
  mtx.unlock();

  return tmp;
}

void TextEditWidget::handleText()
{
  mtx.lock();
  text = lineEdit->text().toStdString();
  mtx.unlock();
  lineEdit->clear();
}



}   // namespace Rcs

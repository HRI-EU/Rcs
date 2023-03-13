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

#include "CmdLineWidget.h"

#include <Rcs_macros.h>

#include <QLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSignalMapper>
#include <QTextStream>
#include <QDebug>
#include <QtGlobal>

#include <algorithm>
#include <sstream>
#include <cstdlib>


namespace Rcs
{

template<typename T>
class ParameterLine : public ParameterCollection::Entry
{
public:
  ParameterLine(std::string name_, std::string description_, T* param_) :
    name(name_), description(description_), param(param_)
  {
  }

  virtual ~ParameterLine()
  {
  }

  virtual void setParam(std::string paramString) = 0;

  virtual std::string getParamAsString() const
  {
    std::stringstream  sstream;
    sstream << (*param);
    return sstream.str();
  }

  std::string getName() const
  {
    return name;
  }

  std::string getDescription() const
  {
    return description;
  }

  std::string name;
  std::string description;
  T* param;
};

class ParameterLineChar : public ParameterLine<char>
{
public:
  ParameterLineChar(std::string name, std::string description, char* param) :
    ParameterLine(name, description, param)
  {
  }
  void setParam(std::string paramString)
  {
    strcpy(param, paramString.c_str());
  }
  std::string getParamAsString() const
  {
    return std::string(param);
  }

};

class ParameterLineInt : public ParameterLine<int>
{
public:
  ParameterLineInt(std::string name, std::string description, int* param) :
    ParameterLine(name, description, param)
  {
  }
  void setParam(std::string paramString)
  {
    *param = atoi(paramString.c_str());
  }
};

class ParameterLineUnsignedInt : public ParameterLine<unsigned int>
{
public:
  ParameterLineUnsignedInt(std::string name, std::string description,
                           unsigned int* param) :
    ParameterLine(name, description, param)
  {
  }
  void setParam(std::string paramString)
  {
    *param = atoi(paramString.c_str());
  }
};

class ParameterLineUnsignedLong : public ParameterLine<unsigned long>
{
public:
  ParameterLineUnsignedLong(std::string name, std::string description,
                            unsigned long* param) :
    ParameterLine(name, description, param)
  {
  }
  void setParam(std::string paramString)
  {
    *param = atol(paramString.c_str());
  }
};

class ParameterLineUnsignedLongLong : public ParameterLine<unsigned long long>
{
public:
  ParameterLineUnsignedLongLong(std::string name, std::string description,
                                unsigned long long* param) :
    ParameterLine(name, description, param)
  {
  }
  void setParam(std::string paramString)
  {
    *param = atoll(paramString.c_str());
  }
};

class ParameterLineBool : public ParameterLine<bool>
{
public:
  ParameterLineBool(std::string name, std::string description, bool* param) :
    ParameterLine(name, description, param)
  {
  }
  void setParam(std::string paramString)
  {
    if (STRCASEEQ(paramString.c_str(), "true"))
    {
      *param = true;
    }
    else
    {
      *param = false;
    }
  }
  std::string getParamAsString() const
  {
    if (*param==true)
    {
      return std::string("true");
    }

    return std::string("false");
  }
};

class ParameterLineString : public ParameterLine<std::string>
{
public:
  ParameterLineString(std::string name, std::string description, std::string* param) :
    ParameterLine(name, description, param)
  {
  }
  void setParam(std::string paramString)
  {
    *param = paramString;
  }
  std::string getParamAsString() const
  {
    return *param;
  }

};

class ParameterLineDouble : public ParameterLine<double>
{
public:
  ParameterLineDouble(std::string name, std::string description, double* param) :
    ParameterLine(name, description, param)
  {
  }
  void setParam(std::string paramString)
  {
    *param = strtod(paramString.c_str(), NULL);
  }
};

class ParameterLineFloat : public ParameterLine<float>
{
public:
  ParameterLineFloat(std::string name, std::string description, float* param) :
    ParameterLine(name, description, param)
  {
  }
  void setParam(std::string paramString)
  {
    *param = strtof(paramString.c_str(), NULL);
  }
};






bool ParameterCollection::getArgument(const char* name, char* value,
                                      const char* description, ...)
{
  char buffer[512] = "";
  if (description)
  {
    va_list args;
    va_start(args, description);
    vsnprintf(buffer, sizeof(buffer), description, args);
    va_end(args);
  }
  paramLine.push_back(new ParameterLineChar(name, buffer, value));
  return true;
}
bool ParameterCollection::getArgument(const char* name, int* value,
                                      const char* description, ...)
{
  char buffer[512] = "";
  if (description)
  {
    va_list args;
    va_start(args, description);
    vsnprintf(buffer, sizeof(buffer), description, args);
    va_end(args);
  }
  paramLine.push_back(new ParameterLineInt(name, buffer, value));
  return true;
}
bool ParameterCollection::getArgument(const char* name, unsigned int* value,
                                      const char* description, ...)
{
  char buffer[512] = "";
  if (description)
  {
    va_list args;
    va_start(args, description);
    vsnprintf(buffer, sizeof(buffer), description, args);
    va_end(args);
  }
  paramLine.push_back(new ParameterLineUnsignedInt(name, buffer, value));
  return true;
}
bool ParameterCollection::getArgument(const char* name, unsigned long* value,
                                      const char* description, ...)
{
  char buffer[512] = "";
  if (description)
  {
    va_list args;
    va_start(args, description);
    vsnprintf(buffer, sizeof(buffer), description, args);
    va_end(args);
  }
  paramLine.push_back(new ParameterLineUnsignedLong(name, buffer, value));
  return true;
}
bool ParameterCollection::getArgument(const char* name, unsigned long long* value,
                                      const char* description, ...)
{
  char buffer[512] = "";
  if (description)
  {
    va_list args;
    va_start(args, description);
    vsnprintf(buffer, sizeof(buffer), description, args);
    va_end(args);
  }
  paramLine.push_back(new ParameterLineUnsignedLongLong(name, buffer, value));
  return true;
}
bool ParameterCollection::getArgument(const char* name, bool* value,
                                      const char* description, ...)
{
  char buffer[512] = "";
  if (description)
  {
    va_list args;
    va_start(args, description);
    vsnprintf(buffer, sizeof(buffer), description, args);
    va_end(args);
  }
  paramLine.push_back(new ParameterLineBool(name, buffer, value));
  return true;
}
bool ParameterCollection::getArgument(const char* name, std::string* value,
                                      const char* description, ...)
{
  char buffer[512] = "";
  if (description)
  {
    va_list args;
    va_start(args, description);
    vsnprintf(buffer, sizeof(buffer), description, args);
    va_end(args);
  }
  paramLine.push_back(new ParameterLineString(name, buffer, value));
  return true;
}
bool ParameterCollection::getArgument(const char* name, double* value,
                                      const char* description, ...)
{
  char buffer[512] = "";
  if (description)
  {
    va_list args;
    va_start(args, description);
    vsnprintf(buffer, sizeof(buffer), description, args);
    va_end(args);
  }
  paramLine.push_back(new ParameterLineDouble(name, buffer, value));
  return true;
}
bool ParameterCollection::getArgument(const char* name, float* value,
                                      const char* description, ...)
{
  char buffer[512] = "";
  if (description)
  {
    va_list args;
    va_start(args, description);
    vsnprintf(buffer, sizeof(buffer), description, args);
    va_end(args);
  }
  paramLine.push_back(new ParameterLineFloat(name, buffer, value));
  return true;
}
bool ParameterCollection::hasArgument(const char* tag,
                                      const char* description, ...)
{
  RLOG(1, "This function does not work as you expect it!");
  return false;
}
size_t ParameterCollection::size() const
{
  return paramLine.size();
}
ParameterCollection::Entry* ParameterCollection::getEntry(size_t index)
{
  return paramLine[index];
}

void ParameterCollection::clear()
{
  for (size_t i=0; i<paramLine.size(); ++i)
  {
    delete paramLine[i];
  }

  paramLine.clear();
}

static bool sortFunc(ParameterCollection::Entry* e1, ParameterCollection::Entry* e2)
{
  return (e1->getName() < e2->getName());
}

void ParameterCollection::sort()
{
  // Alphabetically sort the collection
  std::sort(paramLine.begin(), paramLine.end(), sortFunc);
}











CmdLineGui::CmdLineGui(ParameterCollection* pc, std::string title_) :
  AsyncWidget(), collection(pc), title(title_)
{
  launch();
}

void CmdLineGui::construct()
{
  setWidget(new CmdLineWidget(collection, title));
}



CmdLineWidget::CmdLineWidget(ParameterCollection* collection, std::string title, QWidget* parent) :
  QScrollArea(parent)
{
  if (title.empty())
  {
    setWindowTitle("CmdLineWidget");
  }
  else
  {
    setWindowTitle(QString::fromStdString(title));
  }

  QVBoxLayout* gridLayout = new QVBoxLayout();
  QWidget* scrollWidget = new QWidget(this);
  scrollWidget->setLayout(gridLayout);
  this->setWidget(scrollWidget);

  for (size_t i=0; i<collection->size(); ++i)
  {
    gridLayout->addWidget(new CmdLineEntry(collection->getEntry(i)));
  }

  this->setWidgetResizable(true);
  setObjectName("CmdLineWidget");
}


CmdLineWidget::~CmdLineWidget()
{
  RLOG(5, "Destroying CmdLineWidget()");
}


CmdLineEntry::CmdLineEntry(ParameterCollection::Entry* line_) : line(line_)
{
  QHBoxLayout* hbox = new QHBoxLayout(this);

  QLabel* label = new QLabel(QString::fromStdString(line->getName()));
  label->setTextInteractionFlags(Qt::TextSelectableByMouse);
  hbox->addWidget(label);

  this->textParam = new QLineEdit(this);
  textParam->setText(QString::fromStdString(line->getParamAsString()));
  QString text = textParam->text();
  QFontMetrics fm(textParam->font());
#if QT_VERSION < QT_VERSION_CHECK(5, 11, 0)
  int pixelsWide = fm.width(text);
#else
  int pixelsWide = fm.horizontalAdvance(text);
#endif
  textParam->setFixedWidth(50+pixelsWide);
  connect(textParam, SIGNAL(returnPressed()), this, SLOT(handleText()));
  hbox->addWidget(textParam);

  QLabel* descr = new QLabel(QString::fromStdString(line->getDescription()));
  descr->setTextInteractionFlags(Qt::TextSelectableByMouse);
  hbox->addWidget(descr);

  hbox->addStretch();
}

void CmdLineEntry::handleText()
{
  line->setParam(textParam->text().toStdString());
}


}   // namespace Rcs

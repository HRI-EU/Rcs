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

#include "Rcs_guiFactory.h"
#include <Rcs_macros.h>

#include <QLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSignalMapper>
#include <QTextStream>
#include <QDebug>

#include <algorithm>
#include <sstream>
#include <cstdlib>


namespace Rcs
{

ParameterLine::ParameterLine(std::string name_, std::string description_) :
  name(name_), description(description_)
{
}

ParameterLine::~ParameterLine()
{
}

class ParameterLineChar : public ParameterLine
{
public:
  ParameterLineChar(std::string name, std::string description, char* param_) :
    ParameterLine(name, description), param(param_)
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

protected:
  char* param;
};

class ParameterLineInt : public ParameterLine
{
public:
  ParameterLineInt(std::string name, std::string description, int* param_) :
    ParameterLine(name, description), param(param_)
  {
  }
  void setParam(std::string paramString)
  {
    *param = atoi(paramString.c_str());
  }
  std::string getParamAsString() const
  {
    std::ostringstream sstream;
    sstream << (*param);
    return sstream.str();
  }

protected:
  int* param;
};

class ParameterLineUnsignedInt : public ParameterLine
{
public:
  ParameterLineUnsignedInt(std::string name, std::string description,
                           unsigned int* param_) :
    ParameterLine(name, description), param(param_)
  {
  }
  void setParam(std::string paramString)
  {
    *param = atoi(paramString.c_str());
  }
  std::string getParamAsString() const
  {
    std::ostringstream sstream;
    sstream << (*param);
    return sstream.str();
  }

protected:
  unsigned int* param;
};

class ParameterLineUnsignedLong : public ParameterLine
{
public:
  ParameterLineUnsignedLong(std::string name, std::string description,
                            unsigned long* param_) :
    ParameterLine(name, description), param(param_)
  {
  }
  void setParam(std::string paramString)
  {
    *param = atol(paramString.c_str());
  }
  std::string getParamAsString() const
  {
    std::ostringstream sstream;
    sstream << (*param);
    return sstream.str();
  }

protected:
  unsigned long* param;
};

class ParameterLineUnsignedLongLong : public ParameterLine
{
public:
  ParameterLineUnsignedLongLong(std::string name, std::string description,
                                unsigned long long* param_) :
    ParameterLine(name, description), param(param_)
  {
  }
  void setParam(std::string paramString)
  {
    *param = atoll(paramString.c_str());
  }
  std::string getParamAsString() const
  {
    std::ostringstream sstream;
    sstream << (*param);
    return sstream.str();
  }

protected:
  unsigned long long* param;
};

class ParameterLineBool : public ParameterLine
{
public:
  ParameterLineBool(std::string name, std::string description,
                    bool* param_) :
    ParameterLine(name, description), param(param_)
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

protected:
  bool* param;
};

class ParameterLineString : public ParameterLine
{
public:
  ParameterLineString(std::string name, std::string description, std::string* param_) :
    ParameterLine(name, description), param(param_)
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

protected:
  std::string* param;
};

class ParameterLineDouble : public ParameterLine
{
public:
  ParameterLineDouble(std::string name, std::string description, double* param_) :
    ParameterLine(name, description), param(param_)
  {
  }
  void setParam(std::string paramString)
  {
    *param = strtod(paramString.c_str(), NULL);
  }
  std::string getParamAsString() const
  {
    std::ostringstream sstream;
    sstream << (*param);
    return sstream.str();
  }

protected:
  double* param;
};

class ParameterLineFloat : public ParameterLine
{
public:
  ParameterLineFloat(std::string name, std::string description, float* param_) :
    ParameterLine(name, description), param(param_)
  {
  }
  void setParam(std::string paramString)
  {
    *param = strtof(paramString.c_str(), NULL);
  }
  std::string getParamAsString() const
  {
    std::ostringstream sstream;
    sstream << (*param);
    return sstream.str();
  }

protected:
  float* param;
};






bool ParameterCollection::getArgument(const char* name, char* value,
                                      const char* description, ...)
{
  paramLine.push_back(new ParameterLineChar(name, description, value));
  return true;
}
bool ParameterCollection::getArgument(const char* name, int* value,
                                      const char* description, ...)
{
  paramLine.push_back(new ParameterLineInt(name, description, value));
  return true;
}
bool ParameterCollection::getArgument(const char* name, unsigned int* value,
                                      const char* description, ...)
{
  paramLine.push_back(new ParameterLineUnsignedInt(name, description, value));
  return true;
}
bool ParameterCollection::getArgument(const char* name, unsigned long* value,
                                      const char* description, ...)
{
  paramLine.push_back(new ParameterLineUnsignedLong(name, description, value));
  return true;
}
bool ParameterCollection::getArgument(const char* name, unsigned long long* value,
                                      const char* description, ...)
{
  paramLine.push_back(new ParameterLineUnsignedLongLong(name, description, value));
  return true;
}
bool ParameterCollection::getArgument(const char* name, bool* value,
                                      const char* description, ...)
{
  paramLine.push_back(new ParameterLineBool(name, description, value));
  return true;
}
bool ParameterCollection::getArgument(const char* name, std::string* value,
                                      const char* description, ...)
{
  paramLine.push_back(new ParameterLineString(name, description, value));
  return true;
}
bool ParameterCollection::getArgument(const char* name, double* value,
                                      const char* description, ...)
{
  paramLine.push_back(new ParameterLineDouble(name, description, value));
  return true;
}
bool ParameterCollection::getArgument(const char* name, float* value,
                                      const char* description, ...)
{
  paramLine.push_back(new ParameterLineFloat(name, description, value));
  return true;
}
size_t ParameterCollection::size() const
{
  return paramLine.size();
}
ParameterLine* ParameterCollection::getEntry(size_t index)
{
  return paramLine[index];
}








void* CmdLineWidget::threadFunc(void* arg)
{
  ParameterCollection* collection = (ParameterCollection*) arg;
  CmdLineWidget* gui = new CmdLineWidget(collection);
  gui->show();
  return gui;
}

int CmdLineWidget::create(ParameterCollection* collection)
{
  return RcsGuiFactory_requestGUI(threadFunc, (void*) collection);
}

bool CmdLineWidget::destroy(int handle)
{
  return RcsGuiFactory_destroyGUI(handle);
}


CmdLineWidget::CmdLineWidget(ParameterCollection* collection, QWidget* parent) :
  QScrollArea(parent)
{
  setWindowTitle("CmdLineWidget");

  QVBoxLayout* gridLayout = new QVBoxLayout();
  QWidget* scrollWidget = new QWidget(this);
  scrollWidget->setLayout(gridLayout);
  this->setWidget(scrollWidget);

  for (size_t i=0; i<collection->size(); ++i)
  {
    gridLayout->addWidget(new CmdLineEntry(collection->getEntry(i)));
  }

  this->setWidgetResizable(true);
}


CmdLineEntry::CmdLineEntry(ParameterLine* line_) : line(line_)
{
  QHBoxLayout* hbox = new QHBoxLayout(this);

  QLabel* label = new QLabel(QString::fromStdString(line->name));
  hbox->addWidget(label);

  this->textParam = new QLineEdit(this);
  textParam->setText(QString::fromStdString(line->getParamAsString()));
  connect(textParam, SIGNAL(returnPressed()), this, SLOT(handleText()));
  hbox->addWidget(textParam);

}

void CmdLineEntry::handleText()
{
  line->setParam(textParam->text().toStdString());
}


}   // namespace Rcs

/*******************************************************************************

  Copyright (c) by Honda Research Institute Europe GmbH

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

#ifndef RCS_CMDLINEWIDGET_H
#define RCS_CMDLINEWIDGET_H

#include "AsyncWidget.h"

#include <Rcs_cmdLine.h>

#include <QScrollArea>
#include <QString>
#include <QStringList>
#include <QLineEdit>
#include <QLabel>
#include <QTimer>
#include <QDebug>

#include <vector>
#include <cstddef>

namespace Rcs
{

class ParameterCollection : public CmdLineParser
{
public:
  class Entry
  {
  public:
    virtual ~Entry()
    {
    }
    virtual void setParam(std::string paramString) = 0;
    virtual std::string getParamAsString() const = 0;

    virtual std::string getName() const = 0;
    virtual std::string getDescription() const = 0;
  };


  // String types
  bool getArgument(const char* name, char* value,
                   const char* description=NULL, ...);
  bool getArgument(const char* name, std::string* value,
                   const char* description=NULL, ...);

  // Integer types
  bool getArgument(const char* name, int* value,
                   const char* description=NULL, ...);
  bool getArgument(const char* name, unsigned int* value,
                   const char* description=NULL, ...);
  bool getArgument(const char* name, unsigned long* value,
                   const char* description=NULL, ...);
  bool getArgument(const char* name, unsigned long long* value,
                   const char* description=NULL, ...);
  bool getArgument(const char* name, bool* value,
                   const char* description=NULL, ...);

  // Floating point types
  bool getArgument(const char* tag, double* result,
                   const char* description=NULL, ...);
  bool getArgument(const char* tag, float* result,
                   const char* description=NULL, ...);

  bool hasArgument(const char* tag, const char* description=NULL, ...);

  size_t size() const;
  Entry* getEntry(size_t index);
  void clear();
  void sort();

protected:
  std::vector<Entry*> paramLine;
};



class CmdLineGui : public Rcs::AsyncWidget
{
public:
  CmdLineGui(ParameterCollection* collection,
             std::string title=std::string());

  void construct();

protected:
  ParameterCollection* collection;
  std::string title;
};

class CmdLineWidget : public QScrollArea
{
  Q_OBJECT

public:

  CmdLineWidget(ParameterCollection* collection,
                std::string title=std::string(),
                QWidget* parent=NULL);

  virtual ~CmdLineWidget();

};

class CmdLineEntry : public QWidget
{
  Q_OBJECT

public:
  CmdLineEntry(ParameterCollection::Entry* line);

public slots:
  void handleText();

protected:
  QLineEdit* textParam;
  ParameterCollection::Entry* line;
};

class TextWidget : public QScrollArea
{
  Q_OBJECT
public:
  TextWidget(std::string text, std::string title=std::string())
  {
    QFont font = QFont("Courier");
    font.setStyleHint(QFont::Monospace);
    font.setPointSize(8);
    font.setFixedPitch(true);

    label = new QLabel(QString::fromStdString(text), this);
    label->setFont(font);
    label->setWordWrap(true);
    //label->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
    label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    label->setTextInteractionFlags(Qt::TextSelectableByMouse);

    setWidget(label);
    setWidgetResizable(true);
    setWindowTitle(QString::fromStdString(title));
    setObjectName("Rcs::TextWidget");
    //https://www.appsloveworld.com/cplus/100/39/setting-text-on-a-qlabel-in-a-layout-doesnt-resize?utm_content=cmp-true
    //layout->setSizeConstraint(QLayout::SetMinimumSize);

    QTimer::singleShot(1000, this, SLOT(onResizeToFit()));
  }

  virtual ~TextWidget()
  {
    printf("Destroying TextWidget\n");
  }
  QLabel* label;

public slots:

  void onResizeToFit()
  {
    return;
    qDebug() << "Adjusting size";
    resize(label->sizeHint());
    label->adjustSize();
    //label->updateGeometry();
    resize(label->sizeHint());
  }
};

class TextGui : public Rcs::AsyncWidget
{
public:
  TextGui(std::string text_, std::string title_=std::string()) :
    AsyncWidget(), text(text_), title(title_)
  {
    launch();
  }

  virtual ~TextGui()
  {
    printf("Destroying TextGui\n");
  }

  void construct()
  {
    setWidget(new TextWidget(text, title));
  }

protected:
  std::string text;
  std::string title;
};

}
#endif   // RCS_CMDLINEWIDGET_H

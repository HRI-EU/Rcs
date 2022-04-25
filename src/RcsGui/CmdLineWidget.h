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

#include <QScrollArea>
#include <QString>
#include <QStringList>
#include <QLineEdit>

#include <string>
#include <vector>
#include <cstddef>

namespace Rcs
{

class ParameterLine
{
public:
  ParameterLine(std::string name, std::string description);
  virtual ~ParameterLine();
  virtual void setParam(std::string paramString) = 0;
  virtual std::string getParamAsString() const = 0;

  std::string name;
  std::string description;
};

class ParameterCollection
{
public:
  bool getArgument(const char* name, char* value, const char* description);
  bool getArgument(const char* name, int* value, const char* description);
  bool getArgument(const char* name, std::string* value, const char* description);
  size_t size() const;
  ParameterLine* getEntry(size_t index);

protected:
  std::vector<ParameterLine*> paramLine;
};

class CmdLineWidget : public QScrollArea
{
  Q_OBJECT

public:
  static int create(ParameterCollection* collection);
  static bool destroy(int handle);


private:
  CmdLineWidget(ParameterCollection* collection);
  static void* threadFunc(void* arg);
};

class CmdLineEntry : public QWidget
{
  Q_OBJECT

public:
  CmdLineEntry(ParameterLine* line);

public slots:
  void handleText();

protected:
  QLineEdit* textParam;
  ParameterLine* line;
};

}
#endif   // RCS_CMDLINEWIDGET_H

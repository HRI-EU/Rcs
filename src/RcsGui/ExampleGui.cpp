/*******************************************************************************

  Copyright (c) 2022, Honda Research Institute Europe GmbH

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

#include "ExampleGui.h"

#include "ExampleFactory.h"
#include "Rcs_guiFactory.h"

#include <Rcs_macros.h>
#include <Rcs_utilsCPP.h>
#include <Rcs_timer.h>

#include <QPushButton>
#include <QVBoxLayout>

#include <QSplitter>
#include <QTreeView>
#include <QListView>
#include <QTableView>
#include <QPushButton>
#include <QHeaderView>
#include <QTimer>

#include <sstream>



namespace Rcs
{

/*******************************************************************************
 *
 ******************************************************************************/
class LogLine : public ParameterCollection::Entry
{
public:

  void setParam(std::string paramString)
  {
    RcsLogLevel = atoi(paramString.c_str());
  }

  virtual std::string getParamAsString() const
  {
    std::stringstream  sstream;
    sstream << RcsLogLevel;
    return sstream.str();
  }

  std::string getName() const
  {
    return "Log level";
  }

  std::string getDescription() const
  {
    return "";
  }

};










/*******************************************************************************
 *
 ******************************************************************************/
ExampleGui::ExampleGui(int argc_, char** argv_) :
  AsyncWidget(), argc(argc_), argv(argv_)
{
  RLOG(5, "Before launch");
  launch();
  RLOG(5, "After launch");
}

void ExampleGui::construct()
{
  RLOG(5, "Constructing test");
  QWidget* test = new ExampleWidget(argc, argv);
  RLOG(5, "Setting widget");
  setWidget(test);
  RLOG(5, "Done");
}

class ExampleWorker : public QObject
{
public:
  ExampleWorker(ExampleBase* example_): example(example_)
  {
    RLOG(5, "ExampleWorker created");
  }

  ~ExampleWorker()
  {
    RLOG(5, "ExampleWorker deleted");
  }

  void doWork()
  {
    RLOG(5, "doWork() started");
    example->start();
    RLOG(5, "doWork() finished");
  }

  ExampleBase* example;
};






/*******************************************************************************
 *
 ******************************************************************************/
ExampleItem::ExampleItem(int argc_, char** argv_,
                         const QString& categoryName_,
                         const QString& exampleName) :
  QStandardItem(exampleName),
  parseItem(NULL),
  helpItem(NULL),
  categoryName(categoryName_),
  example(NULL),
  argc(argc_),
  argv(argv_),
  clicked(false),
  parseWindow(NULL),
  helpWindow(NULL)
{
}

ExampleItem::~ExampleItem()
{
  RLOG_CPP(5, "Deleting ExampleItem " << categoryName.toStdString()
           << "::" << text().toStdString());

  // Cleaníng up will properly shut down the running thread and delete
  // the example.
  if (example)
  {
    destroy();
  }
}

void ExampleItem::start()
{
  if (exampleThread.isRunning())
  {
    RLOG(5, "Example is already running");
    return;
  }

  if (!example)
  {
    example = ExampleFactory::create(categoryName.toStdString(),
                                     text().toStdString(), argc, argv);
    RCHECK(example);
  }

  if (parseItem->checkState()==Qt::Checked)
  {
    example->initAlgo();
    example->initGraphics();
    example->initGuis();
  }
  else
  {
    example->init(argc, argv);
  }

  ExampleWorker* worker = new ExampleWorker(example);
  worker->moveToThread(&exampleThread);
  connect(&exampleThread, &QThread::finished, worker, &QObject::deleteLater);
  connect(this, &ExampleItem::startWork, worker, &ExampleWorker::doWork);
  exampleThread.start();
  emit startWork();
}

void ExampleItem::launchHelpWindow()
{
  if (helpWindow)
  {
    RLOG(1, "Help window already launched");
    return;
  }

  RLOG(5, "Launching help window");
  QString title = categoryName + QString(" - ") + text();

  if (!example)
  {
    Rcs::ExampleBase* tmp = ExampleFactory::create(categoryName.toStdString(),
                                                   text().toStdString(),
                                                   argc, argv);
    RCHECK(tmp);
    CmdLineParser argP;
    tmp->initParameters();
    tmp->parseArgs(&argP);
    tmp->initAlgo();
    helpWindow = new TextGui(tmp->help(), title.toStdString());
    delete tmp;
  }
  else
  {
    helpWindow = new TextGui(example->help(), title.toStdString());
  }
  connect(helpWindow->getWidget(), SIGNAL(destroyed(QObject*)), this,
          SLOT(onCloseHelpWindow(QObject*)));
}

// This is what happens here: Before calling this function, the check state of
// the QStandardItem is set to Qt::Unchecked. We call the helpWindow's
// destructor which in turn calls unlaunch() which in turn destroys the QWidget
// of the helpWindow. This triggres the slot onCloseWindow to be called. In
// this callback, we return if the QStandardItem is unchecked, since this means
// the destruction is already ongoing. Otherwise we would get double memory
// frees and crashes.
// The onCloseHelpWindow() will explicitely delete the helpWindow if the
// QStandardItem is checked, because this means that the window has been closed
// by pressing the "close" (x) icon on the window. Then we need to delete the
// class instance inside the callback.
// The same strategy applies to the parser window.
void ExampleItem::closeHelpWindow()
{
  if (helpWindow)
  {
    delete helpWindow;
    helpWindow = NULL;
  }
}

// Gets called when the window is closed
void ExampleItem::onCloseHelpWindow(QObject* obj)
{
  if (helpItem->checkState()==Qt::Checked)
  {
    helpItem->setCheckState(Qt::CheckState::Unchecked);
    closeHelpWindow();
  }
}

void ExampleItem::closeParseWindow()
{
  if (parseWindow)
  {
    delete parseWindow;
    parseWindow = NULL;
  }
}

// Gets called when the window is closed
void ExampleItem::onCloseParseWindow(QObject* obj)
{
  if (parseItem->checkState()==Qt::CheckState::Checked)
  {
    parseItem->setCheckState(Qt::CheckState::Unchecked);
    closeParseWindow();
  }
}

void ExampleItem::launchParseWindow()
{
  if (parseWindow)
  {
    RLOG(1, "Parser gui already launched");
    return;
  }

  if (!example)
  {
    example = ExampleFactory::create(categoryName.toStdString(),
                                     text().toStdString(), argc, argv);
    RCHECK(example);
  }

  if (pc.size() == 0)
  {
    CmdLineParser argP;
    example->initParameters();
    example->parseArgs(&pc);
    example->parseArgs(&argP);
    pc.sort();   // Sort alphabetically
  }

  QString title = categoryName + QString(" - ") + text();
  parseWindow = new CmdLineGui(&pc, title.toStdString());
  connect(parseWindow->getWidget(), SIGNAL(destroyed(QObject*)), this,
          SLOT(onCloseParseWindow(QObject*)));
}

void ExampleItem::stop()
{
  if (!exampleThread.isRunning())
  {
    RLOG(5, "Example is already stopped");
    return;
  }
  RLOG(5, "Stopping example");
  example->stop();
  exampleThread.quit();
  exampleThread.wait();
  RLOG(5, "... stopped");
  if (parseWindow)
  {
    parseWindow->unlaunch();
  }
  if (helpWindow)
  {
    helpWindow->unlaunch();
  }
  // parseWindow->waitUntilWidgetDeleted();
  // helpWindow->waitUntilWidgetDeleted();
}

void ExampleItem::destroy()
{
  stop();
  RLOG(5, "Deleting example");
  delete example;
  pc.clear();
  example = NULL;
}

void ExampleItem::setParseItem(QStandardItem* item)
{
  parseItem = item;
}

void ExampleItem::setHelpItem(QStandardItem* item)
{
  helpItem = item;
}

void ExampleItem::setClicked(bool clickState)
{
  clicked = clickState;
}

void ExampleItem::toggleClicked()
{
  clicked = !clicked;
}

bool ExampleItem::isClicked() const
{
  return clicked;
}



/*******************************************************************************
 *
 ******************************************************************************/
ExampleWidget::ExampleWidget(int argc, char** argv, QWidget* parent) :
  QMainWindow(parent)
{
  setObjectName("Rcs::ExampleGui");

  // This widget's layout is a vertical box which holds the title box on top,
  // and the tree view with all ExampleItems.
  QWidget* mainWidget = new QWidget(this);
  QScrollArea* scrollArea = new QScrollArea();
  scrollArea->setWidget(mainWidget);
  scrollArea->setWidgetResizable(true);
  setCentralWidget(scrollArea);
  QVBoxLayout* vBox = new QVBoxLayout();
  vBox->setMargin(0);
  vBox->setSpacing(0);

  // On top of the widget, we display an edit field that allows to set the
  // log level, and to the right of it, a help button that opens a text window
  // showing some generic help information such as resource paths etc. We
  // put these elements into a horizontal box layout (titleBox).
  // Rcs log level as first field
  QHBoxLayout* titleBox = new QHBoxLayout();
  titleBox->setAlignment(Qt::AlignLeft);
  this->logLine = new LogLine;
  titleBox->addWidget(new CmdLineEntry(logLine));

  QPushButton* helpButton = new QPushButton("Help");
  connect(helpButton, SIGNAL(clicked()), this, SLOT(helpClicked()));
  titleBox->addWidget(helpButton);

  QSpacerItem* spacer = new QSpacerItem(0, 0, QSizePolicy::Expanding, QSizePolicy::Minimum);
  titleBox->addItem(spacer);

  vBox->addLayout(titleBox);



  QTreeView* tree = new QTreeView(mainWidget);
  vBox->addWidget(tree);
  mainWidget->setLayout(vBox);
  setCentralWidget(scrollArea);

  std::set<std::string> categories = ExampleFactory::getCategories();

  this->model = new QStandardItemModel();
  QStandardItem* parentItem = model->invisibleRootItem();

  ExampleFactory::ExampleMap cMap = ExampleFactory::constructorMap();

  std::set<std::string>::iterator categories_it = categories.begin();

  while (categories_it != categories.end())
  {
    std::string category_i = *categories_it;
    QStandardItem* item = new QStandardItem(QString::fromStdString(category_i));
    item->setEditable(false);
    parentItem->appendRow(item);

    int i=0;
    ExampleFactory::ExampleMap::iterator it = cMap.begin();
    while (it != cMap.end())
    {
      if (it->first.first==category_i)
      {
        QString categoryName = QString::fromStdString(it->first.first);
        QString exampleName = QString::fromStdString(it->first.second);
        ExampleItem* exItem = new ExampleItem(argc, argv, categoryName,
                                              exampleName);
        exItem->setEditable(false);
        //exItem->setSelectable(false);
        item->setChild(i, 0, exItem);

        QStandardItem* description = new QStandardItem("");
        description->setEditable(false);
        description->setCheckable(true);
        description->setSelectable(false);
        exItem->setParseItem(description);
        item->setChild(i, 1, description);

        QStandardItem* helpMe = new QStandardItem("");
        helpMe->setEditable(false);
        helpMe->setCheckable(true);
        helpMe->setSelectable(false);
        exItem->setHelpItem(helpMe);
        //QIcon helpIcon;
        //helpIcon.addPixmap(style()->standardPixmap(QStyle::SP_TitleBarContextHelpButton),
        //                    QIcon::Normal, QIcon::Off);
        //helpMe->setIcon(helpIcon);
        item->setChild(i, 2, helpMe);
        i++;
      }
      it++;
    }

    categories_it++;
  }

  model->setHorizontalHeaderItem(0, new QStandardItem("Example"));
  model->setHorizontalHeaderItem(1, new QStandardItem("Parse"));
  model->setHorizontalHeaderItem(2, new QStandardItem("Help"));

  tree->setModel(model);
  tree->expandAll();
  tree->setSelectionMode(QAbstractItemView::MultiSelection);

  // Headers are resized so that all text is visible
  tree->header()->resizeSections(QHeaderView::ResizeToContents);

  mainWidget->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  setMinimumWidth(350);
  const int wHeight = std::min(900, (int)(cMap.size()+categories.size())*25+25);
  setMinimumHeight(wHeight);

  connect(tree,SIGNAL(clicked(const QModelIndex&)), this,
          SLOT(itemClicked(const QModelIndex&)));
  // QTimer::singleShot(100, this, SLOT(onResizeToFit()));
}

ExampleWidget::~ExampleWidget()
{
  RLOG(5, "Deleting ExampleWidget");
  delete this->model;
  delete this->logLine;
}

void ExampleWidget::itemClicked(const QModelIndex& idx)
{
  // Return if click happened somewhere out of the current example item
  if ((idx.parent().row() == -1) && (idx.parent().column() == -1))
  {
    return;
  }

  RLOG(5, "row: %d   col: %d", idx.row(), idx.column());
  QStandardItem* item = model->itemFromIndex(idx);

  if (!item)
  {
    RLOG(4, "Failed to get QStandardItem from model: row: %d   col: %d",
         idx.row(), idx.column());
    return;
  }
  ExampleItem* ei = dynamic_cast<ExampleItem*>(item);

  if (ei)
  {
    ei->toggleClicked();
    RLOG(5, "Item: %d %d -> %s (%s)", idx.parent().row(), idx.row(),
         item->text().toStdString().c_str(),
         ei->isClicked() ? "Clicked" : "Not clicked");

    if (ei->isClicked())
    {
      ei->start();
    }
    else
    {
      //ei->stop();
      ei->destroy();
    }

  }
  // Parse has been clicked. The ExampleItem is located one to the left.
  else if (idx.column()==1)
  {
    QModelIndex eidx = idx.sibling(idx.row(), idx.column()-1);
    ei = dynamic_cast<ExampleItem*>(model->itemFromIndex(eidx));
    RCHECK(ei);
    Qt::CheckState cs = item->checkState();

    switch (cs)
    {
      case Qt::Checked:
        RLOG_CPP(5, "Checked - launching parser window");
        ei->launchParseWindow();
        break;

      case Qt::Unchecked:
        RLOG_CPP(5, "Unchecked - deleting parser window");
        ei->closeParseWindow();
        break;

      default:
        RLOG_CPP(1, "Unknown checked mode: " << cs);
    }
    RLOG_CPP(5, "Clicked on item with: " << item->text().toStdString());
  }
  // Help has been clicked. The ExampleItem is located two to the left.
  else if (idx.column() == 2)
  {
    QModelIndex eidx = idx.sibling(idx.row(), idx.column() - 2);
    ei = dynamic_cast<ExampleItem*>(model->itemFromIndex(eidx));
    RCHECK(ei);
    Qt::CheckState cs = item->checkState();

    switch (cs)
    {
      case Qt::Checked:
        RLOG_CPP(5, "Checked - launching help window");
        ei->launchHelpWindow();
        break;

      case Qt::Unchecked:
        RLOG_CPP(5, "Unchecked - deleting help window");
        ei->closeHelpWindow();
        break;

      default:
        RLOG_CPP(1, "Unknown checked mode: " << cs);
    }
    RLOG_CPP(5, "Clicked on item with: " << item->text().toStdString());
  }
}

void ExampleWidget::helpClicked()
{
  std::stringstream s;
  s << RcsShape_distanceFunctionsToString();
  s << Rcs::getResourcePaths();
  s << Rcs::CmdLineParser::printToString();
  new Rcs::TextGui(s.str());
}

void ExampleWidget::onResizeToFit()
{
  //resize(minimumSizeHint());
  //resize(sizeHint());
  // adjustSize();
}

}   // namespace Rcs

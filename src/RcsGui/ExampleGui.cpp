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



namespace Rcs
{

ExampleGui::ExampleGui(int argc_, char** argv_) :
  AsyncWidget(), argc(argc_), argv(argv_)
{
  RLOG(0, "Before launch");
  launch();
  RLOG(0, "After launch");
}

void ExampleGui::construct()
{
  RLOG(0, "Constructing test");
  QWidget* test = new ExampleWidget(argc, argv);
  RLOG(0, "Setting widget");
  setWidget(test);
  RLOG(0, "Done");
}

class ExampleWorker : public QObject
{
public:
  ExampleWorker(ExampleBase* example_, int argc_, char** argv_) :
    example(example_), argc(argc_), argv(argv_)
  {
    RLOG(0, "ExampleWorker created");
  }

  ~ExampleWorker()
  {
    RLOG(0, "ExampleWorker deleted");
  }

  void doWork()
  {
    // example->init(argc, argv);
    // RLOG(0, "doWork() started");
    example->start();
    RLOG(0, "doWork() finished");
  }

  ExampleBase* example;
  int argc;
  char** argv;
};






ExampleItem::ExampleItem(int argc_, char** argv_,
                         const QString& categoryName_,
                         const QString& exampleName) :
  QStandardItem(exampleName),
  categoryName(categoryName_),
  example(NULL),
  argc(argc_),
  argv(argv_),
  clicked(false),
  parsingFinished(false),
  timer(NULL),
  gui(NULL),
  helpWin(NULL),
  withArgParser(false)
{
  this->timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), SLOT(timerCallback()));
}

ExampleItem::~ExampleItem()
{
  RLOG_CPP(0, "Deleting ExampleItem " << categoryName.toStdString()
           << "::" << text().toStdString());

  // Cleaníng up will properly shut down the running thread and delete the example.
  if (example)
  {
    destroy();
  }
}

void ExampleItem::timerCallback()
{

  if (gui->getWidget()==NULL && (!parsingFinished))
  {
    example->initAlgo();
    example->initGraphics();
    example->initGuis();
    parsingFinished = true;

    emit startWork();
  }

}

void ExampleItem::start()
{
  RLOG(1, "Initializing example");
  example = ExampleFactory::create(categoryName.toStdString(), text().toStdString(), argc, argv);
  RCHECK(example);

  RLOG(1, "Starting example");
  ExampleWorker* worker = new ExampleWorker(example, argc, argv);
  worker->moveToThread(&exampleThread);
  connect(&exampleThread, &QThread::finished, worker, &QObject::deleteLater);
  connect(this, &ExampleItem::startWork, worker, &ExampleWorker::doWork);
  exampleThread.start();


  if (withArgParser)
  {
    CmdLineParser argP;
    example->initParameters();
    pc.clear();
    example->parseArgs(&pc);
    example->parseArgs(&argP);
    gui = new CmdLineGui(&pc);
    helpWin = new TextGui(example->help());
    timer->start(40);
  }
  else
  {
    example->init(argc, argv);
    emit startWork();
  }

}

void startWork()
{
}

void ExampleItem::stop()
{
  RLOG(1, "Stopping example");
  example->stop();
  exampleThread.quit();
  exampleThread.wait();
  RLOG(1, "... stopped");
  timer->stop();
  parsingFinished = false;
  delete helpWin;
  delete gui;
  helpWin = NULL;
  gui = NULL;
}

void ExampleItem::destroy()
{
  stop();
  RLOG(1, "Deleting example");
  delete example;
  example = NULL;
}


ExampleWidget::ExampleWidget(int argc, char** argv, QWidget* parent) :
  QMainWindow(parent)
{
  QWidget* mainWidget = new QWidget(this);
  QTreeView* tree = new QTreeView(mainWidget);
  QHBoxLayout* mainGrid = new QHBoxLayout();
  setObjectName("Rcs::ExampleGui");

  mainGrid->setMargin(0);
  mainGrid->setSpacing(0);
  mainGrid->addWidget(tree);
  mainWidget->setLayout(mainGrid);
  setCentralWidget(mainWidget);

  std::set<std::string> categories = ExampleFactory::getCategories();

  this->model = new QStandardItemModel();
  QStandardItem* parentItem = model->invisibleRootItem();

  typedef std::map<std::string, ExampleFactory::ExampleMaker> ExampleMap;

  ExampleMap cMap = ExampleFactory::constructorMap();

  std::set<std::string>::iterator categories_it = categories.begin();

  while (categories_it != categories.end())
  {
    std::string category_i = *categories_it;
    QStandardItem* item = new QStandardItem(QString::fromStdString(category_i));
    item->setEditable(false);
    parentItem->appendRow(item);

    int i=0;
    ExampleMap::iterator it = cMap.begin();
    while (it != cMap.end())
    {
      std::vector<std::string> strings = String_split(it->first, "_");
      RCHECK_MSG(strings.size()==2, "Splitted into %zu strings: %s",
                 strings.size(), it->first.c_str());

      if (strings[0]==category_i)
      {
        ExampleItem* child = new ExampleItem(argc, argv,
                                             QString::fromStdString(strings[0]),
                                             QString::fromStdString(strings[1]));
        child->setEditable(false);
        //child->setSelectable(false);
        item->setChild(i, 0, child);


        QStandardItem* description = new QStandardItem("");
        description->setEditable(false);
        description->setCheckable(true);
        item->setChild(i, 1, description);

        i++;
      }
      it++;
    }

    categories_it++;
  }

  model->setHorizontalHeaderItem(0, new QStandardItem("Example"));
  model->setHorizontalHeaderItem(1, new QStandardItem("Parse"));

  tree->setModel(model);
  tree->expandAll();
  tree->setSelectionMode(QAbstractItemView::MultiSelection);

  // Headers are resized so that all text is visible
  tree->header()->resizeSections(QHeaderView::ResizeToContents);

  setMinimumWidth(400);
  setMinimumHeight((cMap.size()+categories.size())*25 + 25);

  connect(tree,SIGNAL(clicked(const QModelIndex&)), this,
          SLOT(itemClicked(const QModelIndex&)));
}

ExampleWidget::~ExampleWidget()
{
  RLOG(0, "Deleting ExampleWidget");
  delete this->model;
}

void ExampleWidget::itemClicked(const QModelIndex& idx)
{
  if ((idx.parent().row() == -1) && (idx.parent().column() == -1))
  {
    return;
  }

  QStandardItem* item = model->itemFromIndex(idx);
  RLOG(0, "row: %d   col: %d", idx.row(), idx.column());

  ExampleItem* ei = dynamic_cast<ExampleItem*>(item);

  if (ei)
  {
    ei->clicked = !ei->clicked;
    RLOG(1, "Item: %d %d -> %s (%s)", idx.parent().row(), idx.row(),
         item->text().toStdString().c_str(),
         ei->clicked ? "Clicked" : "Not clicked");

    if (ei->clicked)
    {
      ei->start();
    }
    else
    {
      ei->stop();
      ei->destroy();
    }

  }
  else if (item)
  {
    QModelIndex eidx = idx.sibling(idx.row(), idx.column()-1);
    ExampleItem* sib = dynamic_cast<ExampleItem*>(model->itemFromIndex(eidx));
    RCHECK(sib);
    Qt::CheckState cs = item->checkState();

    switch (cs)
    {
      case Qt::Checked:
        RLOG_CPP(1, "Checked");
        sib->withArgParser = true;
        break;

      case Qt::Unchecked:
        RLOG_CPP(1, "Unchecked");
        sib->withArgParser = false;
        break;

      default:
        RLOG_CPP(1, "Unknown checked mode: " << cs);
    }
    RLOG_CPP(0, "Clicked on item with: " << item->text().toStdString());
  }
}


}   // namespace Rcs

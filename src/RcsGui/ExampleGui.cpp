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

#include <QPushButton>
#include <QVBoxLayout>

#include <QSplitter>
#include <QTreeView>
#include <QListView>
#include <QTableView>
#include <QPushButton>
#include <QHeaderView>


namespace Rcs
{

ExampleGui::ExampleGui(int argc_, char** argv_) : argc(argc_), argv(argv_)
{
  RLOG(0, "Before launch");
  launch();
  RLOG(0, "After launch");
}

void ExampleGui::construct()
{
  RLOG(0, "Constructing test");
  QWidget* test = new TreeTest(argc, argv);
  RLOG(0, "Setting widget");
  setWidget(test);
  RLOG(0, "Done");
}


class ExampleWorker : public QObject
{
public:
  ExampleWorker(ExampleBase* example_, int argc_, char** argv_) : example(example_), argc(argc_), argv(argv_)
  {
    RLOG(0, "ExampleWorker created");
  }

  ~ExampleWorker()
  {
    RLOG(0, "ExampleWorker deleted");
  }

  void doWork()
  {
    example->init(argc, argv);
    RLOG(0, "doWork() started");
    example->start();
    RLOG(0, "doWork() finished");
  }

  ExampleBase* example;
  int argc;
  char** argv;
};






ExampleItem::ExampleItem(int argc_, char** argv_, const QString& categoryName_, const QString& exampleName) :
  QStandardItem(exampleName), categoryName(categoryName_), example(NULL), argc(argc_), argv(argv_), clicked(false)
{
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
  emit startWork();
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
}

void ExampleItem::destroy()
{
  stop();
  RLOG(1, "Deleting example");
  delete example;
  example = NULL;
}


TreeTest::TreeTest(int argc, char** argv, QWidget* parent) : QMainWindow(parent)
{
  QWidget* mainWidget = new QWidget(this);
  QTreeView* tree = new QTreeView(mainWidget);
  QVBoxLayout* mainGrid = new QVBoxLayout();
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

        ExampleItem* child = new ExampleItem(argc, argv, QString::fromStdString(strings[0]), QString::fromStdString(strings[1]));
        child->setEditable(false);
        item->setChild(i++, 0, child);
      }
      it++;
    }

    categories_it++;
  }

  model->setHorizontalHeaderItem(0, new QStandardItem("Example"));
  model->setHorizontalHeaderItem(1, new QStandardItem("Description"));

  tree->setModel(model);
  tree->expandAll();
  tree->setSelectionMode(QAbstractItemView::MultiSelection);

  // Headers are resized so that all text is visible
  tree->header()->resizeSections(QHeaderView::ResizeToContents);


  setFixedWidth(400);
  setFixedHeight(cMap.size()*25);

  connect(tree,SIGNAL(clicked(const QModelIndex&)), this, SLOT(itemClicked(const QModelIndex&)));
}

TreeTest::~TreeTest()
{
  RLOG(0, "Deleting TreeTest");
  delete this->model;
}

void TreeTest::itemClicked(const QModelIndex& idx)
{
  if ((idx.parent().row() == -1) && (idx.parent().column() == -1))
  {
    return;
  }

  QStandardItem* item = model->itemFromIndex(idx);

  ExampleItem* ei = dynamic_cast<ExampleItem*>(item);

  if (ei)
  {
    ei->clicked = !ei->clicked;
    RLOG(1, "Item: %d %d -> %s (%s)", idx.parent().row(), idx.row(), item->text().toStdString().c_str(),
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
  else
  {
    RFATAL("This should never happen");
  }
}
typedef struct
{
  void* ptr[10];
} VoidPointerList;

static void* threadFunc2(void* arg)
{
  VoidPointerList* p = (VoidPointerList*) arg;
  int* argc = (int*) p->ptr[0];
  char** argv = (char**) p->ptr[1];

  TreeTest* w = new TreeTest(*argc, argv);
  w->setWindowTitle("Examples");
  w->show();

  delete argc;
  delete p;

  return w;
}

int TreeTest::create(int argc_, char** argv_)
{
  VoidPointerList* p = new VoidPointerList;

  double* argc = new double;
  *argc = argc_;

  p->ptr[0] = (void*) argc;
  p->ptr[1] = (void*) argv_;

  int handle = RcsGuiFactory_requestGUI(threadFunc2, p);

  return handle;
}

bool TreeTest::destroy(int handle)
{
  return RcsGuiFactory_destroyGUI(handle);
}





#if 0

static void* threadFunc(void* arg)
{
  VoidPointerList* p = (VoidPointerList*) arg;
  int* argc = (int*) p->ptr[0];
  char** argv = (char**) p->ptr[1];

  ExampleGui* w = new ExampleGui(*argc, argv);
  w->setWindowTitle("Examples");
  w->show();

  delete argc;
  delete p;

  return w;
}

int ExampleGui::create(int argc_, char** argv_)
{
  VoidPointerList* p = new VoidPointerList;

  double* argc = new double;
  *argc = argc_;

  p->ptr[0] = (void*) argc;
  p->ptr[1] = (void*) argv_;

  int handle = RcsGuiFactory_requestGUI(threadFunc, p);

  return handle;
}

bool ExampleGui::destroy(int handle)
{
  return RcsGuiFactory_destroyGUI(handle);
}

ExampleGui::ExampleGui(int argc, char** argv)
{
  QScrollArea* scrollWidget = new QScrollArea(this);
  QVBoxLayout* vbox = new QVBoxLayout(scrollWidget);
  setCentralWidget(scrollWidget);
  scrollWidget->setLayout(vbox);

  typedef std::map<std::string, ExampleFactory::ExampleMaker> ExampleMap;

  ExampleMap cMap = ExampleFactory::constructorMap();
  ExampleMap::iterator it = cMap.begin();

  while (it != cMap.end())
  {
    vbox->addWidget(new ExampleWidget("My Category", it->first, argc, argv));
    it++;
  }

}

ExampleGui::~ExampleGui()
{
}

#endif



}   // namespace Rcs

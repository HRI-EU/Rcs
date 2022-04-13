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

#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QScrollArea>

#include <QSplitter>
#include <QTreeView>
#include <QListView>
#include <QTableView>
#include <QPushButton>


namespace Rcs
{

typedef struct
{
  void* ptr[10];
} VoidPointerList;



class ExampleItem : public QStandardItem
{
public:
  ExampleItem(int argc_, char** argv_, const QString& text) :
    QStandardItem(text), example(NULL), argc(argc_), argv(argv_), clicked(false)
  {
  }

  void init()
  {
    RLOG(0, "Initializing example");
    example = ExampleFactory::create(text().toStdString(), argc, argv);
  }

  static void* runThreadFunc(void* arg)
  {
    ExampleBase* example = (ExampleBase*) arg;
    example->start();
    return NULL;
  }

  void start()
  {
    RLOG(0, "Starting example");
    example->init(argc, argv);
    pthread_create(&runThread, NULL, &runThreadFunc, example);
  }

  void stop()
  {
    RLOG(0, "Stopping example");
    example->stop();
    pthread_join(runThread, NULL);
    RLOG(0, "... stopped");
  }

  void destroy()
  {
    // Calling pthread_join on a joined thread will immediately return
    stop();
    RLOG(0, "Deleting example");
    delete example;
    example = NULL;
  }

  ExampleBase* example;
  pthread_t runThread;
  int argc;
  char** argv;
  bool clicked;
};


TreeTest::TreeTest(int argc, char** argv, QWidget* parent) : QMainWindow(parent)
{
  QWidget* mainWidget = new QWidget(this);
  QTreeView* tree = new QTreeView(mainWidget);
  QVBoxLayout* mainGrid = new QVBoxLayout();

  mainGrid->setMargin(0);
  mainGrid->setSpacing(0);
  mainGrid->addWidget(tree);
  mainWidget->setLayout(mainGrid);

  const int numCols = 1;

  std::vector<std::string> categories;
  categories.push_back("Rcs");
  categories.push_back("Biomechanics");
  categories.push_back("Math");

  this->model = new QStandardItemModel();
  QStandardItem* parentItem = model->invisibleRootItem();

  typedef std::map<std::string, ExampleFactory::ExampleMaker> ExampleMap;

  ExampleMap cMap = ExampleFactory::constructorMap();


  for (size_t r = 0; r < categories.size(); r++)
  {
    QStandardItem* item = new QStandardItem(QString::fromStdString(categories[r]));
    item->setEditable(false);
    parentItem->appendRow(item);

    for (int c = 0; c < numCols; c++)
    {

      if (r==0)
      {
        int i=0;
        ExampleMap::iterator it = cMap.begin();
        while (it != cMap.end())
        {
          ExampleItem* child = new ExampleItem(argc, argv, QString::fromStdString(it->first));
          child->setEditable(false);
          item->setChild(i++, c, child);
          RLOG(0, "%s", it->first.c_str());
          it++;
        }
      }
    }
  }

  model->setHorizontalHeaderItem(0, new QStandardItem("Example"));
  model->setHorizontalHeaderItem(1, new QStandardItem("Description"));

  tree->setModel(model);
  tree->expandAll();
  tree->setSelectionMode(QAbstractItemView::MultiSelection);

  setCentralWidget(mainWidget);

  connect(tree,SIGNAL(clicked(const QModelIndex&)), this, SLOT(itemClicked(const QModelIndex&)));
}

void TreeTest::itemClicked(const QModelIndex& idx)
{

  if (idx.parent().row()==-1 && idx.parent().column()==-1)
  {
    return;
  }

  QStandardItem* item = model->itemFromIndex(idx);

  ExampleItem* ei = dynamic_cast<ExampleItem*>(item);

  if (ei)
  {
    ei->clicked = !ei->clicked;
    RLOG(0, "Item: %d %d -> %s (%s)", idx.parent().row(), idx.row(), item->text().toStdString().c_str(),
         ei->clicked ? "Clicked" : "Not clicked");


    if (ei->clicked)
    {
      ei->init();
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
    vbox->addWidget(new ExampleWidget(it->first, argc, argv));
    it++;
  }

}

ExampleGui::~ExampleGui()
{
}




}   // namespace Rcs

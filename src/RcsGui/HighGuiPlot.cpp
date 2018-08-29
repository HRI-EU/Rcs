/*******************************************************************************

  Copyright (c) 2017, Honda Research Institute Europe GmbH.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

  3. All advertising materials mentioning features or use of this software
     must display the following acknowledgement: This product includes
     software developed by the Honda Research Institute Europe GmbH.

  4. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#include "HighGuiPlot.h"
#include "Rcs_guiFactory.h"

#include <Rcs_macros.h>
#include <Rcs_basicMath.h>

#include <qglobal.h>
#include <qwt_scale_engine.h>
#include <qwt_symbol.h>
#include <qwt_text_label.h>
#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_marker.h>

#include <QHBoxLayout>
#include <QAction>
#include <QContextMenuEvent>
#include <QMenu>
#include <QPushButton>
#include <QStyle>
#include <QVariant>


namespace Rcs
{

// color scheme: http://colorbrewer2.org/?type=qualitative&scheme=Paired&n=12
// reorderd
static const char* colormap[] = {"#e31a1c",
                                 "#33a02c",
                                 "#1f78b4",
                                 "#ff7f00",
                                 "#6a3d9a",
                                 "#b15928",
                                 "#a6cee3",
                                 "#b2df8a",
                                 "#fb9a99",
                                 "#fdbf6f",
                                 "#cab2d6",
                                 "#ffff99"
                                };

static const unsigned int num_colors = 12;

typedef struct
{
  void* ptr[10];
} VoidPointerList;


static void* createHighGuiPlot(void* arg)
{
  VoidPointerList* p = (VoidPointerList*) arg;
  RCHECK(arg);
  std::string name = *((std::string*) p->ptr[0]);

  HighGuiPlot* w = new HighGuiPlot(name);
  return w;
}

/******************************************************************************/

HighGuiPlot::HighGuiPlot(const std::string& name, unsigned int capacity,
                         bool linearize) :
  HighGuiWidget(name),
  _new_value(false), _new_limits(false), _new_min(0.0), _new_max(0.0),
  _new_capacity(0), _capacity(0), _linearize(linearize), _pause(false)
{
  QLayout* main_layout = new QHBoxLayout(this);

  _plot = new QwtPlot(QString::fromStdString(name), this);
  _plot->titleLabel()->setStyleSheet("font: bold 14px");
  _plot->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _plot->axisScaleEngine(QwtPlot::xBottom)
  ->setAttribute(QwtScaleEngine::Inverted);

  setCapacity(capacity);

  main_layout->addWidget(_plot);

  // buttons
  _button_layout = new QVBoxLayout();

  // pause button
  QPushButton* pause_button = new QPushButton(this);
  pause_button->setCheckable(true);
  pause_button->setIcon(pause_button->style()
                        ->standardIcon(QStyle::SP_MediaPause));
  pause_button->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  pause_button->setToolTip("Pause plotting");
  connect(pause_button, SIGNAL(toggled(bool)), SLOT(pause(bool)));
  _button_layout->addWidget(pause_button, 0, Qt::AlignTop);

  _button_layout->addStretch();

  main_layout->addItem(_button_layout);
  main_layout->setMargin(0);
}

HighGuiPlot* HighGuiPlot::create(const std::string& name)
{
  VoidPointerList p;
  p.ptr[0] = (void*) &name;

  int handle = RcsGuiFactory_requestGUI(createHighGuiPlot, &p);
  return (HighGuiPlot*) RcsGuiFactory_getPointer(handle);
}

void HighGuiPlot::addValue(double value, unsigned int value_id)
{
  if (value_id >= MaxValueID)
  {
    RLOGS(0, "Cannot add value, please choose a value_id that is lower than %u",
          MaxValueID);
    return;
  }

  lock();
  _new_value = true;
  _new_value_pairs.push_back(std::make_pair(value_id, value));
  unlock();
}

void HighGuiPlot::addValues(const double* value, unsigned int n)
{
  lock();
  _new_value = true;
  for (unsigned int i = 0; i < n; i++)
  {
    _new_value_pairs.push_back(std::make_pair(MaxValueID + i, value[i]));
  }
  unlock();
}

void HighGuiPlot::increaseCapacity()
{
  setCapacity(_capacity * 2);
}

void HighGuiPlot::decreaseCapacity()
{
  setCapacity(_capacity / 2);
}

void HighGuiPlot::setCapacity(unsigned int capacity)
{
  lock();
  if (capacity == 0 || capacity == _capacity)
  {
    unlock();
    return;
  }

  _new_capacity = capacity;
  unlock();
}

void HighGuiPlot::setLimits(double min, double max)
{
  lock();
  _new_min = min;
  _new_max = max;
  _new_limits = true;
  unlock();
}

std::vector<double> HighGuiPlot::linearizeData(const std::vector<double>& raw,
                                               unsigned int start_index)
{
  unsigned int n = raw.size();
  std::vector<double> linearized(n);

  for (unsigned int i = 0; i < n; i++)
  {
    linearized[i] = raw[(i + start_index) %n];
  }

  return linearized;
}

void HighGuiPlot::contextMenuEvent(QContextMenuEvent* event)
{
  QMenu menu(this);
  QAction* inc_action = menu.addAction("Increase capacity (x2)");
  QAction* dec_action = menu.addAction("Decrease capacity (/2)");
  connect(inc_action, SIGNAL(triggered()), SLOT(increaseCapacity()));
  connect(dec_action, SIGNAL(triggered()), SLOT(decreaseCapacity()));
  menu.exec(event->globalPos());
}


void HighGuiPlot::update()
{
  HighGuiWidget::update();

  lock();
  if (_new_capacity > 0)
  {
    // set new capacity
    _capacity = _new_capacity;
    _new_capacity = 0;

    // resice x-axis and set new labels
    _x.resize(_capacity);
    for (unsigned int i = 0; i < _capacity; i++)
    {
      _x[i] = _capacity - i - 1;
    }

    // update data vectors (everything will be cleared)
    std::map<unsigned int, QwtPlotCurve*>::iterator it;
    for (it = _curves.begin(); it != _curves.end(); ++it)
    {
      _data[it->first].first.resize(_capacity);
      _data[it->first].first.clear();
      _data[it->first].second = 0;
      if (!_linearize)
      {
#if QWT_VERSION < 0x060103
        _curves[it->first]->setRawData(_x.data(), _data[it->first].first.data(),
                                       _capacity);
#else
		  _curves[it->first]->setSamples(_x.data(), _data[it->first].first.data(),
			  _capacity);
#endif
      }
    }
  }

  if (_new_limits)
  {
    _new_limits = false;

    if (Math_isNAN(_new_min) || Math_isNAN(_new_max))
    {
      _plot->setAxisAutoScale(QwtPlot::yLeft);
    }
    else
    {
      _plot->setAxisScale(QwtPlot::yLeft, _new_min, _new_max);
    }
  }

  bool new_value = _new_value;
  unlock();

  if (new_value)
  {
    lock();
    std::list<std::pair<unsigned int, double> > new_value_pairs =
      _new_value_pairs;
    _new_value_pairs.clear();
    _new_value = false;
    unlock();

    while (!new_value_pairs.empty())
    {
      unsigned int value_id = new_value_pairs.front().first;
      double value = new_value_pairs.front().second;
      new_value_pairs.pop_front();

      // if curve does not exist yet create it
      std::map<unsigned int, QwtPlotCurve*>::iterator it;
      it = _curves.find(value_id);
      if (it == _curves.end())
      {
        // create curve
        QwtPlotCurve* curve = new QwtPlotCurve();
        _curves[value_id] = curve;
        QColor curve_color(
          (const char*) colormap[(_curves.size()-1) % num_colors]);
        curve->setRenderHint(QwtPlotItem::RenderAntialiased);
        curve->setPen(curve_color);
        curve->attach(_plot);

        // button for hiding the curve
        QPushButton* curve_button = new QPushButton(this);
        curve_button->setCheckable(true);
        curve_button->setChecked(true);
        curve_button->setProperty("value_id", value_id);
        QPixmap pixmap(10,10);
        pixmap.fill(curve_color);
        curve_button->setIcon(pixmap);
        curve_button->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
        curve_button->setToolTip(QString("Hide curve ") +
                                 QString::number(value_id));
        connect(curve_button, SIGNAL(toggled(bool)), SLOT(attach(bool)));
        _button_layout->insertWidget(_button_layout->count() - 1, curve_button);
        curve_button->show();

        // create data
        _data[value_id].first.resize(_capacity, 0.0);
        _data[value_id].second = 0;

        if (!_linearize)
        {
          // set raw data ptr
#if QWT_VERSION < 0x060103
          _curves[value_id]->setRawData(_x.data(), _data[value_id].first.data(),
                                        _capacity);
#else
		  _curves[value_id]->setSamples(_x.data(), _data[value_id].first.data(),
			  _capacity);
#endif

          // create marker
          QwtPlotMarker* marker = new QwtPlotMarker();
          _markers[value_id] = marker;
          marker->setLineStyle(QwtPlotMarker::VLine);
          QPen marker_pen = QPen(QColor::fromRgb(50, 50, 50));
          marker_pen.setWidth(3);
          marker->setLinePen(marker_pen);
#if QWT_VERSION < 0x060103
          QwtSymbol marker_symbol(QwtSymbol::Ellipse, curve_color, curve_color,
                                  QSize(5,5));
          marker->setSymbol(marker_symbol);
#else
		  QwtSymbol* marker_symbol = new QwtSymbol(QwtSymbol::Ellipse, curve_color, curve_color,
			  QSize(5, 5));
		  marker->setSymbol(marker_symbol);

#endif
          marker->setZ(100.0);

          marker->attach(_plot);
        }
      }

      // add data and replot
      std::pair<std::vector<double>, unsigned int>& _data_ref = _data[value_id];
      _data_ref.first[_data_ref.second] = value;


      if (_linearize)
      {
        std::vector<double> linearized_data = linearizeData(_data_ref.first,
                                                            (_data_ref.second + 1) % _capacity);
#if QWT_VERSION < 0x060103
        _curves[value_id]->setData(_x.data(), linearized_data.data(), _capacity);
#else
		RLOG(1, "Fixme");
#endif
      }
      else
      {
        _markers[value_id]->setValue(_x[_data_ref.second]-0.5,
                                     _data_ref.first[_data_ref.second]);
      }

      _data_ref.second = (_data_ref.second + 1) % _capacity;
    }

    if (!_pause)
    {
      _plot->replot();
    }
  }
}

void HighGuiPlot::pause(bool pause)
{
  _pause = pause;
}

void HighGuiPlot::attach(bool attach)
{
  QObject* sender = QObject::sender();
  if (sender)
  {
    unsigned int value_id = sender->property("value_id").toUInt();

    if (attach)
    {
      _curves[value_id]->attach(_plot);
      _markers[value_id]->attach(_plot);
    }
    else
    {
      _curves[value_id]->detach();
      _markers[value_id]->detach();
    }
  }
}


}

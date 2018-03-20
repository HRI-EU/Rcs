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

#ifndef HIGHGUIPLOT_H
#define HIGHGUIPLOT_H

#include "HighGuiWidget.h"

#include <QVBoxLayout>

#include <map>

class QwtPlot;
class QwtPlotCurve;
class QwtPlotMarker;

namespace Rcs
{

class HighGuiPlot : public HighGuiWidget
{
  Q_OBJECT

public:
  static const bool DefaultLinearizeData = false; //!< Data linearization off by default
  static const unsigned int DefaultCapacity = 200; //!< Default number of data points stored
  static const unsigned int MaxValueID = 100000; //!< The largest plot ID the user is allowed to use


public:
  /*!
   * \brief Constructor
   *
   * \todo: Fast data linearization could be achieved by changing the way how
   *        the data is drawn --> inheriting from QwtPlotCurve or QwtData would
   *        probably allow this
   * \param[in] name Title of the plot
   * \param[in] capacity Number of data points shown in the plot
   * \param[in] linearize Enables data linearization. Disabling data
   *                      linearization is significantly faster, but n the
   *                      newest data is not always at the most right side.
   */
  HighGuiPlot(const std::string& name,
              unsigned int capacity = DefaultCapacity,
              bool linearize = DefaultLinearizeData);

  static HighGuiPlot* create(const std::string& name);

  void addValue(double value, unsigned int value_id);

  void addValues(const double* value, unsigned int n);

public Q_SLOTS:
  void increaseCapacity();
  void decreaseCapacity();
  void setCapacity(unsigned int capacity);
  void setLimits(double min, double max);

protected:
  static std::vector<double> linearizeData(const std::vector<double>& raw,
                                           unsigned int start_index);

  virtual void contextMenuEvent(QContextMenuEvent* event);

protected Q_SLOTS:
  virtual void update();
  virtual void pause(bool pause);
  virtual void attach(bool attach);

protected:
  bool _new_value;
  bool _new_limits;
  double _new_min;
  double _new_max;
  unsigned int _new_capacity;
  std::list<std::pair<unsigned int, double> > _new_value_pairs;

  QwtPlot* _plot;
  std::map<unsigned int, QwtPlotCurve*> _curves;
  std::map<unsigned int, QwtPlotMarker*> _markers;
  std::map<unsigned int, std::pair<std::vector<double>, unsigned int> > _data;
  std::vector<double> _x;
  unsigned int _capacity;
  bool _linearize;
  bool _pause;
  QVBoxLayout* _button_layout;
};

}

#endif   // HIGHGUIPLOT_H

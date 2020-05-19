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

#include "Rcs_filters.h"
#include "Rcs_VecNd.h"
#include "Rcs_macros.h"



/*******************************************************************************
 * Linear ramp with 1 dimension
 ******************************************************************************/
Rcs::Ramp1D::Ramp1D(double vmax, double dt)
{
  init(0.0, vmax, dt);
}

Rcs::Ramp1D::Ramp1D(double x, double vmax, double dt)
{
  init(x, vmax, dt);
}

Rcs::Ramp1D::~Ramp1D()
{
}

void Rcs::Ramp1D::init(double x0, double vmax_, double dt_)
{
  this->r      = x0;
  this->target = x0;
  this->vmax   = vmax_;
  this->dt     = dt_;
}

void Rcs::Ramp1D::init(double x0)
{
  this->r      = x0;
  this->target = x0;
}

void Rcs::Ramp1D::setTarget(double targ)
{
  this->target = targ;
}

void Rcs::Ramp1D::setDt(double dt_)
{
  this->dt = dt_;
}

double Rcs::Ramp1D::getDt() const
{
  return this->dt;
}

double Rcs::Ramp1D::iterate()
{
  double dx_max = vmax*dt;

  if (target > r+dx_max)
  {
    r += dx_max;
  }
  else if (target < r-dx_max)
  {
    r -= dx_max;
  }
  else
  {
    r  = target;
  }

  return r;
}

double Rcs::Ramp1D::getPosition() const
{
  return this->r;
}



/*******************************************************************************
 * Linear ramp with n dimensions
 ******************************************************************************/
Rcs::RampND::RampND(double vmax, double dt, size_t dim_) : dim(dim_)
{
  RCHECK(dim>0);

  filt = new Ramp1D*[dim];

  for (size_t i=0; i<dim; i++)
  {
    filt[i] = new Ramp1D(0.0, vmax, dt);
  }

}

Rcs::RampND::RampND(const double* x, double vmax, double dt, size_t dim_) :
  dim(dim_)
{
  RCHECK(dim>0);

  filt = new Ramp1D*[dim];

  for (size_t i=0; i<dim; i++)
  {
    filt[i] = new Ramp1D(x[i], vmax, dt);
  }

}

Rcs::RampND::~RampND()
{
  for (size_t i=0; i<dim; i++)
  {
    delete (filt[i]);
  }

  delete [] filt;
}

void Rcs::RampND::init(const double* x, double vmax_, double dt_)
{
  for (size_t i=0; i<dim; i++)
  {
    filt[i]->init(x[i], vmax_, dt_);
  }
}

void Rcs::RampND::init(const double* x)
{
  for (size_t i=0; i<dim; i++)
  {
    filt[i]->init(x[i]);
  }
}

void Rcs::RampND::setTarget(const double* targ)
{
  for (size_t i=0; i<dim; i++)
  {
    filt[i]->setTarget(targ[i]);
  }
}

void Rcs::RampND::iterate()
{
  for (size_t i=0; i<dim; i++)
  {
    filt[i]->iterate();
  }
}

void Rcs::RampND::getPosition(double* x) const
{
  for (size_t i=0; i<dim; i++)
  {
    x[i] = filt[i]->getPosition();
  }
}

double Rcs::RampND::getPosition(size_t index) const
{
  RCHECK_MSG(index<dim, "index=%zu   dim=%zu", index, dim);
  return filt[index]->getPosition();
}


/*******************************************************************************
 * Asymptotically damped second order low pass filter with 1 dimension
 ******************************************************************************/
Rcs::SecondOrderLPF1D::SecondOrderLPF1D(double x, double tmc, double dt)
{
  init(x, tmc, dt);
}

Rcs::SecondOrderLPF1D::~SecondOrderLPF1D()
{
}

void Rcs::SecondOrderLPF1D::init(double x0, double tmc_, double dt_)
{
  this->x      = x0;
  this->target = x0;
  this->x_dot  = 0.0;
  this->tmc    = tmc_;
  this->dt     = dt_;
  this->xi     = 1.0;
}

void Rcs::SecondOrderLPF1D::init(double x0)
{
  this->x      = x0;
  this->target = x0;
  this->x_dot  = 0.0;
}

void Rcs::SecondOrderLPF1D::setState(double x_, double x_dot_)
{
  x     = x_;
  x_dot = x_dot_;
}

double Rcs::SecondOrderLPF1D::iterate(double goal, double addedAccel)
{
  double x_ddot = computeAcceleration(goal) + addedAccel;
  x += x_dot * dt + 0.5*x_ddot*dt*dt;
  x_dot += x_ddot * dt;

  return x_ddot;
}

double Rcs::SecondOrderLPF1D::iterate(double goal)
{
  double x_ddot = computeAcceleration(goal);
  x     += x_dot*dt + 0.5*x_ddot*dt*dt;
  x_dot += x_ddot*dt;

  return x_ddot;
}

double Rcs::SecondOrderLPF1D::iterate()
{
  return iterate(target);
}

double Rcs::SecondOrderLPF1D::getPosition() const
{
  return x;
}

double* Rcs::SecondOrderLPF1D::getPositionPtr()
{
  return &x;
}

double Rcs::SecondOrderLPF1D::getVelocity() const
{
  return x_dot;
}

double Rcs::SecondOrderLPF1D::computeAcceleration(double goal) const
{
  return (goal-2.0*tmc*xi*x_dot - x)/(tmc*tmc);
}

void Rcs::SecondOrderLPF1D::setDamping(double damping)
{
  this->xi = damping;
}

void Rcs::SecondOrderLPF1D::setTarget(double targ)
{
  this->target = targ;
}

void Rcs::SecondOrderLPF1D::setDt(double dt_)
{
  this->dt = dt_;
}

double Rcs::SecondOrderLPF1D::getTarget() const
{
  return this->target;
}

double Rcs::SecondOrderLPF1D::getDt() const
{
  return this->dt;
}

double Rcs::SecondOrderLPF1D::getTimeConstant() const
{
  return this->tmc;
}

void Rcs::SecondOrderLPF1D::print() const
{
  fprint(stdout);
}

void Rcs::SecondOrderLPF1D::fprint(FILE* out) const
{
  fprintf(out, "2nd order LPF: tmc=%g dt=%g Target=%g x=%g x_dot=%g\n",
          tmc, dt, target, x, x_dot);
}


/*******************************************************************************
 * Ramp filter with 1 dimension
 ******************************************************************************/
Rcs::RampFilter1D::RampFilter1D(double x0, double tmc, double vmax_, double dt):
  Rcs::SecondOrderLPF1D(x0, tmc, dt), vmax(vmax_), r(x0)
{
  init(x, tmc, dt);
}

Rcs::RampFilter1D::~RampFilter1D()
{
}

void Rcs::RampFilter1D::init(double x0, double tmc_, double dt_)
{
  Rcs::SecondOrderLPF1D::init(x0, tmc_, dt_);
  this->r = x0;
}

void Rcs::RampFilter1D::init(double x0)
{
  Rcs::SecondOrderLPF1D::init(x0);
  this->r = x0;
}

double Rcs::RampFilter1D::getRamp() const
{
  return r;
}

double Rcs::RampFilter1D::iterate()
{
  double dx_max = vmax*dt;

  if (target > r+dx_max)
  {
    r += dx_max;
  }
  else if (target < r-dx_max)
  {
    r -= dx_max;
  }
  else
  {
    r  = target;
  }

  return iterate(r);
}

void Rcs::RampFilter1D::fprint(FILE* out) const
{
  fprintf(out, "\t\tRamp filter: vmax = %g   r = %g\n", vmax, r);
  Rcs::SecondOrderLPF1D::fprint(out);
}



/*******************************************************************************
 * Asymptotically damped second order low pass filter with n dimensions
 ******************************************************************************/
Rcs::SecondOrderLPFND::SecondOrderLPFND(double tmc, double dt, size_t dim_) :
  filt(NULL), dim(dim_)
{
  RCHECK(dim>0);

  filt = new SecondOrderLPF1D*[dim];

  for (size_t i=0; i<dim; i++)
  {
    filt[i] = new SecondOrderLPF1D(0.0, tmc, dt);
  }
}

Rcs::SecondOrderLPFND::SecondOrderLPFND(const double* x, double tmc, double dt,
                                        size_t dim_) :
  filt(NULL), dim(dim_)
{
  RCHECK(dim>0);

  filt = new SecondOrderLPF1D*[dim];

  for (size_t i=0; i<dim; i++)
  {
    filt[i] = new SecondOrderLPF1D(x[i], tmc, dt);
  }
}

Rcs::SecondOrderLPFND::~SecondOrderLPFND()
{
  for (size_t i=0; i<dim; i++)
  {
    delete (filt[i]);
  }

  delete [] filt;
}

void Rcs::SecondOrderLPFND::init(const double* x, double tmc_, double dt_)
{
  for (size_t i=0; i<dim; i++)
  {
    filt[i]->init(x[i], tmc_, dt_);
  }
}

void Rcs::SecondOrderLPFND::init(const double* x)
{
  for (size_t i=0; i<dim; i++)
  {
    filt[i]->init(x[i]);
  }
}

void Rcs::SecondOrderLPFND::setState(const double* x, const double* x_dot)
{
  for (size_t i=0; i<dim; i++)
  {
    filt[i]->setState(x[i], x_dot[i]);
  }
}

void Rcs::SecondOrderLPFND::setDim(size_t dim_)
{
  this->dim = dim_;
}

size_t Rcs::SecondOrderLPFND::getDim() const
{
  return this->dim;
}

void Rcs::SecondOrderLPFND::iterate(double* x_ddot, const double* goal,
                                    const double* addedAccel)
{
  for (size_t i = 0; i < dim; i++)
  {
    x_ddot[i] = filt[i]->iterate(goal[i], addedAccel[i]);
  }
}

void Rcs::SecondOrderLPFND::iterate(double* x_ddot, const double* goal)
{
  for (size_t i=0; i<dim; i++)
  {
    x_ddot[i] = filt[i]->iterate(goal[i]);
  }
}

void Rcs::SecondOrderLPFND::iterate(double* x_ddot)
{
  for (size_t i=0; i<dim; i++)
  {
    x_ddot[i] = filt[i]->iterate(filt[i]->getTarget());
  }

}

void Rcs::SecondOrderLPFND::iterate()
{
  for (size_t i=0; i<dim; i++)
  {
    filt[i]->iterate(filt[i]->getTarget());
  }

}

void Rcs::SecondOrderLPFND::getPosition(double* x) const
{
  for (size_t i=0; i<dim; i++)
  {
    x[i] = filt[i]->getPosition();
  }
}

double Rcs::SecondOrderLPFND::getPosition(size_t index) const
{
  RCHECK_MSG(index<dim, "index=%zu   dim=%zu", index, dim);
  return filt[index]->getPosition();
}

void Rcs::SecondOrderLPFND::getVelocity(double* x_dot) const
{
  for (size_t i=0; i<dim; i++)
  {
    x_dot[i] = filt[i]->getVelocity();
  }
}

void Rcs::SecondOrderLPFND::computeAcceleration(double* x_ddot,
                                                const double* goal) const
{
  for (size_t i=0; i<dim; i++)
  {
    x_ddot[i] = filt[i]->computeAcceleration(goal[i]);
  }
}

void Rcs::SecondOrderLPFND::setDamping(double damping)
{
  for (size_t i=0; i<dim; i++)
  {
    filt[i]->setDamping(damping);
  }
}

void Rcs::SecondOrderLPFND::setTarget(const double* target)
{
  for (size_t i=0; i<dim; i++)
  {
    filt[i]->setTarget(target[i]);
  }
}

void Rcs::SecondOrderLPFND::print() const
{
  fprint(stdout);
}

void Rcs::SecondOrderLPFND::fprint(FILE* out) const
{
  for (size_t i=0; i<dim; i++)
  {
    filt[i]->fprint(out);
  }
}


/*******************************************************************************
 * Ramp filter class for n dimensions
 ******************************************************************************/
Rcs::RampFilterND::RampFilterND(double tmc, double vmax_, double dt, size_t dim):
  Rcs::SecondOrderLPFND(tmc, dt, dim), vmax(vmax_)
{
  r = RNALLOC(dim, double);
  VecNd_setZero(r, dim);
}

Rcs::RampFilterND::RampFilterND(double* x, double tmc, double vmax_, double dt,
                                size_t dim):
  Rcs::SecondOrderLPFND(x, tmc, dt, dim), vmax(vmax_)
{
  r = RNALLOC(dim, double);
  VecNd_copy(r, x, dim);
}

Rcs::RampFilterND::~RampFilterND()
{
  RFREE(r);
}

void Rcs::RampFilterND::init(const double* x, double tmc_, double dt_)
{
  Rcs::SecondOrderLPFND::init(x, tmc_, dt_);
  VecNd_copy(r, x, dim);
}

void Rcs::RampFilterND::init(const double* x)
{
  Rcs::SecondOrderLPFND::init(x);
  VecNd_copy(r, x, dim);
}

void Rcs::RampFilterND::getRamp(double* ramp) const
{
  VecNd_copy(ramp, r, dim);
}

void Rcs::RampFilterND::iterate(double* x_ddot)
{
  double dx_max = vmax*filt[0]->getDt();

  for (size_t i=0; i<dim; i++)
  {
    if (filt[i]->getTarget() > r[i]+dx_max)
    {
      r[i] += dx_max;
    }
    else if (filt[i]->getTarget() < r[i]-dx_max)
    {
      r[i] -= dx_max;
    }
    else
    {
      r[i]  = filt[i]->getTarget();
    }

  }

  iterate(x_ddot, r);
}

void Rcs::RampFilterND::iterate()
{
  double* x_ddot = new double[dim];
  iterate(x_ddot);
  delete [] x_ddot;
}

double Rcs::RampFilterND::getRamp(size_t index) const
{
  RCHECK(index<dim);
  return r[index];
}


/******************************************************************************
 * Median filter class for 1 dimension
 *****************************************************************************/
Rcs::MedianFilter1D::MedianFilter1D(unsigned int windowSize_) :
  windowSize(windowSize_), median(0.0)
{
  RCHECK_MSG(this->windowSize > 0,
             "Median filter's window size must be larger than zero");
  this->data = RNALLOC(this->windowSize, double);
  this->buf = RNALLOC(this->windowSize, double);
  init(0.0);
}

Rcs::MedianFilter1D::MedianFilter1D(unsigned int windowSize_, double value) :
  windowSize(windowSize_), median(0.0)
{
  this->data = RNALLOC(this->windowSize, double);
  this->buf = RNALLOC(this->windowSize, double);
  init(value);
}

Rcs::MedianFilter1D::~MedianFilter1D()
{
  RFREE(this->data);
  RFREE(this->buf);
}

void Rcs::MedianFilter1D::init(double value)
{
  VecNd_setElementsTo(this->data, value, this->windowSize);
  this->median = value;
}

void Rcs::MedianFilter1D::addSample(double sample)
{
  // Window size of 1 would otherwise lead to segmentation fault.
  if (this->windowSize > 1)
  {
    memmove(&data[1], &data[0], (windowSize-1)*sizeof(double));
  }

  this->data[0] = sample;
}

void Rcs::MedianFilter1D::print() const
{
  printf("Median filter: size is %d\n", this->windowSize);
  for (unsigned int i=0; i<this->windowSize; i++)
  {
    printf("%f\n", this->data[i]);
  }

}

double Rcs::MedianFilter1D::filt()
{
  VecNd_copy(this->buf, this->data, this->windowSize);
  this->median = VecNd_median(this->buf, this->windowSize);
  return this->median;
}

double Rcs::MedianFilter1D::filt(double sample)
{
  addSample(sample);
  return filt();
}

double Rcs::MedianFilter1D::getMedian() const
{
  return this->median;
}


/******************************************************************************
 * Median filter class for 1 dimension
 *****************************************************************************/
Rcs::MedianFilterND::MedianFilterND(size_t windowSize, size_t dim_) :
  dim(dim_)
{
  RCHECK(dim>0);

  this->filter = new MedianFilter1D*[dim];

  for (size_t i=0; i<dim; i++)
  {
    filter[i] = new MedianFilter1D(windowSize);
  }
}

Rcs::MedianFilterND::MedianFilterND(size_t windowSize, const double* value,
                                    size_t dim_) : dim(dim_)
{
  RCHECK(dim>0);

  this->filter = new Rcs::MedianFilter1D*[dim];

  for (size_t i=0; i<dim; i++)
  {
    filter[i] = new Rcs::MedianFilter1D(windowSize, value[i]);
  }
}

Rcs::MedianFilterND::~MedianFilterND()
{
  for (size_t i=0; i<dim; i++)
  {
    delete (filter[i]);
  }

  delete [] filter;
}

void Rcs::MedianFilterND::init(const double* value)
{
  for (size_t i=0; i<dim; i++)
  {
    filter[i]->init(value[i]);
  }
}

void Rcs::MedianFilterND::addSample(const double* sample)
{
  for (size_t i=0; i<dim; i++)
  {
    filter[i]->addSample(sample[i]);
  }
}

void Rcs::MedianFilterND::print() const
{
  for (size_t i=0; i<dim; i++)
  {
    filter[i]->print();
  }
}

void Rcs::MedianFilterND::filt()
{
  for (size_t i=0; i<dim; i++)
  {
    filter[i]->filt();
  }
}

void Rcs::MedianFilterND::filt(const double* sample)
{
  for (size_t i=0; i<dim; i++)
  {
    filter[i]->filt(sample[i]);
  }
}

void Rcs::MedianFilterND::getMedian(double* median) const
{
  for (size_t i=0; i<dim; i++)
  {
    median[i] = filter[i]->getMedian();
  }
}

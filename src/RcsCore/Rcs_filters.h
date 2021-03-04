/*******************************************************************************

  Copyright (c) 2017, Honda Research Institute Europe GmbH

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

#ifndef RCS_FILTERS_H
#define RCS_FILTERS_H

#include <cstdio>


namespace Rcs
{

/*******************************************************************************
 * Linear ramp with 1 dimension
 ******************************************************************************/
class Ramp1D
{
public:

  Ramp1D(double vmax, double dt);
  Ramp1D(double x, double vmax, double dt);
  virtual ~Ramp1D();
  virtual void init(double x0, double vmax, double dt);
  virtual void init(double x0);
  virtual void setTarget(double targ);
  virtual void setDt(double dt);
  virtual double iterate();
  virtual double getPosition() const;
  virtual double getDt() const;

protected:
  double r, target, vmax, dt;

private:
  Ramp1D();
  Ramp1D(const Ramp1D&);
  Ramp1D& operator=(const Ramp1D&);
};

/*******************************************************************************
 * Linear ramp with n dimensions
 ******************************************************************************/
class RampND
{
public:

  RampND(double vmax, double dt, size_t dim);
  RampND(const double* x, double vmax, double dt, size_t dim);
  virtual ~RampND();
  virtual void init(const double* x0, double vmax, double dt);
  virtual void init(const double* x0);
  virtual void setTarget(const double* targ);
  virtual void iterate();
  virtual void getPosition(double* x) const;
  virtual double getPosition(size_t index) const;

protected:
  Ramp1D** filt;
  size_t dim;

private:
  RampND();
  RampND(const RampND&);
  RampND& operator=(const RampND&);
};

/*******************************************************************************
 * Asymptotically damped second order low pass filter with 1 dimension
 ******************************************************************************/
class SecondOrderLPF1D
{
public:

  SecondOrderLPF1D(double x, double tmc, double dt);
  virtual ~SecondOrderLPF1D();

  virtual void init(double x0, double tmc, double dt);
  virtual void init(double x0);
  virtual void setState(double x, double x_dot);
  virtual double iterate(double goal, double addedAccel);
  virtual double iterate(double goal);
  virtual double iterate();
  virtual double getPosition() const;
  virtual double* getPositionPtr();
  virtual double getVelocity() const;
  virtual double computeAcceleration(double goal) const;
  virtual void setDamping(double damping);
  virtual void setTarget(double targ);
  virtual void setDt(double dt);
  virtual double getTarget() const;
  virtual double getDt() const;
  virtual double getTimeConstant() const;
  virtual void print() const;
  virtual void fprint(FILE* out) const;

protected:
  double x, x_dot, target, tmc, dt, xi;

private:
  SecondOrderLPF1D();
  SecondOrderLPF1D(const SecondOrderLPF1D&);
  SecondOrderLPF1D& operator=(const SecondOrderLPF1D&);
};


/*******************************************************************************
 * Ramp filter with 1 dimension
 ******************************************************************************/
class RampFilter1D : public SecondOrderLPF1D
{
public:

  using SecondOrderLPF1D::iterate;

  RampFilter1D(double x0, double tmc, double vmax, double dt);
  virtual ~RampFilter1D();
  virtual void init(double x0, double tmc, double dt);
  virtual void init(double x0);
  virtual double getRamp() const;
  virtual double iterate();
  virtual void fprint(FILE* out) const;

protected:
  double vmax, r;

private:
  RampFilter1D();
  RampFilter1D(const RampFilter1D&);
  RampFilter1D& operator=(const RampFilter1D&);
};



/*******************************************************************************
 * Asymptotically damped second order low pass filter with n dimensions
 ******************************************************************************/
class SecondOrderLPFND
{
public:

  SecondOrderLPFND(double tmc, double dt, size_t dim);
  SecondOrderLPFND(const double* x, double tmc, double dt, size_t dim);
  virtual ~SecondOrderLPFND();
  virtual void init(const double* x, double tmc, double dt);
  virtual void init(const double* x);
  virtual void setState(const double* x, const double* x_dot);
  virtual void iterate(double* x_ddot, const double* goal, const double* addedAccel);
  virtual void iterate(double* x_ddot, const double* goal);
  virtual void iterate(double* x_ddot);
  virtual void iterate();
  virtual void getPosition(double* x) const;
  virtual double getPosition(size_t index) const;
  virtual void getVelocity(double* x_dot) const;
  virtual void computeAcceleration(double* x_ddot, const double* goal) const;
  virtual size_t getDim() const;
  virtual void setDim(size_t dim);
  virtual void setDamping(double damping);
  virtual void setTarget(const double* target);
  virtual void print() const;
  virtual void fprint(FILE* out) const;

protected:
  SecondOrderLPF1D** filt;
  size_t dim;

private:
  SecondOrderLPFND();
  SecondOrderLPFND(const SecondOrderLPFND&);
  SecondOrderLPFND& operator=(const SecondOrderLPFND&);
};


/*******************************************************************************
 * Ramp filter class for n dimensions
 ******************************************************************************/
class RampFilterND : public SecondOrderLPFND
{
public:

  using SecondOrderLPFND::iterate;

  RampFilterND(double tmc, double vmax, double dt, size_t dim);
  RampFilterND(double* x, double tmc, double vmax, double dt, size_t dim);
  virtual ~RampFilterND();
  virtual void init(const double* x, double tmc, double dt);
  virtual void init(const double* x);
  virtual void getRamp(double* ramp) const;
  virtual void iterate(double* x_ddot);
  virtual void iterate();
  virtual double getRamp(size_t index) const;

protected:
  double vmax, *r;

private:
  RampFilterND();
  RampFilterND(const RampFilterND&);
  RampFilterND& operator=(const RampFilterND&);
};


/******************************************************************************
 * Median filter class for 1 dimension
 *****************************************************************************/
class MedianFilter1D
{
public:
  MedianFilter1D(unsigned int windowSize);
  MedianFilter1D(unsigned int windowSize, double value);
  virtual ~MedianFilter1D();
  virtual void init(double value);
  virtual void addSample(double sample);
  virtual void print() const;
  virtual double filt();
  virtual double filt(double sample);
  virtual double getMedian() const;

private:
  unsigned int windowSize;
  double median;
  double* data;
  double* buf;

private:
  MedianFilter1D();
  MedianFilter1D(const MedianFilter1D&);
  MedianFilter1D& operator=(const MedianFilter1D&);
};


/******************************************************************************
 * Median filter class for n dimension
 *****************************************************************************/
class MedianFilterND
{
public:
  MedianFilterND(size_t windowSize, size_t dim);
  MedianFilterND(size_t windowSize, const double* value, size_t dim);
  virtual ~MedianFilterND();
  virtual void init(const double* value);
  virtual void addSample(const double* sample);
  virtual void print() const;
  virtual void filt();
  virtual void filt(const double* sample);
  virtual void getMedian(double* median) const;

private:
  size_t dim;
  MedianFilter1D** filter;

private:
  MedianFilterND();
  MedianFilterND(const MedianFilterND&);
  MedianFilterND& operator=(const MedianFilterND&);
};

}   // namespace Rcs

#endif   //  RCS_FILTERS_H

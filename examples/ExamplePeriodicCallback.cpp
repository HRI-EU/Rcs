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

#include <PeriodicCallback.h>
#include <Rcs_macros.h>
#include <Rcs_cmdLine.h>
#include <Rcs_resourcePath.h>
#include <Rcs_timer.h>
#include <Rcs_utils.h>
#include <Rcs_math.h>
#include <SegFaultHandler.h>

#include <vector>

RCS_INSTALL_ERRORHANDLERS


bool runLoop = true;


/*******************************************************************************
 * Ctrl-C destructor. Tries to quit gracefully with the first Ctrl-C
 * press, then just exits.
 ******************************************************************************/
void quit(int /*sig*/)
{
  static int kHit = 0;
  runLoop = false;
  fprintf(stderr, "Trying to exit gracefully - %dst attempt\n", kHit + 1);
  kHit++;

  if (kHit == 2)
  {
    fprintf(stderr, "Exiting without cleanup\n");
    exit(0);
  }
}



/*******************************************************************************
 *
 ******************************************************************************/
class MyCallback : public Rcs::PeriodicCallback
{
public:

  MyCallback(const char* msg_) : Rcs::PeriodicCallback(), msg(msg_)
  {
    setClassName(msg);
  }

  void callback()
  {
    printf("Callback \"%s\": %lu\n", this->msg.c_str(), getLoopCount());
  }

  ~MyCallback()
  {
    stop();
    RLOGS(1, "Destroying callback \"%s\"", this->msg.c_str());
  }

  std::string msg;
};


static void testPeriodicCallback(int argc, char** argv)
{
  Rcs::CmdLineParser argP(argc, argv);

  if (argP.hasArgument("-h"))
  {
    return;
  }

  MyCallback p1("Callback 1");
  p1.start(10.0);

  MyCallback p2("Callback 2");
  p2.start(20.0);

  MyCallback p3("Callback 3");
  p3.start(30.0);

  while (runLoop)
  {
    if (p1.getLoopCount() > 10)
    {
      runLoop = false;
    }
    Timer_waitDT(0.1);
  }

  RLOGS(0, "Stopping callback 1");
  p1.stop();
  RLOGS(0, "Stopping callback 2");
  p2.stop();
  RLOGS(0, "Stopping callback 3");
  p3.stop();



  RLOGS(0, "Restarting callback 1");
  runLoop = true;

  p1.start(10.0);

  while (runLoop)
  {
    if (p1.getLoopCount() > 10)
    {
      runLoop = false;
    }
    Timer_waitDT(0.1);
  }

  RLOGS(0, "Stopping restarted callback 1");
  p1.stop();


  RPAUSE_MSG("Hit enter to quit test");
}



/*******************************************************************************
 *
 ******************************************************************************/
class SchedCallback : public Rcs::PeriodicCallback
{
public:

  SchedCallback(const char* msg_) : Rcs::PeriodicCallback(), msg(msg_)
  {
    setClassName(msg);
  }

  void callback()
  {
    Rcs_printThreadInfo(stderr, msg.c_str());
  }

  std::string msg;
};


static void testThreadScheduling(int argc, char** argv)
{
  SchedCallback c1("Thread 1: ");
  SchedCallback c2("Thread 2: ");
  SchedCallback c3("Thread 3: ");

  c2.setSchedulingPolicy(SCHED_RR);

  c1.start(100.0, 10);
  c2.start(100.0, 60);
  c3.start(100.0, 0);

  RPAUSE_MSG("Hit enter to stop threads");

  c1.stop();
  c2.stop();
  c3.stop();
}



/*******************************************************************************
 *
 ******************************************************************************/
class CpuLoadCallback : public Rcs::PeriodicCallback
{
public:

  CpuLoadCallback() : Rcs::PeriodicCallback(), b(0.0), runLoop(true)
  {
    setClassName("CpuLoadCallback");
    setSynchronizationMode(Synchronize_OneShot);
  }

  void callback()
  {
    while (runLoop)
    {
      double a = sin((double)getLoopCount());
      this->b = cos((double)a);
    }
  }

  void stop()
  {
    runLoop = false;
    Rcs::PeriodicCallback::stop();
  }

  std::string msg;
  double b;
  bool runLoop;
};

static void createCpuLoad(int argc, char** argv)
{
  unsigned int nThreads = 1;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-nThreads", &nThreads, "Number of threads (default is %d)",
                   nThreads);

  if (argP.hasArgument("-h"))
  {
    return;
  }

  std::vector<CpuLoadCallback*> cpuHog;

  for (unsigned int i=0; i<nThreads; ++i)
  {
    CpuLoadCallback* cb = new CpuLoadCallback();
    cb->start(100.0);
    cpuHog.push_back(cb);
  }

  RPAUSE_MSG("Hit enter to stop threads");

  for (size_t i=0; i<cpuHog.size(); ++i)
  {
    RLOG_CPP(0, "Stopping thread " << i);
    cpuHog[i]->stop();
    delete cpuHog[i];
  }

}


/*******************************************************************************
 *
 ******************************************************************************/
class BenchmarkCallback : public Rcs::PeriodicCallback
{
public:

  BenchmarkCallback() : Rcs::PeriodicCallback(), limitFreq(0.0)
  {
    setClassName("BenchmarkCallback");
    setSynchronizationMode(Synchronize_Soft);

    double t = Timer_getSystemTime();
    for (unsigned int i=0; i<100; ++i)
    {
      calcSomething();
    }
    t = Timer_getSystemTime() - t;

    this->limitFreq = 1.0/(t/100.0);

    RLOG(0, "f_calc = %g", this->limitFreq);
  }

  double calcSomething()
  {
    const int n = 100;

    MatNd* J = MatNd_create(3 * n, n);
    MatNd* A = MatNd_create(n, n);
    MatNd* A_inv = MatNd_create(n, n);

    MatNd_setRandom(J, -1.0, 1.0);
    MatNd_sqrMulAtBA(A, J, NULL);
    double det = MatNd_choleskyInverse(A_inv, A);

    MatNd_destroy(J);
    MatNd_destroy(A);
    MatNd_destroy(A_inv);

    return det;
  }

  void callback()
  {
    double det = calcSomething();
    RLOG(1, "det=%g", det);
  }

  double limitFreq;
};

static void testSchedulingPriority(int argc, char** argv)
{
  Rcs::CmdLineParser argP(argc, argv);
  int rtPrio = 50;
  unsigned int nThreads = 1;
  argP.getArgument("-prio", &rtPrio, "RT priority (default is %d)", rtPrio);
  argP.getArgument("-nThreads", &nThreads, "Number of threads (default is %d)",
                   nThreads);
  bool rt = argP.hasArgument("-rt", "Real-time scheduling");

  if (argP.hasArgument("-h"))
  {
    return;
  }

  BenchmarkCallback cb;
  std::vector<CpuLoadCallback*> cpuHog;

  if (rt==true)
  {
    cb.setSchedulingPolicy(SCHED_RR);
    RMSG("Starting thread with RT scheduler: priority is %d, frequency is %f",
         rtPrio, 0.9*cb.limitFreq);
  }
  else
  {
    RMSG("Starting thread, frequency is %f", 0.9*cb.limitFreq);
  }

  for (unsigned int i=0; i<nThreads; ++i)
  {
    CpuLoadCallback* cb = new CpuLoadCallback();
    cb->start(100.0);
    cpuHog.push_back(cb);
  }

  cb.start(0.9*cb.limitFreq, rtPrio);

  while (cb.getLoopCount() < 100)
  {
    RLOG(0, "Loop count is %d", (int) cb.getLoopCount());
    Timer_waitDT(0.1);
  }

  RMSG("Stopping threads");

  for (unsigned int i=0; i<nThreads; ++i)
  {
    RLOG(0, "Stopping thread %d", i);
    cpuHog[i]->stop();
  }

  cb.stop();
  cb.print();
}



/******************************************************************************


 *****************************************************************************/
int main(int argc, char** argv)
{
  signal(SIGINT, quit);

  int mode = 0;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");
  argP.getArgument("-m", &mode, "Test mode (default is %d)", mode);

  if ((mode==0) || (argP.hasArgument("-h")))
  {
    printf("\nExamplePeriodicCallback\n");
    printf("\t-m 1: Test PeriodicCallback class\n");
    printf("\t-m 2: Test thread starting and stopping\n");
    printf("\t-m 3: Create CPU load\n");
    printf("\t-m 4: Test scheduling priority\n");
    printf("\n");

    if (mode==0)
    {
      return 0;
    }
  }

  switch (mode)
  {
    case 1:
      testPeriodicCallback(argc, argv);
      break;

    case 2:
      testThreadScheduling(argc, argv);
      break;

    case 3:
      createCpuLoad(argc, argv);
      break;

    case 4:
      testSchedulingPriority(argc, argv);
      break;


    default:
      if (!argP.hasArgument("-h"))
      {
        RFATAL("No mode %d", mode);
      }
  }

  if (argP.hasArgument("-h"))
  {
    argP.print();
  }

  fprintf(stderr, "Thanks for using the ExamplePeriodicCallback\n");

  return 0;
}

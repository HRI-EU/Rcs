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

#include <Atomic.hpp>
#include <HighGui.h>
#include <Rcs_basicMath.h>
#include <Rcs_cmdLine.h>
#include <Rcs_Vec3d.h>
#include <Rcs_VecNd.h>
#include <Rcs_guiFactory.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>

#include <cstdlib>
#include <cstdio>
#include <csignal>


using namespace Rcs;

Rcs::Atomic<bool> runLoop(true);

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

void quit()
{
  quit(0);
}

/*!
 * \brief Simple function used as callback to change plot properties using a
 *        slider.
 */
void changeYMax(double val)
{
  HighGui::configurePlot("Plot 1", 11, HighGuiPlot::DefaultCapacity, -1.0, val);
}

void* differentThread(void*)
{
  unsigned int loop_count = 0;
  while (runLoop)
  {
    HighGui::showLabel("loop_count", 2, "Thread 2 loop count = %u", loop_count);

    double val = HighGui::getSlider("Slider1", 4);
    HighGui::showLabel("slider value", 2,
                       "Thread 2 accessing another thread's slider value %f",
                       val);

    Timer_waitDT(0.1);
    loop_count++;
  }

  return NULL;
}

int main(int argc, char** argv)
{
  // Ctrl-C callback handler
  signal(SIGINT, quit);

  // Parse command line arguments
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel);

  // run another thread for checking if the mutexes work
  pthread_t another_thread;
  pthread_create(&another_thread, 0, differentThread, NULL);

  // connect the quit button via a callback
  HighGui::checkButton("Quit Rcs");
  HighGui::registerButtonCallback("Quit Rcs", 0, quit);


  HighGui::checkButton("Button1");
  HighGui::checkButton("Button2");
  HighGui::checkButton("Button3");

  HighGui::checkButton("Button3 LongLongLongLong Text");
  HighGui::checkButton("Button4");

  HighGui::showLabel("Label1", 1, "Label but no text");

  HighGui::showSlider("Slider1", 0, 0, 1, 0.01);

  Rcs::Atomic<double> val;
  HighGui::showSlider("Slider1", 4, 0, 100, 0.1, &val);
  HighGui::showSlider("Slider2 (coupled by variable to Slider 1)",
                      4, 0, 100, 0.1, &val);

  // callback function to set y max range of "Plot" of window 11
  HighGui::configurePlot("Plot 1", 11, HighGuiPlot::DefaultCapacity, -1.0, 1.0);
  HighGui::showSlider("Plot 1 Y_max", 11, 0.0, 10.0, 0.01);
  HighGui::registerSliderCallback("Plot 1 Y_max", 11, changeYMax);

  // Set capacity and fixed min max range of "Plot 2"
  HighGui::configurePlot("Plot 2", 11, 50, 0.0, 0.75);

  unsigned int loop_count = 0;
  while (runLoop)
  {
    if (loop_count % 20 == 0)
    {
      HighGui::showLabel("Label1", 0, "Loop: %d", loop_count);
    }

    if (HighGui::checkButton("Button that did not exist before"))
    {
      HighGui::showLabel("Label1", 1, "Button XXX clicked at %d", loop_count);
    }

    if (HighGui::checkButton("Button1"))
    {
      HighGui::showLabel("Label1", 1, "Button 1 clicked at %d", loop_count);
      HighGui::showPlot("Button 1", 13, Math_getRandomNumber(0, 100));
    }

    if (HighGui::checkButton("Button2"))
    {
      // let's give both windows some labels
      HighGui::namedWindow("Window 1 (label set by Button2)", 0);
      HighGui::namedWindow("Window 2 (label set by Button2)", 1);
    }

    if (HighGui::checkButton("Button3"))
    {
      HighGui::checkButton("New Button created from another thread", 2);
    }

    if (HighGui::checkButton("Button4"))
    {
      val = 40;
    }

    HighGui::showLabel("Atomic<double>", 3, "Atomic<double> %f", val.get());

    HighGui::showPlot("A random number", 10, Math_getRandomNumber(-1.0, 1.0));
    HighGui::checkButton("Button1", 11);
    HighGui::checkButton("Button2", 11);
    HighGui::checkButton("Button3", 11);
    HighGui::showPlot("Plot 1", 11, sin(loop_count / 10.0), 0);
    HighGui::showPlot("Plot 1", 11, cos(loop_count / 10.0), 1);
    HighGui::showPlot("Plot 1", 11, cos(loop_count / 5.0), 2);
    HighGui::showPlot("Plot 2", 11, sin(loop_count / 2.0), 0);
    HighGui::showPlot("Plot 2", 11, cos(loop_count / 2.0), 1);
    HighGui::showPlot("Plot 3", 11, 0.0, 0);
    HighGui::showPlot("Plot 3", 11, 1.0, 1);
    HighGui::showPlot("Plot 3", 11, 3.0, 3);

    HighGui::showPlot("Slider value", 12, val);

    double three_values[3];
    Vec3d_setRandom(three_values, -1.0, 1.0);
    HighGui::showPlot("Three random values", 12, three_values, 3);

    Timer_waitDT(0.025);
    loop_count++;
  }

  void* status;
  pthread_join(another_thread, &status);
  RcsGuiFactory_shutdown();

  return EXIT_SUCCESS;
}

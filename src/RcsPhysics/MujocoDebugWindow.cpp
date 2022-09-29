/*******************************************************************************

  Copyright 2021 DeepMind Technologies Limited
  Modifications copyright 2022 Honda Research Institute Europe GmbH

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

*******************************************************************************/

#include "MujocoDebugWindow.h"

#include <Rcs_macros.h>
#include <Rcs_timer.h>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <cstdio>
#include <cstring>


namespace Rcs
{

struct MujocoWinData
{
  // MuJoCo data structures
  mjModel* m;                  // MuJoCo model
  mjData* d;                   // MuJoCo data
  mjvCamera cam;               // abstract camera
  mjvOption opt;               // visualization options
  mjvScene scn;                // abstract scene
  mjrContext con;              // custom GPU context

  // Mouse interaction
  bool button_left;
  bool button_middle;
  bool button_right;
  double lastx;
  double lasty;

  // Graphics control
  bool showFrames;

  // Threading control
  bool quitRequest;
  bool threadRunning;
};

static void MujocoWinData_init(MujocoWinData* data)
{
  memset(data, 0, sizeof(MujocoWinData));
}

static MujocoWinData* getWinData(GLFWwindow* window)
{
  Rcs::MujocoDebugWindow* mWin = (Rcs::MujocoDebugWindow*) glfwGetWindowUserPointer(window);
  return mWin->getWinData();
}

// keyboard callback
static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
  // Toggle reference frames
  if (act==GLFW_PRESS)
  {
    MujocoWinData* winData = getWinData(window);

    if (key==GLFW_KEY_R)
    {
      winData->showFrames = !winData->showFrames;
      winData->opt.frame = winData->showFrames ? mjFRAME_BODY : mjFRAME_NONE;
    }
    else if (key==GLFW_KEY_W)
    {
      winData->scn.flags[mjRND_WIREFRAME] = !winData->scn.flags[mjRND_WIREFRAME];
    }
    else if (key==GLFW_KEY_K)
    {
      winData->quitRequest = true;
    }
  }

}


// mouse button callback
static void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
  MujocoWinData* winData = getWinData(window);

  // update button state
  winData->button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
  winData->button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
  winData->button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &winData->lastx, &winData->lasty);
}


// mouse move callback
static void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
  MujocoWinData* winData = getWinData(window);

  // no buttons down: nothing to do
  if (!winData->button_left && !winData->button_middle && !winData->button_right)
  {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - winData->lastx;
  double dy = ypos - winData->lasty;
  winData->lastx = xpos;
  winData->lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (winData->button_right)
  {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  }
  else if (winData->button_left)
  {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  }
  else
  {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(winData->m, action, dx/height, dy/height, &winData->scn, &winData->cam);
}


// scroll callback
static void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
  // emulate vertical mouse motion = 5% of window height
  MujocoWinData* winData = getWinData(window);
  mjv_moveCamera(winData->m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &winData->scn, &winData->cam);
}


// main function
void* MujocoDebugWindow::threadFuncPosix(void* param)
{
  // init GLFW
  if (!glfwInit())
  {
    mju_error("Could not initialize GLFW");
    return NULL;
  }

  Rcs::MujocoDebugWindow* mWin = (Rcs::MujocoDebugWindow*) param;
  MujocoWinData* winData = mWin->getWinData();
  winData->threadRunning = true;

  // create window, make OpenGL context current, request v-sync
  GLFWwindow* window = glfwCreateWindow(640, 480, "Mujoco debug window", NULL, NULL);
  glfwSetWindowUserPointer(window, param);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&winData->cam);
  mjv_defaultOption(&winData->opt);
  mjv_defaultScene(&winData->scn);
  mjr_defaultContext(&winData->con);

  // Adjust line settings for visualization
  winData->m->vis.scale.framelength *= 0.5;
  winData->m->vis.scale.framewidth *= 0.1;

  // create scene and context
  mjv_makeScene(winData->m, &winData->scn, 2000);
  mjr_makeContext(winData->m, &winData->con, mjFONTSCALE_100);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);

  // run main loop, target real-time simulation and 60 fps rendering
  while ((!glfwWindowShouldClose(window)) && (!winData->quitRequest))
  {
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(winData->m, winData->d, &winData->opt, NULL, &winData->cam, mjCAT_ALL, &winData->scn);
    mjr_render(viewport, &winData->scn, &winData->con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
    RLOG(5, "winData->quitRequest: %d", winData->quitRequest);
  }

  RLOG(5, "Quitting endless loop");

  //free visualization storage
  mjv_freeScene(&winData->scn);
  mjr_freeContext(&winData->con);

  // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#else
  glfwDestroyWindow(window);
#endif

  winData->quitRequest = false;
  winData->threadRunning = false;
  RLOG(5, "Exiting thread function");

  return NULL;
}

MujocoDebugWindow::MujocoDebugWindow(mjModel* sim, mjData* simData) : winData(NULL)
{
  winData = new MujocoWinData;
  MujocoWinData_init(winData);
  winData->m = sim;
  winData->d = simData;
}

MujocoDebugWindow::~MujocoDebugWindow()
{
  stop();
  delete winData;
  RLOG(5, "Deleted MujocoDebugWindow");
}

void MujocoDebugWindow::start()
{
  if (getWinData()->threadRunning)
  {
    RLOG(1, "MujocoDebugWindow already running");
    return;
  }

  RLOG(5, "Launching MujocoDebugWindow");
  pthread_create(&graphicsThread, NULL, &threadFuncPosix, this);
  RLOG(5, "Done launching MujocoDebugWindow");
}

void MujocoDebugWindow::stop()
{
  if (!getWinData()->threadRunning)
  {
    RLOG(1, "MujocoDebugWindow already stopped");
    return;
  }

  RLOG(5, "Stopping MujocoDebugWindow");
  winData->quitRequest = true;
  pthread_join(graphicsThread, NULL);

  while (winData->threadRunning)
  {
    RLOG(5, "Waiting for thread function to finish ...");
    Timer_waitDT(0.01);
  }

  RLOG(5, "Stopped MujocoDebugWindow");
}

MujocoWinData* MujocoDebugWindow::getWinData()
{
  return winData;
}



}   // namespace Rcs

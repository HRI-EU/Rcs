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

#include <Rcs_macros.h>
#include <Rcs_gradientTests.h>
#include <Rcs_typedef.h>
#include <Rcs_utils.h>
#include <Rcs_utilsCPP.h>
#include <Rcs_timer.h>
#include <Rcs_parser.h>
#include <Rcs_resourcePath.h>
#include <Rcs_cmdLine.h>
#include <Rcs_math.h>
#include <Rcs_joint.h>
#include <Rcs_body.h>
#include <Rcs_kinematics.h>
#include <Rcs_dynamics.h>

#include <SegFaultHandler.h>



RCS_INSTALL_SEGFAULTHANDLER

bool runLoop = true;



/*******************************************************************************
 * Ctrl-C destructor. Tries to quit gracefully with the first Ctrl-C
 *        press, then just exits.
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
int main(int argc, char** argv)
{
  int mode = 0;
  char xmlFileName[128] = "", directory[128] = "";

  // Ctrl-C callback handler
  signal(SIGINT, quit);

  // This initialize the xml library and check potential mismatches between
  // the version it was compiled for and the actual shared library used.
  LIBXML_TEST_VERSION;

  // Parse command line arguments
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");
  argP.getArgument("-m", &mode, "Test mode");

  // Copy the current working directory
  const char* hgrDir = getenv("SIT");
  RLOG(5, "SIT is set to \"%s\"", hgrDir ? hgrDir : "nothing");

  const char* mkplt = getenv("MAKEFILE_PLATFORM");
  RLOG(5, "MAKEFILE_PLATFORM is set to \"%s\"", mkplt ? mkplt : "nothing");

  const char* hName = getenv("HOSTNAME");
  RLOG(5, "HOSTNAME is set to \"%s\"", hName ? hName : "nothing");

  if (hgrDir != NULL)
  {
    std::string meshDir;
    meshDir = std::string(hgrDir) + std::string("/Data/RobotMeshes/1.0/data");
    Rcs_addResourcePath(meshDir.c_str());
  }

  Rcs_addResourcePath("config");

  REXEC(6)
  {
    RPAUSE();
  }



  switch (mode)
  {
    // ==============================================================
    // Just print out some global information
    // ==============================================================
    case 0:
    {
      fprintf(stderr, "\n\tHere's some useful testing modes:\n\n");
      fprintf(stderr, "\t-dir\t<directory>   Working directory \n");
      fprintf(stderr, "\t-f\t<filename>   XML configuration file \n");
      fprintf(stderr, "\t-dl\t<0...>   Sets the debug level (default 0)\n");
      fprintf(stderr, "\t-m");
      fprintf(stderr, "\t0   Prints this message (default)\n");
      fprintf(stderr, "\t\t1   Prints the graph to stdout, writes a dot "
              "file and shows\n\t\t    it with dotty\n");
      fprintf(stderr, "\t\t2   Runs various gradient tests on the graph\n");
      fprintf(stderr, "\t\t3   Computes gravity compensation with COM "
              "Jacobian and compares it to equations of motion\n");
      fprintf(stderr, "\t\t4   Test joint limit costs with gnuplot\n");
      fprintf(stderr, "\t\t5   Test forward kinematics with velocities\n");
      fprintf(stderr, "\t\t6   Test all graphs in directory\n");
      fprintf(stderr, "\n\nResource path:\n");
      Rcs_printResourcePath();
      break;
    }



    // ==============================================================
    // Graph parsing and dot file output
    // ==============================================================
    case 1:
    {
      if (argP.hasArgument("-h"))
      {
        RMSG("Mode %d: Rcs.exe -m %d -dir <graph-directory> -f "
             "<graph-file>\n\n\t- Creates a graph from an xml file\n\t- "
             "prints it's contents to the console\n\t- creates a dot-file "
             "RcsGraph.dot and\n\t- launches it with the dotty tool\n\n\t"
             "The default "
             "xml file is \"%s\", the default directory is \"%s\"\n",
             mode, mode, xmlFileName, directory);
        break;
      }

      strcpy(xmlFileName, "gScenario.xml");
      strcpy(directory, "config/xml/WAM");
      argP.getArgument("-f", xmlFileName, "RcsGraph's configuration file name");
      argP.getArgument("-dir", directory, "Configuration file directory");
      Rcs_addResourcePath(directory);

      RcsGraph* graph = RcsGraph_create(xmlFileName);
      RCHECK(graph);

      RMSG("Here's the forward tree:");
      RcsGraph_fprint(stderr, graph);
      RcsGraph_writeDotFile(graph, "RcsGraph.dot");
      RcsGraph_destroy(graph);

      REXEC(0)
      {
        int err = system("dotty RcsGraph.dot&");

        if (err == -1)
        {
          RMSG("Couldn't start dot file viewer!");
        }
      }
      break;
    }



    // ==============================================================
    // Gradient tests for graphs
    // ==============================================================
    case 2:
    {
      strcpy(xmlFileName, "gScenario.xml");
      strcpy(directory, "config/xml/WAM");
      argP.getArgument("-f", xmlFileName, "RcsGraph's configuration file name");
      argP.getArgument("-dir", directory, "Configuration file directory");
      Rcs_addResourcePath(directory);

      if (argP.hasArgument("-h"))
      {
        RMSG("Mode %d: Rcs -m %d -dir <%s> -f "
             "<%s>\n\n\t- Creates a graph from an xml file\n\t"
             "- Runs a set of gradient tests on it\n\n\t"
             "On debug level 3 and higher, more verbose information is "
             "printed to the console.\n",
             mode, mode, directory, xmlFileName);

        REXEC(4)
        {
          Rcs_printResourcePath();
        }
        break;
      }

      bool silent = argP.hasArgument("-silent", "No console messages");
      int iter = 0;
      int maxIter = -1;
      argP.getArgument("-iter", &maxIter, "Max. number of tests (default inf)");

      RcsGraph* graph = RcsGraph_create(xmlFileName);
      RCHECK(graph);
      RcsGraph_setState(graph, NULL, NULL);
      MatNd* q_test = MatNd_create(graph->dof, 1);

      // Run tests with random state vectors
      while (runLoop)
      {
        RCSGRAPH_TRAVERSE_JOINTS(graph)
        {
          if (RcsJoint_isRotation(JNT))
          {
            MatNd_set(graph->q, JNT->jointIndex, 0,
                      Math_getRandomNumber(-M_PI_2, M_PI_2));
          }
          else
          {
            MatNd_set(graph->q, JNT->jointIndex, 0,
                      Math_getRandomNumber(-0.1, 0.1));
          }
        }

        MatNd_copy(q_test, graph->q);
        bool success = Rcs_gradientTestGraph(graph, q_test, !silent);

        RMSG("Gradient test %d %s",
             iter, success ? "SUCCEEDED" : "FAILED");
        iter++;

        if (maxIter>=0)
        {
          if (iter > maxIter)
          {
            runLoop = false;
          }
        }
      }

      RcsGraph_destroy(graph);
      MatNd_destroy(q_test);
      break;
    }



    // ==============================================================
    // Test gravity compensation:
    // transpose(COM Jacobian) * (0 0 -mg) = gravity compensation
    // ==============================================================
    case 3:
    {
      strcpy(xmlFileName, "gScenario.xml");
      strcpy(directory, "config/xml/WAM");
      argP.getArgument("-f", xmlFileName, "RcsGraph's configuration file name");
      argP.getArgument("-dir", directory, "Configuration file directory");
      Rcs_addResourcePath(directory);

      RcsGraph* graph = RcsGraph_create(xmlFileName);
      RCHECK(graph);

      MatNd* J_cog_tp = MatNd_create(3, graph->nJ);
      MatNd* F_g      = MatNd_create(3,1);
      MatNd* F_gj     = MatNd_create(graph->nJ, 1);
      MatNd* F_gk     = MatNd_create(graph->nJ, 1);
      MatNd* MM1      = MatNd_create(graph->nJ, graph->nJ);
      MatNd* MM2      = MatNd_create(graph->nJ, graph->nJ);


      // Run tests with random state vectors
      while (runLoop)
      {
        // Set state to random values
        MatNd_setRandom(graph->q, -10.0, 10.0);
        RcsGraph_setState(graph, NULL, NULL);

        // transpose(COM Jacobian) * (0 0 -mg)
        Timer_setZero();
        RcsGraph_COGJacobian(graph, J_cog_tp);
        MatNd_transposeSelf(J_cog_tp);
        MatNd_set(F_g, 2, 0, -RcsGraph_mass(graph)*RCS_GRAVITY);
        MatNd_mul(F_gj, J_cog_tp, F_g);
        double dt1 = Timer_getTime();

        // gravity compensation
        Timer_setZero();
        RcsGraph_computeKineticTerms(graph, NULL, NULL, F_gk);
        double dt2 = Timer_getTime();

        RMSG("\nCOM Jacobian");
        RMSG("\ndt1(COM Jacobian) = %.3f msec, dt2(Kinetic terms wo. M)"
             " = %.3f msec", dt1*1.0e3, dt2*1.0e3);

        REXEC(1)
        {
          RMSG("\nJ_com\t\tEOM\t\tDiff");
          MatNd_printTwoArraysDiff(F_gj, F_gk, 6);
        }


        // Mass matrix individually
        Timer_setZero();
        RcsGraph_computeMassMatrix(graph, MM1);
        double dt3 = Timer_getTime();

        // Mass matrix along with other terms
        Timer_setZero();
        RcsGraph_computeKineticTerms(graph, MM2, NULL, NULL);
        double dt4 = Timer_getTime();

        RMSG("\ndt3(computeMassMatrix) = %.3f msec, dt4(Kinetic terms)"
             " = %.3f msec", dt3*1.0e3, dt4*1.0e3);

        REXEC(1)
        {
          RMSG("\nMM single\t\tMM EOM\t\tDiff");
          MatNd_printTwoArraysDiff(MM1, MM2, 2);
        }

      }

      MatNd_destroy(F_g);
      MatNd_destroy(F_gj);
      MatNd_destroy(F_gk);
      MatNd_destroy(J_cog_tp);
      MatNd_destroy(MM1);
      MatNd_destroy(MM2);
      RcsGraph_destroy(graph);

      break;
    }



    // ==============================================================
    // Test joint limit costs
    // ==============================================================
    case 4:
    {
      strcpy(xmlFileName, "gScenario.xml");
      strcpy(directory, "config/xml/WAM");
      argP.getArgument("-f", xmlFileName, "RcsGraph's configuration file name");
      argP.getArgument("-dir", directory, "Configuration file directory");
      Rcs_addResourcePath(directory);

      RcsGraph* graph = RcsGraph_create(xmlFileName);
      RCHECK(graph);

      RcsJoint* jlJoint = NULL;
      MatNd* dH = MatNd_create(graph->dof, 1);

      // Constraining all joint except the first one
      RCSGRAPH_TRAVERSE_JOINTS(graph)
      {
        if (jlJoint==NULL)
        {
          jlJoint = JNT;
          JNT->constrained = false;
          JNT->q_min = -1.0;
          JNT->q0 = 1.0;
          JNT->q_max = 2.0;
          JNT->weightJL = 1.0;
        }
        else
        {
          JNT->constrained = true;
        }
      }

      RCHECK_MSG(jlJoint, "No joint for testing joint limits found in graph");
      RcsGraph_setState(graph, NULL, NULL);

      FILE* fd = fopen("jlcost.dat", "w+");
      RCHECK(fd);

      for (double q = jlJoint->q_min; q < jlJoint->q_max; q = q + 0.001)
      {
        //*jlJoint->q = q;
        MatNd_set(graph->q, jlJoint->jointIndex, 0, q);
        double c1 = RcsGraph_jointLimitCost(graph, RcsStateFull);
        double c2 = RcsGraph_jointLimitCostPlateau(graph, 0.5, RcsStateFull);
        double c3 = RcsGraph_jointLimitBorderCost(graph, 0.0, RcsStateFull);
        double c4 = RcsGraph_jointLimitBorderCost(graph, 0.5, RcsStateFull);
        double c5 = RcsGraph_jointLimitBorderCost(graph, 0.75, RcsStateFull);
        double c6 = RcsGraph_jointLimitBorderCost(graph, 0.99, RcsStateFull);
        fprintf(fd, "%f   %f   %f   %f   %f   %f   %f\n",
                q*180.0/M_PI, c1, c2, c3, c4, c5, c6);
      }

      fclose(fd);

      REXEC(1)
      {
        // Gradient at the boundary of the workspace
        MatNd_set(graph->q, jlJoint->jointIndex, 0, jlJoint->q_min);
        RcsGraph_jointLimitGradient(graph, dH, RcsStateFull);
        RLOGS(0, "Normal gradient at q_min: %f", dH->ele[jlJoint->jointIndex]);

        MatNd_set(graph->q, jlJoint->jointIndex, 0, jlJoint->q_min);
        RcsGraph_jointLimitBorderGradient(graph, dH, 0.0, RcsStateFull);
        RLOGS(0, "Border gradient at q_min: %f", dH->ele[jlJoint->jointIndex]);

        MatNd_set(graph->q, jlJoint->jointIndex, 0, jlJoint->q_max);
        RcsGraph_jointLimitGradient(graph, dH, RcsStateFull);
        RLOGS(0, "Normal gradient at q_max: %f", dH->ele[jlJoint->jointIndex]);

        MatNd_set(graph->q, jlJoint->jointIndex, 0, jlJoint->q_max);
        RcsGraph_jointLimitBorderGradient(graph, dH, 0.0, RcsStateFull);
        RLOGS(0, "Border gradient at q_max: %f", dH->ele[jlJoint->jointIndex]);
      }

      char postpro[2056];
      double qmin = jlJoint->q_min*180.0/M_PI;
      double q0 = jlJoint->q0*180.0/M_PI;
      double qmax = jlJoint->q_max*180.0/M_PI;
      double lowRange = q0 - qmin;
      double upRange = qmax - q0;
      sprintf(postpro,
              "set grid;\n"
              "set xrange [%f : %f]\n"
              "set yrange [0 : 0.6]\n"
              "set xtics (\"q_min\" %f, \"q0\" %f, \"q_max\" %f)\n"
              "plot \"jlcost.dat\" u 1:2 title \"RcsGraph_jointLimitCost()\" w l, "
              "\"jlcost.dat\" u 1:3 title \"RcsGraph_jointLimitCostPlateau(0.5)\" w l, "
              "\"jlcost.dat\" u 1:4 title \"RcsGraph_jointLimitBorderCost(freeRange=0)\" w l, "
              "\"jlcost.dat\" u 1:5 title \"RcsGraph_jointLimitBorderCost(freeRange=0.5)\" w l, "
              "\"jlcost.dat\" u 1:6 title \"RcsGraph_jointLimitBorderCost(freeRange=0.75)\" w l, "
              "\"jlcost.dat\" u 1:7 title \"RcsGraph_jointLimitBorderCost(freeRange=0.99)\" w l",
              1.25*qmin, 1.25*qmax,
              q0-lowRange, q0, q0+upRange);

      fd = fopen("postpro.gnu", "w+");
      RCHECK(fd);
      fprintf(fd, "%s", postpro);
      fflush(fd);

      int err = system("gnuplot -persist \"postpro.gnu\"");

      if (err == -1)
      {
        RMSG("Couldn't start gnuplot window");
      }


      RcsGraph_destroy(graph);
      fclose(fd);
      MatNd_destroy(dH);

      break;
    }



    // ==============================================================
    // Kinematics with velocities
    // ==============================================================
    case 5:
    {
      strcpy(xmlFileName, "gScenario.xml");
      strcpy(directory, "config/xml/WAM");
      argP.getArgument("-f", xmlFileName, "RcsGraph's configuration file name");
      argP.getArgument("-dir", directory, "Configuration file directory");
      Rcs_addResourcePath(directory);

      if (argP.hasArgument("-h"))
      {
        printf("\n\tMode %d options:\n\n"
               "\t-dir <graph-directory>\n"
               "\t-f <graph-file>\n\n",
               mode);
        break;
      }


      RcsGraph* graph = RcsGraph_create(xmlFileName);
      RCHECK(graph);

      RCSGRAPH_TRAVERSE_JOINTS(graph)
      {
        JNT->constrained = false;
      }
      RcsGraph_setState(graph, NULL, NULL);

      RCSGRAPH_TRAVERSE_JOINTS(graph)
      {
        RLOG(0, "joint %s: jntIndex = %d   jacobiIndex = %d",
             JNT->name, JNT->jointIndex, JNT->jacobiIndex);
      }
      RPAUSE();

      MatNd* q  = MatNd_create(graph->dof, 1);
      MatNd* q_dot = MatNd_create(graph->dof, 1);
      MatNd* JT = MatNd_create(3, graph->dof);
      MatNd* JR = MatNd_create(3, graph->dof);
      MatNd* x_dot = MatNd_create(3, 1);
      MatNd* om = MatNd_create(3, 1);

      while (runLoop)
      {
        MatNd_setRandom(q, -1.0, 1.0);
        MatNd_setRandom(q_dot, -1.0, 1.0);

        RcsGraph_setState(graph, q, q_dot);

        RCSGRAPH_TRAVERSE_JOINTS(graph)
        {
          RLOG(10, "joint %s: jntIndex = %d   jacobiIndex = %d",
               JNT->name, JNT->jointIndex, JNT->jacobiIndex);
        }

        RCHECK_MSG(graph->dof==graph->nJ, "dof=%d   nJ=%d",
                   graph->dof, graph->nJ);
        RLOG(0, "*** dof=%d   nJ=%d ***", graph->dof, graph->nJ);

        RCSGRAPH_TRAVERSE_BODIES(graph)
        {
          RcsGraph_bodyPointJacobian(graph, BODY, NULL, NULL, JT);
          RcsGraph_rotationJacobian(graph, BODY, NULL, JR);
          MatNd_mul(x_dot, JT, graph->q_dot);
          MatNd_mul(om, JR, graph->q_dot);

          MatNd x_dot_i = MatNd_fromPtr(3, 1, BODY->x_dot);
          MatNd om_i = MatNd_fromPtr(3, 1, BODY->omega);

          bool failure = false;

          if ((MatNd_sqrDistance(&x_dot_i, x_dot)>1.0e-8) ||
              (MatNd_sqrDistance(&om_i, om)>1.0e-8))
          {
            failure = true;
          }

          if (failure == true)
          {
            RLOG(0, "Body %s: x_dot = %.4f %.4f %.4f   om = %.4f %.4f %.4f",
                 BODY->name,
                 BODY->x_dot[0], BODY->x_dot[1], BODY->x_dot[2],
                 BODY->omega[0], BODY->omega[1], BODY->omega[2]);
            RLOG(0, "Parent is %s",
                 BODY->parent ? BODY->parent->name : "NULL");

            REXEC(3)
            {
              RcsBody_fprint(stderr, BODY);
            }

            REXEC(4)
            {
              if (failure==true)
              {
                RPAUSE();
              }
            }

            MatNd_printTwoArraysDiff(&x_dot_i, x_dot, 4);
            MatNd_printTwoArraysDiff(&om_i, om, 4);
          }
        }


      }

      RcsGraph_destroy(graph);
      MatNd_destroy(q);
      MatNd_destroy(q_dot);
      MatNd_destroy(JT);
      MatNd_destroy(JR);
      MatNd_destroy(x_dot);
      MatNd_destroy(om);
      break;
    }



    // ==============================================================
    // Check all graph files in directory
    // ==============================================================
    case 6:
    {
      bool success = true;
      char tag[256]="";
      int maxIter = 10;

      strcpy(xmlFileName, "gScenario.xml");
      strcpy(directory, "config/xml/DexBot");
      argP.getArgument("-f", xmlFileName, "Configuration file name");
      argP.getArgument("-dir", directory, "Configuration file directory");
      argP.getArgument("-maxIter", &maxIter, "Gradient test iterations");
      Rcs_addResourcePath(directory);

      bool silent = (RcsLogLevel < 3) ? true : false;

      std::list<std::string> files;
      if (argP.hasArgument("-f", "Graph file name. If none is given, "
                           "all graph files from the directory are "
                           "tested"))
      {
        std::string fileName;
        argP.getArgument("-f", &fileName);
        files.push_back(fileName);
      }
      else
      {
        files = getFilenamesInDirectory(directory, true, ".xml");
      }

      RLOGS(1, "Found %zu files in directory %s", files.size(), directory);

      std::list<std::string>::const_iterator it;
      for (it = files.begin(); it != files.end(); ++it)
      {
        const char* fileName = (*it).c_str();

        if (getXMLFileTag(fileName, tag)==true)
        {
          RLOGS(2, "File \"%s\" has tag \"%s\"", fileName, tag);

          if (STREQ(tag, "Graph"))
          {
            RcsGraph* graph = RcsGraph_create(fileName);
            int nErrors = 0;

            if (graph==NULL)
            {
              nErrors = 1;
            }
            else
            {
              nErrors = RcsGraph_check(graph);
            }

            if (nErrors>0)
            {
              success = false;
              RLOGS(2, "Graph check failed for %s", fileName);
            }
            else
            {
              RLOGS(2, "Graph check succeeded for %s", fileName);

              // For now, we remove all joint couplings
              size_t nCoupledJoints = 0;
              RCSGRAPH_TRAVERSE_JOINTS(graph)
              {
                JNT->coupledTo = NULL;
                if (JNT->coupledJointName != NULL)
                {
                  RFREE(JNT->coupledJointName);
                  JNT->coupledJointName = NULL;
                  nCoupledJoints++;
                }
              }

              if (nCoupledJoints>0)
              {
                RLOGS(2, "Removed %zu coupled joints for graph %s",
                      nCoupledJoints, fileName);
              }

              RcsGraph_makeJointsConsistent(graph);
              RcsGraph_setState(graph, NULL, NULL);

              for (int i=0; i<maxIter; i++)
              {
                MatNd* q_rnd = MatNd_clone(graph->q);
                MatNd_setRandom(q_rnd, -10.0, 10.0);

                success &= Rcs_gradientTestGraph(graph, q_rnd, !silent);

                RLOG(1, "Gradient test %d %s",
                     i, success ? "SUCCEEDED" : "FAILED");
              }

            }
            RcsGraph_destroy(graph);
            RLOGS(0, "%s when checking graph for file \"%s\" %d "
                  "times", success ? "SUCCESS" : "FAILURE",
                  fileName, maxIter);
          }

        }
        else
        {
          RLOGS(2, "File \"%s\" doesn't seem to be a graph file",
                fileName);
        }
      }

      RMSGS("Graph tests %s", (success==true) ? "SUCCEEDED" : "FAILED");
      break;
    }

    default:
    {
      RMSG("there is no mode %d", mode);
    }

  }   // switch(mode)



  // Clean up global stuff. From the libxml2 documentation:
  // WARNING: if your application is multithreaded or has plugin support
  // calling this may crash the application if another thread or a plugin is
  // still using libxml2. It's sometimes very hard to guess if libxml2 is in
  // use in the application, some libraries or plugins may use it without
  // notice. In case of doubt abstain from calling this function or do it just
  // before calling exit() to avoid leak reports from valgrind !
  xmlCleanupParser();

  fprintf(stderr, "Thanks for using the Rcs libraries\n");

  return 0;
}

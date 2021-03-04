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

#include "Rcs_graphicsUtils.h"

#include <Rcs_macros.h>
#include <Rcs_parser.h>
#include <Rcs_resourcePath.h>
#include <Rcs_utils.h>
#include <Rcs_math.h>
#include <RcsViewer.h>

#include <osg/LightModel>
#include <osg/PolygonMode>
#include <osg/ShapeDrawable>
#include <osgShadow/ShadowTexture>
#include <osgShadow/SoftShadowMap>
#include <osgShadow/ParallelSplitShadowMap>
#include <osg/Geode>
#include <osg/Geometry>
#include <osgUtil/LineSegmentIntersector>
#include <osgUtil/SmoothingVisitor>
#include <osgDB/ReadFile>

#include <iomanip>



// The below comment is for astyle, since otherwise it screws up the formatting
// *INDENT-OFF*
#define MULTI_LINE_STRING(a) #a
const char* materialDefs =
  MULTI_LINE_STRING(<Materials>
                    <MaterialDefinition Name="BRASS" Shininess="27.8974">
                    <Ambient R="0.329412" G="0.223529" B="0.027451" A="1.0"/>
                    <Diffuse R="0.780392" G="0.568627" B="0.113725" A="1.0"/>
                    <Specular R="0.992157" G="0.941176" B="0.807843" A="1.0"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="BRONZE" Shininess="25.6">
                    <Ambient R="0.2125" G="0.1275" B="0.054" A="1.0"/>
                    <Diffuse R="0.714" G="0.4284" B="0.18144" A="1.0"/>
                    <Specular R="0.393548" G="0.271906" B="0.166721" A="1.0"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="POLISHED_BRONZE" Shininess="76.8">
                    <Ambient R="0.25" G="0.148" B="0.06475" A="1.0"/>
                    <Diffuse R="0.4" G="0.2368" B="0.1036" A="1.0"/>
                    <Specular R="0.774597" G="0.458561" B="0.200621" A="1.0"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="CHROME" Shininess="76.8">
                    <Ambient R="0.25" G="0.25" B="0.25" A="1.0"/>
                    <Diffuse R="0.4" G="0.4" B="0.4" A="1.0"/>
                    <Specular R="0.774597" G="0.774597" B="0.8774597" A="1.0"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="COPPER" Shininess="12.8">
                    <Ambient R="0.19125" G="0.0735" B="0.0225" A="1.0"/>
                    <Diffuse R="0.7038" G="0.27048" B="0.0828" A="1.0"/>
                    <Specular R="0.256777" G="0.137622" B="0.086014" A="1.0"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="POLISHED_COPPER" Shininess="51.2">
                    <Ambient R="0.2295" G="0.08825" B="0.0275" A="1.0"/>
                    <Diffuse R="0.5508" G="0.2118" B="0.066" A="1.0"/>
                    <Specular R="0.580594" G="0.223257" B="0.0695701" A="1.0"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="GOLD" Shininess="51.2">
                    <Ambient R="0.24725" G="0.1995" B="0.0745" A="1.0"/>
                    <Diffuse R="0.75164" G="0.60648" B="0.22648" A="1.0"/>
                    <Specular R="0.628281" G="0.555802" B="0.366065" A="1.0"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="POLISHED_GOLD" Shininess="83.2">
                    <Ambient R="0.24725" G="0.2245" B="0.0645" A="1.0"/>
                    <Diffuse R="0.34615" G="0.3143" B="0.0903" A="1.0"/>
                    <Specular R="0.797357" G="0.723991" B="0.208006" A="1.0"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="PEWTER" Shininess="9.84615">
                    <Ambient R="0.105882" G="0.058824" B="0.113725" A="1.0"/>
                    <Diffuse R="0.427451" G="0.470588" B="0.541176" A="1.0"/>
                    <Specular R="0.333333" G="0.333333" B="0.521569" A="1.0"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="COPPER" Shininess="51.2">
                    <Ambient R="0.19225" G="0.19225" B="0.19225" A="1.0"/>
                    <Diffuse R="0.50754" G="0.50754" B="0.50754" A="1.0"/>
                    <Specular R="0.508273" G="0.508273" B="0.508273" A="1.0"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="POLISHED_SILVER" Shininess="89.6">
                    <Ambient R="0.23125" G="0.23125" B="0.023125" A="1.0"/>
                    <Diffuse R="0.2775" G="0.2775" B="0.2775" A="1.0"/>
                    <Specular R="0.773911" G="0.773911" B="0.773911" A="1.0"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="EMERALD" Shininess="76.8">
                    <Ambient R="0.0215" G="0.1745" B="0.0215" A="0.55"/>
                    <Diffuse R="0.07568" G="0.61424" B="0.07568" A="0.55"/>
                    <Specular R="0.633" G="0.727811" B="0.633" A="0.55"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="EMERALD_S" Shininess="76.8">
                    <Ambient R="0.0215" G="0.1745" B="0.0215" A="1"/>
                    <Diffuse R="0.07568" G="0.61424" B="0.07568" A="1"/>
                    <Specular R="0.633" G="0.727811" B="0.633" A="1"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="JADE" Shininess="12.8">
                    <Ambient R="0.135" G="0.2225" B="0.1575" A="0.95"/>
                    <Diffuse R="0.54" G="0.89" B="0.63" A="0.95"/>
                    <Specular R="0.316228" G="0.316228" B="0.316228" A="0.95"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="JADE_S" Shininess="12.8">
                    <Ambient R="0.135" G="0.2225" B="0.1575" A="1"/>
                    <Diffuse R="0.54" G="0.89" B="0.63" A="1"/>
                    <Specular R="0.316228" G="0.316228" B="0.316228" A="1"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="OBSIDIAN" Shininess="38.4">
                    <Ambient R="0.05375" G="0.05" B="0.06625" A="0.82"/>
                    <Diffuse R="0.18275" G="0.17" B="0.22525" A="0.82"/>
                    <Specular R="0.332741" G="0.328634" B="0.346435" A="0.82"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="PEARL" Shininess="11.264">
                    <Ambient R="0.25" G="0.20725" B="0.20725" A="0.922"/>
                    <Diffuse R="1.0" G="0.829" B="0.829" A="0.922"/>
                    <Specular R="0.296648" G="0.296648" B="0.296648" A="0.922"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="PEARL_S" Shininess="11.264">
                    <Ambient R="0.25" G="0.20725" B="0.20725" A="1"/>
                    <Diffuse R="1.0" G="0.829" B="0.829" A="1"/>
                    <Specular R="0.296648" G="0.296648" B="0.296648" A="1"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="RUBY" Shininess="76.8">
                    <Ambient R="0.1745" G="0.01175" B="0.01175" A="0.55"/>
                    <Diffuse R="0.61424" G="0.04136" B="0.04136" A="0.55"/>
                    <Specular R="0.727811" G="0.626959" B="0.626959" A="0.55"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="RUBY_S" Shininess="76.8">
                    <Ambient R="0.1745" G="0.01175" B="0.01175" A="1"/>
                    <Diffuse R="0.61424" G="0.04136" B="0.04136" A="1"/>
                    <Specular R="0.727811" G="0.626959" B="0.626959" A="1"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="TURQUOISE" Shininess="12.8">
                    <Ambient R="0.1" G="0.18725" B="0.1745" A="0.8"/>
                    <Diffuse R="0.396" G="0.74151" B="0.69102" A="0.8"/>
                    <Specular R="0.297254" G="0.30829" B="0.306678" A="0.8"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="BLACK_PLASTIC" Shininess="32.0">
                    <Ambient R="0.0" G="0.0" B="0.0" A="1.0"/>
                    <Diffuse R="0.01" G="0.01" B="0.01" A="1.0"/>
                    <Specular R="0.5" G="0.5" B="0.5" A="1.0"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="BLACK_RUBBER" Shininess="10.0">
                    <Ambient R="0.02" G="0.02" B="0.02" A="1.0"/>
                    <Diffuse R="0.01" G="0.01" B="0.01" A="1.0"/>
                    <Specular R="0.4" G="0.4" B="0.4" A="1.0"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="WHITE" Shininess="10.0">
                    <Ambient R="0.25" G="0.25" B="0.25" A="1.0"/>
                    <Diffuse R="1.0" G="1.0" B="1.0" A="1.0"/>
                    <Specular R="0.2" G="0.2" B="0.2" A="1.0"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="PUREWHITE" Shininess="10.0">
                    <Ambient R="1.0" G="1.0" B="1.0" A="1.0"/>
                    <Diffuse R="1.0" G="1.0" B="1.0" A="1.0"/>
                    <Specular R="0.2" G="0.2" B="0.2" A="1.0"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="LIGHTGRAY" Shininess="10.0">
                    <Ambient R="0.25" G="0.25" B="0.25" A="1.0"/>
                    <Diffuse R="0.9" G="0.9" B="0.9" A="1.0"/>
                    <Specular R="0.2" G="0.2" B="0.2" A="1.0"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="GRAY" Shininess="10.0">
                    <Ambient R="0.20" G="0.20" B="0.20" A="1.0"/>
                    <Diffuse R="0.7" G="0.7" B="0.7" A="1.0"/>
                    <Specular R="0.2" G="0.2" B="0.2" A="1.0"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="DARKGRAY" Shininess="10.0">
                    <Ambient R="0.10" G="0.10" B="0.10" A="1.0"/>
                    <Diffuse R="0.5" G="0.5" B="0.5" A="1.0"/>
                    <Specular R="0.1" G="0.1" B="0.1" A="1.0"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="RED" Shininess="100.0">
                    <Ambient R="0.2" G="0.0" B="0.0" A="1.0"/>
                    <Diffuse R="1.0" G="0.0" B="0.0" A="1.0"/>
                    <Specular R="0.2" G="0.2" B="0.2" A="1.0"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="DARKRED" Shininess="100.0">
                    <Ambient R="0.1" G="0.0" B="0.0" A="1.0"/>
                    <Diffuse R="0.8" G="0.0" B="0.0" A="1.0"/>
                    <Specular R="0.1" G="0.1" B="0.1" A="1.0"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="ORANGE" Shininess="100.0">
                    <Ambient R="0.2" G="0.1" B="0.1" A="1.0"/>
                    <Diffuse R="1.0" G="0.5" B="0.0" A="1.0"/>
                    <Specular R="0.2" G="0.2" B="0.2" A="1.0"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="GREEN" Shininess="100.0">
                    <Ambient R="0.0" G="0.2" B="0.0" A="1.0"/>
                    <Diffuse R="0.0" G="1.0" B="0.0" A="1.0"/>
                    <Specular R="0.2" G="0.2" B="0.2" A="1.0"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="BLUE" Shininess="100.0">
                    <Ambient R="0.0" G="0.0" B="0.2" A="1.0"/>
                    <Diffuse R="0.0" G="0.0" B="1.0" A="1.0"/>
                    <Specular R="0.2" G="0.2" B="0.2" A="1.0"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="LIGHTBLUE" Shininess="100.0">
                    <Ambient R="0.91" G="0.99" B="0.99" A="1.0"/>
                    <Diffuse R="0.0" G="0.0" B="0.1" A="1.0"/>
                    <Specular R="0.2" G="0.2" B="0.2" A="1.0"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="CYAN" Shininess="100.0">
                    <Ambient R="0.0" G="0.2" B="0.2" A="1.0"/>
                    <Diffuse R="0.0" G="1.0" B="1.0" A="1.0"/>
                    <Specular R="0.2" G="0.2" B="0.2" A="1.0"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="MAGENTA" Shininess="100.0">
                    <Ambient R="0.2" G="0.0" B="0.2" A="1.0"/>
                    <Diffuse R="1.0" G="0.0" B="1.0" A="1.0"/>
                    <Specular R="0.2" G="0.2" B="0.2" A="1.0"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="YELLOW" Shininess="100.0">
                    <Ambient R="0.2" G="0.2" B="0.0" A="1.0"/>
                    <Diffuse R="1.0" G="1.0" B="0.0" A="1.0"/>
                    <Specular R="0.2" G="0.2" B="0.2" A="1.0"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="LIGHTGRAY_TRANS" Shininess="10.0">
                    <Ambient R="0.25" G="0.25" B="0.25" A="0.3"/>
                    <Diffuse R="0.9" G="0.9" B="0.9" A="0.3"/>
                    <Specular R="0.2" G="0.2" B="0.2" A="0.3"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="RUBY_TRANS" Shininess="76.8">
                    <Ambient R="0.1745" G="0.01175" B="0.01175" A="0.25"/>
                    <Diffuse R="0.61424" G="0.04136" B="0.04136" A="0.25"/>
                    <Specular R="0.727811" G="0.626959" B="0.626959" A="0.25"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="EMERALD_TRANS" Shininess="76.8">
                    <Ambient R="0.0215" G="0.1745" B="0.0215" A="0.25"/>
                    <Diffuse R="0.07568" G="0.61424" B="0.07568" A="0.25"/>
                    <Specular R="0.633" G="0.727811" B="0.633" A="0.25"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="RED_TRANS" Shininess="70.0">
                    <Ambient R="0.2" G="0.0" B="0.0" A="0.3"/>
                    <Diffuse R="0.8" G="0.0" B="0.0" A="0.3"/>
                    <Specular R="0.2" G="0.2" B="0.2" A="0.3"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="DARKRED_TRANS" Shininess="70.0">
                    <Ambient R="0.2" G="0.0" B="0.0" A="0.6"/>
                    <Diffuse R="0.8" G="0.0" B="0.0" A="0.6"/>
                    <Specular R="0.2" G="0.2" B="0.2" A="0.6"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="GREEN_TRANS" Shininess="70.0">
                    <Ambient R="0.0" G="0.2" B="0.0" A="0.3"/>
                    <Diffuse R="0.0" G="0.8" B="0.0" A="0.3"/>
                    <Specular R="0.2" G="0.2" B="0.2" A="0.3"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="BLUE_TRANS" Shininess="70.0">
                    <Ambient R="0.0" G="0.0" B="0.2" A="0.3"/>
                    <Diffuse R="0.0" G="0.0" B="0.8" A="0.3"/>
                    <Specular R="0.2" G="0.2" B="0.2" A="0.3"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="YELLOW_TRANS" Shininess="100.0">
                    <Ambient R="0.2" G="0.2" B="0.0" A="0.3"/>
                    <Diffuse R="1.0" G="1.0" B="0.0" A="0.3"/>
                    <Specular R="0.2" G="0.2" B="0.2" A="0.3"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="VERY_TRANS" Shininess="10.0">
                    <Ambient R="0.25" G="0.25" B="0.25" A="0.15"/>
                    <Diffuse R="0.9" G="0.9" B="0.9" A="0.15"/>
                    <Specular R="0.2" G="0.2" B="0.2" A="0.15"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="DEFAULT" Shininess="10.0">
                    <Ambient R="0.20" G="0.20" B="0.20" A="1.0"/>
                    <Diffuse R="0.7" G="0.7" B="0.7" A="1.0"/>
                    <Specular R="0.2" G="0.2" B="0.2" A="1.0"/>
                    </MaterialDefinition>

                    <MaterialDefinition Name="GLASS" Shininess="96.0">
                    <Ambient R="0.0" G="0.0" B="0.0" A="0.4"/>
                    <Diffuse R="0.705882" G="0.804706" B="0.875294" A="0.7"/>
                    <Specular R="0.9" G="0.9" B="0.9" A="0.7"/>
                    </MaterialDefinition>

                    </Materials>);
// *INDENT-ON*

namespace Rcs
{

/*******************************************************************************
 *
 ******************************************************************************/
osg::Matrix viewMatrixFromHTr(const HTr* A_BI)
{
  osg::Matrix viewMatrix;

  if (A_BI==NULL)
  {
    viewMatrix.makeIdentity();
    return viewMatrix;
  }

  double n[3], u[3], v[3];

  // n is negative x-direction
  n[0] = -A_BI->rot[0][0];
  n[1] = -A_BI->rot[0][1];
  n[2] = -A_BI->rot[0][2];

  // u is negative y-direction
  u[0] = -A_BI->rot[1][0];
  u[1] = -A_BI->rot[1][1];
  u[2] = -A_BI->rot[1][2];

  // v is positive z-direction
  v[0] = A_BI->rot[2][0];
  v[1] = A_BI->rot[2][1];
  v[2] = A_BI->rot[2][2];

  // Origin
  const double* e = &A_BI->org[0];
  double a = -(u[0] * e[0] + u[1] * e[1] + u[2] * e[2]);
  double b = -(v[0] * e[0] + v[1] * e[1] + v[2] * e[2]);
  double c = -(n[0] * e[0] + n[1] * e[1] + n[2] * e[2]);

  viewMatrix.set(u[0], v[0], n[0], 0.0,
                 u[1], v[1], n[1], 0.0,
                 u[2], v[2], n[2], 0.0, a, b, c, 1.0);

  return viewMatrix;
}

/*******************************************************************************
 *
 ******************************************************************************/
void HTr_fromViewMatrix(const osg::Matrixd& viewMatrix, HTr* A_BI)
{
  osg::Vec3 eye, center, up;

  viewMatrix.getLookAt(eye, center, up);

  osg::Vec3 ex = center - eye;
  ex.normalize();

  A_BI->rot[0][0] = ex.x();
  A_BI->rot[0][1] = ex.y();
  A_BI->rot[0][2] = ex.z();

  osg::Vec3 ey = up ^ ex;
  ey.normalize();

  A_BI->rot[1][0] = ey.x();
  A_BI->rot[1][1] = ey.y();
  A_BI->rot[1][2] = ey.z();

  osg::Vec3 ez = ex ^ ey;

  A_BI->rot[2][0] = ez.x();
  A_BI->rot[2][1] = ez.y();
  A_BI->rot[2][2] = ez.z();

  A_BI->org[0] = eye.x();
  A_BI->org[1] = eye.y();
  A_BI->org[2] = eye.z();
}

/*******************************************************************************
 *
 ******************************************************************************/
void HTr_fromMatrix(const osg::Matrixd& viewMatrix, HTr* A_BI)
{
  osg::Matrixd::value_type* rm = (osg::Matrixd::value_type*) viewMatrix.ptr();

  A_BI->rot[0][0] = rm[0];
  A_BI->rot[0][1] = rm[1];
  A_BI->rot[0][2] = rm[2];

  A_BI->rot[1][0] = rm[4];
  A_BI->rot[1][1] = rm[5];
  A_BI->rot[1][2] = rm[6];

  A_BI->rot[2][0] = rm[8];
  A_BI->rot[2][1] = rm[9];
  A_BI->rot[2][2] = rm[10];

  A_BI->org[0] = rm[12];
  A_BI->org[1] = rm[13];
  A_BI->org[2] = rm[14];
}

/*******************************************************************************
 *
 ******************************************************************************/
double distanceToCamera(const osg::Matrixd& viewMatrix, const double I_pt[3])
{
  double k_pt[3];
  HTr A_camI;
  HTr_fromViewMatrix(viewMatrix, &A_camI);

  const double* I_cam = A_camI.org;
  double I_cam_pt[3];

  Vec3d_sub(I_cam_pt, I_pt, I_cam);
  Vec3d_rotate(k_pt, A_camI.rot, I_cam_pt);

  return k_pt[0];
}

/*******************************************************************************
 *
 ******************************************************************************/
osg::Quat QuatFromHTr(const HTr* A_BI)
{
  osg::Quat qA;
  qA.set(osg::Matrix(A_BI->rot[0][0], A_BI->rot[0][1], A_BI->rot[0][2],
                     A_BI->org[0], A_BI->rot[1][0], A_BI->rot[1][1],
                     A_BI->rot[1][2], A_BI->org[1], A_BI->rot[2][0],
                     A_BI->rot[2][1], A_BI->rot[2][2], A_BI->org[2],
                     0.0, 0.0, 0.0, 1.0));
  return qA;
}

/*******************************************************************************
 *
 ******************************************************************************/
osg::Vec4 colorFromString(const char* c)
{
  if (c==NULL)
  {
    RLOG(4, "Color is NULL - returning white");
    return osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f);
  }

  if (STRCASEEQ(c, "RED"))
  {
    return osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f);
  }
  else if (STRCASEEQ(c, "WHITE"))
  {
    return osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f);
  }
  else if (STRCASEEQ(c, "BLACK"))
  {
    return osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f);
  }
  else if (STRCASEEQ(c, "GREEN"))
  {
    return osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f);
  }
  else if (STRCASEEQ(c, "BLUE"))
  {
    return osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f);
  }
  else if (STRCASEEQ(c, "RUBY"))
  {
    return osg::Vec4(0.614, 0.041, 0.041, 1.0);
  }
  else if (STRCASEEQ(c, "YELLOW"))
  {
    return osg::Vec4(1.0, 1.0, 0.0, 1.0f);
  }
  else if (STRCASEEQ(c, "BRASS"))
  {
    return osg::Vec4(0.7803, 0.5686, 0.1137, 1.0);
  }
  else if (STRCASEEQ(c, "PEWTER"))
  {
    return osg::Vec4(0.4274, 0.4705, 0.5411, 1.0);
  }
  else if (STRCASEEQ(c, "BRONZE"))
  {
    return osg::Vec4(0.714, 0.4284, 0.18144, 1.0);
  }
  else if (STRCASEEQ(c, "EMERALD"))
  {
    return osg::Vec4(0.075, 0.6142, 0.07568, 1.0);
  }
  else if (STRCASEEQ(c, "LIGHT_GRAYISH_BLUE"))
  {
    return osg::Vec4(0.8, 0.8, 0.95, 1.0);
  }
  else if (STRCASEEQ(c, "LIGHT_GRAYISH_YELLOW"))
  {
    return osg::Vec4(0.845, 0.845, 0.797, 1.0);
  }
  else if (STRCASEEQ(c, "LIGHT_GRAYISH_GREEN"))
  {
    return osg::Vec4(0.797, 0.845, 0.797, 1.0);
  }
  else if (STRCASEEQ(c, "RED_TRANS"))
  {
    return osg::Vec4(0.8f, 0.0f, 0.0f, 0.3f);
  }
  else if (STRCASEEQ(c, "GREEN_TRANS"))
  {
    return osg::Vec4(0.0f, 0.8f, 0.0f, 0.3f);
  }
  else if (STRCASEEQ(c, "BLUE_TRANS"))
  {
    return osg::Vec4(0.0f, 0.0f, 0.8f, 0.3f);
  }
  else if ((std::string(c).length() == 7 || std::string(c).length() == 9) &&
           std::string(c)[0] == '#')
  {
    double r = (double) hexStringToUInt(std::string(c).substr(1, 2)) / 255;
    double g = (double) hexStringToUInt(std::string(c).substr(3, 2)) / 255;
    double b = (double) hexStringToUInt(std::string(c).substr(5, 2)) / 255;
    double a = (std::string(c).length() == 9) ?
               ((double) hexStringToUInt(std::string(c).substr(7, 2)) / 255) : 1.0;
    return osg::Vec4(r, g, b, a);
  }
  else
  {
    RLOG(4, "Unknown color \"%s\"", c);
    return osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f);
  }
}

/*******************************************************************************
 * Computes a line segment corresponding to the current line of sight,
 * clipped by the near and far plane of the view frustrum. The line
 * is represented in world coordinates.
 ******************************************************************************/
osg::ref_ptr<osg::LineSegment> lineOfSightSegment(const osg::Matrix& projMat,
                                                  const osg::Matrix& viewMat,
                                                  float x, float y)
{
  osg::Matrix matrix = viewMat * projMat;
  osg::Matrix inverseVP;
  inverseVP.invert(matrix);

  osg::Vec3 nearPoint = osg::Vec3(x, y, -1.0f) * inverseVP;
  osg::Vec3 farPoint = osg::Vec3(x, y, 1.0f) * inverseVP;

  osg::ref_ptr<osg::LineSegment> lineOfSight =
    new osg::LineSegment(nearPoint, farPoint);

  return lineOfSight;
}

/*******************************************************************************
 * Compute the mouse tip point in world coordinates. Values x and
 * y describe the mouse tip in normalized screen coordinates
 * [-1 : 1]. The line point corresponds to the eye point. The
 * line direction is determined with the normalized xy-projection
 * of the projection and view matrices and points towards the
 * screen coordinates (x y). The plane normal points into
 * negative view plane normal direction. The plane point is the
 * objects coordinates when initially clicked.
 ******************************************************************************/
void getMouseTip(const osg::Matrix& vm, const osg::Matrix& pm, float x,
                 float y, const double planePt[3], double tip[3])
{
  double linePt[3], lineDir[3], planeNorm[3];

  // The line is directed towards the screen coordinates (x y)
  osg::ref_ptr<osg::LineSegment> line = lineOfSightSegment(pm, vm, x, y);
  osg::Vec3 dir = line.get()->end() - line.get()->start();
  lineDir[0] = dir.x();
  lineDir[1] = dir.y();
  lineDir[2] = dir.z();

  // The line point coincides with the eye position of the observer
  osg::Vec3 eye, center, up;
  vm.getLookAt(eye, center, up);
  linePt[0] = eye.x();
  linePt[1] = eye.y();
  linePt[2] = eye.z();

  // The plane normal points in the negative view plane direction (towards
  // the observer)
  HTr A_camI;
  HTr_fromViewMatrix(vm, &A_camI);
  Vec3d_constMul(planeNorm, A_camI.rot[0], -1.0);

  // Compute the plane-line intersection: is the force point under the mouse
  Vec3d_computePlaneLineIntersection(tip, linePt, lineDir, planePt, planeNorm);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool getMouseTip(const osgGA::GUIEventAdapter& ea,
                 osgGA::GUIActionAdapter& aa,
                 const double I_planePt[3],
                 double I_tip[3])
{
  bool success = true;

  // Mouse tip point for drag force:
  float x = ea.getXnormalized();
  float y = ea.getYnormalized();

  // Get mouse tip in world coordinates
  osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
  if (viewer != NULL)
  {
    Rcs::getMouseTip(viewer->getCamera()->getViewMatrix(),
                     viewer->getCamera()->getProjectionMatrix(),
                     x, y, I_planePt, I_tip);
  }
  else
  {
    RLOG(1, "Couldn't cast to osg::Viewer in mouse tip function");
    success = false;
  }

  return success;
}

/*******************************************************************************
 * Sets the OSG coordinate frame to right handed, x-forward, z-upward
 * convention
 ******************************************************************************/
void setOSGCoordinateFrameToRoboticsConvention()
{
  // Rotate loaded file nodes to standard coordinate conventions
  // (z: up, x: forward)
  osgDB::ReaderWriter::Options* options = new osgDB::ReaderWriter::Options;
  options->setOptionString("noRotation");
  osgDB::Registry::instance()->setOptions(options);
}

/*******************************************************************************
 * Sets a node's material, loads it from file if necessary
 ******************************************************************************/
bool setNodeMaterial(const std::string& matString, osg::Node* node,
                     double alpha)
{
  if (node == NULL)
  {
    RLOG(4, "osg::Node is NULL - doing nothing");
    return false;
  }

  RcsMaterialData* matDataPtr = getMaterial(matString);

  if (matDataPtr == NULL)
  {
    RLOG(4, "Couldn't set material to \"%s\"", matString.c_str());
    return false;
  }

  // set material from matDataPtr, if alpha is between 0.0 and 1.0 then
  // overwrite the materials alpha
  osg::ref_ptr<osg::Material> material = new osg::Material;

  material->setAmbient(osg::Material::FRONT_AND_BACK, matDataPtr->amb);
  material->setDiffuse(osg::Material::FRONT_AND_BACK, matDataPtr->diff);
  material->setSpecular(osg::Material::FRONT_AND_BACK, matDataPtr->spec);
  material->setShininess(osg::Material::FRONT_AND_BACK, matDataPtr->shininess);

  if ((alpha >= 0.0) && (alpha <= 1.0))
  {
    material->setAlpha(osg::Material::FRONT_AND_BACK, alpha);
  }
  else
  {
    RLOG(5, "Material %s: Ignoring alpha value %.16f, must be [0 ... 1]",
         matString.c_str(), alpha);
  }

  // Assign material through state set
  osg::ref_ptr<osg::StateSet> stateset = node->getOrCreateStateSet();

  if (alpha < 1.0)
  {
    stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
  }

  stateset->setMode(GL_BLEND,
                    osg::StateAttribute::OVERRIDE |
                    osg::StateAttribute::ON);

  stateset->setMode(GL_LIGHTING,
                    osg::StateAttribute::OVERRIDE |
                    osg::StateAttribute::ON);

  stateset->setMode(GL_ALPHA_TEST,
                    osg::StateAttribute::OVERRIDE |
                    osg::StateAttribute::ON);

  // removes artifacts inside of transparent objects
  // Disabled since otherwise no backfaces are visible
  //   stateset->setMode(GL_CULL_FACE, osg::StateAttribute::ON);

  stateset->setAttributeAndModes(material.get(),
                                 osg::StateAttribute::OVERRIDE |
                                 osg::StateAttribute::ON);

  // Makes material scale-invariant
  stateset->setMode(GL_RESCALE_NORMAL,
                    // osg::StateAttribute::OVERRIDE |
                    osg::StateAttribute::ON);

  // if we set a material which is translucent then we don't cast shadows
  if (material->getAmbient(osg::Material::FRONT).a() <
      RCS_VIEWER_ALPHA_CAST_SHADOW_THRESHOLD)
  {
    node->setNodeMask(node->getNodeMask() & ~CastsShadowTraversalMask);
  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool createMaterialFromColorString(const std::string& matString,
                                   double amb[4], double diff[4],
                                   double spec[4], double* sh)
{
  if (matString.length() != 7 && matString.length() != 9)
  {
    RLOG(0, "Color string has to be of form #RRGGBB or #RRGGBBAA, you provided"
         " \"%s\" (len=%zu (should be 7 or 9))",
         matString.c_str(), matString.length());
    return false;
  }

  double r = (double) hexStringToUInt(matString.substr(1, 2)) / 255.0;
  double g = (double) hexStringToUInt(matString.substr(3, 2)) / 255.0;
  double b = (double) hexStringToUInt(matString.substr(5, 2)) / 255.0;
  double a = (matString.length() == 9) ?
             ((double) hexStringToUInt(matString.substr(7, 2)) / 255.0) : 1.0;

  amb[0] = 0.2 * r;
  amb[1] = 0.2 * g;
  amb[2] = 0.2 * b;
  amb[3] = a;

  diff[0] = r;
  diff[1] = g;
  diff[2] = b;
  diff[3] = a;

  spec[0] = 0.2;
  spec[1] = 0.2;
  spec[2] = 0.2;
  spec[3] = a;

  *sh = 100.0;

  RLOG(5, "Created color %s", matString.c_str());

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
unsigned int hexStringToUInt(const std::string& str)
{
  unsigned int x;
  std::stringstream ss;
  ss << std::hex << str;
  ss >> x;
  return x;
}

/*******************************************************************************
 *
 ******************************************************************************/
std::string createColorString(unsigned int r,
                              unsigned int g,
                              unsigned int b,
                              unsigned int a)
{
  std::stringstream ss;
  ss << "#";
  ss << std::hex << std::setw(2) << std::setfill('0') << r;
  ss << std::hex << std::setw(2) << std::setfill('0') << g;
  ss << std::hex << std::setw(2) << std::setfill('0') << b;
  ss << std::hex << std::setw(2) << std::setfill('0') << a;
  return ss.str();
}

/*******************************************************************************
 *
 ******************************************************************************/
void colorFromIndex(double color[3], unsigned int index, double saturation,
                    double value)
{
  // average color distance, to get distinct colors
  static const double color_delta = 31;
  double hue  = fmod(index * color_delta, 360.0);

  hsv2rgb(color[0], color[1], color[2], hue, saturation, value);
}

/*******************************************************************************
 *
 ******************************************************************************/
void hsv2rgb(double& r, double& g, double& b, double h, double s, double v)
{
  if (s == 0.0)
  {
    r = g = b = v;
    return;
  }

  double a = h / 60.0;
  int i = floor(a);
  double f = a - i;
  double p = v * (1.0 - s);
  double q = v * (1.0 - s * f);
  double t = v * (1.0 - s * (1.0 - f));

  switch (i)
  {
    case 0:
      r = v;
      g = t;
      b = p;
      break;
    case 1:
      r = q;
      g = v;
      b = p;
      break;
    case 2:
      r = p;
      g = v;
      b = t;
      break;
    case 3:
      r = p;
      g = q;
      b = v;
      break;
    case 4:
      r = t;
      g = p;
      b = v;
      break;
    default:
      r = v;
      g = p;
      b = q;
      break;
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void rgb2hsv(double& h, double& s, double& v, double r, double g, double b)
{
  double c_max = std::max(std::max(r, g), b);
  double c_min = std::min(std::min(r, g), b);
  double delta = c_max - c_min;

  v = c_max;

  if (c_max == 0.0)
  {
    h = 0.0;
    s = 0.0;
    return;
  }

  if (delta == 0.0)
  {
    h = 0.0;
  }
  else if (c_max == r)
  {
    h = 60.0 * ((g - b) / delta);
  }
  else if (c_max == g)
  {
    h = 60.0 * ((b - r) / delta + 2.0);
  }
  else if (c_max == b)
  {
    h = 60.0 * ((r - g) / delta + 4.0);
  }

  if (h < 0.0)
  {
    h += 360.0;
  }

  s = delta / c_max;
}

/*******************************************************************************
 * matlab jet color palette is a linear interpolation:
 * #7F0000 -> #FF7F00 -> #7FFF7F -> #007FFF -> #00007F
 ******************************************************************************/
void jetColorPalette(double color[3], double grayValue)
{
  static double jet[] = {0.5, 0.0, 0.0,
                         1.0, 0.5, 0.0,
                         0.5, 1.0, 0.5,
                         0.0, 0.5, 1.0,
                         0.0, 0.0, 0.5
                        };

  MatNd jetMat = MatNd_fromPtr(5, 3, jet);
  MatNd colorMat = MatNd_fromPtr(1, 3, color);

  MatNd_rowLerp(&colorMat, &jetMat, grayValue);
}

/*******************************************************************************
 * Reads material properties from a file and copies them in the fields.
 ******************************************************************************/
static bool getMaterialFromXmlNode(xmlNodePtr root, const char* material,
                                   double ambient[4], double diffuse[4],
                                   double specular[4], double* shininess)
{
  // Descend one level and search for "MaterialDefinition" node
  xmlNodePtr node = root->children;

  while (node)
  {
    // MaterialDefinition node
    if (isXMLNodeName(node, "MaterialDefinition"))
    {
      char materialName[256] = "";
      getXMLNodePropertyStringN(node, "Name", materialName, 256);

      if (!STREQ(materialName, material))
      {
        node = node->next;
        continue;
      }

      // From here, we have found the material
      getXMLNodePropertyDouble(node, "Shininess", shininess);

      xmlNodePtr child = node->children;

      while (child)
      {
        if (isXMLNodeName(child, "Ambient"))
        {
          getXMLNodePropertyDouble(child, "R", &ambient[0]);
          getXMLNodePropertyDouble(child, "G", &ambient[1]);
          getXMLNodePropertyDouble(child, "B", &ambient[2]);
          getXMLNodePropertyDouble(child, "A", &ambient[3]);
        }

        if (isXMLNodeName(child, "Diffuse"))
        {
          getXMLNodePropertyDouble(child, "R", &diffuse[0]);
          getXMLNodePropertyDouble(child, "G", &diffuse[1]);
          getXMLNodePropertyDouble(child, "B", &diffuse[2]);
          getXMLNodePropertyDouble(child, "A", &diffuse[3]);
        }

        if (isXMLNodeName(child, "Specular"))
        {
          getXMLNodePropertyDouble(child, "R", &specular[0]);
          getXMLNodePropertyDouble(child, "G", &specular[1]);
          getXMLNodePropertyDouble(child, "B", &specular[2]);
          getXMLNodePropertyDouble(child, "A", &specular[3]);
        }

        child = child->next;
      }

      return true;

    } // MaterialDefinition  node

    node = node->next;
  } // while(node)

  return false;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool getMaterialFromFile(const char* materialFile, const char* material,
                         double amb[4], double diff[4], double spec[4],
                         double* sh)
{
  xmlDocPtr doc;
  xmlNodePtr node;

  RCHECK(materialFile);
  RCHECK(material);

  if (!File_exists(materialFile))
  {
    RLOG(1, "Material file \"%s\" not found", materialFile);
    return false;
  }

  node = parseXMLFile(materialFile, "Materials", &doc);

  if (node==NULL)
  {
    RLOG(1, "XML parsing of material file \"%s\" failed", materialFile);
    return false;
  }

  bool success = getMaterialFromXmlNode(node, material,
                                        amb, diff, spec, sh);

  xmlFreeDoc(doc);

  return success;
}

/*******************************************************************************
 * Reads material properties from a character array copies them in the fields.
 ******************************************************************************/
static bool getMaterialFromBuffer(const char* buffer, const char* material,
                                  double amb[4], double diff[4], double spec[4],
                                  double* sh)
{
  if (buffer==NULL)
  {
    RLOG(4, "Couldn't load material from buffer: buffer is NULL");
    return false;
  }

  xmlDocPtr doc;
  xmlNodePtr node = parseXMLMemory(buffer, strlen(buffer), &doc);

  if (node==NULL)
  {
    RLOG(4, "Couldn't parse memory buffer \"%s\"", buffer);
    return false;
  }

  bool success = getMaterialFromXmlNode(node, material, amb, diff, spec, sh);

  xmlFreeDoc(doc);

  return success;
}

/*******************************************************************************
 *
 ******************************************************************************/
static bool getMaterialFromColorFile(const std::string& matString,
                                     double amb[4], double diff[4],
                                     double spec[4], double* sh)

{
  // We do this only the first time the function is called.
  static char matFile[256];
  static bool matFileFound = Rcs_getAbsoluteFileName("colors.xml", matFile);

  if (matFileFound == false)
  {
    matFileFound = Rcs_getAbsoluteFileName("colors/colors.xml", matFile);
  }

  if (matFileFound == false)
  {
    RLOG(4, "Material file \"%s\" not found!", matString.c_str());
    return false;
  }

  return getMaterialFromFile(matFile, matString.c_str(), amb, diff, spec, sh);
}

/*******************************************************************************
 *
 ******************************************************************************/
RcsMaterialData* getMaterial(const std::string& matString_)
{
  // Little map that stores name-pointer pairs of materials files. If a material
  // has already been loaded, we look up its pointer from the map. Otherwise, we
  // load the material and store its pointer in the map.
  static std::map<std::string, RcsMaterialData> matMap;
  static OpenThreads::Mutex matMtx;
  RcsMaterialData* res = NULL;
  std::string matString = matString_;

  matMtx.lock();
  std::map<std::string, RcsMaterialData>::iterator it = matMap.find(matString);

  if (it!=matMap.end())
  {
    NLOG(1, "Found color %s from fast internal map", matString.c_str());
    res = &it->second;
  }
  matMtx.unlock();

  if (res != NULL)
  {
    return res;
  }

  RcsMaterialData matData;
  bool success = false;

  // Create random color
  if (STRCASEEQ(matString_.c_str(), "Random"))
  {
    int rr = Math_getRandomInteger(0, 255);
    int gg = Math_getRandomInteger(0, 255);
    int bb = Math_getRandomInteger(0, 255);
    char rndColor[16];
    snprintf(rndColor, 16, "#%02x%02x%02xff", rr, gg, bb);
    matString = std::string(rndColor);
  }

  // Check if string starts with #
  if (matString.compare(0, 1, "#") == 0)
  {
    success = createMaterialFromColorString(matString,
                                            matData.amb.ptr(),
                                            matData.diff.ptr(),
                                            matData.spec.ptr(),
                                            &matData.shininess);
  }
  else
  {
    success = getMaterialFromBuffer(materialDefs, matString.c_str(),
                                    matData.amb.ptr(),
                                    matData.diff.ptr(),
                                    matData.spec.ptr(),
                                    &matData.shininess);
  }

  if (success==false)
  {
    success = getMaterialFromColorFile(matString,
                                       matData.amb.ptr(),
                                       matData.diff.ptr(),
                                       matData.spec.ptr(),
                                       &matData.shininess);
  }

  if (success==true)
  {
    matMtx.lock();
    matMap[matString] = matData;
    res = &matMap[matString];
    matMtx.unlock();
    return res;
  }



  RLOG(4, "Loading material %s FAILED", matString.c_str());
  return NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
void getGeodes(osg::Node* node, std::vector<osg::Geode*>& geodes)
{
  osg::Geode* geode = dynamic_cast<osg::Geode*>(node);

  if (geode)
  {
    geodes.push_back(geode);
  }
  else
  {
    osg::Group* gp = dynamic_cast<osg::Group*>(node);
    if (gp)
    {
      for (unsigned int ic = 0; ic < gp->getNumChildren(); ic++)
      {
        getGeodes(gp->getChild(ic), geodes);
      }
    }
  }
}

class NodeFinder : public osg::NodeVisitor
{
public:
  NodeFinder(std::string nodeName) :
    osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
    searchName(nodeName)
  {
  }

  virtual void apply(osg::Node& tNnode)
  {
    if (searchName == tNnode.getName())
    {
      node = &tNnode;
    }

    // Keep traversing the rest of the scene graph.
    traverse(tNnode);
  }

  osg::Node* getNode()
  {
    return node.get();
  }

protected:
  std::string searchName;
  osg::ref_ptr<osg::Node> node;
};

osg::Node* findNamedNodeRecursive(osg::Node* root, std::string nodeName)
{
  NodeFinder nf(nodeName);
  root->accept(nf);
  return nf.getNode();
}

/*******************************************************************************
 *
 ******************************************************************************/
osg::Geometry* createGeometryFromMesh(const RcsMeshData* mesh)
{
  osg::Geometry* geometry = new osg::Geometry;
  createGeometryFromMesh(geometry, mesh);
  return geometry;
}

/*******************************************************************************
 *
 ******************************************************************************/
void createGeometryFromMesh(osg::Geometry* geometry, const RcsMeshData* mesh)
{
  double* meshNormals = RcsMesh_createNormalArray(mesh);

  osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
  osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
  const double* v1, *v2, *v3, *n1, *n2, *n3;

  // Since the mesh is represented by face lists, we need to "linearize"
  // the arrays here during construction.
  for (size_t i=0; i<mesh->nFaces; ++i)
  {
    unsigned int fidx0 = 3*mesh->faces[i*3+0];
    unsigned int fidx1 = 3*mesh->faces[i*3+1];
    unsigned int fidx2 = 3*mesh->faces[i*3+2];

    RCHECK_MSG(fidx0<3*mesh->nVertices, "%d %d", fidx0, 3*mesh->nVertices);
    RCHECK_MSG(fidx1<3*mesh->nVertices, "%d %d", fidx1, 3*mesh->nVertices);
    RCHECK_MSG(fidx2<3*mesh->nVertices, "%d %d", fidx2, 3*mesh->nVertices);

    v1 = &mesh->vertices[3*mesh->faces[i*3+0]];
    v2 = &mesh->vertices[3*mesh->faces[i*3+1]];
    v3 = &mesh->vertices[3*mesh->faces[i*3+2]];
    vertices->push_back(osg::Vec3(v1[0], v1[1], v1[2]));
    vertices->push_back(osg::Vec3(v2[0], v2[1], v2[2]));
    vertices->push_back(osg::Vec3(v3[0], v3[1], v3[2]));

    n1 = &meshNormals[3*mesh->faces[i*3+0]];
    n2 = &meshNormals[3*mesh->faces[i*3+1]];
    n3 = &meshNormals[3*mesh->faces[i*3+2]];
    normals->push_back(osg::Vec3(n1[0], n1[1], n1[2]));
    normals->push_back(osg::Vec3(n2[0], n2[1], n2[2]));
    normals->push_back(osg::Vec3(n3[0], n3[1], n3[2]));
  }

  // Prepare geometry node
  geometry->setVertexArray(vertices);
  geometry->setNormalArray(normals);
  geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
  geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0,
                                                vertices->size()));

  RFREE(meshNormals);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool updateGeometryFromMesh(osg::Geometry* geometry, const RcsMeshData* mesh)
{
  osg::Vec3Array* v = static_cast<osg::Vec3Array*>(geometry->getVertexArray());
  const double* v1, *v2, *v3;
  size_t idx = 0;

  // We assume that the vertex normals remain unchanged and skip them here.
  for (unsigned int i = 0; i < mesh->nFaces; i++)
  {
    const unsigned int fidx0 = 3*mesh->faces[i*3+0];
    const unsigned int fidx1 = 3*mesh->faces[i*3+1];
    const unsigned int fidx2 = 3*mesh->faces[i*3+2];

    RCHECK_MSG(fidx0<3*mesh->nVertices, "%d %d", fidx0, 3*mesh->nVertices);
    RCHECK_MSG(fidx1<3*mesh->nVertices, "%d %d", fidx1, 3*mesh->nVertices);
    RCHECK_MSG(fidx2<3*mesh->nVertices, "%d %d", fidx2, 3*mesh->nVertices);

    v1 = &mesh->vertices[fidx0];
    v2 = &mesh->vertices[fidx1];
    v3 = &mesh->vertices[fidx2];
    (*v)[idx++] = (osg::Vec3(v1[0], v1[1], v1[2]));
    (*v)[idx++] = (osg::Vec3(v2[0], v2[1], v2[2]));
    (*v)[idx++] = (osg::Vec3(v3[0], v3[1], v3[2]));
  }

  return true;
}





bool updateGeometryFromMesh2(osg::Geometry* geometry, const RcsMeshData* mesh)
{
  osg::Vec3Array* v = static_cast<osg::Vec3Array*>(geometry->getVertexArray());

  //v->resize(mesh->nVertices);

  for (unsigned int i = 0; i < mesh->nVertices; i++)
  {
    const double* vi = &mesh->vertices[i*3];
    (*v)[i].set(vi[0], vi[1], vi[2]);
  }

  geometry->dirtyBound();

  return true;
}

osg::Geometry* createGeometryFromMesh2(const RcsMeshData* mesh)
{
  osg::Geometry* geometry = new osg::Geometry;
  createGeometryFromMesh2(geometry, mesh);
  return geometry;
}


void createGeometryFromMesh2(osg::Geometry* geometry, const RcsMeshData* mesh)
{
  osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(mesh->nVertices);

  for (unsigned int i = 0; i < mesh->nVertices; i++)
  {
    const double* vi = &mesh->vertices[i * 3];
    (*vertices)[i].set(vi[0], vi[1], vi[2]);
  }

  osg::ref_ptr<osg::DrawElementsUInt> indices = new osg::DrawElementsUInt(GL_TRIANGLES, 3*mesh->nFaces);

  for (unsigned int i = 0; i < 3*mesh->nFaces; i++)
  {
    (*indices)[i] = mesh->faces[i];
  }

  geometry->setVertexArray(vertices.get());
  geometry->addPrimitiveSet(indices.get());

#if 1
  osgUtil::SmoothingVisitor::smooth(*geometry, M_PI_4);
#else
  double* meshNormals = RcsMesh_createNormalArray(mesh);
  osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
  const double* n1, *n2, *n3;

  // Since the mesh is represented by face lists, we need to "linearize"
  // the arrays here during construction.
  for (size_t i=0; i<mesh->nFaces; ++i)
  {
    unsigned int fidx0 = 3*mesh->faces[i*3+0];
    unsigned int fidx1 = 3*mesh->faces[i*3+1];
    unsigned int fidx2 = 3*mesh->faces[i*3+2];

    n1 = &meshNormals[3*mesh->faces[i*3+0]];
    n2 = &meshNormals[3*mesh->faces[i*3+1]];
    n3 = &meshNormals[3*mesh->faces[i*3+2]];
    normals->push_back(osg::Vec3(n1[0], n1[1], n1[2]));
    normals->push_back(osg::Vec3(n2[0], n2[1], n2[2]));
    normals->push_back(osg::Vec3(n3[0], n3[1], n3[2]));
  }
  geometry->setNormalArray(normals);
  geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
#endif
}





} // namespace Rcs

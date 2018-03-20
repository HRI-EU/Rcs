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

#ifndef RCS_GRAPHICSUTILS_H
#define RCS_GRAPHICSUTILS_H

#include <Rcs_HTr.h>

#include <osg/PositionAttitudeTransform>
#include <osg/LightSource>
#include <osgShadow/ShadowedScene>
#include <osg/LineSegment>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgShadow/ShadowMap>


// cast shadows only if alpha is above 0.8
#define RCS_VIEWER_ALPHA_CAST_SHADOW_THRESHOLD (0.8)


/*! \ingroup RcsGraphics
 *  \defgroup RcsGraphicsUtilsFunctions RcsViewer utility functions
 */
namespace Rcs
{

typedef struct
{
  osg::Vec4d amb;
  osg::Vec4d diff;
  osg::Vec4d spec;
  double shininess;

} RcsMaterialData;





/*! \ingroup RcsGraphicsUtilsFunctions
 *  \brief Creates the view matrix in Rcs xyz conventions according to a
 *         given HTr transformation.
 */
osg::Matrix viewMatrixFromHTr(const HTr* A_KI);

/*! \ingroup RcsGraphicsUtilsFunctions
 *  \brief Returns an osg quaternion holding the transformation of the HTr A.
 */
osg::Quat QuatFromHTr(const HTr* A);

/*! \ingroup RcsGraphicsUtilsFunctions
 *  \brief Returns the ambient portion of the color given by string color.
 *         The following colors are defined (case insensitive):
 *         - RED
 *         - WHITE
 *         - BLACK
 *         - GREEN
 *         - BLUE
 *         - RUBY
 *         - YELLOW
 *         - BRASS
 *         - PEWTER
 *         - BRONZE
 *         - EMERALD
 *         - LIGHT_GRAYISH_BLUE
 *         - LIGHT_GRAYISH_YELLOW
 *         - LIGHT_GRAYISH_GREEN
 *         - RED_TRANS
 *         - GREEN_TRANS
 *         - BLUE_TRANS
 *         If the string color is none of them or NULL, WHITE will be returned
 *         and a warning on debug level 4 will be displayed.
 */
osg::Vec4 colorFromString(const char* color);

/*! \ingroup RcsGraphicsUtilsFunctions
 *  \brief Generates a transformation of a transformation matrix in world
 *         coordinates.
 */
void HTr_fromMatrix(const osg::Matrixd& Matrix, HTr* A_KI);

/*! \ingroup RcsGraphicsUtilsFunctions
 *  \brief Generates a transformation of the view matrix in world
 *         coordinates. It is computed through the lookAt function,
 *         therefore the mapping is ill-defined when looking from the
 *         poles. But this will hardly ever happen.
 *         <br>
 *         The native view matrix has the following conventions (see GLU):
 *         Camera x-axis is neg. view matrix z-axis
 *         Camera y-axis is neg. view matrix x-axis
 *         Camera z-axis is pos. view matrix y-axis
 *         <br>
 *         The transformation is located in the eye-point. The x-axis is
 *         pointing in forward direction, the z-axis points up, and the
 *         y-axis follows the right hand rule.
 */
void HTr_fromViewMatrix(const osg::Matrixd& viewMatrix, HTr* A_KI);

/*! \ingroup RcsGraphicsUtilsFunctions
 *  \brief Computes the distance of a point in world coordinates to the
 *         view cameras origin. The distance is measured as the projected
 *         vector to the gaze axis (normal to the view plane).
 */
double distanceToCamera(const osg::Matrixd& viewMat, const double I_pt[3]);

/*! \ingroup RcsGraphicsUtilsFunctions
 *  \brief Computes a line segment corresponding to the current line of
 *         sight, clipped by the near and far plane of the view frustrum.
 *         The line is represented in world coordinates.
 */
osg::ref_ptr<osg::LineSegment>
lineOfSightSegment(const osg::Matrix& projectionMatrix,
                   const osg::Matrix& viewMatrix,
                   float x, float y);

/*! \ingroup RcsGraphicsUtilsFunctions
 *  \brief Compute the mouse tip point in world coordinates. The view- and
 *         projection matrices come from the current view and can for
 *         instance be determined by the osg::Viewer:
 *         <br>
 *         - osg::Matrix vm = viewer->getCamera()->getViewMatrix();
 *         - osg::Matrix pm = viewer->getCamera()->getProjectionMatrix();
 *         <br>
 *         Values x and y describe the mouse tip in normalized screen
 *         coordinates [-1 : 1]. Vector planePt is an arbitraty point inside
 *         the plane.
 */
void getMouseTip(const osg::Matrix& viewMatrix,
                 const osg::Matrix& projectionMatrix,
                 float x, float y, const double planePt[3], double tip[3]);

/*! \ingroup RcsGraphicsUtilsFunctions
 *  \brief Computes the mouse tip in world coordinates.
 */
bool getMouseTip(const osgGA::GUIEventAdapter& ea,
                 osgGA::GUIActionAdapter& aa,
                 const double I_planePt[3], double I_tip[3]);

/*! \ingroup RcsGraphicsUtilsFunctions
 *  \brief Sets the default OSG coordinate frame orientation to right-
 *         handed, x-forward, z-upward convention.
 */
void setOSGCoordinateFrameToRoboticsConvention();

/*! \ingroup RcsGraphicsUtilsFunctions
 *  \brief The template needs to be specialized with an osg::Node* or
 *         anything derived from this. The function then returns a pointer
 *         to the first node of type T that is found under the mouse pointer.
 *         It further determines the mouse tip in world coordinates.
 *         If no node of type T is found under the mouse tip, the array
 *         I_mouseCoords is left unchanged and NULL is returned.
 *         Example usage:
 *
 *         MyNode *n = getNodeUnderMouse<MyNode*>(aa, mouseX, mouseY, pt);
 */
template <typename T>
T getNodeUnderMouse(osgGA::GUIActionAdapter& aa,
                    float mouseX,
                    float mouseY,
                    double I_mouseCoords[3] = NULL)
{
  osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
  if (!viewer)
  {
    return NULL;
  }

  osgUtil::LineSegmentIntersector::Intersections intersections;
  osg::Vec3 I_mouse;
  T foundNode = NULL;

  bool intersects = viewer->computeIntersections(mouseX, mouseY, intersections);

  if ((intersects==true) && (!intersections.empty()))
  {
    osgUtil::LineSegmentIntersector::Intersections::iterator
    hitr = intersections.begin();

    // Find the body to which the mouse currently points to
    if (hitr != intersections.end() && !hitr->nodePath.empty())
    {
      const osg::NodePath& nodePath = hitr->nodePath;
      unsigned int idx = nodePath.size();

      while (idx--)
      {
        // Find the last Rcs::BodyNode in the node path
        foundNode = dynamic_cast<T>(nodePath[idx]);

        if (foundNode == NULL)
        {
          continue;
        }

        // If we get here, we just found the first node in the
        // nodePath. Copy the mouse coordinates (in world frame)
        I_mouse = hitr->getWorldIntersectPoint();
        break;
      }

    }   // !hitr->nodePath.empty()

  }   // if(_viewer->computeIntersections



  // If a body has been found, compute the clicked coordinates in the
  // world coordinate frame
  if (true && I_mouseCoords)
  {
    I_mouseCoords[0] = I_mouse.x();
    I_mouseCoords[1] = I_mouse.y();
    I_mouseCoords[2] = I_mouse.z();
  }

  return foundNode;
}



/*! \ingroup RcsGraphicsUtilsFunctions
 *  \brief See above.
 */
template <typename T>
T getNodeUnderMouse(const osgGA::GUIEventAdapter& ea,
                    osgGA::GUIActionAdapter& aa,
                    double I_mouseCoords[3] = NULL)
{
  return getNodeUnderMouse<T>(aa, ea.getX(), ea.getY(), I_mouseCoords);
}

/*! \ingroup RcsGraphicsUtilsFunctions
 *  \brief Get a material, load it from file if necessary. The returned data
 *         is put into a material map for a fast look-up. Therefore, the caller
 *         must not delete it.
 *
 * Returns NULL if material could not be found
 */
RcsMaterialData* getMaterial(const std::string& matString);


/*! \ingroup RcsGraphicsUtilsFunctions
 *  \brief Sets a node's material, loads it from file if necessary
 *
 * Method was formerly defined in BodyNode
 */
bool setNodeMaterial(const std::string& matString, osg::Node* node,
                     double alpha = -1.0);

/*! \ingroup RcsGraphicsUtilsFunctions
 *  \brief Reads material properties from a file and copies them in the
 *         fields.
 */
bool getMaterialFromFile(const char* materialFile, const char* material,
                         double amb[4], double diff[4], double spec[4],
                         double* sh);

/*! \ingroup RcsGraphicsUtilsFunctions
 *  \brief Creates material properties from a a given color string of type
 *         "#RRGGBB" or "#RRGGBBAA"
 * \param matString The color string
 * \param amb Ambient color (= 0.2 * r, 0.2 * g, 0.2 * b, a)
 * \param diff Diffuse color (= r, g, b, a)
 * \param spec Specular color (= 0.2, 0.2, 0.2, a)
 * \param sh Shininess (= 100.0)
 * \return True on success
 */
bool createMaterialFromColorString(const std::string& matString,
                                   double amb[4], double diff[4],
                                   double spec[4], double* sh);

/*! \ingroup RcsGraphicsUtilsFunctions
 *  \brief Converts a hex string to an unsigned integer (e.g., "FF" --> 255)
 *  \param str String holding a hex number
 *  \return The converted string
 */
unsigned int hexStringToUInt(const std::string& str);

/*! \ingroup RcsGraphicsUtilsFunctions
 *  \brief Creates a color string of form "#RRGGBBAA" from the given color
 *         components
 *  \note No range checking is performed
 *  \param r Red componentent between 0 and 255
 *  \param g Green componentent between 0 and 255
 *  \param b Blue componentent between 0 and 255
 *  \param a Alpha componentent between 0 and 255 (default: 255)
 *  \return Color string of form "#RRGGBBAA"
 */
std::string createColorString(unsigned int r, unsigned int g, unsigned int b,
                              unsigned int a = 255);

/*! \ingroup RcsGraphicsUtilsFunctions
 *  \brief Creates a RGB color value from an index (deterministic)
 *
 *  The function generates colors by mapping the index to hue of the
 *  HSV color space and then transforms it back to RGB.
 */
void colorFromIndex(double color[3], unsigned int index, double saturation=1.0,
                    double value=1.0);

/*! \ingroup RcsGraphicsUtilsFunctions
 *  \brief HSV color to RGB color
 *
 *  \param[out] r Red (range: [0, 1])
 *  \param[out] g Green (range: [0, 1])
 *  \param[out] b Blue (range: [0, 1])
 *  \param[in]  h Hue (range: [0, 360])
 *  \param[in]  s Saturation (range: [0, 1])
 *  \param[in]  v Value (range: [0, 1])
 */
void hsv2rgb(double& r, double& g, double& b, double h, double s, double v);

/*! \ingroup RcsGraphicsUtilsFunctions
 *  \brief RGB color to HSV color
 *
 *  \param[out] h Hue (range: [0, 360])
 *  \param[out] s Saturation (range: [0, 1])
 *  \param[out] v Value (range: [0, 1])
 *  \param[in]  r Red (range: [0, 1])
 *  \param[in]  g Green (range: [0, 1])
 *  \param[in]  b Blue (range: [0, 1])
 */
void rgb2hsv(double& h, double& s, double& v, double r, double g, double b);

/*! \ingroup RcsGraphicsUtilsFunctions
 *  \brief Maps a gray value [0,1] to RGB using the Matlab Jet color palette
 *
 *  \param[out] color RGB color
 *  \param[in] grayValue Scalar gray value between 0.0 and 1.0
 */
void jetColorPalette(double color[3], double grayValue);

void getGeodes(osg::Node* node, std::vector<osg::Geode*>& geodes);

}   // namespace Rcs


#endif // RCS_GRAPHICSUTILS_H


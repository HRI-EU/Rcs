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

#ifndef RCS_GRAPHICSUTILS_H
#define RCS_GRAPHICSUTILS_H

#include <Rcs_HTr.h>
#include <Rcs_mesh.h>

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
 *  \brief Sets a node's material, loads it from file if necessary
 */
bool setNodeMaterial(const std::string& matString, osg::Node* node,
                     double alpha = -1.0);

/*! \ingroup RcsGraphicsUtilsFunctions
 *  \brief Sets a node's alpha value. It overrides all the children's alpha
 *         value. The alpha value is clipped to [0 ... 1], where 0 is fully
 *         transparent and 1 is solid.
 */
bool setNodeAlpha(osg::Node* node, double alpha);

/*! \ingroup RcsGraphicsUtilsFunctions
 *  \brief Traverses the node and all its children, and sets each node's alpha
 *         value in case an osg::material exists for the node. This lets the
 *         nodes keep their colors in case they have no material attached.
 */
void updateNodeAlphaRecursive(osg::Node* node, double alpha);

/*! \ingroup RcsGraphicsUtilsFunctions
 *  \brief Returns a node's alpha value, taken from the ambient color. If no
 *         material has been assigned to the node, a value of 1 is returned.
 */
double getNodeAlpha(osg::Node* node);

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

osg::Node* findNamedNodeRecursive(osg::Node* root, std::string nodeName);

std::vector<osg::Node*> findNamedNodesRecursive(osg::Node* root,
                                                std::string nodeName);

template <typename T>
class ChildNodeCollector : public osg::NodeVisitor
{
public:
  ChildNodeCollector() :
    osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
  {
  }

  ChildNodeCollector(const std::string& name) :
    searchName(name), osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
  {
  }

  virtual void apply(osg::Node& node)
  {
    T* ndPtr = dynamic_cast<T*>(&node);

    if (ndPtr)
    {
      if (searchName.empty() || (node.getName()==searchName))
      {
        collectedNodes.push_back(ndPtr);
      }
    }

    // Keep traversing the rest of the scene graph.
    traverse(node);
  }

  std::vector<T*> getNodes()
  {
    return collectedNodes;
  }

protected:
  std::vector<T*> collectedNodes;
  std::string searchName;
};

template <typename T>
std::vector<T*> findChildrenOfType(osg::Group* root)
{
  ChildNodeCollector<T> nf;
  root->accept(nf);
  return nf.getNodes();
}

template <typename T>
std::vector<T*> findChildrenOfType(osg::Group* root, const std::string& name)
{
  ChildNodeCollector<T> nf(name);
  root->accept(nf);
  return nf.getNodes();
}

osg::Geometry* createGeometryFromMesh(const RcsMeshData* mesh);
void createGeometryFromMesh(osg::Geometry* geo, const RcsMeshData* mesh);
bool updateGeometryFromMesh(osg::Geometry* geometry, const RcsMeshData* mesh);
osg::Geometry* createGeometryFromMesh2(const RcsMeshData* mesh);
void createGeometryFromMesh2(osg::Geometry* geo, const RcsMeshData* mesh);
bool updateGeometryFromMesh2(osg::Geometry* geometry, const RcsMeshData* mesh);

}   // namespace Rcs


#endif // RCS_GRAPHICSUTILS_H

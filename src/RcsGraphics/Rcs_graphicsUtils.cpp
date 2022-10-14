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
#include <Rcs_material.h>
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
 * Combining ambient, diffuse and specular to rgba can be done by adding the
 * components.
 ******************************************************************************/
osg::Vec4 colorFromString2(const char* c)
{
  double rgba[4];
  Rcs_colorFromString(c, rgba);
  return osg::Vec4(rgba[0], rgba[1], rgba[2], rgba[3]);
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

  const RcsMaterial* mPtr = Rcs_getMaterial(matString.c_str());

  if (!mPtr)
  {
    RLOG(4, "Couldn't set material to \"%s\"", matString.c_str());
    return false;
  }

  // set material from matDataPtr, if alpha is between 0.0 and 1.0 then
  // overwrite the materials alpha
  osg::ref_ptr<osg::Material> material = new osg::Material;
  osg::Vec4 amb(mPtr->amb[0], mPtr->amb[1], mPtr->amb[2], mPtr->amb[3]);
  osg::Vec4 diff(mPtr->diff[0], mPtr->diff[1], mPtr->diff[2], mPtr->diff[3]);
  osg::Vec4 spec(mPtr->spec[0], mPtr->spec[1], mPtr->spec[2], mPtr->spec[3]);
  material->setAmbient(osg::Material::FRONT_AND_BACK, amb);
  material->setDiffuse(osg::Material::FRONT_AND_BACK, diff);
  material->setSpecular(osg::Material::FRONT_AND_BACK, spec);
  material->setShininess(osg::Material::FRONT_AND_BACK, mPtr->shininess);

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
 * Sets a node's transparency
 ******************************************************************************/
bool setNodeAlpha(osg::Node* node, double alpha)
{
  if (node == NULL)
  {
    RLOG(4, "osg::Node is NULL - doing nothing");
    return false;
  }

  if ((alpha < 0.0) && (alpha > 1.0))
  {
    RLOG(4, "Ignoring alpha value %.16f, must be [0 ... 1]", alpha);
    return false;
  }

  osg::Material* material = NULL;
  osg::ref_ptr<osg::StateSet> ss = node->getStateSet();
  if (ss.valid())
  {
    material = (osg::Material*)ss->getAttribute(osg::StateAttribute::MATERIAL);
  }

  if (!material)
  {
    material = new osg::Material;
  }

  material->setAlpha(osg::Material::FRONT_AND_BACK, alpha);
  //material->setTransparency(osg::Material::FRONT, alpha);

  // Assign material through state set
  osg::ref_ptr<osg::StateSet> stateset = node->getOrCreateStateSet();

  stateset->setMode(GL_BLEND,
                    osg::StateAttribute::OVERRIDE |
                    osg::StateAttribute::ON);

  stateset->setMode(GL_LIGHTING,
                    osg::StateAttribute::OVERRIDE |
                    osg::StateAttribute::ON);

  stateset->setMode(GL_ALPHA_TEST,
                    osg::StateAttribute::OVERRIDE |
                    osg::StateAttribute::ON);

  stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

  stateset->setAttributeAndModes(material,
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
 * Gets a node's transparency
 ******************************************************************************/
double getNodeAlpha(osg::Node* node)
{
  double alpha = 1.0;
  osg::ref_ptr<osg::StateSet> ss = node->getStateSet();
  if (ss.valid())
  {
    osg::Material* material = (osg::Material*)ss->getAttribute(osg::StateAttribute::MATERIAL);
    if (material)
    {
      osg::Vec4 v = material->getAmbient(osg::Material::FRONT);
      alpha = v[3];
    }
  }

  return alpha;
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

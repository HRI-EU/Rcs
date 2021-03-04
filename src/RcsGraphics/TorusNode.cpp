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

#include "TorusNode.h"

#include <Rcs_macros.h>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/ShadeModel>

#include <cmath>


namespace Rcs
{

TorusNode::TorusNode(double radius, double thickness, double start_angle,
                     double end_angle)
{
  setName("TorusNode");
  osg::ref_ptr<osg::Geode> geode = new osg::Geode;
  osg::ref_ptr<osg::Geometry> torus = createGeometry(radius, thickness,
                                                     start_angle, end_angle);
  geode->addDrawable(torus.get());
  patPtr()->addChild(geode.get());
}

TorusNode::~TorusNode()
{
}

void TorusNode::createHelperArrays(double radius, double thickness,
                                   double start_angle, double end_angle,
                                   unsigned int nSides, unsigned int nRings,
                                   osg::Vec3Array* vertices,
                                   osg::Vec3Array* normals)
{
  double oradius = radius;
  double iradius = thickness / 2.0;

  // add 1 to nRings and nSides, because we need to store more vertices
  // than we have facets
  // unsigned int nRings = 30 + 1;
  // unsigned int nSides = 30 + 1;

  double dpsi = (end_angle - start_angle) / (double)(nRings - 1);
  double dphi = -2.0 * M_PI / (double)(nSides - 1);
  double psi  = start_angle;

  for (unsigned int j = 0; j < nRings; j++)
  {
    double cpsi = cos(psi);
    double spsi = sin(psi);
    double phi = 0.0;

    for (unsigned int i = 0; i < nSides; i++)
    {
      double cphi = cos(phi);
      double sphi = sin(phi);

      if (vertices)
      {
        double x = cpsi * (oradius + cphi * iradius);
        double y = spsi * (oradius + cphi * iradius);
        double z = sphi * iradius;
        vertices->push_back(osg::Vec3(x, y, z));
      }

      if (normals)
      {
        double nx = cpsi * cphi;
        double ny = spsi * cphi;
        double nz = sphi;
        normals->push_back(osg::Vec3(nx, ny, nz));
      }

      phi += dphi;
    }

    psi += dpsi;
  }
}

osg::ref_ptr<osg::Geometry> TorusNode::createGeometry(double radius,
                                                      double thickness,
                                                      double start_angle,
                                                      double end_angle)
{
  osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
  osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;

  // add 1 to nRings and nSides, because we need to store more vertices
  // than we have facets
  unsigned int nRings = 30 + 1;
  unsigned int nSides = 30 + 1;

  createHelperArrays(radius, thickness, start_angle, end_angle,
                     nSides, nRings, vertices, normals);

  // build using quads
  osg::Vec3Array* quad_vertices = new osg::Vec3Array;
  osg::Vec3Array* quad_normals = new osg::Vec3Array;

  for (unsigned int i = 0; i < nSides - 1; i++)
  {
    for (unsigned int j = 0; j < nRings - 1; j++)
    {
      quad_normals->push_back(normals->at(j * nSides + i));
      quad_vertices->push_back(vertices->at(j * nSides + i));

      quad_normals->push_back(normals->at(j * nSides + i + 1));
      quad_vertices->push_back(vertices->at(j * nSides + i + 1));

      quad_normals->push_back(normals->at((j + 1) * nSides + i + 1));
      quad_vertices->push_back(vertices->at((j + 1) * nSides + i + 1));

      quad_normals->push_back(normals->at((j + 1) * nSides + i));
      quad_vertices->push_back(vertices->at((j + 1) * nSides + i));
    }
  }

  // Prepare geometry node
  osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
  geometry->setVertexArray(quad_vertices);
  geometry->setNormalArray(quad_normals);
  geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
  geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0,
                                                quad_vertices->size()));

  return geometry;
}

void TorusNode::resize(double radius, double thickness, osg::Geometry* geometry)
{
  osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;

  // add 1 to nRings and nSides, because we need to store more vertices
  // than we have facets
  unsigned int nRings = 30 + 1;
  unsigned int nSides = 30 + 1;

  createHelperArrays(radius, thickness, 0.0, 2.0*M_PI,
                     nSides, nRings, vertices, NULL);

  // build using quads
  osg::Array* vertexArr = geometry->getVertexArray();
  osg::Vec3Array* quad_vertices = dynamic_cast<osg::Vec3Array*>(vertexArr);
  RCHECK(quad_vertices);
  size_t vidx = 0;

  for (unsigned int i = 0; i < nSides - 1; i++)
  {
    for (unsigned int j = 0; j < nRings - 1; j++)
    {
      (*quad_vertices)[vidx++] = osg::Vec3(vertices->at(j * nSides + i));
      (*quad_vertices)[vidx++] = osg::Vec3(vertices->at(j * nSides + i + 1));
      (*quad_vertices)[vidx++] = osg::Vec3(vertices->at((j + 1) * nSides + i + 1));
      (*quad_vertices)[vidx++] = osg::Vec3(vertices->at((j + 1) * nSides + i));
    }
  }

}


} // namespace Rcs

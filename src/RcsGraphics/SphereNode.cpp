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

#include "SphereNode.h"

#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osgUtil/SmoothingVisitor>



/******************************************************************************

  \brief Constructors.

******************************************************************************/

Rcs::SphereNode::SphereNode(const double pos[3],
                            const double radius,
                            bool resizeable) : NodeBase()
{
  init(pos, radius, resizeable);
}

Rcs::SphereNode::SphereNode() : NodeBase()
{
  init(NULL, 0.5, false);
}

void Rcs::SphereNode::init3(const double pos[3],
                            const double radius,
                            bool resizeable)
{
  osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;

  osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
  vertices->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
  vertices->push_back(osg::Vec3(1.0f, 0.0f, 0.0f));
  vertices->push_back(osg::Vec3(1.0f, 1.0f, 0.0f));
  vertices->push_back(osg::Vec3(0.0f, 1.0f, 0.0f));

  osg::ref_ptr<osg::DrawElementsUInt> quads = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 4);

  (*quads)[0] = 0;
  (*quads)[1] = 1;
  (*quads)[2] = 2;
  (*quads)[3] = 3;



  geometry->setVertexArray(vertices);
  geometry->addPrimitiveSet(quads.get());
  geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
  osgUtil::SmoothingVisitor::smooth(*geometry, M_PI_4);



  //SpherifiedCube(geometry, 16);

  osg::ref_ptr<osg::Geode> geode = new osg::Geode;
  geode->addDrawable(geometry.get());
  this->patPtr()->addChild(geode);
}

void Rcs::SphereNode::init2(const double pos[3],
                            const double radius,
                            bool resizeable)
{
  unsigned int parallels = 16;
  unsigned int meridians = 16;
  osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;

  vertices->push_back(osg::Vec3(0.0f, 1.0f, 0.0f));

  //for (uint32_t j = 0; j < (parallels/2+1) - 1; ++j)
  for (uint32_t j = 0; j < parallels - 1; ++j)
  {
    double const polar = M_PI * double(j+1) / double(parallels);
    double const sp = std::sin(polar);
    double const cp = std::cos(polar);
    for (uint32_t i = 0; i < meridians; ++i)
    {
      double const azimuth = 2.0 * M_PI * double(i) / double(meridians);
      double const sa = std::sin(azimuth);
      double const ca = std::cos(azimuth);
      double const x = sp * ca;
      double const y = cp;
      double const z = sp * sa;
      vertices->push_back(osg::Vec3(x, y, z));
    }
  }
  vertices->push_back(osg::Vec3(0.0f, -1.0f, 0.0f));

  // for (size_t i=0; i<vertices->size(); ++i)
  // {
  //   osg::Vec3 vi = (*vertices)[i];
  //   fprintf(stderr, "Vtx[%zu] = %f %f %f\n", i, vi[0], vi[1], vi[2]);
  // }



  // Tri oben
  osg::ref_ptr<osg::DrawElementsUInt> trisTop = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 3*meridians);

  for (uint32_t i = 0; i < meridians; ++i)
  {
    uint32_t const a = i + 1;
    uint32_t const b = (i + 1) % meridians + 1;
    (*trisTop)[i*3+0] = 0;
    (*trisTop)[i*3+1] = b;
    (*trisTop)[i*3+2] = a;
  }

  // Quads
  osg::ref_ptr<osg::DrawElementsUInt> quads = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS,
                                                                        4*(parallels-2)*(meridians));

  int nQuads = 0;
  //for (uint32_t j = 0; j < (parallels/2+1) - 2; ++j)
  for (uint32_t j = 0; j < parallels - 2; ++j)
  {
    uint32_t aStart = j * meridians + 1;
    uint32_t bStart = (j + 1) * meridians + 1;
    for (uint32_t i = 0; i < meridians; ++i)
    {
      const uint32_t a = aStart + i;
      const uint32_t a1 = aStart + (i + 1) % meridians;
      const uint32_t b = bStart + i;
      const uint32_t b1 = bStart + (i + 1) % meridians;

      (*quads)[nQuads*4+0] = a;
      (*quads)[nQuads*4+1] = a1;
      (*quads)[nQuads*4+2] = b1;
      (*quads)[nQuads*4+3] = b;
      nQuads++;
    }
  }

  // Tri unten
  osg::ref_ptr<osg::DrawElementsUInt> trisBottom = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 3*meridians);

  for (uint32_t i = 0; i < meridians; ++i)
  {
    uint32_t const a = i + meridians * (parallels - 2) + 1;
    uint32_t const b = (i + 1) % meridians + meridians * (parallels - 2) + 1;
    (*trisBottom)[i*3+0] = vertices->size()-1;
    (*trisBottom)[i*3+1] = a;
    (*trisBottom)[i*3+2] = b;
  }

  osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
  geometry->setVertexArray(vertices);
  //geometry->addPrimitiveSet(trisTop.get());
  //geometry->addPrimitiveSet(trisBottom.get());
  geometry->addPrimitiveSet(quads.get());
  geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
  osgUtil::SmoothingVisitor::smooth(*geometry, M_PI_4);
  osg::ref_ptr<osg::Geode> geode = new osg::Geode;
  geode->addDrawable(geometry.get());
  this->patPtr()->addChild(geode);


}

void Rcs::SphereNode::init(const double pos[3],
                           const double radius,
                           bool resizeable)
{
  setName("SphereNode");
  this->sphere = new osg::Sphere();
  sphere->setRadius(radius);
  osg::ShapeDrawable* shape = new osg::ShapeDrawable(sphere.get());

  if (resizeable==true)
  {
    shape->setUseDisplayList(false);
  }

  osg::Geode* geode = new osg::Geode();
  geode->addDrawable(shape);
  this->patPtr()->addChild(geode);

  if (pos != NULL)
  {
    setPosition(pos);
  }

}

void Rcs::SphereNode::setRadius(double r)
{
  sphere->setRadius(r);
}
/*
From: https://github.com/caosdoar/spheres/ (MIT license)
void UVSphere(uint32_t meridians, uint32_t parallels, Mesh &mesh)
{
  mesh.vertices.emplace_back(0.0f, 1.0f, 0.0f);
  for (uint32_t j = 0; j < parallels - 1; ++j)
  {
    double const polar = M_PI * double(j+1) / double(parallels);
    double const sp = std::sin(polar);
    double const cp = std::cos(polar);
    for (uint32_t i = 0; i < meridians; ++i)
    {
      double const azimuth = 2.0 * M_PI * double(i) / double(meridians);
      double const sa = std::sin(azimuth);
      double const ca = std::cos(azimuth);
      double const x = sp * ca;
      double const y = cp;
      double const z = sp * sa;
      mesh.vertices.emplace_back(x, y, z);
    }
  }
  mesh.vertices.emplace_back(0.0f, -1.0f, 0.0f);

  for (uint32_t i = 0; i < meridians; ++i)
  {
    uint32_t const a = i + 1;
    uint32_t const b = (i + 1) % meridians + 1;
    mesh.addTriangle(0, b, a);
  }

  for (uint32_t j = 0; j < parallels - 2; ++j)
  {
    uint32_t aStart = j * meridians + 1;
    uint32_t bStart = (j + 1) * meridians + 1;
    for (uint32_t i = 0; i < meridians; ++i)
    {
      const uint32_t a = aStart + i;
      const uint32_t a1 = aStart + (i + 1) % meridians;
      const uint32_t b = bStart + i;
      const uint32_t b1 = bStart + (i + 1) % meridians;
      mesh.addQuad(a, a1, b1, b);
    }
  }

  for (uint32_t i = 0; i < meridians; ++i)
  {
    uint32_t const a = i + meridians * (parallels - 2) + 1;
    uint32_t const b = (i + 1) % meridians + meridians * (parallels - 2) + 1;
    mesh.addTriangle(mesh.vertices.size() - 1, a, b);
  }
}
*/

namespace CubeToSphere
{
static const osg::Vec3 origins[6] =
{
  osg::Vec3(-1.0, -1.0, -1.0),
  osg::Vec3(1.0, -1.0, -1.0),
  osg::Vec3(1.0, -1.0, 1.0),
  osg::Vec3(-1.0, -1.0, 1.0),
  osg::Vec3(-1.0, 1.0, -1.0),
  osg::Vec3(-1.0, -1.0, 1.0)
};
static const osg::Vec3 rights[6] =
{
  osg::Vec3(2.0, 0.0, 0.0),
  osg::Vec3(0.0, 0.0, 2.0),
  osg::Vec3(-2.0, 0.0, 0.0),
  osg::Vec3(0.0, 0.0, -2.0),
  osg::Vec3(2.0, 0.0, 0.0),
  osg::Vec3(2.0, 0.0, 0.0)
};
static const osg::Vec3 ups[6] =
{
  osg::Vec3(0.0, 2.0, 0.0),
  osg::Vec3(0.0, 2.0, 0.0),
  osg::Vec3(0.0, 2.0, 0.0),
  osg::Vec3(0.0, 2.0, 0.0),
  osg::Vec3(0.0, 0.0, 2.0),
  osg::Vec3(0.0, 0.0, -2.0)
};
}

void Rcs::SphereNode::SpherifiedCube(osg::Geometry* geometry, unsigned int divisions)
{
  osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
  const double step = 1.0 / double(divisions);
  const osg::Vec3 step3(step, step, step);

  for (uint32_t face = 0; face < 6; ++face)
  {
    const osg::Vec3 origin = CubeToSphere::origins[face];
    const osg::Vec3 right = CubeToSphere::rights[face];
    const osg::Vec3 up = CubeToSphere::ups[face];
    for (uint32_t j = 0; j < divisions + 1; ++j)
    {
      const osg::Vec3 j3(j, j, j);
      for (uint32_t i = 0; i < divisions + 1; ++i)
      {
        const osg::Vec3 i3(i, i, i);
        const osg::Vec3 p = origin + step3 * (i3 * right + j3 * up);
        //const osg::Vec3 p2 = p * p;
        const osg::Vec3 p2 = osg::Vec3(p[0]*p[0], p[1]*p[1], p[2]*p[2]);
        const osg::Vec3 n
        (
          p[0] * std::sqrt(1.0 - 0.5 * (p2[1] + p2[2]) + p2[1]*p2[2] / 3.0),
          p[1] * std::sqrt(1.0 - 0.5 * (p2[2] + p2[0]) + p2[2]*p2[0] / 3.0),
          p[2] * std::sqrt(1.0 - 0.5 * (p2[0] + p2[1]) + p2[0]*p2[1] / 3.0)
        );
        vertices->push_back(n);
      }
    }
  }


  osg::ref_ptr<osg::DrawElementsUInt> tris = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 6*6*divisions*divisions);
  uint32_t count = 0;

  const uint32_t k = divisions + 1;

  for (uint32_t face = 0; face < 6; ++face)
  {
    for (uint32_t j = 0; j < divisions; ++j)
    {
      const bool bottom = j < (divisions / 2);

      for (uint32_t i = 0; i < divisions; ++i)
      {
        const bool left = i < (divisions / 2);
        const uint32_t a = (face * k + j) * k + i;
        const uint32_t b = (face * k + j) * k + i + 1;
        const uint32_t c = (face * k + j + 1) * k + i;
        const uint32_t d = (face * k + j + 1) * k + i + 1;
        if (bottom ^ left)
        {
          (*tris)[count*6+0] = a;
          (*tris)[count*6+1] = b;
          (*tris)[count*6+2] = d;
          (*tris)[count*6+3] = b;
          (*tris)[count*6+4] = c;
          (*tris)[count*6+5] = d;
          // mesh.addQuadAlt(a, c, d, b);
        }
        else
        {
          (*tris)[count*6+0] = a;
          (*tris)[count*6+1] = b;
          (*tris)[count*6+2] = c;
          (*tris)[count*6+3] = a;
          (*tris)[count*6+4] = c;
          (*tris)[count*6+5] = d;
          // mesh.addQuad(a, c, d, b);
        }

        count++;
      }
    }
  }


  geometry->setVertexArray(vertices);
  geometry->addPrimitiveSet(tris.get());
  geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
  osgUtil::SmoothingVisitor::smooth(*geometry, M_PI_4);
}

// void addQuad(uint32_t a, uint32_t b, uint32_t c, uint32_t d)
//  {
//    triangles.emplace_back(a);
//    triangles.emplace_back(b);
//    triangles.emplace_back(c);
//    triangles.emplace_back(a);
//    triangles.emplace_back(c);
//    triangles.emplace_back(d);
//  }



// void addQuadAlt(uint32_t a, uint32_t b, uint32_t c, uint32_t d)
//  {
//    triangles.emplace_back(a);
//    triangles.emplace_back(b);
//    triangles.emplace_back(d);
//    triangles.emplace_back(b);
//    triangles.emplace_back(c);
//    triangles.emplace_back(d);
//  }

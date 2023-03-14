/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH

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

#include "Rcs_material.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_utils.h"
#include "Rcs_resourcePath.h"
#include "Rcs_basicMath.h"
#include "Rcs_VecNd.h"

#include <string>
#include <sstream>
#include <map>
#include <pthread.h>



// The below comment is for astyle, since otherwise it screws up the formatting
// *INDENT-OFF*
#define MULTI_LINE_STRING(a) #a
static const char* materialDefs =
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

                    <MaterialDefinition Name="BRICKRED" Shininess="10.0">
                    <Ambient R="0.1" G="0.0" B="0.0" A="1.0"/>
                    <Diffuse R="0.5569" G="0.1059" B="0.1059" A="1.0"/>
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

                    <MaterialDefinition Name="DARKGREEN" Shininess="10.0">
                    <Ambient R="0.0" G="0.1" B="0.0" A="1.0"/>
                    <Diffuse R="0.2353" G="0.3843" B="0.2235" A="1.0"/>
                    <Specular R="0.1" G="0.1" B="0.1" A="1.0"/>
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


// Little map that stores name-pointer pairs of materials files. If a material
// has already been loaded, we look up its pointer from the map. Otherwise, we
// load the material and store its pointer in the map.
static std::map<std::string, RcsMaterial> materialMap;


/*******************************************************************************
 *
 ******************************************************************************/
static unsigned int hexStringToUInt(const std::string& str)
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
static bool createMaterialFromColorString(const std::string& matString,
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
static bool getMaterialFromFile(const char* materialFile, const char* material,
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
const RcsMaterial* Rcs_getMaterial(const char* materialName)
{
  std::string matString = std::string(materialName);
  RcsMaterial* res = NULL;

  static pthread_mutex_t mLock = PTHREAD_MUTEX_INITIALIZER;

  pthread_mutex_lock(&mLock);
  std::map<std::string, RcsMaterial>::iterator it = materialMap.find(matString);

  if (it!=materialMap.end())
  {
    NLOG(1, "Found color %s from fast internal map", matString.c_str());
    res = &it->second;
  }
  pthread_mutex_unlock(&mLock);

  if (res)
  {
    return res;
  }



  RcsMaterial matData;
  bool success = false;

  // Create random color
  if (STRCASEEQ(materialName, "Random"))
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
                                            matData.amb,
                                            matData.diff,
                                            matData.spec,
                                            &matData.shininess);
  }
  else
  {
    pthread_mutex_lock(&mLock);
    success = getMaterialFromBuffer(materialDefs, matString.c_str(),
                                    matData.amb,
                                    matData.diff,
                                    matData.spec,
                                    &matData.shininess);
    pthread_mutex_unlock(&mLock);
  }

  if (success==false)
  {
    pthread_mutex_lock(&mLock);
    success = getMaterialFromColorFile(matString,
                                       matData.amb,
                                       matData.diff,
                                       matData.spec,
                                       &matData.shininess);
    pthread_mutex_unlock(&mLock);
  }

  if (success==true)
  {
    pthread_mutex_lock(&mLock);
    materialMap[matString] = matData;
    res = &materialMap[matString];
    pthread_mutex_unlock(&mLock);
    return res;
  }



  RLOG(4, "Loading material %s FAILED", materialName);
  return NULL;
}


/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs_colorFromString(const char* c, double rgba[4])
{
  if (c==NULL)
  {
    RLOG(4, "Color is NULL - returning white");
    VecNd_set4(rgba, 1.0f, 1.0f, 1.0f, 1.0f);
    return false;
  }

  bool success = true;

  if ((std::string(c).length() == 7 || std::string(c).length() == 9) &&
      std::string(c)[0] == '#')
  {
    double r = (double) hexStringToUInt(std::string(c).substr(1, 2)) / 255;
    double g = (double) hexStringToUInt(std::string(c).substr(3, 2)) / 255;
    double b = (double) hexStringToUInt(std::string(c).substr(5, 2)) / 255;
    double a = (strlen(c) == 9) ?
               ((double) hexStringToUInt(std::string(c).substr(7, 2)) / 255) : 1.0;
    VecNd_set4(rgba, r, g, b, a);
  }
  else if (STRCASEEQ(c, "RED"))
  {
    VecNd_set4(rgba, 1.0, 0.0, 0.0, 1.0);
  }
  else if (STRCASEEQ(c, "WHITE"))
  {
    VecNd_set4(rgba, 1.0, 1.0, 1.0, 1.0);
  }
  else if (STRCASEEQ(c, "BLACK"))
  {
    VecNd_set4(rgba, 0.0, 0.0, 0.0, 0.0);
  }
  else if (STRCASEEQ(c, "GREEN"))
  {
    VecNd_set4(rgba, 0.0, 1.0, 0.0, 1.0);
  }
  else if (STRCASEEQ(c, "BLUE"))
  {
    VecNd_set4(rgba, 0.0, 0.0, 1.0, 1.0);
  }
  else if (STRCASEEQ(c, "RUBY"))
  {
    VecNd_set4(rgba, 0.614, 0.041, 0.041, 1.0);
  }
  else if (STRCASEEQ(c, "YELLOW"))
  {
    VecNd_set4(rgba, 1.0, 1.0, 0.0, 1.0);
  }
  else if (STRCASEEQ(c, "BRASS"))
  {
    VecNd_set4(rgba, 0.7803, 0.5686, 0.1137, 1.0);
  }
  else if (STRCASEEQ(c, "PEWTER"))
  {
    VecNd_set4(rgba, 0.4274, 0.4705, 0.5411, 1.0);
  }
  else if (STRCASEEQ(c, "BRONZE"))
  {
    VecNd_set4(rgba, 0.714, 0.4284, 0.18144, 1.0);
  }
  else if (STRCASEEQ(c, "EMERALD"))
  {
    VecNd_set4(rgba, 0.075, 0.6142, 0.07568, 1.0);
  }
  else if (STRCASEEQ(c, "LIGHT_GRAYISH_BLUE"))
  {
    VecNd_set4(rgba, 0.8, 0.8, 0.95, 1.0);
  }
  else if (STRCASEEQ(c, "LIGHT_GRAYISH_YELLOW"))
  {
    VecNd_set4(rgba, 0.845, 0.845, 0.797, 1.0);
  }
  else if (STRCASEEQ(c, "LIGHT_GRAYISH_GREEN"))
  {
    VecNd_set4(rgba, 0.797, 0.845, 0.797, 1.0);
  }
  else if (STRCASEEQ(c, "RED_TRANS"))
  {
    VecNd_set4(rgba, 0.8f, 0.0, 0.0, 0.3f);
  }
  else if (STRCASEEQ(c, "GREEN_TRANS"))
  {
    VecNd_set4(rgba, 0.0, 0.8f, 0.0, 0.3f);
  }
  else if (STRCASEEQ(c, "BLUE_TRANS"))
  {
    VecNd_set4(rgba, 0.0, 0.0, 0.8f, 0.3f);
  }
  else
  {
    const RcsMaterial* mat = Rcs_getMaterial(c);
    if (mat)
    {
      for (size_t i=0; i<3; ++i)
      {
        rgba[i] = mat->amb[i] + mat->diff[i] + mat->spec[i];
        rgba[3] += mat->amb[i] + mat->diff[i] + mat->spec[i];
      }
      for (size_t i=0; i<3; ++i)
      {
        rgba[i] = Math_clip(rgba[i], 0.0, 1.0);
      }

      rgba[3] = 1.0;
    }
    else
    {
      RLOG(4, "Unknown color \"%s\"", c);
      VecNd_set4(rgba, 1.0, 1.0, 1.0, 1.0);
      success = false;
    }
  }

  return success;
}

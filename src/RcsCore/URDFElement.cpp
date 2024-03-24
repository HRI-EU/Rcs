/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
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

#include "URDFElement.h"
#include "Rcs_macros.h"

#include <sstream>


namespace Rcs
{

//------------------------------------------------------------------------

URDFElement::URDFElement(const std::string& tag)
  : m_tag(tag)
  , m_depth(0)
  , m_subElements()
  , m_attributes(nullptr)
{
}

//------------------------------------------------------------------------

std::unique_ptr<URDFAttributes>& URDFElement::getAttributes()
{
  return m_attributes;
}

//------------------------------------------------------------------------

void URDFElement::addSubElement(std::unique_ptr<URDFNodeInterface>&& ele)
{
  m_subElements.push_back(std::move(ele));
  for (auto& ele : m_subElements)
  {
    ele->setDepth(this->getDepth() + 1);
  }
}

//------------------------------------------------------------------------

void URDFElement::setAttributes(std::unique_ptr<URDFAttributes>&& attributes)
{
  m_attributes = std::move(attributes);
}

//------------------------------------------------------------------------

void URDFElement::addAttribute(const std::string& tag, const std::string& value)
{
  if (!m_attributes)
  {
    m_attributes = std::unique_ptr<URDFAttributes>(new (std::nothrow) URDFAttributes());
  }

  if (m_attributes)
  {
    m_attributes->addAttribute(tag, value);
  }
  else
  {
    RFATAL("no attributes member. nullptr error.");
  }
}

//------------------------------------------------------------------------

void URDFElement::setDepth(size_t depth)
{
  m_depth = depth;
  for (auto& ele : m_subElements)
  {
    ele->setDepth(this->getDepth() + 1);
  }
}

//------------------------------------------------------------------------

size_t URDFElement::getDepth() const
{
  return m_depth;
}

//------------------------------------------------------------------------

std::string URDFElement::toString()
{
  std::stringstream ss;

  ss << std::string(m_depth * 2, ' ') << "<" << m_tag;
  if (m_attributes)
  {
    ss << " " << m_attributes->toString();
  }

  if (m_subElements.size() == 0)
  {
    ss << "/>";
    return ss.str();
  }

  ss << ">";

  for (const auto& ele : m_subElements)
  {
    ss << "\n" << ele->toString();
  }

  ss << "\n" << std::string(m_depth * 2, ' ') << "</" << m_tag << ">";

  return ss.str();
}

}

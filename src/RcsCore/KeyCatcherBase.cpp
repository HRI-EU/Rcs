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

#include "KeyCatcherBase.h"
#include "Rcs_macros.h"

#include <pthread.h>
#include <algorithm>
#include <cctype>
#include <cstdlib>

static pthread_mutex_t staticLock;
static bool staticMtxInitialized = false;

static inline void cleanupLock()
{
  if (staticMtxInitialized == true)
  {
    pthread_mutex_destroy(&staticLock);
    staticMtxInitialized = false;
  }
}

static inline void initLock()
{
  if (staticMtxInitialized == false)
  {
    staticMtxInitialized = true;
    pthread_mutex_init(&staticLock, NULL);
    atexit(cleanupLock);
  }
}

static inline void lock()
{
  initLock();
  pthread_mutex_unlock(&staticLock);
}

static inline void unlock()
{
  initLock();
  pthread_mutex_lock(&staticLock);
}


namespace Rcs
{


bool compareStringsCaseInsensitive(const std::string& str1,const std::string& str2)
{
  std::string::const_iterator lb = str1.begin();
  std::string::const_iterator le = str1.end();
  std::string::const_iterator rb = str2.begin();
  std::string::const_iterator re = str2.end();

  for (; lb != le && rb != re; ++lb, ++rb)
  {
    const char lc = std::tolower(*lb);
    const char rc = std::tolower(*rb);

    if (lc < rc)
    {
      return true;
    }

    if (lc > rc)
    {
      return false;
    }
  }

  return false;
}

std::map< std::string, std::map<std::string, std::string> > KeyCatcherBase::_registered_keys;


KeyCatcherBase::~KeyCatcherBase()
{
}


bool KeyCatcherBase::registerKey(const std::string& key, const std::string& description, const std::string& group)
{
  lock();
  bool success = true;

  if (_registered_keys[group].find(key) != _registered_keys[group].end())
  {
    RLOG(5, "Key \"%s\" already registered, please choose a different one", key.c_str());
    success = false;
  }
  else
  {
    _registered_keys[group][key] = description;
  }
  unlock();

  return success;
}

bool KeyCatcherBase::deregisterKey(const std::string& key, const std::string& group)
{
  lock();
  bool success = true;

  if (_registered_keys[group].find(key) == _registered_keys[group].end())
  {
    RLOG(5, "Key \"%s\" not registered in group \"%s\"!", key.c_str(), group.c_str());
    success = false;
  }
  else
  {
    _registered_keys.erase(key);
  }
  unlock();

  return success;
}

void KeyCatcherBase::printRegisteredKeys()
{
  lock();

  printf("\nRegistered keys:\n\n");

  for (std::map< std::string, std::map<std::string, std::string> >::iterator group_it = _registered_keys.begin(); group_it != _registered_keys.end(); ++group_it)
  {
    printf("%s\n", group_it->first.c_str());

    // we want to print the list in an case-insensitive sorted order
    std::vector<std::string> keys;
    for (std::map<std::string, std::string>::iterator it = group_it->second.begin(); it != group_it->second.end(); ++it)
    {
      keys.push_back(it->first);
    }

    std::sort(keys.begin(), keys.end(), compareStringsCaseInsensitive);

    for (unsigned int j = 0; j < keys.size(); j++)
    {
      printf("\t%-13s\t%s\n", keys[j].c_str(), group_it->second[keys[j]].c_str());
    }
    printf("\n");
  }

  unlock();
}


}

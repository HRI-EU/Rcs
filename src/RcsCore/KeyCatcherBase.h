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

#ifndef RCS_KEYCATCHERBASE_H
#define RCS_KEYCATCHERBASE_H

#include <string>
#include <map>
#include <vector>

#include <pthread.h>

/*!
 * \brief Global Rcs namespace encapsulating all Rcs classes
 */
namespace Rcs
{

/*! \brief Keyboard-shortcut registry and thread-safe key-state store.
 *
 *         The class serves two purposes: a global (static) registry of
 *         keyboard shortcuts with descriptions and grouping, and a per-instance
 *         256-entry key-state array that application code polls via
 *         getAndResetKey. Both the OSG-backed Rcs::KeyCatcher (which feeds key
 *         events from the OSG event loop) and headless viewers (e.g. the
 *         browser bridge in RcsWebViewer, which feeds them from a network thread) use
 *         this class directly. Every per-instance accessor takes the internal
 *         pthread mutex, so producers and the polling application thread may run
 *         concurrently.
 */
class KeyCatcherBase
{
public:
  KeyCatcherBase();
  virtual ~KeyCatcherBase(); 

  virtual bool getAndResetKey(char c);
  virtual bool getAndResetKey(int i);
  bool getKey(char c);
  void setKey(char c);
  void resetKey(char c);
  void toggleKey(char c);

  static bool registerKey(const std::string& key, const std::string& description, const std::string& group = "Main");
  static bool deregisterKey(const std::string& key, const std::string& group = "Main");
  static void deregisterKeys();
  static void printRegisteredKeys();
  static std::string printRegisteredKeysToString();

private:
  KeyCatcherBase(const KeyCatcherBase&);
  KeyCatcherBase& operator=(const KeyCatcherBase&);

  bool _charPressed[256];
  pthread_mutex_t _mutex;
  static std::map< std::string, std::map<std::string, std::string> > _registered_keys;
};

}

#endif // RCS_KEYCATCHERBASE_H

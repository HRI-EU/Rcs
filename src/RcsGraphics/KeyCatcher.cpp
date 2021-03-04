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
#include "KeyCatcher.h"

#include <Rcs_macros.h>

#include <iostream>


using namespace Rcs;



/*******************************************************************************

  \brief Constructor.

*******************************************************************************/

KeyCatcher::KeyCatcher()
{
  pthread_mutex_init(&_lock, NULL);
  for (int i = 0; i < 256; i++)
  {
    _charPressed[i] = false;
  }

  KeyCatcherBase::registerKey("F12", "Print all available keys to console",
                              "KeyCatcher");
  setName("KeyCatcher");
}




/*******************************************************************************

  \brief Destructor.

*******************************************************************************/

KeyCatcher::~KeyCatcher()
{
  pthread_mutex_destroy(&_lock);
}



/*******************************************************************************

  \brief Handles keydown events.

*******************************************************************************/

bool KeyCatcher::handle(const osgGA::GUIEventAdapter& ea,
                        osgGA::GUIActionAdapter& aa)
{

  switch (ea.getEventType())
  {

    case (osgGA::GUIEventAdapter::KEYDOWN):
    {

      // key 'A' to 'Z' are ASCII codes 65 - 90
      if ((ea.getKey() >= 65) && (ea.getKey() <= 90))
      {
        unsigned int asciiCode =  ea.getKey();
        pthread_mutex_lock(&_lock);
        _charPressed[asciiCode] = !_charPressed[asciiCode];
        pthread_mutex_unlock(&_lock);
        RLOG(5, "Toggled %c (%u)", asciiCode, asciiCode);
        return false;
      }

      // key 'a' to 'z' are ASCII codes 97 - 122
      else if ((ea.getKey() >= 97) && (ea.getKey() <= 122))
      {
        unsigned int asciiCode =  ea.getKey();
        pthread_mutex_lock(&_lock);
        _charPressed[asciiCode] = !_charPressed[asciiCode];
        pthread_mutex_unlock(&_lock);
        RLOG(5, "Toggled %c (%u)", asciiCode, asciiCode);
        return false;
      }

      // ESC
      else if (ea.getKey() == 27)
      {
        exit(0);
        return false;
      }

      // Enter
      else if (ea.getKey() == 13)
      {
        unsigned int asciiCode =  ea.getKey();
        pthread_mutex_lock(&_lock);
        _charPressed[asciiCode] = !_charPressed[asciiCode];
        pthread_mutex_unlock(&_lock);
        RLOG(5, "Toggled Enter (%u)", asciiCode);
        return false;
      }

      else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_F12)
      {
        KeyCatcherBase::printRegisteredKeys();
        return false;
      }

      // Rest of the ASCII table
      else
      {
        unsigned int asciiCode =  ea.getKey();
        if (asciiCode < 256)
        {
          pthread_mutex_lock(&_lock);
          _charPressed[asciiCode] = !_charPressed[asciiCode];
          pthread_mutex_unlock(&_lock);
          RLOG(5, "Toggled %c (%u)", asciiCode, asciiCode);
        }
        else
        {
          RLOG(5, "Unhandled ASCII code %c (%u)", asciiCode, asciiCode);
        }
        return false;
      }

    }   // case KEYDOWN

    default:
      break;

  }    // switch(ea.getEventType())



  return false;
}



/*******************************************************************************

  \brief Returns true if the key is pressed, false otherwise.

*******************************************************************************/

bool KeyCatcher::getKey(const char key)
{
  pthread_mutex_lock(&_lock);
  bool isPressed = _charPressed[(unsigned int) key];
  pthread_mutex_unlock(&_lock);
  return isPressed;
}



/*******************************************************************************

  \brief Returns the state of the key and resets it to unpressed.

*******************************************************************************/

bool KeyCatcher::getAndResetKey(char c)
{
  pthread_mutex_lock(&_lock);
  bool isPressed = _charPressed[(unsigned int) c];
  _charPressed[(unsigned int) c] = false;
  pthread_mutex_unlock(&_lock);
  return isPressed;
}

/*******************************************************************************

  \brief Returns the state of the key and resets it to unpressed.

*******************************************************************************/

bool KeyCatcher::getAndResetKey(int i)
{
  pthread_mutex_lock(&_lock);
  bool isPressed = _charPressed[(unsigned int) i];
  _charPressed[(unsigned int) i] = false;
  pthread_mutex_unlock(&_lock);
  return isPressed;
}



/*******************************************************************************

  \brief Sets the state of the key to pressed.

*******************************************************************************/

void KeyCatcher::setKey(const char key)
{
  if ((unsigned int) key >= 256)
  {
    RLOG(0, "Key %c (ASCII %u) out of ASCII range - ignoring", key, key);
  }

  pthread_mutex_lock(&_lock);
  _charPressed[(unsigned int) key] = true;
  pthread_mutex_unlock(&_lock);
}



/*******************************************************************************

  \brief Resets the state of the key to unpressed.

*******************************************************************************/

void KeyCatcher::resetKey(const char key)
{
  if ((unsigned int) key >= 256)
  {
    RLOG(0, "Key %c (ASCII %u) out of ASCII range - ignoring", key, key);
  }

  pthread_mutex_lock(&_lock);
  _charPressed[(unsigned int) key] = false;
  pthread_mutex_unlock(&_lock);
}

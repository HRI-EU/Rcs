

#ifndef TEMPLATE_H
#define TEMPLATE_H
//#include <Any.h>
#include <Rcs_macros.h>

template<typename T> void logSomething()
{
  //ANY_LOG(0, "Log message", ANY_LOG_INFO);
  RLOG(0, "Log message");
}

#endif

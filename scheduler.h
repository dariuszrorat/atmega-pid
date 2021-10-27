#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "Arduino.h"   

#define MAX_TASKS   (3)

typedef struct
{
  void (*pfn)(void);
  unsigned int period;
} Task;

class Scheduler
{
  public:
    void init(void);
    unsigned short add(void ( *)(void), unsigned int);
    void  dispatch(void);
};

static Scheduler Sch;

#endif

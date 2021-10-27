#include "scheduler.h"

unsigned int taskindex;
unsigned int taskcounter;
Task tasks[MAX_TASKS]; 

void Scheduler::init(void)
{
  taskcounter = 0;
  taskindex = 0;
}

unsigned short Scheduler::add(void (*fn)(), unsigned int period)
{
  if (taskindex < MAX_TASKS)
  {
    tasks[taskindex].pfn = fn;
    tasks[taskindex].period = period;
    taskindex++;
  }
  
  return taskindex;
}

void Scheduler::dispatch(void)
{
  unsigned short i;
  for (i = 0; i < taskindex; i++)
  {
    if ((taskcounter % tasks[i].period) == 0)
    {
      (*tasks[i].pfn)();
    }
  }

  // default 1ms delay
  delay(1);
  taskcounter++;
}

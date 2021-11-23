#ifndef MMICODE_H
#define MMICODE_H

#include "Arduino.h"
#include "strings.h"

#define MAX_CALLBACKS (25)

typedef struct
{
  void (*pfn)(long, long, long);
  String command;
} Callback;

class Mmicode
{
  public:
    void init(void);
    unsigned int add(String, void ( *) (long, long, long));
    bool exec(String code);
};

static Mmicode Mmi;

#endif

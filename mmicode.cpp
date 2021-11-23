#include "mmicode.h"

unsigned int cindex;
unsigned int ccounter;

Callback callbacks[MAX_CALLBACKS];

void Mmicode::init(void)
{
  ccounter = 0;
  cindex = 0;
}

unsigned int Mmicode::add(String command, void (*fn)(long param1, long param2, long param3))
{
  if (cindex < MAX_CALLBACKS)
  {
    callbacks[cindex].pfn = fn;
    callbacks[cindex].command = command;
    cindex++;
  }

  return cindex;
}


bool Mmicode::exec(String code)
{
  int len = code.length();
  if (len == 0) return false;
  if ((len > 0) && (code[len - 1] != '#')) return false;

  String cmd = code.substring(0, 4);
  String params = code.substring(4, code.length() - 1);
  String param0 = trimAll(getParam(params, '*', 0));
  String param1 = trimAll(getParam(params, '*', 1));
  String param2 = trimAll(getParam(params, '*', 2));
  long par0 = param0.toInt();
  long par1 = param1.toInt();
  long par2 = param2.toInt();

  unsigned short i;
  for (i = 0; i < cindex; i++)
  {
    if (callbacks[i].command == cmd)
    {
      (*callbacks[i].pfn)(par0, par1, par2);
      return true;
    }
  }
  return false;
}

#ifndef STRINGS_H
#define STRINGS_H

#include <arduino.h>

String trimAll(String s);
String getParam(String data, char separator, int index);
String filledStr(String s, int maxlen);


#endif

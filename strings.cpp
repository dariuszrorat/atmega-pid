#include "strings.h"


String trimAll(String s)
{
  String result = "";
  for (int i = 0; i < s.length(); i++)
  {
    if ((s[i] != ' ') && (s[i] != '#') && (s[i] != '*'))
    {
      result += s[i];
    }
  }
  return result;
}

String getParam(String data, char separator, int index)
{
  int maxIndex = data.length() - 1;
  int j = 0;
  String chunkVal = "";

  for (int i = 0; i <= maxIndex && j <= index; i++)
  {
    chunkVal.concat(data[i]);

    if (data[i] == separator)
    {
      j++;

      if (j > index)
      {
        chunkVal.trim();
        return chunkVal;
      }

      chunkVal = "";
    }
    else if ((i == maxIndex) && (j < index)) {
      chunkVal = "";
      return chunkVal;
    }
  }
}

String leftFilledStr(String s, int maxlen)
{
  int len = s.length();
  if (len > maxlen)
  {
    return s.substring(0, maxlen);
  }

  int i;
  String result = "";
  for (i = 0; i < (maxlen-len); i++)
  {
    result += " ";
  }
  result += s;
  return result;
}

String rightFilledStr(String s, int maxlen)
{
  int len = s.length();
  if (len > maxlen)
  {
    return s.substring(0, maxlen);
  }

  int i;
  for (i = len; i < maxlen; i++)
  {
    s += " ";
  }
  return s;
}

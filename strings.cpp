    #include "strings.h"

String boolToString(bool b, String valueTrue, String valueFalse)
{
    return b ? valueTrue : valueFalse;    
}

String floatToString(float val)
{
    String result = "";
    char charVal[10];
    dtostrf(val, 4, 4, charVal);
    for (int i = 0; i < 4; i++)
    {
        result += charVal[i];
    }
    return result;
}

unsigned int hex2int(String a, unsigned int len)
{
    unsigned int i;
    unsigned int val = 0;

    for (i = 0; i < len; i++)
        if (a[i] <= 57)
            val += (a[i] - 48) * (1 << (4 * (len - 1 - i)));
        else
            val += (a[i] - 55) * (1 << (4 * (len - 1 - i)));
    return val;
}

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
/*
String getParam(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
*/
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

String filledStr(String s, int maxlen)
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

#ifndef ParseIncluded
#define ParseIncluded

#include "Util.h"

//////////////////////////////////////////////////////////////////////////
//
// Parse is a simple parser which extracts tokens from a line string
//
////////////////////////////////////////////////////////////////////////////


class Parse {
public:
    Parse(const byte* buf, int len);
    Parse(const char* str);
    ~Parse();

    Parse& Next(const char* delimiters);

    bool operator==(const char* str);
    bool operator!=(const char* str) {return !(*this == str);}
    bool GetToken(char *str, int len);
    byte GetDelimiter();

protected:
    const byte* Buf;
    int Len;
    int TokenBegin;
    int TokenEnd;
    int Delimiter;
};

#endif // ParseIncluded

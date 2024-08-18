#ifndef Logger_h
#define Logger_h

#include "Arduino.h"

#define USE_LOGGER 

class Logger {
    public:
        Logger();
        void static begin();
        void static print(char * str);
        void static print(String str);
        void static print(float str);
    private:
        Logger static * _instance;
};

#endif

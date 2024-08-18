#include "Logger.h"

Logger::Logger()
{
}

void Logger::begin()
{
    #ifdef USE_LOGGER
        Serial.begin(115200);
        delay(4000);
    #endif
}
void Logger::print(char * str)
{
    #ifdef USE_LOGGER
        Serial.println(str);
    #endif
}
void Logger::print(String str)
{
    #ifdef USE_LOGGER
        Serial.println(str);
    #endif
}
void Logger::print(float str)
{
    #ifdef USE_LOGGER
        Serial.println(str);
    #endif
}
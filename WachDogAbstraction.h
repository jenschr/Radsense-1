#ifndef WatchDogAbstraction_h
#define WatchDogAbstraction_h

// In version 3.x, Espressif broke backward compatibility for WDT
// This file is just to abstract away that difference for
// anyone not familiar with ESP-IDF (or using Arduino)

#include <esp_task_wdt.h>
#include <esp_arduino_version.h>

// The ESP32 C3 only has one core
#if !defined(CONFIG_FREERTOS_NUMBER_OF_CORES)
#define CONFIG_FREERTOS_NUMBER_OF_CORES 1
#endif

class WatchDogAbstraction {
    public:
        WatchDogAbstraction()
        {
            // intentionally empty
        }
        void begin( uint32_t secondsUntilReset ){
            if(!isStarted){
                #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
                    esp_task_wdt_config_t twdt_config = {
                        .timeout_ms = secondsUntilReset*1000,
                        .idle_core_mask = (1 << CONFIG_FREERTOS_NUMBER_OF_CORES) - 1,    // Bitmask of all cores
                        .trigger_panic = true,
                    };
                    esp_task_wdt_init(&twdt_config); //enable panic so ESP32 restarts
                #else
                    esp_task_wdt_init(secondsUntilReset, true); //enable panic so ESP32 restarts
                #endif
                esp_task_wdt_add(NULL); //add current thread to WDT watch
                isStarted = true;
            }
            
        }
        // Maintain the watchdog
        void reset(){
            #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
                esp_task_wdt_reset();
            #else
                esp_task_wdt_reset();
            #endif
        }
    private:
        bool isStarted = false;
};
#endif

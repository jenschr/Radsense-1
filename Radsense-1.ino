#include <Arduino.h>
#include <Wire.h>
#include <NimBLEDevice.h>
#include <math.h>
#include "ld2410.h"
#include "State.h"
#include "Logger.h"
#include "WachDogAbstraction.h"

#define SERVICE_UUID         "b71eb828-6e9a-4db3-b11b-0ea799461a10"
#define CHARACTERISTIC_UUID  "825fdfcc-9771-11ee-b11b-0ea799461a10"
#define CHARACTERISTIC_UUID2 "825fdfcc-9771-11ee-b11b-0ea799461a11"
#define CHARACTERISTIC_UUID3 "825fdfcc-9771-11ee-b11b-0ea799461a12"
#define MTU_SIZE 100

#define FIRMWARE_VERSION 1.71
#define RADAR_SERIAL Serial1
#define WDT_TIMEOUT 15

#define SCL_PIN 1
#define SDA_PIN 0
#define LED_PIN 5 // v1.6 = 2, v1.7 = 5
#define SETTINGS_PIN 8
#define IN1_PIN 4
#define IN2_PIN 2 // v1.6 = 5, v1.7 = 2
#define RELAY_OUT1_PIN 6
#define RELAY_OUT2_PIN 7
#define NEOPIXEL_PIN 3
#define BOOT_ESP_PIN 9 // Never use apart from booting?
#define RADAR_OUT_PIN 10
#define RADAR_RX_PIN 21
#define RADAR_TX_PIN 20
#define NEOPIXEL_COUNT 8

#define MM_TO_IN 0.393701

WatchDogAbstraction wdt;

enum class PresenceState {
  CLOSE = 0,
  NEAR = 1,
  DISTANT = 2,
  NOTHING = 3,
  UNKNOWN = 5,
};
PresenceState currentState;

// Visual feedback
State deviceState;

// BLE
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristicData = NULL;
BLECharacteristic* pCharacteristicSettings = NULL;
BLECharacteristic* pCharacteristicDebug = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool bleOutputEnabled = false;
int bluetoothShouldBeTurnedOffAfterThisManyMinutes = 2;
unsigned long turnBluetoothOffAt = 0;
int count = 0;

// Radar
ld2410 radar;
uint32_t lastReading = 0;
bool radarConnected = false;
float readings[4];
int requiredConsecutiveReads = 60;
const int msBetweenReads = 30;
int minDistance = 30;
int maxDistance = 330;
int firstRange;
int secondRange;
int lastDistanceDetected;
int minEnergy = 20; // 20 is default
int uniqueSamplesPerMinute = 0;
int uniqueSamplesPerSecond = 0;

// Potmeter adjustment
int analogValue1 = 0;
int analogValue2 = 0;
int analogValueOld1 = 0;
int analogValueOld2 = 0;

// Settings
bool wasSettingsPressed = false;
unsigned long now = 0;
unsigned long lastSettingsPress = 0;
unsigned long nextDebugOutput = 0;
unsigned long nextSettingsOutput = 0;
int timeSinceLastPress = 0;
int timeSinceLastRelease = 0;
int lastDoublePress = 0;
bool clickDetected;
bool singleClickDetected;
bool doubleClickDetected;
bool doubleClickFastDetected;

// Averages
int lastSecondSampled = 0;
int sampleBuffer[600];              // This is the maximum number of samples we can analyze
int sampleBufferMaxPosition = 100;  // What is the longest into the buffer we want to analyze
int sampleIndex = 0;                // Holds our current position in the buffer
int uniqueSampleCount = 0;

int sampleCountPerSecond = 0;
const int numSamplesInBuffer = 60;        // The buffer is always 60, but we might not use all of it
int sampleBufferMinute[numSamplesInBuffer];
int sampleBufferSeconds[numSamplesInBuffer];
int uniqueSamples[numSamplesInBuffer];

bool isStickyRelaysMode()
{
  if( deviceState.getDeviceMode() == DeviceMode::BOTH_SLOW || deviceState.getDeviceMode() == DeviceMode::BOTH_MEDIUM || deviceState.getDeviceMode() == DeviceMode::BOTH_FAST ){
    return true;
  }
  return false;
}

void updateZoneLeds()
{
  switch( currentState ){
      case PresenceState::CLOSE   : 
        if( deviceState.getLedMode() != LedMode::OFF ){ // Do not tutn on, if LEDs are off
          deviceState.setPixels(50,50,50,7,1);
        }
        
        if( isStickyRelaysMode() ){
          if( deviceState.getLedMode() != LedMode::OFF ){ // Do not tutn on, if LEDs are off
            deviceState.setPixels(50,50,50,6,1);
          }
        } else {
          deviceState.setPixels(0,0,0,6,1);
        }
        break;
      case PresenceState::NEAR    : 
        deviceState.setPixels(0,0,0,7,1);
        if( deviceState.getLedMode() != LedMode::OFF ){ // Do not tutn on, if LEDs are off
          deviceState.setPixels(50,50,50,6,1);
        }
        break;
      case PresenceState::DISTANT :
        deviceState.setPixels(0,0,0,6,2);
        break;
      case PresenceState::NOTHING :
        deviceState.setPixels(0,0,0,6,2);
        break;
    }
}

void setPresenceState( PresenceState newState ){
  if( currentState != newState )
  {
    switch( newState ){
      case PresenceState::CLOSE   : 
        digitalWrite(RELAY_OUT1_PIN, HIGH);
        if( isStickyRelaysMode() ){
          digitalWrite(RELAY_OUT2_PIN, HIGH);
        } else {
          digitalWrite(RELAY_OUT2_PIN, LOW);
        }
        Logger::print(F("CLOSE!"));
        break;
      case PresenceState::NEAR    : 
        digitalWrite(RELAY_OUT1_PIN, LOW);
        digitalWrite(RELAY_OUT2_PIN, HIGH);
        Logger::print(F("NEAR!"));
        break;
      case PresenceState::DISTANT :
        digitalWrite(RELAY_OUT1_PIN, LOW);
        digitalWrite(RELAY_OUT2_PIN, LOW);
        Logger::print(F("DISTANT!"));
        break;
      case PresenceState::NOTHING :
        digitalWrite(RELAY_OUT1_PIN, LOW);
        digitalWrite(RELAY_OUT2_PIN, LOW);
        Logger::print(F("NOTHING!"));
        break;
    }
    currentState = newState;
  }
  updateZoneLeds();
}

/**  None of these are required as they will be handled by the library with defaults. **
 **                       Remove as you see fit for your needs                        */  
class MyServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) {
      deviceConnected = true;
      Logger::print(F("Client connected"));
    };

    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) {
      deviceConnected = false;
      bleOutputEnabled = false;
      BLEDevice::stopAdvertising();
      Logger::print(F("Client disconnected"));
    }
/***************** New - Security handled here ********************
****** Note: these are the same return values as defaults ********/
  uint32_t onPassKeyRequest()
  {
    Logger::print(F("Server PassKeyRequest"));
    return 123456; 
  }

  bool onConfirmPIN(uint32_t pass_key){
    Logger::print(F("The passkey YES/NO number: "));
    Logger::print(pass_key);
    return true; 
  }

  void onAuthenticationComplete(ble_gap_conn_desc desc){
    Logger::print(F("Starting BLE work!"));
  }
/*******************************************************************/
};

void setupBluetooth()
{
  // Create the BLE Device
  BLEDevice::init("RadSense1");
  BLEDevice::setMTU(MTU_SIZE);
  
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  NimBLEService*        pService = pServer->createService("F00D");
  
  // Create a BLE Characteristic
  pCharacteristicData = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      NIMBLE_PROPERTY::NOTIFY
                    );

  pCharacteristicSettings = pService->createCharacteristic(
                      CHARACTERISTIC_UUID2,
                      NIMBLE_PROPERTY::NOTIFY
                    );

  pCharacteristicDebug = pService->createCharacteristic(
                      CHARACTERISTIC_UUID3,
                      NIMBLE_PROPERTY::NOTIFY
                    );

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->setName("Radsense1");
  pAdvertising->addServiceUUID(pService->getUUID());
  pAdvertising->enableScanResponse(false);
}

void setupOutputs()
{
  pinMode(IN1_PIN, INPUT);
  pinMode(IN2_PIN, INPUT);
  pinMode(SETTINGS_PIN, INPUT_PULLUP);
  pinMode(RELAY_OUT1_PIN, OUTPUT);
  pinMode(RELAY_OUT2_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(RELAY_OUT1_PIN, LOW);
  digitalWrite(RELAY_OUT2_PIN, LOW);
  digitalWrite(LED_PIN, HIGH);
  pinMode(RADAR_RX_PIN, INPUT);
  pinMode(RADAR_TX_PIN, OUTPUT);
}

void updateBluetoothTurnoffTime()
{
  turnBluetoothOffAt = millis();
  turnBluetoothOffAt += 1000*60*bluetoothShouldBeTurnedOffAfterThisManyMinutes;
  if( !bleOutputEnabled )
  {
    BLEDevice::startAdvertising();
    bleOutputEnabled = true;
    Logger::print(F("BLE advertising turned on"));
  }
}

/*
When reading the sensor every 30ms, we'll get 30 samples per second.
There is a limit to how fast you can read the sensor, so there is
a tradeoff between speed and precision:
- The more samples, the higher certainty of the positioning.
- The less samples, the faster switching.
*/
void updateSampleSpeed()
{
  switch( deviceState.getDeviceMode() )
  {
    case DeviceMode::DEFAULT_SLOW :    requiredConsecutiveReads = 60; break;
    case DeviceMode::DEFAULT_MEDIUM :  requiredConsecutiveReads = 30; break;
    case DeviceMode::DEFAULT_FAST :    requiredConsecutiveReads = 15; break;
    case DeviceMode::BOTH_SLOW :       requiredConsecutiveReads = 60; break;
    case DeviceMode::BOTH_MEDIUM :     requiredConsecutiveReads = 30; break;
    case DeviceMode::BOTH_FAST :       requiredConsecutiveReads = 15; break;
    default : requiredConsecutiveReads = 60; break;
  }
}

void updateFilter()
{
  switch( deviceState.getFilterMode() )
  {
    case FilterMode::WEAK :            minEnergy = 60; break;
    case FilterMode::MEDIUM :          minEnergy = 85; break;
    case FilterMode::STRONG :          minEnergy = 98; break;
    default : minEnergy = 60; break;
  }
}

void animateRgbLeds()
{
  while(1)
  {
    for(int i=0;i<NEOPIXEL_COUNT;i++)
    {
      deviceState.setPixels(0,0,0,0,NEOPIXEL_COUNT); // all off
      deviceState.setPixels(100,0,0,i,1);
      delay(250);
    }
  }
}

void getPresenceStateAsString( char * str, PresenceState state )
{
  switch (state)
  {
    case PresenceState::CLOSE: sprintf(str,"CLOSE"); break;
    case PresenceState::NEAR: sprintf(str,"NEAR"); break;
    case PresenceState::DISTANT: sprintf(str,"DISTANT"); break;
    case PresenceState::NOTHING: sprintf(str,"NOTHING"); break;
    default: sprintf(str,"UNKNOWN"); break;
  }
}

void setup(void)
{
  deviceState.begin(NEOPIXEL_PIN, NEOPIXEL_COUNT);
  deviceState.setLedSubMode(0);

  Logger::begin();
  Logger::print(F("RadSense1 firmware version "));
  Logger::print(FIRMWARE_VERSION);

  setupOutputs();
  Wire.begin(SDA_PIN,SCL_PIN);

  // After prefs are read, we can re-set the update frequency
  updateSampleSpeed();
  updateFilter();

  // Also print out potmeter values
  analogValue1 = analogRead(IN1_PIN);
  Logger::print(F("analogValue1: "));
  Logger::print(analogValue1);
  analogValue2 = analogRead(IN2_PIN);
  Logger::print(F("analogValue2: "));
  Logger::print(analogValue2);
  
  Logger::print(F("\nStartup"));
  digitalWrite(LED_PIN, LOW);
  
  //radar.debug(Logger::print); //Uncomment to show debug information from the library on the Serial Monitor. By default this does not show sensor reads as they are very frequent.
  RADAR_SERIAL.begin(256000, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN); //UART for monitoring the radar
  delay(500);
  Logger::print(F("LD2410 radar sensor initialising: "));
  if(radar.begin(RADAR_SERIAL))
  {
    Logger::print(F("OK"));
    Logger::print(F("LD2410 firmware version: "));
    Logger::print(radar.firmware_major_version);
    Logger::print('.');
    Logger::print(radar.firmware_minor_version);
    Logger::print('.');
    Logger::print(radar.firmware_bugfix_version);
  }
  else
  {
    Logger::print(F("not connected"));
    int blinks = 0;
    while(1){
      deviceState.setPixels(255,0,0);
      delay(200);
      deviceState.setPixels(0,0,0);
      delay(200);
      // Maintain the watchdog only for the first 20 blinks
      if( blinks > 20){
        esp_restart();
      }
      blinks++;
    }
  }

  deviceState.setLedSubMode(2);
  setupBluetooth();

  wdt.begin(WDT_TIMEOUT);

  // Turn off the radar's builtin bluetooth config
  bool wasTurnedOff = !radar.bluetoothOff();
  radar.requestRestart();
}

void updateAnalogReadings()
{
  analogValue1 = analogRead(IN1_PIN);
  analogValue2 = analogRead(IN2_PIN);

  if( analogValue1 != analogValueOld1 || analogValue2 != analogValueOld2 )
  {
    // position moving targets
    float percentage1 = analogValue1/(float)4095;
    float percentage2 = analogValue2/(float)4095;
    float total = percentage1 + percentage2;
    //Logger::print.print("% "+String(percentage1)+"-"+String(percentage2)+" -:- ");
    firstRange = minDistance+((maxDistance-minDistance)*percentage1);
    secondRange = firstRange+ ((maxDistance-firstRange)*percentage2);
    //Logger::print.println(String(minDistance)+"0-"+String(firstRange)+"-"+String(secondRange)+"-"+String(maxDistance));
    analogValueOld1 = analogValue1;
    analogValueOld2 = analogValue2;
  }
}

void deleteOneFromAllBins()
{
  // First we delete a sample from all four bin's
  if( readings[0] > 0 ){
    readings[0] = readings[0]-1;
  }
  if( readings[1] > 0 ){
    readings[1] = readings[1]-1;
  }
  if( readings[2] > 0 ){
    readings[2] = readings[2]-1;
  }
  if( readings[3] > 0 ){
    readings[3] = readings[3]-1;
  }
}

void debugOutput( int distance, int energy )
{
  return;
  Logger::print(readings[0]);
  Logger::print(F(" : "));
  Logger::print(readings[1]);
  Logger::print(F(" : "));
  Logger::print(readings[2]);
  Logger::print(F(" : "));
  Logger::print(readings[3]);
  Logger::print(F(", "));
  Logger::print(distance);
  Logger::print(F(":"));
  Logger::print(energy);
}

void updateTheBins( int distance, int firstRange, int secondRange, int energy )
{
  // Select where to put the new sample
  if( distance < firstRange && energy > minEnergy ){
    if( readings[0]<requiredConsecutiveReads ){
      readings[0] = readings[0]+2;
    }
  } else if(distance < secondRange && energy > minEnergy){
    if( readings[1]<requiredConsecutiveReads ){
      readings[1] = readings[1]+2;
    }
  } else if(distance < maxDistance && energy > minEnergy){
    if( readings[2]<requiredConsecutiveReads ){
      readings[2] = readings[2]+2;
    }
  } else {
    if( readings[3]<requiredConsecutiveReads ){
      readings[3] = readings[3]+2;
    }
  }
}

void sendBleDebug( int distance, int firstRange, int secondRange, int energy, unsigned long now )
{
  // notify of changed value
  if (deviceConnected) {
    updateBluetoothTurnoffTime();
    // up to 10 times per second, we'll send out the current data
    if( now > nextDebugOutput){
      nextDebugOutput += 100;
      const int bleStringLength = BLEDevice::getMTU();
      char toSendInfo[MTU_SIZE];
      if( deviceState.getLedMode() == LedMode::IMPERIAL )
      {
        int firstRangeIn = round((float)firstRange * MM_TO_IN);
        int secondRangeIn = round((float)secondRange * MM_TO_IN);
        int distanceIn = round((float)distance * MM_TO_IN);
        sprintf(toSendInfo, "Zone1 %din, Zone2 %din, D %din", firstRangeIn,secondRangeIn,distanceIn);
      }
      else
      {
        sprintf(toSendInfo, "Zone1 %dcm\nZone2 %dcm\nDetect %dcm\nEnergy:%d", firstRange,secondRange,distance,energy);
      }
      pCharacteristicData->setValue(toSendInfo);
      pCharacteristicData->notify();
    }

    // Once every second, we'll also send out the current device settings as a
    // different characteristic
    if( now > nextSettingsOutput){
      nextSettingsOutput += 1000;
      char toSendSettings[MTU_SIZE];
      char deviceMode[20];
      deviceState.getDeviceModeAsString(deviceMode);
      char ledMode[20];
      deviceState.getLedModeAsString(ledMode);
      char filterMode[20];
      deviceState.getFilterModeAsString(filterMode);
      char stateAsString[20];
      getPresenceStateAsString(stateAsString, currentState);
      sprintf(toSendSettings, "Mode %s\nLed %s\nFilter %s\nPresence: %s", deviceMode, ledMode, filterMode, stateAsString);
      pCharacteristicSettings->setValue(toSendSettings);
      pCharacteristicSettings->notify();

      char toSendDebug[MTU_SIZE];
      sprintf(toSendDebug, "PerSec:%d\nPerMin: %d", currentState, uniqueSamplesPerSecond, uniqueSamplesPerMinute);
      pCharacteristicDebug->setValue(toSendDebug);
      pCharacteristicDebug->notify();
    }
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
      delay(100); // give the bluetooth stack the chance to get things ready
      turnBluetoothOffAt = 0; // by setting it this low, it'll force disconnect
      oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
      // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
  }
}

void gotoNextPrecisionMode()
{
  Logger::print(F("gotoNextPrecisionMode"));
  deviceState.setPixels(0,0,0,0,NEOPIXEL_COUNT);
  switch( deviceState.getDeviceMode() )
  {
    case DeviceMode::DEFAULT_SLOW : deviceState.setDeviceMode( DeviceMode::DEFAULT_MEDIUM ); break;
    case DeviceMode::DEFAULT_MEDIUM : deviceState.setDeviceMode( DeviceMode::DEFAULT_FAST ); break;
    case DeviceMode::DEFAULT_FAST : deviceState.setDeviceMode( DeviceMode::BOTH_SLOW ); break;
    case DeviceMode::BOTH_SLOW : deviceState.setDeviceMode( DeviceMode::BOTH_MEDIUM ); break;
    case DeviceMode::BOTH_MEDIUM : deviceState.setDeviceMode( DeviceMode::BOTH_FAST ); break;
    case DeviceMode::BOTH_FAST : deviceState.setDeviceMode( DeviceMode::DEFAULT_SLOW ); break;
    default : deviceState.setDeviceMode( DeviceMode::DEFAULT_SLOW ); break;
  }
  updateSampleSpeed();
  setPresenceState(currentState);
  deviceState.showActiveDeviceMode();
  updateZoneLeds();
}

void gotoNextLedMode()
{
  Logger::print(F("gotoNextLedMode"));
  deviceState.setPixels(0,0,0,0,NEOPIXEL_COUNT);
  switch( deviceState.getLedMode() )
  {
    case LedMode::METRIC: deviceState.setLedMode( LedMode::IMPERIAL ); break;
    case LedMode::IMPERIAL:      deviceState.setLedMode( LedMode::OFF ); deviceState.setPixels(0,0,0,0,NEOPIXEL_COUNT); break;
    case LedMode::OFF:           deviceState.setLedMode( LedMode::METRIC ); break;
    default: deviceState.setLedMode( LedMode::METRIC ); break;
  }
  deviceState.showActiveLedMode();
  updateZoneLeds();
}

void gotoPreviousLedMode()
{
  // Yeah, this is dumb but it works
  Logger::print(F("gotoPreviousLedMode"));
  switch( deviceState.getLedMode() )
  {
    case LedMode::METRIC: deviceState.setLedMode( LedMode::OFF ); break;
    case LedMode::IMPERIAL:      deviceState.setLedMode( LedMode::METRIC ); break;
    case LedMode::OFF:           deviceState.setLedMode( LedMode::IMPERIAL ); break;
    default: deviceState.setLedMode( LedMode::METRIC ); break;
  }
}


void gotoNextFilterMode()
{
  Logger::print(F("gotoNextFilterMode"));
  switch( deviceState.getFilterMode() )
  {
    case FilterMode::WEAK: deviceState.setFilterMode( FilterMode::MEDIUM ); break;
    case FilterMode::MEDIUM: deviceState.setFilterMode( FilterMode::STRONG ); break;
    case FilterMode::STRONG: deviceState.setFilterMode( FilterMode::WEAK ); break;
    default: deviceState.setLedMode( LedMode::METRIC ); break;
  }
  updateFilter();
  deviceState.showActiveFilterMode();
}

void handleInteraction( unsigned long now )
{
  bool isSettingsPressed = digitalRead(SETTINGS_PIN);
  if( isSettingsPressed != wasSettingsPressed )
  {
    if( !isSettingsPressed ) // onPressDown
    {
      singleClickDetected = true;
      updateBluetoothTurnoffTime();

      // Figure out interactivity
      timeSinceLastPress = now - lastSettingsPress;
      if( timeSinceLastPress < 500 ) // Double-press
      {
        // register a double-press
        doubleClickDetected = true;
        Logger::print(F("Double!"));

        // Check if we're double-clicking fast!
        unsigned long currentDoublePress = now;
        int timeSincePreviousDoublePress = currentDoublePress - lastDoublePress;
        Logger::print(F("Double repeat? "));
        Logger::print(timeSincePreviousDoublePress);
        if( timeSincePreviousDoublePress < 5000 ) // Fast Double-press
        {
          doubleClickFastDetected = true;
          Logger::print(F("Double repeat!"));
        }
        lastDoublePress = currentDoublePress;
      }
    }
    else // onRelease
    {
      timeSinceLastRelease = now - lastSettingsPress;
      if( timeSinceLastRelease > 5000 && lastSettingsPress != 0 ) // Really long press...
      {
        Logger::print(F("VeryLongPress"));
        gotoNextFilterMode();
      }
    }

    // Update before next loop
    lastSettingsPress = now;
    wasSettingsPressed = isSettingsPressed;
  }

  // Handle settings Interaction only after half a second
  if( now > (lastSettingsPress+500) )
  {
    if( doubleClickFastDetected )
    {
      doubleClickFastDetected = false;
      Logger::print(F("Change LED mode"));
      gotoNextLedMode();
    }
    else if( doubleClickDetected )
    {
      doubleClickDetected = false; // always reset!
      singleClickDetected = false; // always reset!
      Logger::print(F("Show LED mode"));
      deviceState.showActiveLedMode();
    }
    else if( singleClickDetected )
    {
      singleClickDetected = false; // always reset!
      Logger::print(F("timeSinceLastPress: "));
      Logger::print(timeSinceLastPress);
      if( timeSinceLastPress > 5000 ) // Less than 5 seconds since last press?
      {
        Logger::print(F("Show Device mode"));
        deviceState.showActiveDeviceMode();
      }
      else
      {
        Logger::print(F("Change Device mode"));
        gotoNextPrecisionMode();
      }
    }
  }

  if( singleClickDetected && now > (lastSettingsPress+5000) ) // Reset clicks after 5 seconds
  {
    Logger::print(F("Reset clicks"));
    doubleClickFastDetected = false;
    doubleClickDetected = false; // always reset!
    singleClickDetected = false; // always reset!
  }
}

// Second attempt that removes the minute-buffer and only looks at the previous sample
int countUniqueSamples( unsigned long int time, int currentValue )
{
  // TODO: evaluate if we need to drop samples beyond max. we likely do

  // Always save the sample
  sampleBuffer[sampleIndex] = currentValue;

  // Make sure we don't overrun the buffer on next iteration
  if( sampleIndex >= (sampleBufferMaxPosition-1) ){
    sampleIndex=0;
  } else {
    sampleIndex++;
  }
  
  // Update value only once per second
  int currentSecond = time/1000;
  if( currentSecond != lastSecondSampled )
  {
    // Loop through all samples
    for(int sampleNumber=0;sampleNumber<(sampleBufferMaxPosition-1);sampleNumber++)
    {
      // Compare each sample
      int firstSample = sampleBufferSeconds[sampleNumber];
      int secondSample = sampleBufferSeconds[sampleNumber+1];
      if(firstSample != secondSample)
      {
        uniqueSampleCount++;
      }
    }

    // Less than 2 values is not useful, so just remove
    if( uniqueSampleCount < 3 ) { uniqueSampleCount = 0; }

    // Output the result
    Logger::print(F("uniqueSampleCount: "));
    Logger::print(uniqueSampleCount);
    
    // Update before next loop
    lastSecondSampled = currentSecond;
    sampleCountPerSecond = 0;
  }
  return uniqueSampleCount;
}

/* The radar obviously cannot detect someone that isn't there, so as
long as any samples are different from the previous one's, we're
counting them using the uniqueSamplesPerMinute variable.

If nobody is present, we are not getting new samples so the value 
will go down. This means that turning off might take some seconds
(20-60 based on speed setting).

The below method clearly overcomplicates things by dividing into two
buffers, but it's very solid in terms of detecting. Changing zones
will still be very fast, so it's only in the case of someone
"disappearing" that this is used.
*/
int findUniqueSamplesPerMinute( unsigned long int time, int currentValue )
{
  // Always maintain and average the samples
  if( sampleCountPerSecond < (requiredConsecutiveReads/2) && currentValue < firstRange) // prevent exceeding the buffer
  {
    sampleBufferSeconds[sampleCountPerSecond] = currentValue;
    sampleCountPerSecond++;
  }
  
  // Update value only when the second is changing
  int currentSecond = time/1000;
  int currentSecondInMinute = currentSecond%requiredConsecutiveReads;
  uniqueSamplesPerSecond = 0;
  if( currentSecond != lastSecondSampled )
  {
    // Loop through all samples
    for(int sampleNumber=0;sampleNumber<requiredConsecutiveReads;sampleNumber++)
    {
      // Read each sample
      int sampleToTest = sampleBufferSeconds[sampleNumber];

      // Check if already in collection
      bool valueAlreadyInCollection = false;
      for(int i=0;i<uniqueSamplesPerSecond;i++)
      {
        if(uniqueSamples[i] == sampleToTest)
        {
          valueAlreadyInCollection = true;
        }
      }

      // if not already in collection, add it
      if( !valueAlreadyInCollection )
      {
        uniqueSamples[uniqueSamplesPerSecond] = sampleToTest;
        uniqueSamplesPerSecond++;
      }
    }

    // If we're not detecting anything, there's no need to count unique samples
    // This solves a bug that might cause a non-detect period on initial detect
    // and also improves response time.
    if( currentState == PresenceState::NOTHING && uniqueSamplesPerSecond > 1 ){
      uniqueSamplesPerSecond = 0;
    }

    // Less than 2 values is not useful, so just remove
    if( uniqueSamplesPerSecond < 3 ) { uniqueSamplesPerSecond = 0; }
    sampleBufferMinute[currentSecondInMinute] = uniqueSamplesPerSecond;

    // Find changes per minutes
    int total = 0;
    for(int i=0;i<requiredConsecutiveReads;i++)
    {
      total += sampleBufferMinute[i];
    }
    uniqueSamplesPerMinute = total;

    // Reset the buffers
    for(int i=0;i<requiredConsecutiveReads;i++)
    {
      sampleBufferSeconds[i] = 0;
      uniqueSamples[i] = 0;
    }

    // Update before next loop
    lastSecondSampled = currentSecond;
    sampleCountPerSecond = 0;
  }
  return 0;
}

void readTheRadar( unsigned long now )
{
  bool wasRead = radar.read();
  if(deviceState.getLedMode() != LedMode::OFF){
    digitalWrite(LED_PIN, wasRead); // Blink LED for debug
  }

  int energy = radar.stationaryTargetEnergy();
  int distanceToSave = 0;
  if(radar.isConnected() && now - lastReading > msBetweenReads && energy >= minEnergy)  //Report every 1000ms
  {
    lastReading = now;
    deleteOneFromAllBins();

    if( radar.presenceDetected() || radar.movingTargetDetected() )
    {
      lastDistanceDetected = radar.stationaryTargetDistance()-minDistance;
      distanceToSave = lastDistanceDetected;
    }
    else
    {
      distanceToSave = 500;
    }

    // Select where to put the new sample
    updateTheBins( distanceToSave, firstRange, secondRange, energy );
    sendBleDebug( distanceToSave, firstRange, secondRange, energy, now );
    debugOutput( distanceToSave, energy);
    if( radar.stationaryTargetEnergy() >= 98)
    {
      findUniqueSamplesPerMinute( now, lastDistanceDetected );
      countUniqueSamples( now, lastDistanceDetected );
    }
  }
  else if( now - lastReading > msBetweenReads ) // No radar, but time to update?
  {
    deleteOneFromAllBins();
    distanceToSave = 500;
    updateTheBins( distanceToSave, firstRange, secondRange, energy );
    sendBleDebug( distanceToSave, firstRange, secondRange, energy, now );
    lastReading = now;
    debugOutput(distanceToSave, energy);
    findUniqueSamplesPerMinute( now, lastDistanceDetected );
    countUniqueSamples( now, lastDistanceDetected );
  }
}

void loop()
{
  // Always update the LED based on current state
  int subState = static_cast<int>(currentState);
  if( !deviceState.animating()){
    deviceState.setLedSubMode(subState);
  } else {
    deviceState.maintain();
  }
  
  // Sample time just once per loop and rather pass it around
  now = millis();

  // Handle settings-button interaction
  handleInteraction( now );
  
  // Handle BLE connection
  if(bleOutputEnabled && now > turnBluetoothOffAt && !deviceConnected)
  {
    bleOutputEnabled = false;
    BLEDevice::stopAdvertising();
    Logger::print(F("Stop BLE advertising"));  // stop advertising the BLE service
  }

  // Handle reading analog values
  analogValue1 = analogRead(IN1_PIN);
  analogValue2 = analogRead(IN2_PIN);
  float percentage1 = analogValue1/(float)4095;
  float percentage2 = analogValue2/(float)4095;
  float total = percentage1 + percentage2;
  firstRange = minDistance+((maxDistance-minDistance)*percentage1);
  secondRange = firstRange+ ((maxDistance-firstRange)*percentage2);
  lastDistanceDetected = 500;
  
  readTheRadar( now );

  // Color & relay output
  if( readings[0] >= requiredConsecutiveReads ){
    setPresenceState(PresenceState::CLOSE);
  } else if(readings[1] >= requiredConsecutiveReads){
    setPresenceState(PresenceState::NEAR);
  } else if(readings[2] >= requiredConsecutiveReads){
    setPresenceState(PresenceState::DISTANT);
  } else if(readings[3] >= requiredConsecutiveReads && uniqueSamplesPerMinute <= 5){
    setPresenceState(PresenceState::NOTHING);
  }

  // Maintain the watchdog
  wdt.reset();
}

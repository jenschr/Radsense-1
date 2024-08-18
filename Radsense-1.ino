// Drop by https://github.com/ncmreynolds/ld2410
// once in a while to help with issues

#include <Arduino.h>
#include <esp_task_wdt.h>
#include <Wire.h>
#include <NimBLEDevice.h>
#include <math.h>
#include "ld2410.h"
#include "State.h"
#include "Logger.h"

#define SERVICE_UUID        "b71eb828-6e9a-4db3-b11b-0ea799461a10"
#define CHARACTERISTIC_UUID "825fdfcc-9771-11ee-b11b-0ea799461a10"

#define FIRMWARE_VERSION 1.61
#define RADAR_SERIAL Serial1
#define WDT_TIMEOUT 10

#define SCL_PIN 1
#define SDA_PIN 0
#define LED_PIN 2
#define SETTINGS_PIN 8 // v1.3 = 3, v1.4 = 8
#define IN1_PIN 4
#define IN2_PIN 5
#define RELAY_OUT1_PIN 6
#define RELAY_OUT2_PIN 7
#define NEOPIXEL_PIN 3 // v1.3 = 8, v1.4 = 3
#define BOOT_PIN 9 // Never use apart from booting?
#define RADAR_OUT_PIN 10
#define RADAR_RX_PIN 20 // v1.5 = 20, v1.6 = 21
#define RADAR_TX_PIN 21 // v1.5 = 21, v1.6 = 20
#define NEOPIXEL_COUNT 6

enum class PresenceState {
  CLOSE = 0,
  NEAR = 1,
  DISTANT = 2,
  NOTHING = 3,
  UNKNOWN = 5,
};
PresenceState currentState;
PresenceState previousState;

// Visual feedback
State deviceState;

// BLE
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
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
int distance;
int minEnergy = 20; // 20 is default

// Potmeter adjustment
int analogValue1 = 0;
int analogValue2 = 0;
int analogValueOld1 = 0;
int analogValueOld2 = 0;

// Settings
bool wasSettingsPressed = false;
unsigned long now = 0;
unsigned long lastSettingsPress = 0;
int timeSinceLastPress = 0;
int timeSinceLastRelease = 0;
int lastDoublePress = 0;
bool clickDetected;
bool singleClickDetected;
bool doubleClickDetected;
bool doubleClickFastDetected;

bool isStickyRelaysMode()
{
  if( deviceState.getDeviceMode() == DeviceMode::BOTH_SLOW || deviceState.getDeviceMode() == DeviceMode::BOTH_MEDIUM || deviceState.getDeviceMode() == DeviceMode::BOTH_FAST ){
    return true;
  }
  return false;
}

void setPresenceState( PresenceState newState ){
  if( previousState != newState )
  {
    switch( newState ){
      case PresenceState::CLOSE   : 
        digitalWrite(RELAY_OUT1_PIN, HIGH);
        isStickyRelaysMode() ? digitalWrite(RELAY_OUT2_PIN, HIGH) : digitalWrite(RELAY_OUT2_PIN, LOW);
        break;
      case PresenceState::NEAR    : 
        digitalWrite(RELAY_OUT1_PIN, LOW);
        digitalWrite(RELAY_OUT2_PIN, HIGH);
        break;
      case PresenceState::DISTANT :
        digitalWrite(RELAY_OUT1_PIN, LOW);
        digitalWrite(RELAY_OUT2_PIN, LOW);
        break;
      case PresenceState::NOTHING :
        digitalWrite(RELAY_OUT1_PIN, LOW);
        digitalWrite(RELAY_OUT2_PIN, LOW);
        break;
    }
    previousState = newState;
  }
}

/**  None of these are required as they will be handled by the library with defaults. **
 **                       Remove as you see fit for your needs                        */  
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
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
  BLEDevice::init("RadSense1 Maketronics");
  BLEDevice::setMTU(100);

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      NIMBLE_PROPERTY::NOTIFY
                    );

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  /** Note, this could be left out as that is the default value */
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
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
    case FilterMode::WEAK :            minEnergy = 35; break;
    case FilterMode::MEDIUM :          minEnergy = 60; break;
    case FilterMode::STRONG :          minEnergy = 98; break;
    default : minEnergy = 20; break;
  }
}

void setup(void)
{
  deviceState.begin(NEOPIXEL_PIN, NEOPIXEL_COUNT);

  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  delay(4100);

  deviceState.setLedSubMode(0);

  Logger::begin();

  Logger::print(F("\nRadSense1 firmware version "));
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
    while(1){
      deviceState.setPixels(255,0,0);
      delay(200);
      deviceState.setPixels(0,0,0);
      delay(200);
      // Maintain the watchdog
      esp_task_wdt_reset();
    }
  }

  deviceState.setLedSubMode(2);
  setupBluetooth();

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

void updateTheBins( int distance, int firstRange, int secondRange )
{
  // Select where to put the new sample
  if( distance < firstRange ){
    if( readings[0]<requiredConsecutiveReads ){
      readings[0] = readings[0]+2;
    }
  } else if(distance < secondRange){
    if( readings[1]<requiredConsecutiveReads ){
      readings[1] = readings[1]+2;
    }
  } else if(distance < maxDistance){
    if( readings[2]<requiredConsecutiveReads ){
      readings[2] = readings[2]+2;
    }
  } else {
    if( readings[3]<requiredConsecutiveReads ){
      readings[3] = readings[3]+2;
    }
  }
}

void sendBleDebug( int distance, int firstRange, int secondRange, int energy )
{
  // notify of changed value
  if (deviceConnected) {
    updateBluetoothTurnoffTime();
    const int bleStringLength = BLEDevice::getMTU();
    char toSend[100];
    if( deviceState.getLedMode() == LedMode::IMPERIAL )
    {
      int firstRangeIn = round((float)firstRange*0.393701);
      int secondRangeIn = round((float)secondRange*0.393701);
      int distanceIn = round((float)distance*0.393701);
      sprintf(toSend, "Zone 1 %din, Zone 2 %din, Detect %din, e:%d", firstRangeIn,secondRangeIn,distanceIn, energy);
    }
    else
    {
      sprintf(toSend, "Zone 1 %dcm, Zone 2 %dcm, Detect %dcm, e:%d", firstRange,secondRange,distance, energy);
    }
    pCharacteristic->setValue(toSend);
    pCharacteristic->notify();
    // bluetooth stack will go into congestion, if too many packets are sent,
    // but as long as the loop takes more than 3ms, we don't need more delay
    //delay(50);
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
  setPresenceState(previousState);
  deviceState.showActiveDeviceMode();
}

void gotoNextLedMode()
{
  Logger::print(F("gotoNextLedMode"));
  switch( deviceState.getLedMode() )
  {
    case LedMode::TRAFFIC_LIGHT: deviceState.setLedMode( LedMode::IMPERIAL ); break;
    case LedMode::IMPERIAL:      deviceState.setLedMode( LedMode::OFF ); break;
    case LedMode::OFF:           deviceState.setLedMode( LedMode::TRAFFIC_LIGHT ); break;
    default: deviceState.setLedMode( LedMode::TRAFFIC_LIGHT ); break;
  }
  deviceState.showActiveLedMode();
}

void gotoPreviousLedMode()
{
  // Yeah, this is dumb but it works
  Logger::print(F("gotoPreviousLedMode"));
  switch( deviceState.getLedMode() )
  {
    case LedMode::TRAFFIC_LIGHT: deviceState.setLedMode( LedMode::OFF ); break;
    case LedMode::IMPERIAL:      deviceState.setLedMode( LedMode::TRAFFIC_LIGHT ); break;
    case LedMode::OFF:           deviceState.setLedMode( LedMode::IMPERIAL ); break;
    default: deviceState.setLedMode( LedMode::TRAFFIC_LIGHT ); break;
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
    default: deviceState.setLedMode( LedMode::TRAFFIC_LIGHT ); break;
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

void readTheRadar()
{
  bool wasRead = radar.read();
  digitalWrite(LED_PIN, wasRead); // Blink LED for debug

  int energy = radar.stationaryTargetEnergy();
  if(radar.isConnected() && millis() - lastReading > msBetweenReads && energy >= minEnergy)  //Report every 1000ms
  {
    lastReading = millis();
    deleteOneFromAllBins();

    if( radar.presenceDetected() || radar.movingTargetDetected() )
    {
      distance = radar.stationaryTargetDistance()-minDistance;
    }
    else
    {
      distance = 500;
    }

    // Select where to put the new sample
    updateTheBins( distance, firstRange, secondRange );
    sendBleDebug( distance, firstRange, secondRange, energy );
    debugOutput(distance, energy);
  }
  else if( millis() - lastReading > msBetweenReads ) // No radar, but time to update?
  {
    deleteOneFromAllBins();
    distance = 500;
    updateTheBins( distance, firstRange, secondRange );
    sendBleDebug( distance, firstRange, secondRange, energy );
    lastReading = millis();
    debugOutput(distance, energy);
  }
}

void loop()
{
  // Always update the LED based on current state
  int subState = static_cast<int>(previousState);
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
  if(bleOutputEnabled && millis() > turnBluetoothOffAt && !deviceConnected)
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
  distance = 500;
  
  readTheRadar();

  // Color & relay output
  if( readings[0] >= requiredConsecutiveReads ){
    setPresenceState(PresenceState::CLOSE);
  } else if(readings[1] >= requiredConsecutiveReads){
    setPresenceState(PresenceState::NEAR);
  } else if(readings[2] >= requiredConsecutiveReads){
    setPresenceState(PresenceState::DISTANT);
  } else if(readings[3] >= requiredConsecutiveReads){
    setPresenceState(PresenceState::NOTHING);
  }

  // Maintain the watchdog
  esp_task_wdt_reset();
}

#include "State.h"

State::State(){
    _activeLedMode = LedMode::TRAFFIC_LIGHT;
    _activeDeviceMode = DeviceMode::DEFAULT_MEDIUM;
    currentAnimation = AnimationType::NONE;
}

void State::begin(int ledPin, int numLeds)
{
  _ledPin = ledPin;
  _numLeds = numLeds;
  pixels = new Adafruit_NeoPixel(_numLeds, _ledPin, NEO_GRB + NEO_KHZ800);
  pixels->begin();
  setPixels(0, 0, 255);

  preferences.begin("my-app", false);
  unsigned int savedLedMode = preferences.getUInt("LedMode", 1);
  setLedMode( intToLedMode(savedLedMode) );
  Logger::print(F("savedLedMode "));
  Logger::print(savedLedMode);

  unsigned int savedDeviceMode = preferences.getUInt("DeviceMode", 1);
  setDeviceMode( intToDeviceMode(savedDeviceMode) );
  Logger::print(F("savedDeviceMode "));
  Logger::print(savedDeviceMode);

  unsigned int savedFilterMode = preferences.getUInt("FilterMode", 0);
  setFilterMode( intToFilterMode(savedFilterMode) );
  Logger::print(F("savedFilterMode "));
  Logger::print(savedFilterMode);
}

bool State::animating()
{
  return currentAnimation != AnimationType::NONE;
}

void State::maintain()
{
  if( currentAnimation == AnimationType::HOLD ) // LED Modes
  {
    unsigned long now = millis();
    if( now - animationStart > 3000 ){ // finished animating state
      currentAnimation = AnimationType::NONE;
    } else if( now - animationStart > 2000 ){
      setLedSubMode(3);
    } else if( now - animationStart > 1500 ){
      setLedSubMode(2);
    } else if( now - animationStart > 1000 ){
      setLedSubMode(1);
    } else if( now - animationStart > 500 ){
      setLedSubMode(0);
    } else {
      setLedSubMode(3);
    }
  }
  else if( currentAnimation == AnimationType::FADE ) // Device modes
  {
    unsigned long now = millis();
    if( now - animationStart > 3000 ){ // finished animating state
      currentAnimation = AnimationType::NONE;
    } else if( now - animationStart > 2000 ){
      setDeviceSubMode(3);
    } else if( now - animationStart > 1500 ){
      setDeviceSubMode(2);
    } else if( now - animationStart > 1000 ){
      setDeviceSubMode(1);
    } else if( now - animationStart > 500 ){
      setDeviceSubMode(0);
    } else {
      setDeviceSubMode(3);
    }
  }
  else if( currentAnimation == AnimationType::FADE2 ) // Filter modes
  {
    unsigned long now = millis();
    if( now - animationStart > 3000 ){ // finished animating state
      currentAnimation = AnimationType::NONE;
    } else if( now - animationStart > 2000 ){
      setFilterSubMode(3);
    } else if( now - animationStart > 1500 ){
      setFilterSubMode(2);
    } else if( now - animationStart > 1000 ){
      setFilterSubMode(1);
    } else if( now - animationStart > 500 ){
      setFilterSubMode(0);
    } else {
      setFilterSubMode(3);
    }
  }
}

// LED modes handle how the LEDs light up for debugging
void State::showActiveLedMode()
{
  currentAnimation = AnimationType::HOLD;
  animationStart = millis();
}

void State::setLedMode( LedMode newState )
{
  if( newState != _activeLedMode )
  {
    Logger::print(F("LedMode::setLedMode "));
    Logger::print((int)newState);
    _activeLedMode = newState;
    
    // Save to prefs
    int tmpMode = (int)_activeLedMode;
    preferences.putUInt("LedMode", tmpMode);
    Logger::print(F("Wrote LedMode to prefs "));
    Logger::print(tmpMode);
  }
}

void State::setLedSubMode( int newState )
{
  if( _activeSubState != newState )
  {
    int statePosition = static_cast<int>(_activeLedMode)*STATESIZE;
    int subStatePosition = newState*STATEWIDTH;
    int r = ALLSTATES[statePosition+subStatePosition+0];
    int g = ALLSTATES[statePosition+subStatePosition+1];
    int b = ALLSTATES[statePosition+subStatePosition+2];
    /*
    Logger::print(millis());
    Logger::print(", ");
    Logger::print(statePosition);
    Logger::print(", ");
    Logger::print(statePosition+subStatePosition+0);
    Logger::print(", ");
    Logger::print(statePosition+subStatePosition+1);
    Logger::print(", ");
    Logger::print(statePosition+subStatePosition+2);
    Logger::print(", ");
    
    Logger::print("LedState::setLedSubMode ");
    Logger::println(newState);
    */
    setPixels(r, g, b);
  }
  _activeSubState = newState;
}

LedMode State::getLedMode()
{
  return _activeLedMode;
}

// Device modes handle how the device works

void State::showActiveDeviceMode()
{
  Logger::print(F("DeviceMode::showActiveDeviceMode "));
  currentAnimation = AnimationType::FADE;
  animationStart = millis();
}

void State::setDeviceMode( DeviceMode newDeviceMode )
{
  if( newDeviceMode != _activeDeviceMode )
  {
    Logger::print(F("DeviceMode::setDeviceMode "));
    Logger::print((int)newDeviceMode);
    _activeDeviceMode = newDeviceMode;
    // Save to prefs
    int tmpMode = (int)_activeDeviceMode;
    preferences.putUInt("DeviceMode", tmpMode);
    Logger::print(F("Wrote DeviceMode to prefs "));
    Logger::print(tmpMode);
  }
}

void State::setDeviceSubMode( int newDeviceMode )
{
  int statePosition = (static_cast<int>(_activeDeviceMode)+LEDMODES) * STATESIZE;
  int subStatePosition = newDeviceMode*STATEWIDTH;

  if( _activeSubState != newDeviceMode )
  {
    fadeBrightness = 255;
    /*
    Logger::print(millis());
    Logger::print(", ");
    Logger::print(statePosition);
    Logger::print(", ");
    Logger::print(statePosition+subStatePosition+0);
    Logger::print(", ");
    Logger::print(statePosition+subStatePosition+1);
    Logger::print(", ");
    Logger::print(statePosition+subStatePosition+2);
    Logger::print(", ");
    
    Logger::print("State::setDeviceSubMode ");
    Logger::println(newDeviceMode);
    */
  }

  // Always fade...
  int r = ALLSTATES[statePosition+subStatePosition+0] > 10 ? fadeBrightness : 5;
  int g = ALLSTATES[statePosition+subStatePosition+1] > 10 ? fadeBrightness : 5;
  int b = ALLSTATES[statePosition+subStatePosition+2] > 10 ? fadeBrightness : 5;
  setPixels(r, g, b);
  fadeBrightness -= 1;
  fadeBrightness < 0 ? fadeBrightness = 0 : fadeBrightness;
  
  _activeSubState = newDeviceMode;
}

DeviceMode State::getDeviceMode()
{
  return _activeDeviceMode;
}

void State::showActiveFilterMode()
{
  Logger::print(F("DeviceMode::showActiveFilterMode "));
  currentAnimation = AnimationType::FADE2;
  animationStart = millis();
}

void State::setFilterMode( FilterMode newFilterMode )
{
  if( newFilterMode != _activeFilterMode )
  {
    Logger::print(F("State::setFilterMode "));
    Logger::print((int)newFilterMode);
    _activeFilterMode = newFilterMode;
    // Save to prefs
    int tmpMode = (int)_activeFilterMode;
    preferences.putUInt("FilterMode", tmpMode);
    Logger::print(F("Wrote FilterMode to prefs "));
    Logger::print(tmpMode);
  }
}

void State::setFilterSubMode( int newFilterMode )
{
  int statePosition = (static_cast<int>(_activeFilterMode)+LEDMODES+DEVICEMODES) * STATESIZE;
  int subStatePosition = newFilterMode*STATEWIDTH;

  if( _activeSubState != newFilterMode )
  {
    fadeBrightness = 255;
    /*
    Logger::print(millis());
    Logger::print(", ");
    Logger::print(statePosition);
    Logger::print(", ");
    Logger::print(statePosition+subStatePosition+0);
    Logger::print(", ");
    Logger::print(statePosition+subStatePosition+1);
    Logger::print(", ");
    Logger::print(statePosition+subStatePosition+2);
    Logger::print(", ");
    
    Logger::print("State::setFilterSubMode ");
    Logger::print(newFilterMode);
    */
  }

  // Always fade...
  int r = ALLSTATES[statePosition+subStatePosition+0] > 10 ? fadeBrightness : 5;
  int g = ALLSTATES[statePosition+subStatePosition+1] > 10 ? fadeBrightness : 5;
  int b = ALLSTATES[statePosition+subStatePosition+2] > 10 ? fadeBrightness : 5;
  setPixels(r, g, b);
  fadeBrightness -= 1;
  fadeBrightness < 0 ? fadeBrightness = 0 : fadeBrightness;
  
  _activeSubState = newFilterMode;
}

FilterMode State::getFilterMode()
{
  return _activeFilterMode;
}

void State::setPixels(uint8_t r, uint8_t g, uint8_t b)
{
  for(int i=0;i<_numLeds;i++)
  {
    pixels->setPixelColor(i, r, g, b);
  }
  pixels->show();
}

LedMode State::intToLedMode( int state )
{
  switch( state )
  {
    case 0: return LedMode::OFF;
    case 1: return LedMode::TRAFFIC_LIGHT;
    case 2: return LedMode::IMPERIAL;
    default: return LedMode::TRAFFIC_LIGHT;
  }
}

DeviceMode State::intToDeviceMode( int state )
{
  switch( state )
  {
    case 0: return DeviceMode::DEFAULT_SLOW;
    case 1: return DeviceMode::DEFAULT_MEDIUM;
    case 2: return DeviceMode::DEFAULT_FAST;
    case 3: return DeviceMode::BOTH_SLOW;
    case 4: return DeviceMode::BOTH_MEDIUM;
    case 5: return DeviceMode::BOTH_FAST;
    default: return DeviceMode::DEFAULT_MEDIUM;
  }
}

FilterMode State::intToFilterMode( int state )
{
  switch( state )
  {
    case 0: return FilterMode::WEAK;
    case 1: return FilterMode::MEDIUM;
    case 2: return FilterMode::STRONG;
    default: return FilterMode::STRONG;
  }
}
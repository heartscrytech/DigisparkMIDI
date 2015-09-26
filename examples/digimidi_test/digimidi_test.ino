#include <DigiMIDI.h>


DigiMIDIDevice midi;


#define DEBUG 1  // Set to 1 to enable, 0 to disable
 
#if DEBUG
#define DebugPin 1  // Digispark model A onboard LED
#define DebugBlink 75
#define DebugPause 300
#define debugDelay(ms) delay(ms)  // Change if you need a special delay function (e.g. if you use libraries that need regular refresh calls)
 
void _debugBlink(int n) {
  for ( int i = 0 ; i < n ; i++ ) {
    digitalWrite(DebugPin, HIGH);
    debugDelay(DebugBlink);
    digitalWrite(DebugPin, LOW);
    debugDelay(DebugBlink);
  }
  debugDelay(DebugPause);
}
 
void _debugSetup() {
  pinMode(DebugPin, OUTPUT);
}
 
#define debugBlink(n) _debugBlink(n)  // Do the blink when DEBUG is set
#define debugSetup() _debugSetup()
#else
#define debugBlink(n)  // Make the calls disappear when DEBUG is 0
#define debugSetup()
#endif


void setup() {
  // put your setup code here, to run once:
  debugSetup();
  debugBlink(3);
}

void loop() {
  // put your main code here, to run repeatedly:
  midi.update();
  
  midi.delay(500);
  debugBlink(1);
  // Note number, velocity (opt=channel)
  midi.sendNoteOn(62,32);
  debugBlink(1);
}

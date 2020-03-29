#define PIN_MS0 0
#define PIN_SS0 1
#define PIN_SS1 2
#define PIN_SS2 3
#define PIN_SS3 4
#define PIN_KEY_READ 5
#define PIN_POWER_LED 13
#define PIN_KNOB_0 A0
#define PIN_KNOB_1 A1
#define PIN_KNOB_2 A2
#define PIN_KNOB_3 A3
#define PIN_KNOB_4 A4
#define PIN_KNOB_5 A5
#define PIN_KNOB_6 A6
#define PIN_KNOB_7 A7
#define PIN_AUDIO_OUT A21

#define BASE_OCTAVE (2)
#define M_TAU (6.283185307179586)

const int keyCount = 32;

struct {
  bool isDown;
} keys[keyCount];

unsigned short knobs[8];

unsigned long timeNow;
unsigned long timePrev;

void setup() {
  pinMode(PIN_POWER_LED, OUTPUT);
  digitalWriteFast(PIN_POWER_LED, HIGH);
  
  analogWriteResolution(12);
  
  pinMode(PIN_MS0, OUTPUT);
  pinMode(PIN_SS0, OUTPUT);
  pinMode(PIN_SS1, OUTPUT);
  pinMode(PIN_SS2, OUTPUT);
  pinMode(PIN_SS3, OUTPUT);
  
  pinMode(PIN_KEY_READ, INPUT_PULLDOWN);
  
  pinMode(PIN_KNOB_0, INPUT);
  pinMode(PIN_KNOB_1, INPUT);
  pinMode(PIN_KNOB_2, INPUT);
  pinMode(PIN_KNOB_3, INPUT);
  pinMode(PIN_KNOB_4, INPUT);
  pinMode(PIN_KNOB_5, INPUT);
  pinMode(PIN_KNOB_6, INPUT);
  pinMode(PIN_KNOB_7, INPUT);
  
  pinMode(PIN_AUDIO_OUT, OUTPUT);
  
  Serial.begin(9600);
}

float keyIndexToFreq(int key) {
  const int notesPerOctave = 12;
  
  float freqs[notesPerOctave] = {
    21.8267578125, // F
    23.1246484375, // F#
    24.4997265625, // G
    25.9565625,    // G#
    27.5,          // A
    29.135234375,  // A#
    30.8676953125, // B
    32.703203125,  // C
    34.6478125,    // C#
    36.708046875,  // D
    38.890859375,  // D#
    41.2034375,    // E
  };
  
  int freqIndex = key % notesPerOctave;
  int octave = BASE_OCTAVE + (key / notesPerOctave);
  
  return freqs[freqIndex] * powf(2, octave);
}

void updateKeyState() {
  static unsigned long lastKeyReadTime = 0;
  static int keyIndex = 0;
  
  if (timeNow - lastKeyReadTime < 200) return; // Too soon to read the key
  
  keys[keyIndex].isDown = digitalRead(PIN_KEY_READ) == HIGH;
  
  if (++keyIndex >= 32) {
    keyIndex = 0;
    static int knobIndex = 0;
    
    switch (knobIndex) {
      case 0: knobs[knobIndex] = analogRead(PIN_KNOB_0); break;
      case 1: knobs[knobIndex] = analogRead(PIN_KNOB_1); break;
      case 2: knobs[knobIndex] = analogRead(PIN_KNOB_2); break;
      case 3: knobs[knobIndex] = analogRead(PIN_KNOB_3); break;
      case 4: knobs[knobIndex] = analogRead(PIN_KNOB_4); break;
      case 5: knobs[knobIndex] = analogRead(PIN_KNOB_5); break;
      case 6: knobs[knobIndex] = analogRead(PIN_KNOB_6); break;
      case 7: knobs[knobIndex] = analogRead(PIN_KNOB_7); break;
    }
    
    if (++knobIndex > 7) knobIndex = 0;
  }
  
  int SS0Setting = keyIndex & 0x01 ? HIGH : LOW;
  int SS1Setting = keyIndex & 0x02 ? HIGH : LOW;
  int SS2Setting = keyIndex & 0x04 ? HIGH : LOW;
  int SS3Setting = keyIndex & 0x08 ? HIGH : LOW;
  int MS0Setting = keyIndex & 0x10 ? HIGH : LOW;
  
  digitalWriteFast(PIN_MS0, MS0Setting);
  digitalWriteFast(PIN_SS0, SS0Setting);
  digitalWriteFast(PIN_SS1, SS1Setting);
  digitalWriteFast(PIN_SS2, SS2Setting);
  digitalWriteFast(PIN_SS3, SS3Setting);
  
  lastKeyReadTime = micros();
}

float getOutputSample() {
  unsigned timeDelta = timeNow - timePrev;
  static unsigned long basePhaseUs = 0;
  
  basePhaseUs += timeDelta;
  float baseAngle = (basePhaseUs / 1000000.0f) * M_TAU;
  
  float summedSamples = 0;
  unsigned char keyDownCount = 0;
  
  for (int k = 0; k < keyCount; k++) {
    if (keys[k].isDown) {
      float s = sinf(baseAngle * keyIndexToFreq(k));
      
      // Apply distortion
      for (int i = 0; i < knobs[2] / 100; i++) {
        s = sinf(M_PI * (sinf(s) / 2));
      }
      
      summedSamples += s;
      keyDownCount++;
    } else keys[k].vol = 0;
  }
  
  if (keyDownCount == 0) basePhaseUs = 0;
  
  return summedSamples;
}

void loop() {
  timeNow = micros();
  static int count = 0;
  
  // Temporal resolution monitoring
  // if (count++ > 10000) {
  //   Serial.println(timeNow - timePrev);
  //   count = 0;
  // }
  
  updateKeyState();
  
  int intSample = 2048 + getOutputSample() * knobs[0] * 2;
  analogWrite(PIN_AUDIO_OUT, intSample);
  
  timePrev = timeNow;
}



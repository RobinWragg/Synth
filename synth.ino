int16_t sinLookup16Bit[256] = {
  0,804,1607,2410,3211,4011,4807,5601,6392,7179,7961,8739,9511,10278,11038,11792,12539,13278,14009,14732,15446,16150,16845,17530,18204,18867,19519,20159,20787,21402,22004,22594,23169,23731,24278,24811,25329,25831,26318,26789,27244,27683,28105,28510,28897,29268,29621,29955,30272,30571,30851,31113,31356,31580,31785,31970,32137,32284,32412,32520,32609,32678,32727,32757,32766,32757,32727,32678,32609,32520,32412,32284,32137,31970,31785,31580,31356,31113,30851,30571,30272,29955,29621,29268,28897,28510,28105,27683,27244,26789,26318,25831,25329,24811,24278,23731,23169,22594,22004,21402,20787,20159,19519,18867,18204,17530,16845,16150,15446,14732,14009,13278,12539,11792,11038,10278,9511,8739,7961,7179,6392,5601,4807,4011,3211,2410,1607,804,0,-804,-1607,-2410,-3211,-4011,-4807,-5601,-6392,-7179,-7961,-8739,-9511,-10278,-11038,-11792,-12539,-13278,-14009,-14732,-15446,-16150,-16845,-17530,-18204,-18867,-19519,-20159,-20787,-21402,-22004,-22594,-23169,-23731,-24278,-24811,-25329,-25831,-26318,-26789,-27244,-27683,-28105,-28510,-28897,-29268,-29621,-29955,-30272,-30571,-30851,-31113,-31356,-31580,-31785,-31970,-32137,-32284,-32412,-32520,-32609,-32678,-32727,-32757,-32766,-32757,-32727,-32678,-32609,-32520,-32412,-32284,-32137,-31970,-31785,-31580,-31356,-31113,-30851,-30571,-30272,-29955,-29621,-29268,-28897,-28510,-28105,-27683,-27244,-26789,-26318,-25831,-25329,-24811,-24278,-23731,-23169,-22594,-22004,-21402,-20787,-20159,-19519,-18867,-18204,-17530,-16845,-16150,-15446,-14732,-14009,-13278,-12539,-11792,-11038,-10278,-9511,-8739,-7961,-7179,-6392,-5601,-4807,-4011,-3211,-2410,-1607,-804
};

int16_t sinDistortedLookup16Bit[256] = {
  0,9522,11995,13729,15107,16269,17282,18185,19004,19753,20446,21092,21695,22263,22799,23307,23789,24247,24685,25102,25501,25883,26249,26600,26937,27260,27570,27868,28155,28430,28694,28948,29192,29426,29650,29865,30072,30269,30458,30639,30811,30976,31132,31281,31422,31556,31682,31801,31913,32018,32115,32206,32289,32366,32436,32499,32555,32605,32648,32684,32714,32737,32753,32763,32766,32763,32753,32737,32714,32684,32648,32605,32555,32499,32436,32366,32289,32206,32115,32018,31913,31801,31682,31556,31422,31281,31132,30976,30811,30639,30458,30269,30072,29865,29650,29426,29192,28948,28694,28430,28155,27868,27570,27260,26937,26600,26249,25883,25501,25102,24685,24247,23789,23307,22799,22263,21695,21092,20446,19753,19004,18185,17282,16269,15107,13729,11995,9522,-145,-9522,-11995,-13729,-15107,-16269,-17282,-18185,-19004,-19753,-20446,-21092,-21695,-22263,-22799,-23307,-23789,-24247,-24685,-25102,-25501,-25883,-26249,-26600,-26937,-27260,-27570,-27868,-28155,-28430,-28694,-28948,-29192,-29426,-29650,-29865,-30072,-30269,-30458,-30639,-30811,-30976,-31132,-31281,-31422,-31556,-31682,-31801,-31913,-32018,-32115,-32206,-32289,-32366,-32436,-32499,-32555,-32605,-32648,-32684,-32714,-32737,-32753,-32763,-32766,-32763,-32753,-32737,-32714,-32684,-32648,-32605,-32555,-32499,-32436,-32366,-32289,-32206,-32115,-32018,-31913,-31801,-31682,-31556,-31422,-31281,-31132,-30976,-30811,-30639,-30458,-30269,-30072,-29865,-29650,-29426,-29192,-28948,-28694,-28430,-28155,-27868,-27570,-27260,-26937,-26600,-26249,-25883,-25501,-25102,-24685,-24247,-23789,-23307,-22799,-22263,-21695,-21092,-20446,-19753,-19004,-18185,-17282,-16269,-15107,-13729,-11995,-9522
};

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
#define KEY_COUNT 32
#define RELEASE_DURATION 3000

uint32_t timeNow;
uint32_t timePrev;

struct Key {
  bool isActive, isDown;
  uint32_t elapsedUs, keyDownDuration;
  float volume, keyUpVolume;
} keys[KEY_COUNT];

struct Envelope {
  uint16_t attackKnob;
  uint16_t decayKnob;
  uint16_t sustainKnob;
  
  uint32_t attackDuration;
  uint32_t decayDuration;
  float sustainVolume;
} env;

uint16_t masterVolume2048 = 0;
float distortionFactor = 0;

uint32_t nextDebugFlash = 0;
bool debugFlashHigh = false;

void setup() {
  pinMode(PIN_POWER_LED, OUTPUT);
  digitalWriteFast(PIN_POWER_LED, HIGH);
  
  analogWriteResolution(12);
  
  pinMode(PIN_MS0, OUTPUT);
  pinMode(PIN_SS0, OUTPUT);
  pinMode(PIN_SS1, OUTPUT);
  pinMode(PIN_SS2, OUTPUT);
  pinMode(PIN_SS3, OUTPUT);
  
  pinMode(PIN_KEY_READ, INPUT);
  
  pinMode(PIN_KNOB_0, INPUT);
  pinMode(PIN_KNOB_1, INPUT);
  pinMode(PIN_KNOB_2, INPUT);
  pinMode(PIN_KNOB_3, INPUT);
  pinMode(PIN_KNOB_4, INPUT);
  pinMode(PIN_KNOB_5, INPUT);
  pinMode(PIN_KNOB_6, INPUT);
  pinMode(PIN_KNOB_7, INPUT);
  
  pinMode(PIN_AUDIO_OUT, OUTPUT);
  
  for (int i = 0; i < KEY_COUNT; i++) {
    keys[i].isActive = false;
    keys[i].isDown = false;
  }
  
  Serial.begin(9600);
  
  delay(1000);
  
  // for (int i = 0; i < 256; i++) {
  //   double frac = i / 256.0f;
  //   double theta = frac * M_TAU;
  //   Serial.print(int(cbrt(sin(theta)) * 32767));
  //   Serial.print(",");
  // }
}

float keyIndexTo8BitFreq(int key) {
  const int notesPerOctave = 12;
  
  const static float freqs8Bit[notesPerOctave] = {
    // Frequency    Nanoseconds  8-bit
    21.8267578125 / 1000000.0f * 256, // F
    23.1246484375 / 1000000.0f * 256, // F#
    24.4997265625 / 1000000.0f * 256, // G
    25.9565625    / 1000000.0f * 256, // G#
    27.5          / 1000000.0f * 256, // A
    29.135234375  / 1000000.0f * 256, // A#
    30.8676953125 / 1000000.0f * 256, // B
    32.703203125  / 1000000.0f * 256, // C
    34.6478125    / 1000000.0f * 256, // C#
    36.708046875  / 1000000.0f * 256, // D
    38.890859375  / 1000000.0f * 256, // D#
    41.2034375    / 1000000.0f * 256, // E
  };
  
  int freqIndex = key % notesPerOctave;
  int octave = BASE_OCTAVE + (key / notesPerOctave);
  
  return freqs8Bit[freqIndex] * powf(2, octave);
}

void calcEnvelope() {
  env.attackDuration = env.attackKnob * env.attackKnob;
  env.decayDuration = env.decayKnob * env.decayKnob;
  env.sustainVolume = env.sustainKnob / 1024.0f;
}

void computeKeyEnvelope(Key &key) {
  if (key.isDown) {
    if (key.elapsedUs < env.attackDuration) {
      key.volume = key.elapsedUs / (float)env.attackDuration;
    } else if (key.elapsedUs < env.attackDuration + env.decayDuration) {
      uint32_t decayElapsed = key.elapsedUs - env.attackDuration;
      float lerpInput = decayElapsed / (float)env.decayDuration;
      key.volume = 1 + lerpInput * (env.sustainVolume - 1);
    } else key.volume = env.sustainVolume;
  } else {
    uint32_t releaseElapsed = key.elapsedUs - key.keyDownDuration;
    
    if (releaseElapsed > RELEASE_DURATION) {
      key.isActive = false;
      key.volume = 0.0f;
    } else {
      float lerpInput = releaseElapsed / (float)RELEASE_DURATION;
      key.volume = key.keyUpVolume - lerpInput * key.keyUpVolume;
    }
  }
}

void updateKeyState() {
  static uint32_t lastKeyReadTime = 0;
  static uint8_t keyIndex = 0;
  
  if (timeNow - lastKeyReadTime < 500) return; // Too soon to read the key
  
  if (keyIndex < 32) {
    bool keyIsDown = digitalRead(PIN_KEY_READ) == HIGH;
    
    // Reset the note duration timer if the key's state has changed
    if (keyIsDown != keys[keyIndex].isDown) {
      keys[keyIndex].elapsedUs = 0;
      keys[keyIndex].isDown = keyIsDown;
      if (keyIsDown) keys[keyIndex].isActive = true;
      else {
        keys[keyIndex].keyDownDuration = keys[keyIndex].elapsedUs;
        keys[keyIndex].keyUpVolume = keys[keyIndex].volume;
      }
    }
    
    keyIndex++;
  } else {
    keyIndex = 0;
    static int knobIndex = 0;
    
    switch (knobIndex) {
      case 0: masterVolume2048 = analogRead(PIN_KNOB_0) * 2; break;
      case 1: analogRead(PIN_KNOB_1); break;
      case 2: distortionFactor = analogRead(PIN_KNOB_2) / 1024.0f; break;
      case 3: analogRead(PIN_KNOB_3); break;
      case 4: analogRead(PIN_KNOB_4); break;
      
      // ADSR
      case 5: env.attackKnob = analogRead(PIN_KNOB_5); break;
      case 6: env.decayKnob = analogRead(PIN_KNOB_6); break;
      case 7: env.sustainKnob = analogRead(PIN_KNOB_7); break;
      case 8: calcEnvelope(); break;
    }
    
    if (++knobIndex > 8) knobIndex = 0;
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

float sin16Bit(uint8_t theta8Bit) {
  return sinLookup16Bit[theta8Bit] / 32768.0f; // TODO
}

float sin3Root16Bit(uint8_t theta8Bit) {
  return sinDistortedLookup16Bit[theta8Bit] / 32768.0f; // TODO
}

float getOutputSample() {
  uint32_t timeDelta = timeNow - timePrev;
  static uint32_t basePhase32Bit = 0;
  
  basePhase32Bit += timeDelta;
  
  float summedSamples = 0;
  for (int k = 0; k < KEY_COUNT; k++) {
    if (keys[k].isActive) {
      Key &key = keys[k];
      key.elapsedUs += timeDelta;
      
      // TODO: Try recursive sine for distortion: sin(sin(theta) * pi/2) etc.
      float sinInput = basePhase32Bit * keyIndexTo8BitFreq(k);
      float sinResult = sin16Bit(sinInput);
      float sin3RootResult = sin3Root16Bit(sinInput);
      
      float blendedSinResult = sinResult + distortionFactor * (sin3RootResult - sinResult);
      
      computeKeyEnvelope(key);
      summedSamples += blendedSinResult * key.volume;
    }
  }
  
  return summedSamples;
}



void loop() {
  timeNow = micros();
  
  // Temporal resolution monitoring
  // static int count = 0;
  // if (count++ > 1000) {
  //   Serial.println(timeNow - timePrev);
  //   count = 0;
  // }
  
  updateKeyState();
  
  int intSample = 2048 + getOutputSample() * masterVolume2048;
  analogWrite(PIN_AUDIO_OUT, intSample);
  
  timePrev = timeNow;
}




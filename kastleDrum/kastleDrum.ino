/* wonkystuff fork of Bastl software. Original banner comment below.
 * This update 2021 by John Tuffen
 * open source license: CC BY SA
 *
 * https://wonkystuff.net/
 *
 * History:
 * 1. Removed commented-out code for clarity, fixed bug which prevented compilation and tidied warnings.
 *    in Arduino 1.8.13
 */

/*
  KASTLE DRUM

The Kastle Drum is a special edition of Bastl’s classic mini modular synth focusing on algorithmic industrial glitchy drums
How do you generate rhythm on a patchable drum machine that has neither buttons, nor a programmable sequencer? You discover it!

Drum sound synthesis with a unique dynamic acceleration charged envelope makes this rhythm box surprisingly versatile and extremely fun to play.
The built-in VC clock generator with a stepped pattern sequencer can either run on its own or it can be synchronized to analog clock,
while retaining the triangle LFO for parameter modulation.

The Kastle Drum is a mini modular synthesizer with a headphone output,
2 in/out ports for interfacing other gear, and it runs on just 3 AA batteries. It is ideal for beginners in modular synthesis,
but it will add some quite unique functionality to any modular synthesizer system. It delivers the fun of modular synthesis at a
low price and fits into your pocket so you can play it anywhere!



Kastle Drum Features
  -8 drum synthesis styles
  -”noises” output for less tonal content
  -DRUM selects drum sounds
  -acceleration charge dynamic envelope
  -decay time
  -PITCH control with offset and CV input with attenuator
  -voltage-controllable clock with square and triangle output
  -stepped voltage generator with random, 8 step and 16 step loop mode
  -2 I/O CV ports that can be routed to any patch point
  -the main output can drive headphones
  -3x AA battery operation or USB power selectable by a switch
  -open source
  -durable black & gold PCB enclosure



  Writen by Vaclav Pelousek 2020
  based on the earlier kastle v1.5
  open source license: CC BY SA
  http://www.bastl-instruments.com

  -this is the code for the VCO chip of the Kastle
  -software written in Arduino 1.8.12 - used to flash ATTINY 85 running at 8mHz
  http://highlowtech.org/?p=1695
  -created with help of the heavenly powers of internet and several tutorials that you can google out
  -i hope somebody finds this code usefull (i know it is a mess :( )

  thanks to
  -Lennart Schierling for making some work on the register access
  -Uwe Schuller for explaining the capacitance of zener diodes
  -Peter Edwards for making the inspireing bitRanger
  -Ondrej Merta for being the best boss
  -and the whole bastl crew that made this project possible
  -Arduino community and the forums for picking ups bits of code to setup the timers & interrupts
  -v1.5 and kastle drum uses bits of code from miniMO DCO http://www.minimosynth.com/
*/

#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/io.h>        // Adds useful constants
#include <util/delay.h>

#include "TR_HH_AT.h"

//global variables
#define WSMAP_POINTS 5
uint16_t wsMap[10] = {
  0, 63, 127, 191, 234,   15, 100, 160, 210, 254
};


uint8_t _out;
uint16_t time;
uint8_t mode;
uint8_t analogChannelRead = 1;
uint8_t analogValues[4];
uint8_t lastAnalogValues[4];
uint8_t out;
uint8_t pwmCounter;
uint8_t _clocks;
bool flop;
uint8_t incr = 6, _incr = 6;
uint8_t lastOut;
uint8_t bitShift = 3;
uint16_t osc2offset = 255;
uint8_t lastAnalogChannelRead;
bool firstRead = false;

uint8_t pwmIncrement, _upIncrement, _downIncrement, upIncrement, downIncrement;
bool quantizer;
const uint8_t analogToDigitalPinMapping[4] = {
  PORTB5, PORTB2, PORTB4, PORTB3
};


//defines for synth types
//all are dual oscillator setups -
#define NOISE 1 // phase distortion -
#define FM 0 //aka phase modulation
#define TAH 2 //aka track & hold modulation (downsampling with T&H)


#define LOW_THRES 150
#define HIGH_THRES 162
#define LOW_MIX 300
#define HIGH_MIX 900

//defines for synth parameters
#define PITCH 3
#define WS_1 2
#define WS_2 1

const uint8_t PROGMEM sinetable[128] = {
  0,   0,   0,   0,   1,   1,   1,   2,   2,   3,   4,   5,   5,   6,   7,   9,
  10,  11,  12,  14,  15,  17,  18,  20,  21,  23,  25,  27,  29,  31,  33,  35,
  37,  40,  42,  44,  47,  49,  52,  54,  57,  59,  62,  65,  67,  70,  73,  76,
  79,  82,  85,  88,  90,  93,  97,  100, 103, 106, 109, 112, 115, 118, 121, 124,
  128, 131, 134, 137, 140, 143, 146, 149, 152, 155, 158, 162, 165, 167, 170, 173,
  176, 179, 182, 185, 188, 190, 193, 196, 198, 201, 203, 206, 208, 211, 213, 215,
  218, 220, 222, 224, 226, 228, 230, 232, 234, 235, 237, 238, 240, 241, 243, 244,
  245, 246, 248, 249, 250, 250, 251, 252, 253, 253, 254, 254, 254, 255, 255, 255,
};

//the actual table that is read to generate the sound
unsigned char wavetable[256];


uint32_t curveMap(uint8_t value, uint8_t numberOfPoints, uint16_t * tableMap) {
  uint32_t inMin = 0, inMax = 255, outMin = 0, outMax = 255;
  for (int i = 0; i < numberOfPoints - 1; i++) {
    if (value >= tableMap[i] && value <= tableMap[i + 1]) {
      inMax = tableMap[i + 1];
      inMin = tableMap[i];
      outMax = tableMap[numberOfPoints + i + 1];
      outMin = tableMap[numberOfPoints + i];
      i = numberOfPoints + 10;
    }
  }
  return map(value, inMin, inMax, outMin, outMax);
}


void createLookup() {
  for (uint16_t i = 0; i < 256; i++) {
    // mapLookup[i]=curveMap(i,WSMAP_POINTS,wsMap);
  }
}


bool XYmode;
uint8_t startupRead = 0;

uint8_t decayVolume;
uint16_t decayTime = 50;
uint8_t _sample;
uint8_t _saw, _lastSaw;

uint8_t decayVolume2 = 0;

void setup()  { //happends at the startup
  writeWave(0);
  digitalWrite(5, HIGH); //turn on pull up resistor for the reset pin
  //set outputs
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);

  setTimers(); //setup interrupts

  //setup ADC and run it in interrupt
  init();
  connectChannel(analogChannelRead);
  startConversion();
  _delay_us(100);
  while (startupRead < 12) {
    loop();
  }
  XYmode = true; //if all pots are hight and mode is HIGH than render XY mode instead
  if (analogValues[0] < HIGH_THRES)  XYmode = false;
  for (uint8_t i = 1; i < 4; i++) if (analogValues[i] < 200) XYmode = false; //HIGH_THRES

}

void setTimers(void)
{
  PLLCSR |= (1 << PLLE);               // Enable PLL (64 MHz)
  _delay_us(100);                      // Wait for a steady state
  while (!(PLLCSR & (1 << PLOCK)));    // Ensure PLL lock
  PLLCSR |= (1 << PCKE);               // Enable PLL as clock source for timer 1
  PLLCSR |= (1 << LSM); //low speed mode 32mhz
  cli();                               // Interrupts OFF (disable interrupts globally)


  TCCR0A = 2 << COM0A0 | 2 << COM0B0 | 3 << WGM00;
  TCCR0B = 0 << WGM02 | 1 << CS00;



  //  setup timer 0 to run fast for audiorate interrupt

  TCCR1 = 0;                  //stop the timer
  TCNT1 = 0;                  //zero the timer
  GTCCR = _BV(PSR1);          //reset the prescaler
  OCR1A = 255;                //set the compare value
  OCR1C = 255;

  TIMSK = _BV(OCIE1A);// | _BV(TOIE0);    //TOIE0    //interrupt on Compare Match A
  //start timer, ctc mode, prescaler clk/1
  TCCR1 = _BV(CTC1) | _BV(CS12);

  sei();
}

uint16_t clocks() {
  //return _clocks;
  return TCNT0 | (_clocks << 8);
}
void writeWave(int wave) {
  switch (wave) {
    case 0:
      sineWave();
      break;
    case 1:
      triangleWave();
      break;
    case 2:
      squareWave();
      break;
    case 3:
      sawtoothWave();
      break;
    case 4:
      digitalWrite(0, LOW);
      zeroWave();
      break;
  }
}

//functions to populate the wavetable
void sineWave() {                                       //too costly to calculate on the fly, so it reads from the sine table. We use 128 values, then mirror them to get the whole cycle
  for (int i = 0; i < 128; ++i) {
    wavetable[i] = pgm_read_byte_near(sinetable + i);
  }
  wavetable[128] = 255;
  for (int i = 129; i < 256; ++i) {
    wavetable[i] = wavetable[256 - i] ;
  }
}
void sawtoothWave() {
  for (int i = 0; i < 256; ++i) {
    wavetable[i] = i; // sawtooth
  }
}
void triangleWave() {
  for (int i = 0; i < 128; ++i) {
    wavetable[i] = i * 2;
  }
  int value = 255;
  for (int i = 128; i < 256; ++i) {
    wavetable[i] = value;
    value -= 2;
  }
}
void squareWave() {
  for (int i = 0; i < 128; ++i) {
    wavetable[i] = 255;
  }
  for (int i = 128; i < 256; ++i) {
    wavetable[i] = 1;                  //0 gives problems (offset and different freq), related to sample  = ((wavetable[phase >> 8]*amplitude)>>8);
  }
}
void zeroWave() {
  for (int i = 0; i < 256; ++i) {
    wavetable[i] = 1;                  //0 gives problems
  }
}

byte sample, sample90;
unsigned int _phase, _lastPhase;
unsigned int frequency;
byte sample2;
unsigned int _phase2, _phase4, _phase5, _phase6;
unsigned int frequency2, frequency4, frequency5, frequency6;
uint8_t _phs, _phs90;
uint8_t _phase3;
uint8_t pitchEnv;

ISR(TIMER1_COMPA_vect)  // render primary oscillator in the interupt
{
  OCR0A = sample;//(sample+sample2)>>1;
  OCR0B = sample2;//_phs;// sample90;

  //_lastPhase=_phase;
  if (pitchEnv) {
    _phase += (frequency + (decayVolume));
    _phase3 += (frequency + (decayVolume)); //frequency;//
    _phase2 += (frequency2 + (decayVolume));
  }
  else {
    _phase += frequency;
    _phase3 += frequency;
    _phase2 += frequency2;
  }
  if(!(_phase%4) )_phase4 += frequency4;


  if (mode) { // other than FM, FM=0
  }
  else {
    _phs = (_phase + (analogValues[WS_2] * wavetable[_phase2 >> 8])) >> 6;
    sample = ((wavetable[_phs] ) * decayVolume) >> 8;
  }
}

uint16_t sampleEnd;
const uint8_t multiplier[24] = {
  2, 2, 2, 3, 1, 2, 4, 4, 3, 4, 5, 2, 1, 5, 6, 8, 3, 8, 7, 8, 7, 6, 8, 16
};
void setFrequency2(uint16_t input) {

  mode = input >> 7;

  if (mode > 6) bitWrite(TCCR0B, CS00, 0), bitWrite(TCCR0B, CS01, 1);
  else bitWrite(TCCR0B, CS00, 1), bitWrite(TCCR0B, CS01, 0);

  frequency2 = (input << 4) + 1;
  frequency4 = 512 - input;
}

void setLength(uint8_t _length) {
  sampleEnd = map(_length, 0, 255, 0, sampleLength);
}

void setFrequency(uint16_t input) {
  int addEnv = 0; //((pitchEnv*decayVolume)>>7);
  if (   mode == NOISE ) frequency = ((input - 200) << 2) + 1; //NOISE
  else if (mode > 3) frequency = (input) + 1;
  else frequency = ((input + addEnv) << 2) + 1;
  frequency5 = frequency;
}

int ultimateFold(int _input) {
  int _output = _input;
  while (_output > 255 || _output < 0) {
    if (_output > 255) _output = 255 - (_output - 255);
    if (_output < 0) _output = 0 - _output;
  }
  return _output;
}

uint8_t _sample2, _lastSample2;
#define SAMPLE_PHASE_SHIFT 3
void synthesis() {
  if (XYmode) {
    sample2=decayVolume;
  }
  else {
    if ((_phase3 >> 2) >= (analogValues[WS_1]) << 4) {
      _phase3 = 0;
    }
    _lastSample2 = sample2;
    _sample2 = (char)pgm_read_byte_near(sampleTable + (_phase3) ) + 128;

    if (analogValues[PITCH] > wavetable[(_phase4 >> 9) + (_sample2>>5)]) _sample2 = _lastSample2;
    else _sample2 = abs(_sample2 - _lastSample2);
    sample2 =  ((_sample2 * (decayVolume2)) >> 8);
  }
  if (mode == FM) {

    if (XYmode) {
      _phs90 = _phs + 64;
    }
    else {
    }

  }

  if (mode == NOISE) {
    if ((_phase >> 2) >= (analogValues[WS_2] - 100u) << 5) {
      _phase = 0;
    }
    _sample = (char)pgm_read_byte_near(sampleTable + (_phase >> 2));
    _sample = (_sample * wavetable[_phase2 >> 8]) >> 8;
    sample = (_sample * decayVolume) >> 8;
  }

  if (mode == TAH) {
    if ((_phase2 >> 8) < analogValues[WS_2] + 5u)  _phs = _phase >> 8, sample = ((wavetable[_phs] ) * decayVolume) >> 8;
    if (XYmode) {
      _phs90 = _phs + 64;
    }
    else {
    }
  }

  if (mode > 2) {
    if ((_phase >> 2) >= (analogValues[WS_2] - 100u) << 5) {
    }
  }
  if (mode == 3) {
    if ((_phase >> SAMPLE_PHASE_SHIFT) > sample2Length) _phase = 0;
    _sample = (char)pgm_read_byte_near(sample2Table + (_phase >> SAMPLE_PHASE_SHIFT)) + 128;
    sample = (_sample * decayVolume) >> 8;
  }

  if (mode == 4) {
    if ((_phase >> SAMPLE_PHASE_SHIFT) > sample3Length) _phase = 0;
    _sample = (char)pgm_read_byte_near(sample3Table + (_phase >> SAMPLE_PHASE_SHIFT));
    sample = (_sample * decayVolume) >> 8;
  }

  if (mode == 5) {
    if ((_phase >> SAMPLE_PHASE_SHIFT) > sample4Length) _phase = 0;
    _sample = (char)pgm_read_byte_near(sample4Table + (_phase >> SAMPLE_PHASE_SHIFT)) + 128;
    sample = (_sample * decayVolume) >> 8;
  }

  if (mode == 6) {
    if ((_phase >> SAMPLE_PHASE_SHIFT) > sampleLength) _phase = 0;
    _sample = (char)pgm_read_byte_near(sampleTable + (_phase >> SAMPLE_PHASE_SHIFT));
    sample = (_sample * decayVolume) >> 8;
  }

  if (mode == 7) {
    if ((_phase >> SAMPLE_PHASE_SHIFT) > sample4Length) _phase = 0;
    _sample = (char)pgm_read_byte_near(sample4Table + (_phase >> SAMPLE_PHASE_SHIFT)) + 128;
    sample = (_sample * decayVolume) >> 8;
  }


}
void loop() {
  synthesis();
  renderDecay();
  trigDetect();


}
uint8_t trigState = 0;
uint8_t lastTrigState = 0;
void trigDetect() { //rather trigger machine

  lastTrigState = trigState;

  if (analogValues[0] < LOW_THRES)  trigState = 0;
  else if (analogValues[0] > HIGH_THRES) trigState = 2;
  else trigState = 1;

  if (lastTrigState != trigState) trigger(trigState, lastTrigState), _phase = 0;

}

void trigger(uint8_t intensity_1, uint8_t intensity_2) {
  if (abs(intensity_1 - intensity_2) == 1) decayVolume = 128, decayVolume2 = 255;
  else decayVolume = 255, decayVolume2 = 150;


}


uint8_t analogChannelSequence[6] = {0, 1, 0, 2, 0, 3};
uint8_t analogChannelReadIndex;

void setDecay() {
  if (analogValues[WS_2] > 100) decayTime = constrain(analogValues[WS_2] - 120, 1, 255), pitchEnv = 0;
  else decayTime = (100 - analogValues[WS_2]), pitchEnv = 255;
}
ISR(ADC_vect) { // interupt triggered ad completion of ADC counter
  startupRead++;
  if (!firstRead) { // discard first reading due to ADC multiplexer crosstalk
    //update values and remember last values
    lastAnalogValues[analogChannelRead] = analogValues[analogChannelRead];
    analogValues[analogChannelRead] = getConversionResult() >> 2;
    //set ADC MULTIPLEXER to read the next channel
    lastAnalogChannelRead = analogChannelRead;
    if (!analogChannelRead) trigDetect();
    analogChannelReadIndex++;
    if (analogChannelReadIndex > 5) analogChannelReadIndex = 0;
    analogChannelRead = analogChannelSequence[analogChannelReadIndex];
    connectChannel(analogChannelRead);
    // set controll values if relevant (value changed)
    if (lastAnalogChannelRead == PITCH && lastAnalogValues[PITCH] != analogValues[PITCH]) setFrequency(analogValues[PITCH] << 2), decayVolume2 = constrain(decayVolume2 + ((abs(lastAnalogValues[PITCH] - analogValues[PITCH]) << 3)), 0, 255);; //constrain(mapLookup[,0,1015)); //
    if (lastAnalogChannelRead == WS_1 && lastAnalogValues[WS_1] != analogValues[WS_1])  setFrequency2(analogValues[WS_1] << 2), decayVolume = constrain(decayVolume + ((abs(lastAnalogValues[WS_1] - analogValues[WS_1]) << 2)), 0, 255);
    if (lastAnalogChannelRead == WS_2 && lastAnalogValues[WS_2] != analogValues[WS_2]) analogValues[WS_2] = analogValues[WS_2], setDecay();
    firstRead = true;
    //start the ADC - at completion the interupt will be called again
    startConversion();

  }
  else {
    /*
      at the first reading off the ADX (which will not used)
      something else will happen the input pin will briefly turn to output to
      discarge charge built up in passive mixing ciruit using zener diodes
      because zeners have some higher unpredictable capacitance, various voltages might get stuck on the pin
    */

    if ( mode == NOISE) {
    }

    else {
      if (analogValues[analogChannelRead] < 200) bitWrite(DDRB, analogToDigitalPinMapping[analogChannelRead], 1);
      bitWrite(DDRB, analogToDigitalPinMapping[analogChannelRead], 0);
      bitWrite(PORTB, analogToDigitalPinMapping[analogChannelRead], 0);
    }
    firstRead = false;
    startConversion();

  }
}


uint16_t decayCounter = 0;

uint16_t decayCounter2 = 0;

void renderDecay() {
  if (decayTime != 0) {
    if (1) {
      decayCounter += 6;
      if (decayCounter >= decayTime)
      {
        decayCounter = 0;
        if (decayVolume > 0) decayVolume -= ((decayVolume >> 6) + 1);
      }
    }
  }

  if (decayTime != 0) {
    if (1) {
      decayCounter2 += 6;
      if (decayCounter2 >= (decayTime))
      {
        decayCounter2 = 0;
        if (decayVolume2 > 0) decayVolume2 -= ((decayVolume2 >> 6) + 1);
      }
    }
  }

}



// #### FUNCTIONS TO ACCES ADC REGISTERS
void init() {

  ADMUX  = 0;
  bitWrite(ADCSRA, ADEN, 1); //adc enabled
  bitWrite(ADCSRA, ADPS2, 1); // set prescaler
  bitWrite(ADCSRA, ADPS1, 1); // set prescaler
  bitWrite(ADCSRA, ADPS0, 1); // set prescaler
  bitWrite(ADCSRA, ADIE, 1); //enable conversion finished interupt
  bitWrite(SREG, 7, 1);
  // prescaler = highest division
}


// channel 8 can be used to measure the temperature of the chip
void connectChannel(uint8_t number) {
  ADMUX &= (11110000);
  ADMUX |= number;
}

void startConversion() {
  bitWrite(ADCSRA, ADSC, 1); //start conversion
}

bool isConversionFinished() {
  return (ADCSRA & (1 << ADIF));
}

bool isConversionRunning() {
  return !(ADCSRA & (1 << ADIF));
}

uint16_t getConversionResult() {
  uint16_t result = ADCL;
  return result | (ADCH << 8);
}

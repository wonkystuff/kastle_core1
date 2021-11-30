/* wonkystuff fork of Bastl software. Original banner comment below.
 * This update 2021 by John Tuffen
 * open source license: CC BY SA
 *
 * https://wonkystuff.net/
 *
 * History:
 * See git log.
 */

/*
KASTLE VCO v 1.5


 Features
 -5 synthesis modes = phase modulation, phase distortion, tarck & hold modulation, formant synthesis, noise wtf
 -regular & alternative waveform output by 2 PWM channels
 -3 sound parameters controlled by voltage inputs
 -voltage selectable synthesis modes on weak I/O Reset pin


 Writen by Vaclav Pelousek 2016
 open source license: CC BY SA
 http://www.bastl-instruments.com


 -software written in Arduino 1.0.6 - used to flash ATTINY 85 running at 8mHz
 -created with help of the heavenly powers of internet and several tutorials that you can google out
 -i hope somebody finds this code usefull (i know it is a mess :( )

 thanks to
 -Lennart Schierling for making some work on the register access
 -Uwe Schuller for explaining the capacitance of zener diodes
 -Peter Edwards for making the inspireing bitRanger
 -Ondrej Merta for being the best boss
 -and the whole bastl crew that made this project possible
 -Arduino community and the forums for picking ups bits of code to setup the timers & interrupts
 -v1.5 uses bits of code from miniMO DCO http://www.minimosynth.com/
 */

#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/io.h>        // Adds useful constants
#include <util/delay.h>

#include "TR_HH_AT.h"

//global variables
uint8_t mode;
uint8_t analogChannelRead=1;
uint8_t analogValues[4];
uint8_t lastAnalogValues[4];
uint8_t lastAnalogChannelRead;
uint8_t _sample;
uint8_t _saw, _lastSaw;

//defines for synth types
//all are dual oscillator setups -
#define NOISE 1 // phase distortion -
#define FM    0 // aka phase modulation
#define TAH   2 // aka track & hold modulation (downsampling with T&H)

#define LOW_THRES  160
#define HIGH_THRES 220

// Some definitions relevant to the core1.æ - mapping to
// physical knob labels
#define KNOB_A  (0u)
#define KNOB_B  (1u)
#define KNOB_C  (3u)
#define KNOB_D  (2u)

//defines for synth parameters
#define PITCH  KNOB_D
#define WS_1   KNOB_C
#define WS_2   KNOB_B
#define MODE   KNOB_A

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
uint8_t wavetable[256];

void
setup(void)
{
  // happens at the startup
  writeWave(0);

  //set outputs
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);

  setTimers(); //setup interrupts

  //setup ADC and run it in interrupt
  initADC();
  connectChannel(analogChannelRead);
  startConversion();
}

void
setTimers(void)
{
  PLLCSR |= (1 << PLLE);               // Enable PLL (64 MHz)
  _delay_us(100);                      // Wait for a steady state
  while (!(PLLCSR & (1 << PLOCK)));    // Ensure PLL lock
  PLLCSR |= (1 << PCKE);               // Enable PLL as clock source for timer 1
  PLLCSR |= (1 << LSM); //low speed mode 32mhz
  cli();                               // Interrupts OFF (disable interrupts globally)

  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00) | _BV(WGM01);
  TCCR0B = _BV(CS00);

  //  setup timer 0 to run fast for audiorate interrupt

  TCCR1 = 0;                  //stop the timer
  TCNT1 = 0;                  //zero the timer
  GTCCR = _BV(PSR1);          //reset the prescaler
  OCR1A = 255;                //set the compare value
  OCR1C = 255;

  TIMSK = _BV(OCIE1A);   //interrupt on Compare Match A
  //start timer, ctc mode, prescaler clk/1
  TCCR1 = _BV(CTC1) | _BV(CS12) | _BV(CS10);

  sei();
}

void
writeWave(int wave)
{
  switch (wave)
  {
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
void
sineWave(void)
{                                       //too costly to calculate on the fly, so it reads from the sine table. We use 128 values, then mirror them to get the whole cycle
  for (int i = 0; i < 128; ++i)
  {
    wavetable[i] = pgm_read_byte_near(sinetable + i);
  }
  wavetable[128] = 255;
  for (int i = 129; i < 256; ++i) {
    wavetable[i] = wavetable[256 - i] ;
  }
}

void
sawtoothWave(void)
{
  for (int i = 0; i < 256; ++i)
  {
    wavetable[i] = i; // sawtooth
  }
}

void
triangleWave(void)
{
  for (int i = 0; i < 128; ++i)
  {
    wavetable[i] = i * 2;
  }
  int value = 255;
  for (int i = 128; i < 256; ++i)
  {
    wavetable[i] = value;
    value -= 2;
  }
}

void
squareWave(void)
{
  for (int i = 0; i < 128; ++i)
  {
    wavetable[i] = 255;
  }
  for (int i = 128; i < 256; ++i)
  {
    wavetable[i] = 1;                  //0 gives problems (offset and different freq), related to sample  = ((wavetable[phase >> 8]*amplitude)>>8);
  }
}

void
zeroWave(void)
{
  for (int i = 0; i < 256; ++i)
  {
    wavetable[i] = 1;                  //0 gives problems
  }
}

uint8_t  sample, sample90;
uint16_t _phase, _lastPhase;
uint16_t frequency;
uint8_t  sample2;
uint16_t _phase2, _phase4, _phase5,_phase6;
uint16_t frequency2, frequency4, frequency5, frequency6;
uint8_t  _phs,_phs90;
uint8_t  _phase3;

// Sample-rate interrupt here:
ISR(TIMER1_COMPA_vect)
{
  // render primary oscillator in the interupt

  OCR0B = sample;
  OCR0A = sample2;

  //_lastPhase=_phase;
  _phase += frequency;

  _phase2 += frequency2;
  _phase4 += frequency4;
  _phase5 += frequency5;

  switch (mode)
  {
    case FM:
    {
      _lastSaw = _saw;
      _saw=(((255-(_phase>>8)) * (analogValues[WS_2])) >> 8);

      sample2 = ((_saw * wavetable[_phase4 >> 8] ) >> 8) + ((wavetable[_phase4 >> 8] * (255-analogValues[WS_2])) >> 8);

      if(_lastSaw < _saw)
      {
        _phase4 = 64 << 8;
      }

      uint8_t shft=abs(_saw - _lastSaw);

      if(shft > 3)
      {
        _phase5 += shft << 8;
      }
      _phs=(_phase + (analogValues[WS_2] * wavetable[_phase2 >> 8])) >> 6;
      sample = (wavetable[_phs] );
    }
    break;

    case NOISE:
      _phase3 = _phase2 >> 8;
      sample2 = (wavetable[_phase3 + (_phase>>8)]);

      if((_phase >> 2) >= (analogValues[WS_2] - 100u ) << 5)
      {
        _phase=0;
      }
      _sample = 0x80 + pgm_read_byte_near(sampleTable + (_phase >> 2));
      _sample = (_sample * wavetable[_phase2 >> 8]) >> 8;
      sample  = _sample;
      break;

    case TAH:
      _phase6 += frequency6;
      if ((_phase2 >> 8) >= analogValues[WS_2])
      {
        _phs=_phase>>8;
        sample = (wavetable[_phs] );
      }
      sample2 = (wavetable[_phase2 >> 8] + wavetable[_phase4 >> 8] + wavetable[_phase5 >> 8] + wavetable[_phase6 >> 8]) >> 2;
      break;
  }
}

uint16_t sampleEnd;
const uint8_t multiplier[24] = {
  2, 2, 2, 3, 1, 2, 4, 4,
  3, 4, 5, 2, 1, 5, 6, 8,
  3, 8, 7, 8, 7, 6, 8, 16
};

void
setFrequency2(uint16_t input)
{
  if (mode == NOISE)
  {
    frequency2 = (((input-300) << 2) + 1) / 2;
  }
  else if (mode == TAH)
  {
    uint8_t multiplierIndex=analogValues[WS_2]>>5;
    frequency2 = (input << 2) + 1;
    frequency4 = (frequency2 + 1) * multiplier[multiplierIndex];
    frequency5 = (frequency2 - 3) * multiplier[multiplierIndex+8];
    frequency6 = (frequency2 + 7) * multiplier[multiplierIndex+16];
  }
  else
  {
    frequency2 = (input<<2)+1;
    frequency4 = frequency2;
    frequency5 = frequency2;
  }
}

void
setLength(uint8_t _length)
{
  sampleEnd=map(_length,0,255,0,sampleLength);
}

void
setFrequency(uint16_t input)
{
  if(mode == NOISE)
  {
    frequency = ((input-200)<<2) + 1;
  }
  else
  {
    frequency = (input<<2) + 1;
  }
}

void
loop(void)
{
  modeDetect();
}

void
modeDetect(void)
{
  if (analogValues[MODE]<LOW_THRES)
  {
    mode = NOISE;
  }
  else if (analogValues[MODE]>HIGH_THRES)
  {
    mode = TAH;
  }
  else
  {
    mode = FM;
  }
}


uint8_t analogChannelSequence[6] = {
  0, 1, 0, 2, 0, 3
};
uint8_t analogChannelReadIndex;

ISR(ADC_vect)
{
  // interupt triggered at completion of ADC counter

  //update values and remember last values
  lastAnalogValues[analogChannelRead] = analogValues[analogChannelRead];
  analogValues[analogChannelRead] = getConversionResult()>>2;

  lastAnalogChannelRead=analogChannelRead;

  analogChannelReadIndex++;
  if(analogChannelReadIndex>5)
  {
    analogChannelReadIndex=0;
  }
  analogChannelRead=analogChannelSequence[analogChannelReadIndex];

  //set ADC MULTIPLEXER to read the next channel
  connectChannel(analogChannelRead);

  // set control values if relevant (value changed)
  if(lastAnalogChannelRead==MODE)
  {
    modeDetect();
  }
  else if(lastAnalogChannelRead==PITCH)
  {
    setFrequency(analogValues[PITCH]<<2);//constrain(mapLookup[,0,1015));
  }
  else if(lastAnalogChannelRead==WS_1)
  {
    setFrequency2(analogValues[WS_1]<<2);
  }

  //start the ADC - at completion the interupt will be called again
  startConversion();
}


// #### FUNCTIONS TO ACCES ADC REGISTERS
void
initADC(void)
{
  ADMUX  = 0;
  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0) | _BV(ADIE);
}


// channel 8 can be used to measure the temperature of the chip
void
connectChannel(uint8_t number)
{
  ADMUX &= (11110000);
  ADMUX |= number;
}

void
startConversion(void)
{
  ADCSRA |= _BV(ADSC);  //start conversion
}

uint16_t
getConversionResult(void)
{
  uint16_t result = ADCL;
  return result | (ADCH<<8);
}

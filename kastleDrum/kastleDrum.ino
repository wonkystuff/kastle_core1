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

uint8_t mode;
uint8_t analogChannelRead = 1;
uint8_t analogValues[4];
uint8_t lastAnalogValues[4];
uint8_t lastAnalogChannelRead;

//defines for synth types
//all are dual oscillator setups -
#define NOISE (1u) // phase distortion -
#define FM    (0u) // aka phase modulation
#define TAH   (2u) // aka track & hold modulation (downsampling with T&H)

// Some definitions relevant to the core1.æ - mapping to
// physical knob labels
#define KNOB_A  (0u)
#define KNOB_B  (1u)
#define KNOB_C  (3u)
#define KNOB_D  (2u)

//defines for synth parameters
#define PITCH   KNOB_C
#define SND_SEL KNOB_D
#define DECAY   KNOB_A
#define TRIG    KNOB_B

#if (TRIG == KNOB_A)
#define LOW_THRES  (150u)
#define HIGH_THRES (162u)
#else
#define LOW_THRES  (80u)
#define HIGH_THRES (160u)
#endif

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

uint8_t decayVolume;
uint8_t decayVolume2;
uint16_t decayTime;

void
setup(void)
{
  //happens at the startup
  writeWave(0);
  digitalWrite(5, HIGH); //turn on pull up resistor for the reset pin
  //set outputs
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);

  setTimers(); //setup interrupts

  //setup ADC and run it in interrupt
  initADC();
  connectChannel(analogChannelRead);
  startConversion();
  _delay_us(100);
}

void
setTimers(void)
{
  PLLCSR |= (1 << PLLE);               // Enable PLL (64 MHz)
  _delay_us(100);                      // Wait for a steady state
  while (!(PLLCSR & (1 << PLOCK)));    // Ensure PLL lock
  PLLCSR |= (1 << PCKE);               // Enable PLL as clock source for timer 1

  cli();                               // Interrupts OFF (disable interrupts globally)

  // Setup Timer 0 for fast PWM output on both A & B channels
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00) | _BV(WGM01);
  TCCR0B = _BV(CS00);

  //  setup timer 1 to run fast for audiorate interrupt

  TCCR1 = 0;                  // stop the timer
  TCNT1 = 0;                  // zero the timer
  GTCCR = _BV(PSR1);          // reset the prescaler
  OCR1A = 255;                // set the compare value
  OCR1C = 255;

  TIMSK = _BV(OCIE1A);// | _BV(TOIE0);    //TOIE0    //interrupt on Compare Match A
  //start timer, ctc mode, prescaler clk/16
  TCCR1 = _BV(CTC1) | _BV(CS12) | _BV(CS10);

  sei();
}

void
writeWave(int wave)
{
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

// functions to populate the wavetable
void
sineWave(void)
{
  // too costly to calculate on the fly, so it reads from the sine table. We use 128 values, then mirror them to get the whole cycle
  for (int i = 0; i < 128; ++i)
  {
    wavetable[i] = pgm_read_byte_near(sinetable + i);
  }
  wavetable[128] = 255;
  for (int i = 129; i < 256; ++i)
  {
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
  for (int i = 0; i < 128; ++i) {
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

uint16_t _phase;
uint16_t _phase2;
uint16_t _phase4;
uint8_t  _phs;
uint8_t  _phase3;
uint16_t frequency;
uint16_t frequency2;
uint16_t frequency4;
uint8_t  pitchEnv;
uint8_t  sample2;

#define SAMPLE_PHASE_SHIFT 3

ISR(TIMER1_COMPA_vect)  // render primary oscillator in the interupt
{
  static uint8_t  sample;   // Needs to persist from one sample-tick to the next
  uint8_t _sample=0;

  // output the samples first because there's gonna be a lot of jitter in
  // the following code...
  OCR0B = sample;
  OCR0A = sample2;

  if (pitchEnv)
  {
    _phase  += (frequency  + (decayVolume >> 1));
    _phase3 += (frequency  + (decayVolume >> 1)); //frequency;//
    _phase2 += (frequency2 + (decayVolume >> 1));
  }
  else
  {
    _phase  += frequency;
    _phase3 += frequency;
    _phase2 += frequency2;
  }
  if(!(_phase%4))
  {
    _phase4 += frequency4;
  }

  if (mode == FM)
  {
    _phs = (_phase + (analogValues[DECAY] * wavetable[_phase2 >> 8])) >> 6;
    _sample = wavetable[_phs];
  }
  else if (mode == NOISE)
  {
    if ((_phase >> 2) >= (analogValues[DECAY] - 100u) << 5)
    {
      _phase = 0;
    }
    _sample = (char)pgm_read_byte_near(sampleTable + (_phase >> 2)) + 128;
    _sample = (_sample * wavetable[_phase2 >> 8]) >> 8;
  }
  else if (mode == TAH)
  {
    if ((_phase2 >> 8) < analogValues[DECAY] + 5u)
    {
      _phs = _phase >> 8;
    }
    _sample = wavetable[_phs];
  }
  else if (mode == 3)
  {
    if ((_phase >> SAMPLE_PHASE_SHIFT) > sample2Length)
    {
      _phase = 0;
    }
    _sample = (char)pgm_read_byte_near(sample2Table + (_phase >> SAMPLE_PHASE_SHIFT)) + 128;
  }
  else if (mode == 4)
  {
    if ((_phase >> SAMPLE_PHASE_SHIFT) > sample3Length)
    {
      _phase = 0;
    }
    _sample = (char)pgm_read_byte_near(sample3Table + (_phase >> SAMPLE_PHASE_SHIFT)) + 128;
  }
  else if (mode == 5)
  {
    if ((_phase >> SAMPLE_PHASE_SHIFT) > sample4Length)
    {
      _phase = 0;
    }
    _sample = (char)pgm_read_byte_near(sample4Table + (_phase >> SAMPLE_PHASE_SHIFT)) + 128;
  }
  else if (mode == 6)
  {
    if ((_phase >> SAMPLE_PHASE_SHIFT) > sampleLength)
    {
      _phase = 0;
    }
    _sample = (char)pgm_read_byte_near(sampleTable + (_phase >> SAMPLE_PHASE_SHIFT)) + 128;
  }
  else if (mode == 7)
  {
    if ((_phase >> SAMPLE_PHASE_SHIFT) > sample4Length)
    {
      _phase = 0;
    }
    _sample = (char)pgm_read_byte_near(sample4Table + (_phase >> SAMPLE_PHASE_SHIFT)) + 128;
  }
  sample = (_sample * decayVolume) >> 8;
  renderDecay();
  synthesis();
}

void
setFrequency2(uint16_t input)
{
  if (mode > 6)
  {
    TCCR0B = _BV(CS01);
  }
  else
  {
    TCCR0B = _BV(CS00);
  }
  frequency2 = (input << 4) + 1;
  frequency4 = 512 - input;
}

void
setFrequency(uint16_t input)
{
  if (mode == NOISE)
    frequency = ((input - 200) << 2) + 1; //NOISE
  else if (mode > 2)
    frequency = (input) + (1 << SAMPLE_PHASE_SHIFT);
  else
    frequency = (input << 2) + (1 << SAMPLE_PHASE_SHIFT);
}

void
synthesis(void)
{
  uint8_t _sample2;
  uint8_t _lastSample2;
  if ((_phase3 >> 2) >= (analogValues[SND_SEL]) << 4)
  {
    _phase3 = 0;
  }
  _lastSample2 = sample2;
  _sample2 = (char)pgm_read_byte_near(sampleTable + (_phase3) ) + 128;

  if (analogValues[PITCH] > wavetable[(_phase4 >> 9) + (_sample2>>5)])
  {
    _sample2 = _lastSample2;
  }
  else
  {
    _sample2 = abs(_sample2 - _lastSample2);
  }
  sample2 =  ((_sample2 * (decayVolume2)) >> 8);
}

void
loop(void)
{
  // <twiddles fingers>
}

void
trigDetect(void)
{
  //rather trigger machine
  static uint8_t trigState = 0;
  static uint8_t lastTrigState = 0;

  lastTrigState = trigState;

  if (analogValues[TRIG] < LOW_THRES)
  {
    trigState = 0;
  }
  else if (analogValues[TRIG] > HIGH_THRES)
  {
    trigState = 2;
  }
  else
  {
    trigState = 1;
  }

  if (lastTrigState != trigState)
  {
    trigger(trigState, lastTrigState);
    _phase = 0;
  }
}

void
trigger(uint8_t intensity_1, uint8_t intensity_2)
{
  if (abs(intensity_1 - intensity_2) == 1)
  {
    decayVolume = 128;
    decayVolume2 = 255;
  }
  else
  {
     // difference is 2
     decayVolume = 255;
     decayVolume2 = 150;
  }
}

void
setDecay(void)
{
  if (analogValues[DECAY] > 100)
  {
    decayTime = constrain(analogValues[DECAY] - 120, 1, 255);
    pitchEnv = 0;
  }
  else
  {
    decayTime = (100 - analogValues[DECAY]);
    pitchEnv = 255;
  }
}

const uint8_t analogChannelSequence[6] = {TRIG, PITCH, TRIG, SND_SEL, TRIG, DECAY};
uint8_t analogChannelReadIndex;

ISR(ADC_vect)
{
  // interupt triggered ad completion of ADC counter
  //update values and remember last values
  lastAnalogValues[analogChannelRead] = analogValues[analogChannelRead];
  analogValues[analogChannelRead] = getConversionResult() >> 2;
  //set ADC MULTIPLEXER to read the next channel
  lastAnalogChannelRead = analogChannelRead;
  if (!analogChannelRead)
  {
    trigDetect();
  }
  analogChannelReadIndex++;
  if (analogChannelReadIndex > 5)
  {
    analogChannelReadIndex = 0;
  }
  analogChannelRead = analogChannelSequence[analogChannelReadIndex];
  connectChannel(analogChannelRead);
  // set controll values if relevant (value changed)
  if (lastAnalogChannelRead == PITCH && lastAnalogValues[PITCH] != analogValues[PITCH])
  {
    setFrequency(analogValues[PITCH]);
    decayVolume2 = constrain(decayVolume2 + ((abs(lastAnalogValues[PITCH] - analogValues[PITCH]) << 3)), 0, 255); //constrain(mapLookup[,0,1015)); //
  }
  if (lastAnalogChannelRead == SND_SEL && lastAnalogValues[SND_SEL] != analogValues[SND_SEL])
  {
    mode = analogValues[SND_SEL] >> 5;
    setFrequency2(analogValues[SND_SEL] << 2);
    decayVolume = constrain(decayVolume + ((abs(lastAnalogValues[SND_SEL] - analogValues[SND_SEL]) << 2)), 0, 255);
  }
  if (lastAnalogChannelRead == DECAY && lastAnalogValues[DECAY] != analogValues[DECAY])
  {
#if (DECAY == KNOB_A)
    // KNOB_A only runs from 128-255, so scale it to 0-254 (or so):
    analogValues[DECAY] = (analogValues[DECAY] & 0x7f) << 1;
#endif
    setDecay();
  }
  //start the ADC - at completion the interupt will be called again
  startConversion();
}

#define DECAYRATE  (3u)

void
renderDecay(void)
{
  static uint16_t decayCounter = 0;

  if (decayTime != 0) {
    decayCounter += DECAYRATE;
    if (decayCounter >= decayTime)
    {
      decayCounter = 0;
      if (decayVolume > 0)
      {
        decayVolume -= ((decayVolume >> 6) + 1);
      }
      if (decayVolume2 > 0)
      {
        decayVolume2 -= ((decayVolume2 >> 6) + 1);
      }
    }
  }
}

// #### FUNCTIONS TO ACCES ADC REGISTERS
void
initADC(void)
{

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
void
connectChannel(uint8_t number)
{
  ADMUX &= (11110000);
  ADMUX |= number;
}

void
startConversion(void)
{
  bitWrite(ADCSRA, ADSC, 1); //start conversion
}

uint16_t
getConversionResult(void)
{
  uint16_t result = ADCL;
  return result | (ADCH << 8);
}

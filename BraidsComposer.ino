/*********************************
BraidsComposer 1.1.1

This is a port of the Mutable Instruments Braids Macro-oscillator by Olivier Gillet.
It runs on the Microbe Modular Equation Composer without too much trouble at 48kHz (half the original 96KHz).
The porting code (everything in this file) was written by Mathieu Routhier (mrouthier@gmail.com).

Thank you to Olivier Gillet from Mutable Instruments, for his generosity towards this port project and for making his code available.
Thank you to Bret Truchan from Microbe Modular, for making the Equation Composer and for answering many of my questions.

---------------
Input mappings
---------------

PRG : selects the oscillator (see the Braids documentation).
SR  : is the pitch (1V/Oct).  Unfortunately, negative voltages can't be sent to the Equation Composer hardware.  Therefore it can only handle C4 and upwards.
MOD : an octave shift.  Very useful because of the limitations of SR.
PARAM1 : Timbre.
PARAM2 : Color.
PARAM3 : Unused.
GATE : strikes the oscillator.  It has a different action, depending on the selected oscillator.

*********************************/

// uncomment this to enable the system to output the SR value whenever GATE goes high.  The values are useful for calibration.
#define PRINT_SR_MEASUREMENTS_ON_GATE

// Here's the calibration process:
// 1. Apply a 1V signal on SR (this is usually equivalent to the pitch for C5)
// 2. Trigger GATE a few times and use the printed value as CALIBRATION_1V
// 3. Apply a 3V signal on SR (this is usually equivalent to the pitch for C7)
// 4. Trigger GATE a few times and use the printed value as CALIBRATION_3V

#define CALIBRATION_1V 813
#define CALIBRATION_3V 2465

#include "Arduino.h"
#include "defines.h"
#include "DueTimer.h"
#include "macro_oscillator.h"
#include <limits.h>


braids::MacroOscillator osc;

#define SAMPLESPERBLOCK 24

uint8_t sync_buffer[SAMPLESPERBLOCK];

int16_t audioData[2][SAMPLESPERBLOCK];
volatile int outputBlock = 0; // the block index that the interrupt is currently outputting
volatile int outputSample = 0; // the index of the next sample (from outputBlock) that the interrupt will output
volatile int renderBlock = 0; // the block index that the render loop should fill
volatile int renderedBlock = 0; // the block index that the render loop has completed filling


void setup()
{
#ifdef PRINT_SR_MEASUREMENTS_ON_GATE
    Serial.begin(9600);
#endif

    analogReadResolution(ANALOG_READ_RESOLUTION);
    analogWriteResolution(ANALOG_READ_RESOLUTION);

    // Enable the DAC
    analogWrite(DAC1, 0);

    // Set the pinmode for digital pins.  This is not required for the analog inputs.
    pinMode(PIN_GATE, INPUT);

    // trick to accelerate pin read time
    REG_ADC_MR = (REG_ADC_MR & 0xFFF0FFFF) | 0x00020000;

    // init Braids
    memset(sync_buffer, 0, sizeof(sync_buffer));
    osc.Init();

    outputBlock = 0;
    outputSample = 0;
    renderBlock = 0;
    renderedBlock = -1;

    // manually call loop to fill the first block
    loop();

    Timer0.attachInterrupt(audioRateInterrupt).setFrequency(48000).start();
}

void loop()
{
    // wait for a block to be ready
    if(renderBlock == renderedBlock)
        return;

    uint16_t prgValue = analogRead(PIN_PRG);
    uint16_t srValue = analogRead(PIN_SR);
    uint16_t modValue = analogRead(PIN_MOD);
    uint16_t oneValue = analogRead(PIN_PARAM_1);
    uint16_t twoValue = analogRead(PIN_PARAM_2);
    uint16_t gateValue = digitalRead(PIN_GATE);

    static bool strike = false;

    if(gateValue)
    {
        if(strike == false)
        {
            strike = true;
            osc.Strike();

            #ifdef PRINT_SR_MEASUREMENTS_ON_GATE
            Serial.print("SR: ");
            Serial.print(srValue);
            Serial.println();
            #endif
        }
    }
    else
    {
        strike = false;
    }

    braids::MacroOscillatorShape shape = (braids::MacroOscillatorShape)map(prgValue, 0, MAX_CV, 0, braids::MACRO_OSC_SHAPE_LAST-1);
    osc.set_shape(shape);

    // correct SR values based on my personal calibration measurement. They are possibly different for each board.
    srValue = map(srValue, CALIBRATION_1V, CALIBRATION_3V, MAX_CV/5.7, MAX_CV*3/5.67);

    uint16_t octaveShift = modValue * 8 / MAX_CV;

    uint32_t pitch = octaveShift*12*128 + srValue*128/(5*12);

    if(pitch > 128*128)
        pitch = 128*128;

    osc.set_pitch(pitch);

    osc.set_parameters(oneValue<<3, twoValue<<3);

    int16_t *audio = audioData[renderBlock];

    osc.Render(sync_buffer, audio, SAMPLESPERBLOCK);

    // write down that this block was rendered
    renderedBlock = renderBlock;
}

void audioRateInterrupt()
{
    int16_t *audio = audioData[outputBlock];
    int32_t unsignedSample = (int32_t)audio[outputSample] + SHRT_MAX; // signed to unsigned
    uint16_t twelveBits = unsignedSample >> 4; // to 12 bits

    analogWrite(DAC1, twelveBits);

    ++outputSample;
    ++outputSample;

    if(outputSample >= SAMPLESPERBLOCK)
    {
        // tell the rendering loop to start writing in the block that was just finished
        renderBlock = outputBlock;

        //start reading from the next block
        outputBlock = (outputBlock+1) & 1;
        outputSample = 0;
    }

}

/*

  
    https://learn.sparkfun.com/tutorials/tiny-avr-programmer-hookup-guide#programming-in-arduino
*/

#include <stdlib.h>
#include <EEPROM.h>

#include <avr/io.h>        // Adds useful constants
#include <util/delay.h>    // Adds delay_ms and delay_us functions
#include <avr/interrupt.h>
#include <TinyWireS.h>

//************ USER PARAMETERS***********************

#define PWMPIN 1 // PWMPin
#define COOPIN 3 // 
#define REVPIN 4 //
#define FullWidth 192 //

#define I2C_SLAVE_ADDRESS 0x2D

// The default buffer size, Can't recall the scope of defines right now
#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE ( 16 )
#endif

// I2C Register Address definition
#define PWM_REG 0x10
#define COO_REG 0x11
#define REV_REG 0x12

// Volatile Variables
volatile uint8_t PWMDuty = 128;
volatile uint8_t COO = 0;
volatile uint8_t REV = 1;

volatile byte reg_position;

volatile bool bPWMStatus = false;
volatile uint8_t nDuty = FullWidth / 2;
volatile uint8_t nIdle = FullWidth / 2;

// 
void Timer0CompareInit(bool bOption = false);

/**
 * This is called for each read request we receive, never put more than one byte of data (with TinyWireS.send) to the 
 * send-buffer when using this callback
 */
void requestEvent()
{ // 
    switch(reg_position)
    {
      case PWM_REG:
        TinyWireS.send(PWMDuty);
      break;

      case COO_REG:
        TinyWireS.send(COO);
      break;

      case REV_REG:
        TinyWireS.send(REV);
      break;

      default:
        TinyWireS.send(PWMDuty);
      break;
    }
}

/**
 * The I2C data received -handler
 *
 */
void receiveEvent(uint8_t howMany)
{
    if (howMany < 1)
    {
        return;
    }
    if (howMany > TWI_RX_BUFFER_SIZE)
    {
        return;
    }

    reg_position = TinyWireS.receive();
    howMany--;
    if (!howMany)
    {
        // This write was only to set the buffer for next read
        return;
    }
    while(howMany--)
    {
        switch(reg_position)
        {
          case PWM_REG:
          {
            PWMDuty = TinyWireS.receive();
            nDuty = ((uint16_t)FullWidth * (uint16_t)PWMDuty) >> 8;
            nIdle = FullWidth - nDuty;
          }
          break;

          case COO_REG:
          {
            COO = TinyWireS.receive();
            digitalWrite(COOPIN, COO);
          }
          break;

          case REV_REG:
          {
            REV = TinyWireS.receive();
            digitalWrite(REVPIN, REV);
          }
          break;

          default:
            TinyWireS.receive();
          break;
        }
    }
}

//**********MODE*************************************

// the setup routine runs once when you press reset:
void setup()  { 

    // Setup I/O Pins
    pinMode(PWMPIN, OUTPUT);
    digitalWrite(PWMPIN, LOW);
    pinMode(COOPIN, OUTPUT);
    digitalWrite(COOPIN, LOW);
    pinMode(REVPIN, OUTPUT);
    digitalWrite(REVPIN, LOW);

    TinyWireS.begin(I2C_SLAVE_ADDRESS);
    TinyWireS.onReceive(receiveEvent);
    TinyWireS.onRequest(requestEvent);
    
    Timer0CompareInit(true);
} 

void Timer0Stop()
{
  digitalWrite(PWMPIN, LOW);
  bPWMStatus = false;
  TCCR0A = 0;
  TCCR0B = 0;
  TIMSK &= ~(1 << OCIE0A);
  TIFR &= ~(1 << OCF0A);
  TIFR &= ~(1 << OCF0B);
  
}

/* 64HZ PWM IS IMPLEMENTED BY TIMER0 COMPARE INTERRUPT */
void Timer0CompareInit(bool bOption)
{
  if(bOption)
  {
    digitalWrite(PWMPIN, LOW);
    bPWMStatus = false;
    OCR0A = nIdle;
  }
  else
  {
    if(!bPWMStatus)
    { // Duty
      bPWMStatus = true;
      OCR0A = nDuty;
    }
    else
    { // Idle
      bPWMStatus = false;
      OCR0A = nIdle;
    }    
  }
  /*
  Control Register A for Timer/Counter-0 (Timer/Counter-0 is configured using two registers: A and B)
  TCCR0A is 8 bits: [COM0A1:COM0A0:COM0B1:COM0B0:unused:unused:WGM01:WGM00]
  0<<COM0A0: sets bits COM0A0 and COM0A1, which (in Fast PWM mode) clears OC0A on compare-match, and sets OC0A at BOTTOM
  1<<COM0B0: sets bits COM0B0 and COM0B1, which (in Fast PWM mode) clears OC0B on compare-match, and sets OC0B at BOTTOM
  3<<WGM00: sets bits WGM00 and WGM01, which (when combined with WGM02 from TCCR0B below) enables Fast PWM mode
  */
  TCCR0A = 1<<COM0B0 | 2 << WGM00;
  /*
  Control Register B for Timer/Counter-0 (Timer/Counter-0 is configured using two registers: A and B)
  TCCR0B is 8 bits: [FOC0A:FOC0B:unused:unused:WGM02:CS02:CS01:CS00]
  0<<WGM02: bit WGM02 remains clear, which (when combined with WGM00 and WGM01 from TCCR0A above) enables Fast PWM mode
  1<<CS00: sets bits CS01 (leaving CS01 and CS02 clear), which tells Timer/Counter-0 to not use a prescalar
  */
  TCCR0B = 0 << WGM02 | 3 << CS00;

  // Interrupt relevant part
  TIMSK |= (1 << OCIE0A);
  TCNT0 = 0;
  
  TIFR &= ~(1 << OCF0A);
  TIFR &= ~(1 << OCF0B);
}

/*
*/
ISR(TIMER0_COMPA_vect)
{
  Timer0CompareInit();
}

// the loop routine runs over and over again forever:
void loop()  
{
  TinyWireS_stop_check();
}

/* Name:            avr001.c
 * Project:         AVR001 - USB touch-switch stats
 * Author:          Vladimir Stankovic
 * Creation Date:   20140310
 * Copyright:       N/A
 * License:         N/A
 */

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include "usbdrv.h"
#include "oddebug.h"
#include "reqdefs.h"
#include "USI_TWI_Master.h"
#include "TinyWireM.h"
#include "usi_i2c_master.h"


#define MEM_24C64_ADDRESS   0x50

#define TIMER1_PRESCALER_16384
#define PINUP   PINB3
#define PINDOWN PINB4

volatile uint8_t timer0_ticks;
volatile uint16_t gCounter;
volatile uint8_t gOffsetCounter;
short isUsbInitialized;

static uchar currAddress = 0;
static uchar bytesRemaining = 0;
/*
// EEPROM

#include <inttypes.h>
#define USI_SEND         0              // indicates sending to TWI
#define USI_RCVE         1              // indicates receiving from TWI
#define USI_BUF_SIZE    16              // bytes in message buffer

static uint8_t USI_Buf[USI_BUF_SIZE];             // holds I2C send and receive data
static uint8_t USI_BufIdx = 0;                    // current number of bytes in the send buff
static uint8_t USI_LastRead = 0;                  // number of bytes read so far
static uint8_t USI_BytesAvail = 0;                // number of bytes requested but not read

// EEPROM
*/

typedef enum
{
    START,
    INIT,
    RECORD,
    UPLOAD,
    DELETE
} state_t;

state_t state = INIT;

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

int isPinPressed(int pin)
{
    return (PINB & (1 << pin)) != (1 << pin);
}

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
    usbRequest_t    *rq = (usbRequest_t *)data;
    static uchar    dataBuffer[4];  /* buffer must stay valid when usbFunctionSetup returns */

    if(rq->bRequest == REQ_GET_HEADER) // set the application header code
    {
        uchar data = 0;
        uchar buffer[3];

        //eeprom_read_block(&data, 0, 1);
        eeprom_read_block(buffer, 0, 3);

        dataBuffer[0] = buffer[0];//gCounter;//0xff;
        dataBuffer[1] = buffer[1];//isPinPressed(PINUP);//isPinPressed(PINB3);//(PINB & (1 << PINB4)) == (1 << PINB4);//0xff;
        dataBuffer[2] = buffer[2];//isPinPressed(PINDOWN); //isPinPressed(PINB4);
        dataBuffer[3] = data;//0x01;
        usbMsgPtr = (unsigned short)dataBuffer;
        return 4;
    }

    return 0;   /* default for not implemented requests: return no data back to host */
}

/* ------------------------------------------------------------------------- */



// vector name for ATtiny26 defined in /usr/avr/include/avr/iotn26.h
// vector name for ATtiny25/45/85 defined in /usr/avr/include/avr/iotnx5.h

/* ------------------------------------------------------------------------- */
/* -----------------------------    Timer 0    ----------------------------- */
/* ------------------------------------------------------------------------- */
ISR(TIM0_OVF_vect)
{
    TCNT0 = 0;
    //PORTB ^= 0x01;
}

/* ------------------------------------------------------------------------- */
/* -----------------------------    Timer 1    ----------------------------- */
/* ------------------------------------------------------------------------- */
ISR(TIM1_OVF_vect)
{
    if(++gOffsetCounter == 3)
    {
        TCNT1 = 61;
        gOffsetCounter = 0;
    }
    else
    {
        TCNT1 = 60;
    }

    ++gCounter;
}

void initTimers()
{
    TCNT0 = 0x00;
    TCNT1 = 60;

    // enable timer overflow interrupt for Timer0
    TIMSK = (1 << TOIE0) | (1 << TOIE1);

    // start timer0 with /1024 prescaler
    TCCR0B = (1 << CS02) | (1 << CS00);

#ifdef TIMER1_PRESCALER_16384
    TCCR1 = (1 << CS13) | (1 << CS11) | (1 << CS10) | (1 << CS12);
#else
    TCCR1 = (1 << CS13) | (1 << CS11) | (1 << CS10);
#endif
}

void disableTimers()
{
    TIMSK &= ~((1 << TOIE0) | (1 << TOIE1));
}

#define abs(x) ((x) > 0 ? (x) : (-x))

// Called by V-USB after device reset
void    usbEventResetReady(void)
{
    int frameLength, targetLength = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);
    int bestDeviation = 9999;
    uchar trialCal, bestCal, step, region;

    // do a binary search in regions 0-127 and 128-255 to get optimum OSCCAL
    for(region = 0; region <= 1; region++) {
        frameLength = 0;
        trialCal = (region == 0) ? 0 : 128;

        for(step = 64; step > 0; step >>= 1) {
            if(frameLength < targetLength) // true for initial iteration
                trialCal += step; // frequency too low
            else
                trialCal -= step; // frequency too high

            OSCCAL = trialCal;
            frameLength = usbMeasureFrameLength();

            if(abs(frameLength-targetLength) < bestDeviation) {
                bestCal = trialCal; // new optimum found
                bestDeviation = abs(frameLength -targetLength);
            }
        }
    }

    OSCCAL = bestCal;
}

/* EXTERN EEPROM */
/*
void begin(){ // initialize I2C lib
  USI_TWI_Master_Initialise();
}

void beginTransmission(uint8_t slaveAddr){ // setup address & write bit
  USI_BufIdx = 0;
  USI_Buf[USI_BufIdx] = (slaveAddr<<TWI_ADR_BITS) | USI_SEND;
}

void send(uint8_t data){ // buffers up data to send
  if (USI_BufIdx >= USI_BUF_SIZE) return;         // dont blow out the buffer
  USI_BufIdx++;                                   // inc for next byte in buffer
  USI_Buf[USI_BufIdx] = data;
}

uint8_t endTransmission(){ // actually sends the buffer
  short xferOK = 0; //false;
  uint8_t errorCode = 0;
  xferOK = USI_TWI_Start_Read_Write(USI_Buf,USI_BufIdx+1); // core func that does the work
  USI_BufIdx = 0;
  if (xferOK != 0) return 0;
  else {                                  // there was an error
    errorCode = USI_TWI_Get_State_Info(); // this function returns the error number
    return errorCode;
  }
}

uint8_t requestFrom(uint8_t slaveAddr, uint8_t numBytes){ // setup for receiving from slave
  short xferOK = 0; // false;
  uint8_t errorCode = 0;
  USI_LastRead = 0;
  USI_BytesAvail = numBytes; // save this off in a global
  numBytes++;                // add extra byte to transmit header
  USI_Buf[0] = (slaveAddr<<TWI_ADR_BITS) | USI_RCVE;   // setup address & Rcve bit
  xferOK = USI_TWI_Start_Read_Write(USI_Buf,numBytes); // core func that does the work
  // USI_Buf now holds the data read
  if (xferOK != 0) return 0;
  else {                                  // there was an error
    errorCode = USI_TWI_Get_State_Info(); // this function returns the error number
    return errorCode;
  }
}

uint8_t receive(){ // returns the bytes received one at a time
  USI_LastRead++;     // inc first since first uint8_t read is in USI_Buf[1]
  return USI_Buf[USI_LastRead];
}

uint8_t available(){ // the bytes available that haven't been read yet
  return USI_BytesAvail - (USI_LastRead);
}
*/
int __attribute__((noreturn)) main(void)
{




    gCounter = 0;
    gOffsetCounter = 0;
    isUsbInitialized = FALSE;

    //DDRB  = 0x09; // PB0 is output pin
    //PORTB = (1 << PORTB0) | (1 << PORTB2);
    PINB = (1 << PINB4) | (1 << PINB4);

    cli(); // Clear ISR

    //initTimers();

//    wdt_enable(WDTO_1S); // Enable watch-dog timer

//    usbInit();
//    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
//    uchar i = 0;
//    while(--i){             /* fake USB disconnect for > 250 ms */
//        wdt_reset();
//        _delay_ms(1);
//    }
//    usbDeviceConnect();

    sei(); // enable interrupts

    // Temporary
    for(;;)
    {
        switch (state) {
        case START:
            break;
        case INIT:
            if(isPinPressed(PINUP) && isPinPressed(PINDOWN))
            {
                state = RECORD;
                initTimers();

/*
                PORT_USI |= (1<<PORT_USI_SDA);           // Enable pullup on SDA, to set high as released state.
                PORT_USI |= (1<<PORT_USI_SCL);           // Enable pullup on SCL, to set high as released state.



                //DDR_USI  |= (1 << PORT_USI_SCL);           // Enable SCL as output.
                //DDR_USI  |= (1 << PORT_USI_SDA);           // Enable SDA as output.

                DDRB = 0x05;

                int i = 0;
                for (i = 0; i < 5; ++i)
                {
                    PORT_USI |= (1 << PORT_USI_SDA);
                    PORT_USI &= ~(1 << PORT_USI_SCL);
                    //PORT_USI |= 0x04;
                    //PORT_USI &= ~0x01;
                    _delay_ms(500);
                    PORT_USI &= ~(1 << PORT_USI_SDA);
                    PORT_USI |= (1 << PORT_USI_SCL);
                    //PORT_USI |= 0x01;
                    //PORT_USI &= ~0x04;
                    _delay_ms(500);
                }
*/

                /*
                DDRB = 0x05;
                int i = 0;
                for (i = 0; i < 5; ++i)
                {
                    PORT_USI = 0xff;
                    _delay_ms(500);
                    PORT_USI = 0x00;
                    _delay_ms(500);
                }
                */


                //begin(); // 24c64
//                USI_TWI_Master_Initialise();
                // DDRB
                //USI_TWI_Master_UnInitialise();
                //DDRB  |= (1<<PINB2);           // Enable SCL as output.
                //DDRB  |= (1<<PINB0);           // Enable SDA as output.

                //DDRB &= ~(1 << PINB2);



    /*
                DDR_USI  |= (1<<PIN_USI_SDA);

                DDRB  = 0x01; // PB0 is output pin
                DDRB &= ~(1<<PORTB0);
                PORTB = (1 << PORTB0);
                PINB = (1 << PINB4) | (1 << PINB4);
    */

/*
                cli();
                usbInit();
                usbDeviceDisconnect();
                uchar i = 0;
                while(--i){
                    wdt_reset();
                    _delay_ms(1);
                }
                usbDeviceConnect();

                sei(); // enable interrupts
*/
            }

            break;
        case RECORD:
            /*
                uchar   usbFunctionWrite(uchar *data, uchar len)
                {
                    if(bytesRemaining == 0)
                        return 1;               // end of transfer
                    if(len > bytesRemaining)
                        len = bytesRemaining;
                    eeprom_write_block(data, (uchar *)0 + currentAddress, len);
                    currentAddress += len;
                    bytesRemaining -= len;

             */
        {
            uchar data = 7;//gCounter; // reading lower byte of gCounter
            //eeprom_write_block(&data, 0, 1);

            state = UPLOAD;

            //USI_TWI_Master_Initialise();
/*
            beginTransmission(MEM_24C64_ADDRESS);
            send(1 >> 8);
            send(1 & 0xff);
            send(data);
            data = endTransmission();
*/

            /*
            if(USI_TWI_Master_Start() == FALSE)
            {
                data = USI_TWI_Get_State_Info();
            }
            else
            {
                data = TRUE;
            }
            */


            unsigned char i2c_transmit_buffer[3];
            unsigned char i2c_transmit_buffer_len = 3;


            i2c_transmit_buffer[0] = (MEM_24C64_ADDRESS << 1) | 0; // Write
            i2c_transmit_buffer[1] = 0x01;
            i2c_transmit_buffer[2] = 6;
            USI_I2C_Master_Start_Transmission(i2c_transmit_buffer, i2c_transmit_buffer_len);

            i2c_transmit_buffer[0] = (MEM_24C64_ADDRESS << 1) | 1; // Read
            i2c_transmit_buffer[1] = 0x01;
            i2c_transmit_buffer[2] = 2;
            USI_I2C_Master_Start_Transmission(i2c_transmit_buffer, i2c_transmit_buffer_len);


            /*
            unsigned char rdata = 9;
            beginTransmission(MEM_24C64_ADDRESS);
            //send(1 >> 8);
            //send(1 & 0xff);
            //endTransmission();
            if(requestFrom(MEM_24C64_ADDRESS, 1))
            {
                if(available())
                {
                    rdata = receive();
                    rdata = receive();
                    rdata = receive();
                }
                else
                    rdata = 5;
            }
            else
            {
                rdata = 3;
            }

            //USI_TWI_Master_UnInitialise();
            data = rdata;
            */

            //eeprom_write_block(&data, 0, 1);
            eeprom_write_block(i2c_transmit_buffer, 0, 3);


            break;
        }
        case UPLOAD:
            /*
                uchar   usbFunctionRead(uchar *data, uchar len)
                {
                    if(len > bytesRemaining)
                        len = bytesRemaining;
                    eeprom_read_block(data, (uchar *)0 + currentAddress, len);
                    currentAddress += len;
                    bytesRemaining -= len;
                    return len;
                }
             */

            if(isUsbInitialized == FALSE)
            {
                //USI_TWI_Master_UnInitialise();

                cli();

                wdt_enable(WDTO_1S); // Enable watch-dog timer

                usbInit();
                usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
                uchar i = 0;
                while(--i){             /* fake USB disconnect for > 250 ms */
                    wdt_reset();
                    _delay_ms(1);
                }
                usbDeviceConnect();

                sei(); // enable interrupts

                isUsbInitialized = TRUE;
                break;
            }


            wdt_reset();
            usbPoll();

            break;
        case DELETE:
            break;
        default:
            break;
        }

        //wdt_reset();
        //usbPoll();

        if(gCounter == 5)
        {
            //cli();
            gCounter = 0;
            //PORTB ^= 0x01; // Toggling PB0
            //sei();
        }
    }
}

/* ------------------------------------------------------------------------- */

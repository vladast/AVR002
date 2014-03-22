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

//#include <ioavr.h>
//#include <inavr.h>

#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include "usbdrv.h"
#include "oddebug.h"
#include "reqdefs.h"

#include "24c64.h"

#define TIMER1_PRESCALER_16384

#define SWITCH1             PINC1
#define SWITCH2             PINC2
#define SWITCH3             PINC3
#define STATUS              PORTC0

#define SW_WITH             SWITCH1
#define SW_THROW            SWITCH2
#define SW_WITHOUT          SWITCH3

#define REQ_GET_STATE       REQ_GET_DATA1
#define REQ_GET_SESSION     REQ_GET_DATA2
#define REQ_GET_ERROR       REQ_GET_DATA3

#define REQ_RESET_SESSION   REQ_SET_DATA2

#define MEMLOC_STATE        0x00
#define MEMLOC_SESSION      0x01
#define MEMLOC_ERROR        0x02

#define MEM_HW_ADDRESS      0x50

#define FALSE   0
#define TRUE    1

volatile uint8_t timer0_ticks;
volatile uint16_t gCounter;
volatile uint8_t gOffsetCounter;
short isUsbInitialized;

static uchar currAddress = 0;
static uchar bytesRemaining = 0;


typedef enum
{
    START   = 0xA1,
    INIT    = 0xB2,
    RECORD  = 0xC3,
    UPLOAD  = 0xD4,
    DELETE  = 0xE5
} state_t;

state_t state = INIT;

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

int isPinPressed(int pin)
{
    return (PINC & (1 << pin)) != (1 << pin);
}

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
    usbRequest_t    *rq = (usbRequest_t *)data;
    static uchar    dataBuffer[4];  /* buffer must stay valid when usbFunctionSetup returns */

    if(rq->bRequest == REQ_GET_HEADER) // set the application header code
    {
        uchar data = 0;
        uchar buffer[4] = {0};

        //eeprom_read_block(&data, 0, 1);
        eeprom_read_block(buffer, 0, 4);

//        dataBuffer[0] = gCounter;
//        dataBuffer[1] = isPinPressed(SWITCH1);
//        dataBuffer[2] = isPinPressed(SWITCH2);
//        dataBuffer[3] = data;

        dataBuffer[0] = buffer[0];
        dataBuffer[1] = buffer[1];
        dataBuffer[2] = buffer[2];
        dataBuffer[3] = buffer[3];

        usbMsgPtr = (unsigned short)dataBuffer;
        return 4;
    }

    return 0;   /* default for not implemented requests: return no data back to host */
}

/* ------------------------------------------------------------------------- */



// vector name for ATtiny26 defined in /usr/avr/include/avr/iotn26.h
// vector name for ATtiny25/45/85 defined in /usr/avr/include/avr/iotnx5.h
// vector name for Atmega8 defined in /usr/avr/include/avr/iom8.h

/* ------------------------------------------------------------------------- */
/* -----------------------------    Timer 1    ----------------------------- */
/* ------------------------------------------------------------------------- */
ISR (TIMER1_COMPA_vect)
{
    // Increment counter every 1 sec
    ++gCounter;
}

void initTimers()
{
    // OCRn =  [ (clock_speed / Prescaler_value) * Desired_time_in_Seconds ] - 1
    OCR1A = 15624; // 1sec

    TCCR1B |= _BV(WGM12); // Mode 4, CTC on OCR1A

    TIMSK |= _BV(OCIE1A); // Set interrupt on compare match

    TCCR1B |= _BV(CS12) | _BV(CS10); // set prescaler to 1024 and start the timer
}

void disableTimers()
{
    //TIMSK &= ~((1 << TOIE0) | (1 << TOIE1));
    TIMSK ^= _BV(OCIE1A);
}

void Wait()
{
    uint8_t i;

    for(i=0;i<100;i++)
        _delay_loop_2(0);
}

int __attribute__((noreturn)) main(void)
{
    for(;;)
    {
        switch (state) {
        case START:
            break;
        case INIT:
            if(isPinPressed(SWITCH3))
            {
                state = RECORD;

                gCounter = 0;
                gOffsetCounter = 0;
                isUsbInitialized = FALSE;

                DDRC = 0x01;
                PORTC = _BV(PORTC0);
                cli();
                initTimers();
                sei();

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

            unsigned char messageBuf[4];


            //Init EEPROM
            EEOpen();

            _delay_loop_2(0);

            Wait();

            uint8_t failed = 0;
            uint16_t address = 0;

            int read = 1;

            if(read == 0)
            {

//                if(EEWriteByte(address,7) == 0)
//                {
//                    failed=1;
//                    Wait();
//                    messageBuf[0] = 6;
//                    messageBuf[1] = 6;
//                    messageBuf[2] = 6;
//                    messageBuf[3] = 6;
//                }
//                else
//                {
//                    Wait();
//                    messageBuf[0] = 7;
//                    messageBuf[1] = 7;
//                    messageBuf[2] = 7;
//                    messageBuf[3] = 7;
//                }

                uint16_t i;
                for (i = 0; i < 4; ++i)
                {
                    EEWriteByte(i, i);
                    Wait();
                }
            }
            else
            {

//                if(EEReadByte(address) != 7)
//                {
//                    failed=1;
//                    Wait();
//                    messageBuf[0] = 3;
//                    messageBuf[1] = 3;
//                    messageBuf[2] = 3;
//                    messageBuf[3] = 3;
//                }
//                else
//                {
//                    Wait();
//                    messageBuf[0] = 4;
//                    messageBuf[1] = 4;
//                    messageBuf[2] = 4;
//                    messageBuf[3] = 4;
//                }

                uint16_t i;
                for (i = 0; i < 4; ++i)
                {
                    messageBuf[i] = EEReadByte(i);
                    Wait();
                }
            }

            eeprom_write_block(messageBuf, 0, 4);
            PORTC ^= _BV(PORTC0);

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

                wdt_enable(WDTO_1S);

                usbInit();
                usbDeviceDisconnect();
                uchar i = 0;
                while(--i){
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

        if(gCounter == 5)
        {
            //cli();
            gCounter = 0;
            //PORTC ^= _BV(PORTC0);
            //sei();
        }
    }
}

/* ------------------------------------------------------------------------- */

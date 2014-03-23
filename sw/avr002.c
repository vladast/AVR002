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

#define BUTTON_WITH         SWITCH1
#define BUTTON_THROW        SWITCH2
#define BUTTON_WITHOUT      SWITCH3

#define SWID_UNKNOWN        0x00
#define SWID_WITH           0xe1
#define SWID_THROW          0xe2
#define SWID_WITHOUT        0xe3

#define REQ_GET_STATE       REQ_GET_DATA1
#define REQ_GET_SESSION     REQ_GET_DATA2
#define REQ_GET_ERROR       REQ_GET_DATA3

#define REQ_RESET_SESSION   REQ_SET_DATA2

// Predefined memory locations on CPU's EEPROM
#define MEMADDR_STATE       0
#define MEMADDR_SESSION     1
#define MEMADDR_ERROR       2
#define MEMADDR_ENTRIES     3

#define MEM_HW_ADDRESS      0x50

#define USB_CONNECT_CNT     1000L

#define FALSE   0
#define TRUE    1

volatile uint8_t    timer0_ticks;
volatile uint16_t   gCounter;
volatile uint16_t   gUsbConnectCounter;
volatile uint8_t    gOffsetCounter;
volatile uint8_t    gSessionCounter;
volatile uint16_t   gEntryCounter;
volatile short      isButtonWithPressed = FALSE;
volatile short      isButtonThrowPressed = FALSE;
volatile short      isButtonWithoutPressed = FALSE;
volatile int        idPressedButton = SWID_UNKNOWN;
short               isUsbInitialized = FALSE;
short               isStarted = FALSE;

//static uchar currAddress = 0;
//static uchar bytesRemaining = 0;


typedef enum
{
    START   = 0xA1, // Device is being started
    INIT    = 0xB2, // Initialize device
    RECORD  = 0xC3, // Record events (touch-switch states)
    UPLOAD  = 0xD4, // Upload records to USB host
    DELETE  = 0xE5, // Erase external EEPROM
    RESES   = 0xF6  // Reinit session counter
} state_t;

volatile state_t state = INIT;

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

int isPinPressed(int pin)
{
    return ((PINC & (1 << pin)) != (1 << pin)) ? TRUE : FALSE;
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
/* -----------------------------    Timer 0    ----------------------------- */
/* ------------------------------------------------------------------------- */
ISR (TIMER0_OVF_vect)
{
    // Temp: Toggle LED with every interrupt
    PORTC ^= _BV(PORTC0);

    isButtonWithPressed = isPinPressed(BUTTON_WITH);
    isButtonThrowPressed = isPinPressed(BUTTON_THROW);
    isButtonWithoutPressed = isPinPressed(BUTTON_WITHOUT);

    if(isButtonWithPressed == TRUE || isButtonThrowPressed == TRUE || isButtonWithoutPressed == TRUE)
    {
        if(isButtonWithPressed == TRUE && isButtonThrowPressed == TRUE ||
                isButtonWithPressed == TRUE && isButtonWithoutPressed == TRUE ||
                isButtonThrowPressed == TRUE && isButtonWithoutPressed == TRUE)
        {
            // Invalid state
        }
        else
        {
            if(isButtonWithPressed == TRUE)
            {
                idPressedButton = SWID_WITH;
            }
            else if(isButtonWithoutPressed == TRUE)
            {
                idPressedButton = SWID_WITHOUT;
            }
            else if(isButtonThrowPressed == TRUE)
            {
                idPressedButton = SWID_THROW;

                if(gUsbConnectCounter >= USB_CONNECT_CNT)
                    state = UPLOAD;
                    gUsbConnectCounter = 0;
            }
        }
    }
    else
    {
        gUsbConnectCounter = 0;
    }
}

/* ------------------------------------------------------------------------- */
/* -----------------------------    Timer 1    ----------------------------- */
/* ------------------------------------------------------------------------- */
ISR (TIMER1_COMPA_vect)
{
    // Increment counter every 1 sec
    ++gCounter;
    ++gUsbConnectCounter;
}

void initTimer0()
{
    TCCR0 |= _BV(CS02) | _BV(CS00);
    TCNT0 = 0;
    TIMSK |= _BV(TOIE0);
}

void initTimer1()
{
    // Using comparison for Timer1 for precision
    // OCRn =  [ (clock_speed / Prescaler_value) * Desired_time_in_Seconds ] - 1
    OCR1A = 15624; // 1sec

    TIMSK |= _BV(OCIE1A); // Set interrupt on compare match

    TCCR1B |= _BV(WGM12) |              // Mode 4, CTC on OCR1A
              _BV(CS12) | _BV(CS10);    // set prescaler to 1024
}

void initTimers()
{
    initTimer0();
    initTimer1();
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

void initSession()
{
    gSessionCounter = 0;
    eeprom_write_block(&gSessionCounter, MEMADDR_SESSION, 1);
}

void createNewSession()
{
    eeprom_read_block(&gSessionCounter, MEMADDR_SESSION, 1);
    ++gSessionCounter;
    eeprom_write_block(&gSessionCounter, MEMADDR_SESSION, 1);
}

void readEntryCount()
{
    eeprom_read_block(&gEntryCounter, MEMADDR_ENTRIES, 1);
}

void storeEntryCount()
{
    eeprom_write_block(&gEntryCounter, MEMADDR_ENTRIES, 1);
}

void initEntryCount()
{
    gEntryCounter = 0;
    storeEntryCount();
}

void checkStartState()
{
    eeprom_read_block(&state, MEMADDR_STATE, 1);

    if (state != START ||
        state != INIT ||
        state != RECORD ||
        state != UPLOAD ||
        state != DELETE)
    {
        // Upon start, no state was stored --> fresh start!
        state = START;
        eeprom_write_block(&state, MEMADDR_STATE, 1);

        initSession();
        initEntryCount();

        idPressedButton = SWID_UNKNOWN;
    }
}

int __attribute__((noreturn)) main(void)
{
    // Check whether state entry is initialized
    checkStartState();

    // Increment session number
    createNewSession();

    for(;;)
    {
        switch (state)
        {
        case START:
            break;
        case INIT:
            eeprom_write_block(&state, MEMADDR_STATE, 1);
            if(isPinPressed(SWITCH3))
            {
                state = RECORD;

                gCounter = 0;
                gOffsetCounter = 0;
                gUsbConnectCounter = 0;
                isUsbInitialized = FALSE;

                // Define PC0 as output
                DDRC = 0x01;
                PORTC = _BV(PORTC0);

                // Initialize timers
                cli();
                initTimers();
                sei();

                //Init EEPROM
                EEOpen();

                _delay_loop_2(0);

                Wait();

            }

            break;
        case RECORD:
            eeprom_write_block(&state, MEMADDR_STATE, 1);
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



            unsigned char messageBuf[4];




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
                for (i = 0; i < 40; ++i)
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
                for (i = 0; i < 40; ++i)
                {
                    messageBuf[i] = EEReadByte(i);
                    Wait();
                }
            }

            state = UPLOAD;
            eeprom_write_block(messageBuf, 0, 4);

            // Temp: toggle LED
            PORTC ^= _BV(PORTC0);

            break;
        }
        case UPLOAD:
            eeprom_write_block(&state, MEMADDR_STATE, 1);
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
            eeprom_write_block(&state, MEMADDR_STATE, 1);
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

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
#include "usbdefs.h"

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
#define MEMADDR_STATE       0x0000
#define MEMADDR_SESSION     0x0001
#define MEMADDR_ERROR       0x0002
#define MEMADDR_ENTRIES     0x0003

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
volatile uint16_t   gEventCounter;
volatile short      isButtonWithPressed = FALSE;
volatile short      isButtonThrowPressed = FALSE;
volatile short      isButtonWithoutPressed = FALSE;
volatile int        idPressedButton = SWID_UNKNOWN;
volatile short      isButtonValueReady = FALSE; // Returned to FALSE when written onto eeprom
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

void initSession()
{
    uint8_t sessionCounter = 0x00;
    eeprom_write_block(&sessionCounter, MEMADDR_SESSION, 1);
    gSessionCounter = sessionCounter;
}

uint8_t readSession()
{
    uint8_t sessionCounter;
    eeprom_read_block(&sessionCounter, MEMADDR_SESSION, 1);
    return sessionCounter;
}

void createNewSession()
{
    uint8_t sessionCounter;

    while(!eeprom_is_ready());
    eeprom_read_block(&sessionCounter, MEMADDR_SESSION, 1);

    sessionCounter++;

    while(!eeprom_is_ready());
    eeprom_write_block(&sessionCounter, MEMADDR_SESSION, 1);
    gSessionCounter = sessionCounter;
}

uint16_t readEntryCount()
{
    eeprom_read_block(&gEntryCounter, MEMADDR_ENTRIES, 2);
    return gEntryCounter;
}

void storeEntryCount(uint16_t entryCount)
{
    gEntryCounter = entryCount;
    eeprom_write_block(&gEntryCounter, MEMADDR_ENTRIES, 2);
}

void initEntryCount()
{
    gEntryCounter = 0;
    storeEntryCount(gEntryCounter);
}

uint8_t readCurrentState()
{
    uint8_t currentState;
    eeprom_read_block(&currentState, MEMADDR_STATE, 1);
    return currentState;
}

void checkStartState()
{
    eeprom_read_block(&state, MEMADDR_STATE, 1);

    if(state == 0xff)
    {
        // Upon start, no state was stored --> fresh start after session reset or reprogram!
        state = START;
    }
}

void setCurrentState(state_t state_to_store)
{
    state = state_to_store;
    eeprom_write_block(&state, MEMADDR_STATE, 1);
}

uint8_t readErrorCode()
{
    uint8_t errorCode;
    eeprom_read_block(&errorCode, MEMADDR_ERROR, 1);
    return errorCode;
}

void storeErrorCode(uint8_t errorCode)
{
    eeprom_write_block(&errorCode, MEMADDR_ERROR, 1);
    return errorCode;
}

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
    usbRequest_t    *rq = (usbRequest_t *)data;
    static uchar    dataBuffer[4];

    if (rq->bRequest == REQ_GET_HEADER) // set the application header code
    {
        dataBuffer[0] = AVR002_CODE >> 0; // Lower byte of AVR002_CODE value
        dataBuffer[1] = AVR002_CODE >> 8; // Higher byte of AVR002_CODE value

        usbMsgPtr = (unsigned short)dataBuffer;
        return 2; // Device code value is 2 byte long (uint16_t)
    }
    else if (rq->bRequest == REQ_GET_DATA1) // Get state
    {
        dataBuffer[0] = readCurrentState();
        usbMsgPtr = (unsigned short)dataBuffer;
        return 1; // Device's state value is 1 byte long (uint8_t)
    }
    else if (rq->bRequest == REQ_SET_DATA1)
    {
        if(rq->wValue.bytes[0] == INIT)
        {
            setCurrentState(INIT);
        }

    }
    else if (rq->bRequest == REQ_GET_DATA2) // Get session
    {
        dataBuffer[0] = readSession();
        usbMsgPtr = (unsigned short)dataBuffer;
        return 1; // Device's session value is 1 byte long (uint8_t)
    }
    else if (rq->bRequest == REQ_GET_DATA3) // Get error-code
    {
        dataBuffer[0] = readErrorCode();
        usbMsgPtr = (unsigned short)dataBuffer;
        return 1; // Device's session value is 1 byte long (uint8_t)
    }
    else if (rq->bRequest == REQ_GET_DATA4) // Get entry count
    {
        uint16_t entryCount = readEntryCount();
        dataBuffer[0] = entryCount >> 0; // Lower byte of entryCount
        dataBuffer[1] = entryCount >> 8; // Higher byte of entryCount
        usbMsgPtr = (unsigned short)dataBuffer;
        return 2; // Device's entry count value is 2 byte long (uint16_t)
    }

    return 0;
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
    isButtonWithPressed = isPinPressed(BUTTON_WITH);
    isButtonThrowPressed = isPinPressed(BUTTON_THROW);
    isButtonWithoutPressed = isPinPressed(BUTTON_WITHOUT);

    if(isButtonValueReady == FALSE)
        PORTC ^= _BV(PORTC0);

    if(isButtonValueReady == FALSE &&
            (isButtonWithPressed == TRUE || isButtonThrowPressed == TRUE || isButtonWithoutPressed == TRUE))
    {
        if(isButtonWithPressed == TRUE && isButtonThrowPressed == TRUE ||
                isButtonWithPressed == TRUE && isButtonWithoutPressed == TRUE ||
                isButtonThrowPressed == TRUE && isButtonWithoutPressed == TRUE)
        {
            // Invalid state
            PORTC ^= _BV(PORTC0);
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
            isButtonValueReady = TRUE;
        }
    }
    else
    {
        idPressedButton = SWID_UNKNOWN;
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
    /*
    uint8_t i;

    for(i=0;i<100;i++)
        _delay_loop_2(0);
        */
    _delay_us(50);
}

int __attribute__((noreturn)) main(void)
{
    // Check whether state entry is initialized
    checkStartState();

    for(;;)
    {
        switch (state)
        {
        case START:
            eeprom_write_block(&state, MEMADDR_STATE, 1);

            initSession();
            initEntryCount();

            idPressedButton = SWID_UNKNOWN;
            isButtonValueReady = FALSE;

            state = INIT;
            break;
        case INIT:
            eeprom_write_block(&state, MEMADDR_STATE, 1);

            if(isPinPressed(BUTTON_THROW))
            {
                // Increment session number
                createNewSession();

                gCounter = 0;
                gOffsetCounter = 0;
                gUsbConnectCounter = 0;
                gEventCounter = 0;
                //isUsbInitialized = FALSE;

                // Define PC0 as output
                DDRC = _BV(PORTC0);
                PORTC = _BV(PORTC0);

                // Initialize timers
                cli();
                initTimers();
                sei();

                //Init EEPROM
                EEOpen();

                state = RECORD;
            }

            break;
        case RECORD:
            eeprom_write_block(&state, MEMADDR_STATE, 1);

            if(isButtonValueReady == TRUE)
            {
                uint16_t entryCount = readEntryCount();

                // TODO: Write to ext. memory

                // TODO: Increment entry count and store it in local eeprom
                storeEntryCount(++entryCount);
            }

/*
            unsigned char messageBuf[4];


            int read = 0;

            if(read == 0)
            {
                uint16_t i;
                for (i = 0; i < 4; ++i)
                {
                    EEWriteByte(i, i + 5);
                    Wait();
                }
            }
            else
            {
                uint16_t i;
                for (i = 0; i < 4; ++i)
                {
                    messageBuf[i] = EEReadByte(i);
                    Wait();
                }
            }


            eeprom_write_block(messageBuf, 0, 4);
*/

            state = UPLOAD;

            // Temp: toggle LED
            PORTC ^= _BV(PORTC0);

            break;
        case UPLOAD:
        {
            eeprom_write_block(&state, MEMADDR_STATE, 1);

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
        }
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
            ++gEventCounter;
        }
    }
}

/* ------------------------------------------------------------------------- */

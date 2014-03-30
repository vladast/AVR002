/*
 * Copyright (C) 2014  Vladimir Stankovic
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
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
#define SWID_WITH           0x01
#define SWID_THROW          0x02
#define SWID_WITHOUT        0x03

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

#define USB_CONNECT_CNT     5L      // Count 5 Timer1 interrupts
#define SWITCH_PRESS_CNT    50L     // Count 20 Timer0 interrupts

#define SAMPLING_TIME_WINDOW 1

#define FALSE   0
#define TRUE    1

volatile uint8_t    timer0_ticks;
volatile uint16_t   gCounter = 0;
volatile uint16_t   gPrevCounter = 0;
volatile uint16_t   gUsbConnectCounter;
volatile uint8_t    gOffsetCounter;
volatile uint8_t    gSessionCounter;
volatile uint16_t   gEntryCounter;
volatile uint16_t   gEventCounter;
//volatile uint8_t    gButtonLedCounter;
volatile short      isButtonWithPressed = FALSE;
volatile short      isButtonThrowPressed = FALSE;
volatile short      isButtonWithoutPressed = FALSE;
volatile short      isButtonLedOn = FALSE;
volatile short      isUsbConnectCounterStarted = FALSE;
volatile int        idPressedButton = SWID_UNKNOWN;
volatile short      isButtonValueReady = FALSE; // Returned to FALSE when written onto eeprom
short               isUsbInitialized = FALSE;
short               isStarted = FALSE;

volatile uint16_t   gEEpromCurrentAddress = 0;

//static uchar currAddress = 0;
//static uchar bytesRemaining = 0;


typedef enum
{
    START   = 0xa1, // Device is being started
    INIT    = 0xb2, // Initialize device
    RECORD  = 0xc3, // Record events (touch-switch states)
    UPLOAD  = 0xd4, // Upload records to USB host
    DELETE  = 0xe5, // Erase external EEPROM
    RESES   = 0xf6  // Reinit session counter
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
    uint8_t sessionCounter = 0;
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
    state_t startState;
    eeprom_read_block(&startState, MEMADDR_STATE, 1);

//    START   = 0xA1, // Device is being started
//    INIT    = 0xB2, // Initialize device
//    RECORD  = 0xC3, // Record events (touch-switch states)
//    UPLOAD  = 0xD4, // Upload records to USB host
//    DELETE  = 0xE5, // Erase external EEPROM
//    RESES   = 0xF6  // Reinit session counter

    if(startState != START && startState != INIT && startState != RECORD && startState != UPLOAD && startState != DELETE && startState != RESES)
    {
        // Upon start, no state was stored --> fresh start after session reset or reprogram!

        storeErrorCode(startState);
        //state = START;

        /* [Issue #2] temp fix */
        state = INIT;
        /* [Issue #2] temp fix */
    }
    else
    {
        state = INIT; // temp
    }

    /* [Issue #2] temp fix */
    if(readSession() == 0xff)
    {
        initSession();
    }
    /* [Issue #2] temp fix */

    /*else if (state == RECORD)
    {
        state = UPLOAD;
    }*/
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
    else if (rq->bRequest == REQ_SET_DATA1) // Set state
    {
        if(rq->wValue.bytes[0] == INIT)
        {
            // TODO: Gracefully disconnect USB
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

        gEEpromCurrentAddress = 0;

        return 2; // Device's entry count value is 2 byte long (uint16_t)
    }
    else if (rq->bRequest == REQ_GET_DATA5) // Get entry count
    {
        dataBuffer[0] = EEReadByte(gEEpromCurrentAddress++);
        usbMsgPtr = (unsigned short)dataBuffer;
        return 1; // Device's entry count value is 2 byte long (uint16_t)
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

    // Check if any switch is being pressed
    if((isButtonWithPressed == TRUE || isButtonThrowPressed == TRUE || isButtonWithoutPressed == TRUE))
    {
        if(isButtonWithPressed == TRUE && isButtonThrowPressed == TRUE ||
                isButtonWithPressed == TRUE && isButtonWithoutPressed == TRUE ||
                isButtonThrowPressed == TRUE && isButtonWithoutPressed == TRUE)
        {
            // Invalid state
            //PORTC ^= _BV(PORTC0);
            // TODO: Store error-code with event count for the event
            return;
        }

        if(isButtonValueReady == FALSE)
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
            }

            if(state == RECORD)
            {
                isButtonValueReady = TRUE;

                PORTC = _BV(PORTC0);
                isButtonLedOn = TRUE;

                isUsbConnectCounterStarted = FALSE;
                gUsbConnectCounter = 0;
            }
        }
        else
        {
            if(idPressedButton == SWID_THROW && isButtonLedOn == TRUE)
            {
                if(isUsbConnectCounterStarted == FALSE)
                {
                    isUsbConnectCounterStarted = TRUE;
                    gUsbConnectCounter = 0;
                }
                else
                {
                    // Check if "throw" switch is being pressedd for long time
                    if(gUsbConnectCounter >= USB_CONNECT_CNT)
                    {
                        state = UPLOAD;
                        gUsbConnectCounter = 0;
                        isUsbConnectCounterStarted = FALSE;

                        isButtonLedOn = FALSE;
                        PORTC = 0;

                        // ... AND ERASE THE LAST STORED ENTRY FROM EEPROM AND DECREMENT EVENT COUNTER
                    }
                }
            }
            else
            {
                isUsbConnectCounterStarted = FALSE;
                gUsbConnectCounter = 0;
            }
        }
    }
    else // On raising edge, when user lifts up finger from switch, turn-off LED
    {
        isButtonLedOn = FALSE;
        PORTC = 0;
    }


    /*
    else
    {
        idPressedButton = SWID_UNKNOWN;
        isUsbConnectCounterStarted = FALSE;
        gUsbConnectCounter = 0;
    }
    */

}

/* ------------------------------------------------------------------------- */
/* -----------------------------    Timer 1    ----------------------------- */
/* ------------------------------------------------------------------------- */
ISR (TIMER1_COMPA_vect)
{
    // Increment counter every 1 sec
    ++gCounter;
    if(isUsbConnectCounterStarted == TRUE)
        ++gUsbConnectCounter;
}

void initTimer0()
{
    TCCR0 |= _BV(CS02);
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

            if(gCounter % SAMPLING_TIME_WINDOW == 0)
            {
                while(isButtonLedOn == TRUE);

                if(isButtonValueReady == TRUE && state == RECORD)
                {
                    uint8_t dataToStore = 0x00;
                    uint16_t entryCount = readEntryCount();
                    uint16_t counterDiff = (gCounter - gPrevCounter) / SAMPLING_TIME_WINDOW;
                    gPrevCounter = gCounter;

                    if(counterDiff <= 0x1F) // 0x1F: All five bits reserved for CNT value are set
                    {
                        // Store SWID and OV together with CNT
                        dataToStore = (idPressedButton << 6) | (0 << 6) | (uint8_t)counterDiff;
                        EEWriteByte(entryCount++, dataToStore);
                    }
                    else
                    {
                        // Store SWID and OV together with 5 lower bits of CNT
                        dataToStore = (idPressedButton << 6) | (1 << 6) | (uint8_t)counterDiff;
                        EEWriteByte(entryCount++, dataToStore);

                        // + store the upper bits of CNT in the next memory slot
                        dataToStore = (counterDiff >> 5);
                        EEWriteByte(entryCount++, dataToStore);
                    }

                    storeEntryCount(entryCount);
                    isButtonValueReady = FALSE;
                }
            }
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
    }
}

/* ------------------------------------------------------------------------- */

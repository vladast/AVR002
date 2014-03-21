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
//#include "USI_TWI_Master.h"
//#include "TinyWireM.h"
//#include "usi_i2c_master.h"


#define MEM_24C64_ADDRESS   0x50

#define TIMER1_PRESCALER_16384
#define SWITCH1 PINC1
#define SWITCH2 PINC2
#define SWITCH3 PINC3
#define STATUS  PORTC0

#define FALSE   0
#define TRUE    1

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
    return (PINC & (1 << pin)) != (1 << pin);
}

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
    usbRequest_t    *rq = (usbRequest_t *)data;
    static uchar    dataBuffer[4];  /* buffer must stay valid when usbFunctionSetup returns */

    if(rq->bRequest == REQ_GET_HEADER) // set the application header code
    {
        uchar data = 0;

        eeprom_read_block(&data, 0, 1);

        dataBuffer[0] = gCounter;
        dataBuffer[1] = isPinPressed(SWITCH1);
        dataBuffer[2] = isPinPressed(SWITCH2);
        dataBuffer[3] = data;
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
ISR(TIMER0_OVF_vect)
{
    //TCNT0 = 0;
    //PORTB ^= 0x01;
}

/* ------------------------------------------------------------------------- */
/* -----------------------------    Timer 1    ----------------------------- */
/* ------------------------------------------------------------------------- */
ISR(TIMER1_OVF_vect)
{
/*
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
*/


    /*
    if(PORTC & (1 << PORTC0))
    {
        PORTC = 0x00;
    }
    else
    {
        PORTC = 0x01;
    }
    */

    /*
    if(gCounter == 0)
        TCNT1 = 0x0FFF;

     ++gCounter;
     */

    //++gCounter;
    //PORTC ^= _BV(PORTC0);

}

ISR (TIMER1_COMPA_vect)
{
    // action to be done every 1 sec
    ++gCounter;
    //PORTC ^= _BV(PORTC0);
}

void initTimers()
{
/*
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
*/

    /*
    TCNT1L = 0xFF;
    TCNT1H = 0xAF;
    // Normal mode (WGM13:0 = 0)
    TIMSK = _BV(TOIE1);
    TCCR1A = 0;
    TCCR1B = _BV(CS12) | _BV(CS10);
    */

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

/*
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
*/

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

            eeprom_write_block(&data, 0, 1);
            //eeprom_write_block(i2c_transmit_buffer, 0, 3);


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
            PORTC ^= _BV(PORTC0);
            //sei();
        }
    }
}

/* ------------------------------------------------------------------------- */

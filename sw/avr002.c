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
//#include "USI_TWI_Master.h"
//#include "TinyWireM.h"
//#include "usi_i2c_master.h"
#include "TWI_Master.h"
#include "24c64.h"


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

/* TWI from AVR 315 */
#define TWI_GEN_CALL         0x00  // The General Call address is 0

// Sample TWI transmission commands
#define TWI_CMD_MASTER_WRITE 0x10
#define TWI_CMD_MASTER_READ  0x20

// Sample TWI transmission states, used in the main application.
#define SEND_DATA             0x01
#define REQUEST_DATA          0x02
#define READ_DATA_FROM_BUFFER 0x03

unsigned char TWI_Act_On_Failure_In_Last_Transmission ( unsigned char TWIerrorMsg )
{
                    // A failure has occurred, use TWIerrorMsg to determine the nature of the failure
                    // and take appropriate actions.
                    // Se header file for a list of possible failures messages.

                    // Here is a simple sample, where if received a NACK on the slave address,
                    // then a retransmission will be initiated.

  if ( (TWIerrorMsg == TWI_MTX_ADR_NACK) | (TWIerrorMsg == TWI_MRX_ADR_NACK) )
    TWI_Start_Transceiver();

  return TWIerrorMsg;
}


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
    unsigned char TWI_targetSlaveAddress = 0x50, TWI_operation = TRUE;

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

            unsigned char messageBuf[4];


            //TWI_Master_Initialise();
            //sei();
            //data = TWI_Get_State_Info();

            while (1 == 0 /*TWI_operation != FALSE*/)
            {
                if(1 == 0)
                {
                    //messageBuf[0] = (TWI_targetSlaveAddress<<TWI_ADR_BITS) | (FALSE<<TWI_READ_BIT); // The first byte must always consit of General Call code or the TWI slave address.
                    messageBuf[0] = 0xA0;
                    messageBuf[1] = 0x00; // A15 - A8 address byte
                    messageBuf[2] = 0x00; // A7 - A0 address byte
                    messageBuf[3] = 0xBB; // data byte
                    TWI_Start_Transceiver_With_Data( messageBuf, 4);

                    messageBuf[0] = 0xA0;
                    messageBuf[1] = 0x00; // A15 - A8 address byte
                    messageBuf[2] = 0x01; // A7 - A0 address byte
                    messageBuf[3] = 0xCC; // data byte
                    TWI_Start_Transceiver_With_Data( messageBuf, 4);

                    messageBuf[0] = 0xA0;
                    messageBuf[1] = 0x00; // A15 - A8 address byte
                    messageBuf[2] = 0x02; // A7 - A0 address byte
                    messageBuf[3] = 0xDD; // data byte
                    TWI_Start_Transceiver_With_Data( messageBuf, 4);
                    //data = TWI_Get_State_Info();

                    messageBuf[0] = TWI_Get_State_Info();
                    eeprom_write_block(messageBuf, 0, 4);

                    TWI_operation = FALSE;
                }
                else
                {
                    if(TWI_operation == TRUE)
                    {
//                        messageBuf[0] = 0xA1;
//                        TWI_Start_Transceiver_With_Data( messageBuf, 1);
//                        //TWI_Get_Data_From_Transceiver( messageBuf, 1);
//                        eeprom_write_block(messageBuf, 0, 1);
//                        TWI_operation = READ_DATA_FROM_BUFFER;



                        messageBuf[0] = 0xA1;
                        messageBuf[1] = 0x00; // A15 - A8 address byte
                        messageBuf[2] = 0x00; // A7 - A0 address byte
                        //messageBuf[3] = 0x08; // dummy data
                        TWI_Start_Transceiver_With_Data( messageBuf, 3);
                        //TWI_Get_Data_From_Transceiver( messageBuf, 2 );

                        TWI_operation = REQUEST_DATA;         // To release resources to other operations while waiting for the TWI to complete,
                                                              // we set a operation mode and continue this command sequence in a "followup"
                                                              // section further down in the code.
                        //data = TWI_Get_State_Info();

                        //TWI_Get_Data_From_Transceiver( messageBuf, 4);

                        messageBuf[0] = TWI_Get_State_Info();

                        eeprom_write_block(messageBuf, 0, 1);
                    }
                }

                //TWI_operation = REQUEST_DATA;
                if ( ! TWI_Transceiver_Busy() )
                {
                // Check if the last operation was successful
                  if ( TWI_statusReg.lastTransOK )
                  {

                    if ( TWI_operation ) // Section for follow-up operations.
                    {
                    // Determine what action to take now
                      if (TWI_operation == REQUEST_DATA)
                      { // Request/collect the data from the Slave
                        //messageBuf[0] = (TWI_targetSlaveAddress<<TWI_ADR_BITS) | (TRUE<<TWI_READ_BIT); // The first byte must always consit of General Call code or the TWI slave address.
                          messageBuf[0] = 0xA1;
                          messageBuf[1] = 0x00;
                          messageBuf[2] = 0x00;
                          messageBuf[3] = 0xA1;
                        TWI_Start_Transceiver_With_Data( messageBuf, 3 );
                        //eeprom_write_block(messageBuf, 0, 1);
                        TWI_operation = READ_DATA_FROM_BUFFER; // Set next operation
                      }
                      else if (TWI_operation == READ_DATA_FROM_BUFFER)
                      { // Get the received data from the transceiver buffer
                        TWI_Get_Data_From_Transceiver( messageBuf, 1);
                        //data = messageBuf[1];        // Store data on PORTB.
                        TWI_operation = FALSE;        // Set next operation

                        eeprom_write_block(messageBuf, 0, 4);
                      }
                    }
                  }
                  else // Got an error during the last transmission
                  {
                      data = 9;
                    // Use TWI status information to detemine cause of failure and take appropriate actions.
                    TWI_Act_On_Failure_In_Last_Transmission( TWI_Get_State_Info( ) );
                    eeprom_write_block(&data, 0, 1);
                  }
                }
            }

            //eeprom_write_block(&data, 0, 1);


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

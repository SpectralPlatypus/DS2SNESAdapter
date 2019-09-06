#define F_CPU 16000000UL
//#define DEBUG 1

#include <avr/io.h>
#include <avr/interrupt.h>
#include "definitions.h"

#include <util/delay.h>
#include <string.h>

/* ========== PORT MAPPINGS ============ */
#define IO_PORT      PORTD
#define IO_DDR       DDRD
#define IO_LATCH     PD2
#define IO_CLK       PD3

#define DATA_PORT    PORTC
#define DATA_DDR     DDRC
#define DATA_BIT     PC0

#define PSX_ACK_PORT PORTC
#define PSX_ACK_DDR  DDRC
#define PSX_ACK_PIN  PINC
#define PSX_ACK_BIT  PC0

#define PSX_PORT    PORTB
#define PSX_PIN     PINB
#define PSX_DDR     DDRB
#define PSX_CLK     PB5
#define PSX_DATA    PB4
#define PSX_CMD     PB3
#define PSX_ATT     PB2

#define TEST_PORT    PORTC
#define TEST_DDR     DDRC
#define TEST_BIT     PC5
/* ================================== */

// Message constants
#define PSX_MSG_ID 0x41
#define PSX_MSG_MYSTERY 0x5A

// Functions macros for pin driving tasks
#define PSX_SS_ENABLE() (PSX_PORT &= ~(1<<PSX_ATT))
#define PSX_SS_DISABLE() (PSX_PORT |= (1<<PSX_ATT))

#define PSX_CLK_LO() (PSX_PORT &= ~(1<<PSX_CLK))
#define PSX_CLK_HI() (PSX_PORT |= (1<<PSX_CLK))

typedef enum
{
    ST_IDLE,
    ST_READY,
    ST_MYSTERY,
    ST_REC_BUF0,
    ST_REC_BUF1
} psxCommState;

static struct map_ent snes_mapping[] =
{
    { PSX_X, SNES_B },
    { PSX_SQUARE, SNES_Y },
    { PSX_SELECT, SNES_SELECT },
    { PSX_START, SNES_START },
    { PSX_UP, SNES_UP },
    { PSX_RIGHT, SNES_RIGHT },
    { PSX_DOWN, SNES_DOWN },
    { PSX_LEFT, SNES_LEFT },
    { PSX_L1, SNES_L },
    { PSX_R1, SNES_R },
    { PSX_TRIANGLE, SNES_X },
    { PSX_O, SNES_A },
    { 0, 0 },
};

static struct map_ent tdo_mapping[] =
{
    {0, 0},
    {0, 0},
    { PSX_DOWN, TDO_DOWN },
    { PSX_UP, TDO_UP },
    { PSX_RIGHT, TDO_RIGHT },
    { PSX_LEFT, TDO_LEFT },
    { PSX_O, TDO_A },
    { PSX_X, TDO_B },
    { PSX_SQUARE, TDO_Y },
    { PSX_SELECT, TDO_SELECT },
    { PSX_START, TDO_START },
    { PSX_L1, TDO_L },
    { PSX_R1, TDO_R },
    { PSX_TRIANGLE, TDO_X },
    { 0, 0 },
    { 0, 0 }
    /* 1s after this */
};

static struct map_ent *type_mapping = &tdo_mapping[0];
static uint8_t lastBuf[2];
static uint8_t psxBuf[2];
static uint8_t currentByte = 0x0;
static uint8_t btnCounter = 0x0;
static psxCommState state = ST_IDLE;

void blinkLed();
void io_init();
void psx_spi_init();
uint8_t psxSpiReadWriteByte(uint8_t);

int main(void)
{
    io_init();

    // Enable SPI as Master
    psx_spi_init();
    PSX_SS_DISABLE();
    // Comm will not work if PSX controller is plugged in already
    // Solution is a 100ms delay after config
    _delay_ms(100);

    uint8_t volatile b = 0;

    while(1)
    {
        switch(state)
        {
        case ST_IDLE:
            PSX_CLK_HI();
            PSX_SS_ENABLE();
            b = psxSpiReadWriteByte(0x1);
            if(b == 0xff)
            {
                state = ST_READY;
            }
            break;
        case ST_READY:
            b = psxSpiReadWriteByte(0x42);
            if(b == PSX_MSG_ID)
            {
                state = ST_MYSTERY;
            }
            else
            {
                PSX_SS_DISABLE();
                state = ST_IDLE;
            }
            break;
        case ST_MYSTERY:
            b = psxSpiReadWriteByte(0x0);
            if(b == PSX_MSG_MYSTERY)
            {
                state = ST_REC_BUF0;
            }
            else
            {
                PSX_SS_DISABLE();
                state = ST_IDLE;
            }
            break;
        case ST_REC_BUF0:
            b = psxSpiReadWriteByte(0x0);
            psxBuf[0] = b;
            state = ST_REC_BUF1;
            break;
        case ST_REC_BUF1:
            b = psxSpiReadWriteByte(0x0);
            psxBuf[1] = b;
            state = ST_IDLE;

            //Final check
#ifdef DEBUG
            if((lastBuf[1] & (PSX_X)) &&
                    ((psxBuf[1] & (PSX_X))==0))
            {
                blinkLed();
            }
#endif
            PSX_SS_DISABLE();

            // Copy over to last buffer, for testing only
            memcpy(lastBuf,psxBuf,2);
            _delay_ms(16);
            break;
        default:
            state = ST_IDLE;
            break;
        }
    }
}

void io_init()
{

    //Setup Latch and CLK ports as inputs, use internal pull-up and falling edge interrupt trigger
    IO_DDR &= ~((1<<IO_CLK)|(1<<IO_LATCH));
    IO_PORT |= (1<<IO_CLK)|(1<<IO_LATCH);
    DATA_DDR |= (1<<DATA_BIT);
    //Default output is nothing pressed
    DATA_PORT |= (1<<DATA_BIT);
    //Rising Edge Interrupt
    EICRA = 0b00001111; //(3 << ISC10)|(3 << ISC00);
    EIMSK |= (1 << INT0);

    DDRC |= _BV(TEST_BIT);
    // Set ACK Pin as input
    //PSX_ACK_DDR &= ~_BV(PSX_ACK_BIT);
}

void psx_spi_init()
{
    PSX_DDR |= (1<<PSX_CMD) | (1<<PSX_ATT) | (1<<PSX_CLK);
    PSX_DDR &= ~(1<<PSX_DATA);

    PSX_PORT |= (1<<PSX_CLK);
    PSX_PORT &= ~(1<<PSX_CMD);
}

uint8_t psxSpiReadWriteByte(uint8_t writeData)
{
    uint8_t readData = 0;
    PSX_CLK_HI();
    _delay_us(8);
    for(int i = 0; i < 8; i++)
    {
        if(writeData & (1 << i))
            PSX_PORT |= (1<<PSX_CMD);
        else
            PSX_PORT &= ~(1<<PSX_CMD);

        PSX_CLK_LO();
        _delay_us(4);
        
        // read data
        if(PSX_PIN & (1<<PSX_DATA))
        {
            readData |= (1<<i);
        }
        PSX_CLK_HI();
        _delay_us(4);
    }

    PSX_PORT |= (1<<PSX_CMD);
    return readData;
}

// Function for button testing
void blinkLed()
{
    static int kek = 1;
    if(kek)
    {
        PORTC |= _BV(TEST_BIT);
        kek = 0;
    }
    else
    {
        PORTC &= ~_BV(TEST_BIT);
        kek = 1;
    }
}

ISR (INT0_vect)
{
    //Latch interrupt, 12us second later CLK will start pulsing
    //First shift the first button out (B), and then initialize CLK Int
    if(psxBuf[1] & type_mapping[0].p)
        DATA_PORT |= (1<<DATA_BIT);
    else
    {
        DATA_PORT &= ~(1<<DATA_BIT);
    }
    currentByte = psxBuf[1]; 
    btnCounter = 0x1;
    //Enable CLK Int
    EIMSK |= (1 << INT1);
}
ISR (INT1_vect)
{
    if(currentByte & type_mapping[btnCounter].p) // Button not pressed
        DATA_PORT |= (1<<DATA_BIT);
    else
        DATA_PORT &= ~(1<<DATA_BIT);

    btnCounter++;
    if(btnCounter ==  0x08)
    {
        currentByte = psxBuf[0];
    }
    else if(btnCounter == 0x0F)
    {
        DATA_PORT |= (1<<DATA_BIT);
        //This is where INT1 would be disabled if it didn't break everything
        //EIMSK &= ~(1 << INT1);
    }
}

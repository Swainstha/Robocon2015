/*
 * header.h
 *
 * Created: 12/16/2014 3:45:43 PM
 *  Author: Rabing
 */ 


#ifndef HEADER_H_
#define HEADER_H_

#if defined(atMega32)
// define ports, pins
//
//spi slave select pin
#define DD_SS           B,4
// wire master MISO to slave MISO for atmega32
#define DD_MISO         B,6

// wire master MOSI to slave MOSI
#define DD_MOSI         B,5

// wire master SCK to slave SCK
#define DD_SCK          B,7

// wire master to slave1 select pin
#define SLAVE1          B,0

// wire master to slave2 select pin
#define SLAVE2          B,2

// wire master to slave3 select pin
#define SLAVE3          B,3

// wire master to slave4 select pin
#define SLAVE4          B,1

//uart tx pin
#define TX_PIN          D,1
//uart rx pin
#define RX_PIN          D,0

#define DD_INT0         D,2

#define DD_INT1         D,3

#define DD_SCL          C,0

#define DD_SDA          C,1

#define DD_OC1A         D,5

#define DD_OC1B         D,4

#endif


#if defined(atMega8)
// define ports, pins
//
//spi slave select pin
#define DD_SS           B,2
// wire master MISO to slave MISO for atmega32
#define DD_MISO         B,4

// wire master MOSI to slave MOSI
#define DD_MOSI         B,3

// wire master SCK to slave SCK
#define DD_SCK          B,5

// wire master to slave1 select pin
#define SLAVE1          B,1

// wire master to slave2 select pin
#define SLAVE2          B,2

// wire master to slave3 select pin
#define SLAVE3          B,3

// wire master to slave4 select pin
#define SLAVE4          B,0

//uart tx pin
#define TX_PIN          D,1
//uart rx pin
#define RX_PIN          D,0

#define DD_INT0         D,2

#define DD_INT1         D,3

#define DD_SCL          C,5

#define DD_SDA          C,4

#define DD_OC1A         B,1

#define DD_OC1B         B,2

#endif

#define INPUT2(port,pin) DDR ## port &= ~_BV(pin)
#define OUTPUT2(port,pin) DDR ## port |= _BV(pin)
#define CLEAR2(port,pin) PORT ## port &= ~_BV(pin)
#define SET2(port,pin) PORT ## port |= _BV(pin)
#define TOGGLE2(port,pin) PORT ## port ^= _BV(pin)
#define READ2(port,pin) ((PIN ## port & _BV(pin))?1:0)
#define REGISTER_SET1( REGISTER, BIT1 ) REGISTER|=_BV( BIT1 )
#define REGISTER_SET2( REGISTER, BIT1, BIT2 ) REGISTER|=_BV( BIT1 )|_BV( BIT2 )
#define REGISTER_SET3( REGISTER, BIT1, BIT2, BIT3 ) REGISTER|=_BV( BIT1 )|_BV( BIT2 )|_BV( BIT3 )
#define REGISTER_SET4( REGISTER, BIT1, BIT2, BIT3, BIT4 ) REGISTER|=_BV( BIT1 )|_BV( BIT2 )|_BV( BIT3 )|_BV( BIT4 )
#define REGISTER_SET5( REGISTER, BIT1, BIT2, BIT3, BIT4, BIT5 ) REGISTER|=_BV( BIT1 )|_BV( BIT2 )|_BV( BIT3 )|_BV( BIT4 )|_BV( BIT5 )
#define REGISTER_SET6( REGISTER, BIT1, BIT2, BIT3, BIT4, BIT5, BIT6 ) REGISTER|=_BV( BIT1 )|_BV( BIT2 )|_BV( BIT3 )|_BV( BIT4 )|_BV( BIT5 )|_BV( BIT6 )
#define REGISTER_RESET( REGISTER,BIT ) REGISTER&=~_BV( BIT )
//
#define INPUT(x) INPUT2(x)
#define OUTPUT(x) OUTPUT2(x)
#define CLEAR(x) CLEAR2(x)
#define SET(x) SET2(x)
#define TOGGLE(x) TOGGLE2(x)n
#define READ(x) READ2(x)
#define PULLUP_ON(x) INPUT2(x); SET2(x)
#define PULLUP_OFF(x) INPUT2(x); CLEAR2(x)
#define Slave_Connect(x) CLEAR2(x)
#define Slave_Disconnect(x) SET2(x)


# define _16bitTo8bit( _16BitNum, _8BitHigh, _8BitLow ) _8BitLow=_16BitNum;_8BitHigh=( _16BitNum>>8 );
# define _8bitTo16bit( _16BitNum, _8BitHigh, _8BitLow ) _16BitNum=(int)_8BitLow;_16BitNum|=( (int)_8BitHigh<<8 );



#define General_interrupt_enable() sei()

#endif /* HEADER_H_ */
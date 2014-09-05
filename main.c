
/*
 * Some code inspired by and/or taken from Richard Scott Teal,
 * https://github.com/Cognoscan/msp430-libs, published under the MIT license
 */

#include <msp430.h>
#include <stdint.h>

/*
 * MSP430G2553, LP1.5, HW UART
 *
 * Serial terminal at 9600 8N1
 *
 * IR receiver on P2.0
 */

#include "printf.h"

void UART_Transmit(char data);

void __attribute__((interrupt (USCIAB0TX_VECTOR))) UART_Tx_ISR(void);
void __attribute__((interrupt (TIMER1_A0_VECTOR))) Timer1_A0_ISR(void);
void __attribute__((interrupt (TIMER1_A1_VECTOR))) Timer1_A1_ISR(void);

volatile uint8_t tx_store[16];
volatile uint8_t tx_head;
volatile uint8_t tx_tail;
volatile uint8_t transmitting;

volatile uint16_t Redge;
volatile uint16_t Fedge;
enum {
	MARK = 0,
	SPACE
};
volatile uint16_t status;

// #define NEC_TIME_UNIT	(1120)				// 560uS @ 0.5uS tick
#define NEC_TIME_UNIT		(560)			// 560uS @ 1uS tick
#define NEC_START_MARK		(16)			// Start bit mark time in timeunits
#define SAM_START_MARK		(8)			// Start bit mark time in timeunits (Samsung variant)
#define NEC_START_SPACE		(8)			// Start bit space time in timeunits
#define NEC_BIT_MARK		(1)			// Bit mark time in timeunits
#define NEC_BIT_0_SPACE		(1)			// Bit 0 space time in timeunits
#define NEC_BIT_1_SPACE		(3)			// Bit 1 space time in timeunits
#define NEC_STOP_MARK		(1)			// Stop bit mark time in timeunits
#define NEC_STOP_SPACE		(1)			// Stop bit space time in timeunits
#define NEC_REPEAT_BITS		(1)			// Repeat sequence bit count
#define NEC_REPEAT_SPACE	(4)			// Repeat bit space after AGC in timeunits
#define NEC_NDATABITS		(32)			// Protocol sends 32 data bits

enum {
	STATE_INACTIVE = 0,
	STATE_REPEAT,
	STATE_START_BIT,
	STATE_DATA_BIT,
	STATE_STOP_BIT,
};

// IR Receiver Variables
volatile uint16_t ir_count;				// Keeps track of bit being read in
volatile uint16_t byte_count;
volatile uint8_t ir_data;				// Holds the IR data as it is read in
volatile uint8_t state;					// Stores the current receiver state

typedef struct nec {
	uint8_t addr_hi;
	uint8_t addr_lo;
	uint8_t cmd;
	uint8_t inv;
} nec_t;
volatile nec_t nec;
volatile nec_t nec_repeat;

uint8_t jiffy2timeunit(uint16_t a, uint16_t b);
#define measure_mark()		(status |= BIT0)
#define measure_space()		(status &= ~BIT0)
#define measuring_mark()	((status & BIT0) == BIT0)
#define measuring_space()	(!(measuring_mark()))

#define set_timeout()		(status |= BIT1)
#define clear_timeout()		(status &= ~BIT1)
#define start_timeout(tmo)	({								\
					clear_timeout();					\
					TA1CCR1 = TA1R + ((tmo) * NEC_TIME_UNIT);		\
					TA1CCTL1 = CCIE;					\
				})
#define stop_timeout()		(TA1CCTL1 = 0)
#define timed_out()		((status & BIT1) == BIT1)

#define get_mark(tmo)		({								\
					measure_mark();						\
					start_timeout(tmo);					\
					__bis_status_register(LPM0_bits);			\
					if(timed_out())						\
					{							\
						state = STATE_INACTIVE;				\
						break;						\
					}							\
					jiffy2timeunit(Fedge, Redge);				\
				})
#define get_space(tmo)		({								\
					measure_space();					\
					start_timeout(tmo);					\
					__bis_status_register(LPM0_bits);			\
					if(timed_out())						\
					{							\
						state = STATE_INACTIVE;				\
						break;						\
					}							\
					jiffy2timeunit(Redge, Fedge);				\
				})

void main(void) {

	uint8_t mark, space;

	WDTCTL = WDTPW | WDTHOLD;

	DCOCTL = CALDCO_8MHZ;
	BCSCTL1 = CALBC1_8MHZ;
	BCSCTL2 = 0;

	P1DIR = ~BIT1;
	P1SEL = BIT2;
	P1SEL2 = BIT2;
	P1OUT = 0;

	P1DIR |= BIT0;
	P1OUT &= ~BIT0;

	P2DIR = ~(BIT0|BIT3);					// P2.0 is input (TA1.CCI0A), P2.3 is TA1.CCI0B
	P2SEL = BIT0 | BIT3;					// TA1.0 on P2.0, TA1.0 on P2.3
	P2SEL2 = 0;
	P2OUT = 0;

	P3DIR = 0xFF;
	P3OUT = 0;

	TA1CCTL0 = CAP | CM_3 | SCS | CCIS_0 | CCIE;		// Capture, rising, synchronous, CCI0A/TA1CCR0, with interrupts
	TA1CCTL1 = 0;						// Used for the timeout counter
	TA1CCTL2 = 0;
	TA1CTL = TASSEL_2 | ID_3 | MC_2 | TACLR;		// SMCLK/8 == 1MHz, tick=1uS, Continuous up

	transmitting = 0;

	UCA0BR0 = 65;						// 9600 baud @ 8MHz clk
	UCA0BR1 = 3;
	UCA0MCTL = UCBRS_2;
	UCA0CTL0 = 0;
	UCA0CTL1 = UCSSEL_2 | UCSWRST;

	UCA0CTL1 &= ~UCSWRST;

	__eint();

	UART_Transmit(0);

	__delay_cycles(1000);

	my_puts("NEC CIR decoder\r\n");
	my_puts("***************\r\n");

	Redge = 0;
	Fedge = 0;
	state = STATE_INACTIVE;
	status = 0;
	while(1)
	{
		switch(state)
		{
			case STATE_INACTIVE:
				nec.addr_hi = 0;
				nec.addr_lo = 0;
				nec.cmd = 0;
				nec.inv = 0;
				ir_data = 0;
				ir_count = 0;
				byte_count = 0;
				state = STATE_START_BIT;
			break;
			case STATE_START_BIT:
				mark = get_mark(20);
				space = get_space(10);
				if((mark == NEC_START_MARK ||
				    mark == SAM_START_MARK) &&
				   (space == NEC_START_SPACE ||
				    space == NEC_REPEAT_SPACE))
				{
					if(space == NEC_REPEAT_SPACE)
					{
						state = STATE_REPEAT;
					}
					else
					{
						nec_repeat.addr_hi = 0;
						nec_repeat.addr_lo = 0;
						nec_repeat.cmd = 0;
						nec_repeat.inv = 0;
						state = STATE_DATA_BIT;
					}
				}
				else
				{
					state = STATE_INACTIVE;
				}
			break;
			case STATE_DATA_BIT:
				mark = get_mark(2);
				space = get_space(4);
				if(mark == NEC_BIT_MARK &&
				   (space == NEC_BIT_0_SPACE ||
				    space == NEC_BIT_1_SPACE))
				{
					ir_data >>= 1;
					if(space == NEC_BIT_1_SPACE)
					{
						ir_data |= 0x80;
					}
					ir_count++;
				}
				else
				{
					state = STATE_INACTIVE;
				}

				if(ir_count == 8)
				{
					uint8_t inv;
					uint8_t addr_hi;
					byte_count++;

					switch(byte_count)
					{
						case 1:
							nec.addr_hi = ir_data;
							nec_repeat.addr_hi = ir_data;
						break;
						case 2:
							nec.addr_lo = ir_data;
							nec_repeat.addr_lo = ir_data;
						break;
						case 3:
							nec.cmd = ir_data;
							nec_repeat.cmd = ir_data;
						break;
						case 4:
							addr_hi = ~nec.addr_hi;
							inv = ~nec.cmd;
							nec.inv = ir_data;
							if(inv == nec.inv)
							{
								nec_repeat.addr_hi = nec.addr_hi;
								nec_repeat.addr_lo = nec.addr_lo;
								nec_repeat.cmd = nec.cmd;
								nec_repeat.inv = nec.inv;
								my_printf("   addr = %X %X cmd = %X inv = %X %s\r\n",
									  nec.addr_hi, nec.addr_lo, nec.cmd, nec.inv,
									  addr_hi == nec.addr_lo ? "Normal" : "Extended");
								state = STATE_START_BIT;
							}
							else
							{
								state = STATE_INACTIVE;
							}
						break;
						default:
						break;
					}
					ir_count = 0;
				}
			break;
			case STATE_REPEAT:
				my_printf("R: addr = %X %X cmd = %X inv = %X\r\n",
					   nec_repeat.addr_hi, nec_repeat.addr_lo, nec_repeat.cmd, nec_repeat.inv);
				state = STATE_START_BIT;
			break;
			default:
				state = STATE_INACTIVE;
			break;
		}
		if(TA1CCTL0 & COV)
		{
			my_puts("(COV)");
			TA1CCTL0 &= ~COV;
		}
	}
}

void UART_Transmit(char data)
{
	if (!transmitting)
	{
		UCA0TXBUF = data;
		transmitting = 1;
		IE2 |= UCA0TXIE;
	} else {
		uint8_t busy = 1;
		while (busy)
		{
			if(!(((tx_head - tx_tail) == 1) || ((tx_tail - tx_head) == 15)))
			{
				busy = 0;
			}
		}
		tx_store[tx_tail] = data;
		tx_tail++;
		tx_tail &= 0x0F;
	}
}

void __attribute__((interrupt (USCIAB0TX_VECTOR))) UART_Tx_ISR(void)
{
	if ((tx_head - tx_tail) != 0)
	{
		UCA0TXBUF = tx_store[tx_head];
		tx_head++;
		tx_head &= 0x0F;
	} else {
		transmitting = 0;
		IE2 &= ~UCA0TXIE;
	}
	__bic_status_register_on_exit(LPM0_bits);
}

void __attribute__((interrupt (TIMER1_A0_VECTOR))) Timer1_A0_ISR(void)
{
	if(TA1CCTL0 & CCI)
	{
		Fedge = TA1CCR0;
		if(measuring_mark())
		{
			__bic_status_register_on_exit(LPM0_bits);
		}
	}
	else
	{
		Redge = TA1CCR0;
		if(measuring_space())
		{
			__bic_status_register_on_exit(LPM0_bits);
		}
	}

	stop_timeout();
}

void __attribute__((interrupt (TIMER1_A1_VECTOR))) Timer1_A1_ISR(void)
{
	if((TA1IV & TA1IV_TACCR1) == TA1IV_TACCR1)
	{
		state = STATE_INACTIVE;
		set_timeout();
		stop_timeout();
		__bic_status_register_on_exit(LPM0_bits);
	}
}

uint8_t jiffy2timeunit(uint16_t a, uint16_t b)
{
	uint16_t q;
	uint16_t r;

	q = (a - b) / NEC_TIME_UNIT;
	r = (a - b) % NEC_TIME_UNIT;

	// 30% tolerance
	if(r > (NEC_TIME_UNIT - (3 * NEC_TIME_UNIT / 10)))
	{
		q++;
	}

	return q & 0xff;
}

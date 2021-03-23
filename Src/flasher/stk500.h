/***************************
* This is a basic implementation of the
* Atmel STK500 protocol for AVR flashers.
*/

#ifndef __STK500_H__
#define __STK500_H__

/* Main function handling STK500 commands */
int stk500_parse_cmd(void);

#endif /* __STK500_H__ */

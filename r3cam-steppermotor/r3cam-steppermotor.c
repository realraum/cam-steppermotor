/*
 *  spreadspace avr utils
 *
 *
 *  Copyright (C) 2012 Christian Pointner <equinox@spreadspace.org>
 *
 *  This file is part of spreadspace avr utils.
 *
 *  spreadspace avr utils is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  any later version.
 *
 *  spreadspace avr utils is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with spreadspace avr utils. If not, see <http://www.gnu.org/licenses/>.
 */


#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/power.h>

#include "util.h"
#include "led.h"

/*
             LUFA Library
     Copyright (C) Dean Camera, 2012.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/
#include <LUFA/Drivers/USB/USB.h>
#include "lufa-descriptor-usbserial.h"

USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
  {
    .Config =
      {
        .ControlInterfaceNumber         = 0,

        .DataINEndpointNumber           = CDC_TX_EPNUM,
        .DataINEndpointSize             = CDC_TXRX_EPSIZE,
        .DataINEndpointDoubleBank       = false,

        .DataOUTEndpointNumber          = CDC_RX_EPNUM,
        .DataOUTEndpointSize            = CDC_TXRX_EPSIZE,
        .DataOUTEndpointDoubleBank      = false,

        .NotificationEndpointNumber     = CDC_NOTIFICATION_EPNUM,
        .NotificationEndpointSize       = CDC_NOTIFICATION_EPSIZE,
        .NotificationEndpointDoubleBank = false,
      },
  };

void EVENT_USB_Device_ConfigurationChanged(void)
{
  CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
}

void EVENT_USB_Device_ControlRequest(void)
{
  CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}
/* end LUFA CDC-ACM specific definitions*/


/* Start Our Code */

#define M_PORT PORTB
#define M_DDR DDRB
#define M_DIRECTION  0
#define M_CLK 1
#define M_ENABLE 2
#define M_RESET 3
#define M_FULLSTEPS 4

static uint8_t m_clk_divisor_ = 0;
static uint8_t m_clk_divisor_counter_ = 0;
static uint16_t m_steps_to_go_ = 0;

inline void m_timer_enable(void)
{
    TIMSK0 |= (1<<TOIE0);    
}

inline void m_timer_disable(void)
{
    TIMSK0 &= ~(1<<TOIE0);    
}

void motor_stop(void)
{
    m_timer_disable();
    M_PORT &= ~(1 << M_ENABLE);
    m_steps_to_go_ = 0;
}

ISR(TIMER0_OVF_vect)
{
    if (m_steps_to_go_ == 0)
        motor_stop();
    if (m_clk_divisor_counter_ == 0)
    {
        m_clk_divisor_counter_ = m_clk_divisor_;
        M_PORT ^=  (1 << M_CLK);
        m_steps_to_go_--;
    }
    else 
        m_clk_divisor_counter_--;
}

void motor_set_speed(uint8_t speed)
{
    m_clk_divisor_ = 0xFF - speed;
}

void motor_run(uint16_t steps, uint8_t direction)
{
    //reset by pulling reset low for 100ms
    M_PORT &= ~(1 << M_RESET);
    _delay_ms(100);
    M_PORT |= (1 << M_RESET);
    
    m_clk_divisor_counter_ = 0;
    m_steps_to_go_ = steps;
    
    if (direction)
        M_PORT |= (1 << M_DIRECTION);
    else
        M_PORT &= ~(1 << M_DIRECTION);
    
    //enable motor
    M_PORT |= (1 << M_ENABLE);
    m_timer_enable();    
}

void init_pins()
{
    M_DDR = 0x0F;
    M_PORT = (1 << M_RESET) | (1 << M_FULLSTEPS);
    
    // Configure timer 0 to generate a timer overflow interrupt every
    // 560us (NEC Protocol)
    TCCR0A = 0x00;
    TIMSK0 = (0<<TOIE0);
    //TCCR0B = 1<<WGM02 | 1<<CS02 | 0<<CS01| 1<<CS00;   //Teiler 1024
    TCCR0B = 1<<WGM02 | 1<<CS02 | 0<<CS01| 0<<CS00;   //Teiler 256
    //OCR0A = 139;        // (1+139)*8 = 1120 -> 70us @ 16 MHz -> 1*alpha
    OCR0A = 255;        // (1+139)*8 = 1120 -> 70us @ 16 MHz -> 1*alpha
}

uint8_t cur_speed = 0xFF;
void handle_cmd(uint8_t cmd)
{
  switch(cmd) {
  case '0': led_off(); break;
  case '1': led_on(); break;
  case 't': led_toggle(); break;
  case 'r': reset2bootloader(); break;
  case 's': motor_stop(); break;
  case 'c': motor_run(300,0); break;
  case 'w': motor_run(300,1); break;
  case 'C': motor_run(1000,0); break;
  case 'W': motor_run(1000,1); break;
  case '+': motor_set_speed(++cur_speed); break;
  case '-': motor_set_speed(--cur_speed); break;
  default: CDC_Device_SendString(&VirtualSerial_CDC_Interface, "error\n"); return;
  }
  CDC_Device_SendString(&VirtualSerial_CDC_Interface, "ok\n");
}



int main(void)
{
  MCUSR &= ~(1 << WDRF);
  wdt_disable();

  cpu_init();
  led_init();
  USB_Init();
  init_pins();
  sei();

  for(;;) {
    int16_t BytesReceived = CDC_Device_BytesReceived(&VirtualSerial_CDC_Interface);
    while(BytesReceived > 0) {
      int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
      if(!(ReceivedByte < 0)) {
        handle_cmd(ReceivedByte);
      }
      BytesReceived--;
    }

    CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
    USB_USBTask();
  }
}

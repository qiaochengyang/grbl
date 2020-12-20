/*
  serial.c - Low level functions for sending and recieving bytes via the serial port
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"
// #define UART_BUFFERED
#define RX_RING_BUFFER (RX_BUFFER_SIZE+1)
#define TX_RING_BUFFER (TX_BUFFER_SIZE+1)

uint8_t serial_rx_buffer[RX_RING_BUFFER];
uint8_t serial_rx_buffer_head = 0;
volatile uint8_t serial_rx_buffer_tail = 0;

uint8_t serial_tx_buffer[TX_RING_BUFFER];
uint8_t serial_tx_buffer_head = 0;
volatile uint8_t serial_tx_buffer_tail = 0;


// Returns the number of bytes available in the RX serial buffer.
uint8_t serial_get_rx_buffer_available()
{
  uint8_t rtail = serial_rx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_rx_buffer_head >= rtail) { return(RX_BUFFER_SIZE - (serial_rx_buffer_head-rtail)); }
  return((rtail-serial_rx_buffer_head-1));
}


// Returns the number of bytes used in the RX serial buffer.
// NOTE: Deprecated. Not used unless classic status reports are enabled in config.h.
uint8_t serial_get_rx_buffer_count()
{
  uint8_t rtail = serial_rx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_rx_buffer_head >= rtail) { return(serial_rx_buffer_head-rtail); }
  return (RX_BUFFER_SIZE - (rtail-serial_rx_buffer_head));
}


// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
uint8_t serial_get_tx_buffer_count()
{
  uint8_t ttail = serial_tx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_tx_buffer_head >= ttail) { return(serial_tx_buffer_head-ttail); }
  return (TX_RING_BUFFER - (ttail-serial_tx_buffer_head));
}


void serial_init()
{
  // // Set baud rate
  // #if BAUD_RATE < 57600
  //   uint16_t UBRR0_value = ((F_CPU / (8L * BAUD_RATE)) - 1)/2 ;
  //   UCSR0A &= ~(1 << U2X0); // baud doubler off  - Only needed on Uno XXX
  // #else
  //   uint16_t UBRR0_value = ((F_CPU / (4L * BAUD_RATE)) - 1)/2;
  //   UCSR0A |= (1 << U2X0);  // baud doubler on for high baud rates, i.e. 115200
  // #endif
  // UBRR0H = UBRR0_value >> 8;
  // UBRR0L = UBRR0_value;

  // // enable rx, tx, and interrupt on complete reception of a byte
  // UCSR0B |= (1<<RXEN0 | 1<<TXEN0 | 1<<RXCIE0);

  // // defaults to 8-bit, no parity, 1 stop bit
  //   //
  // Enable the GPIO Peripheral used by the UART.
  //
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

  //
  // Enable UART0
  //
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

  //
  // Configure GPIO Pins for UART mode.
  //
  MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
  MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
  MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

  //
  // Use the internal 16MHz oscillator as the UART clock source.
  //
  UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
  MAP_UARTConfigSetExpClk(UART0_BASE, 16000000, 115200,
                            (UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_WLEN_8));
  //
  // Initialize the UART for console I/O.
  //
  // UARTStdioConfig(0, 115200, 16000000);
  MAP_UARTFIFOEnable(UART0_BASE);
  MAP_UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
  //
  // Flush both the buffers.
  //
  UARTFlushRx();
  UARTFlushTx(true);  
  MAP_UARTIntDisable(UART0_BASE, 0xFFFFFFFF);
  MAP_UARTIntEnable(UART0_BASE, UART_INT_TX | UART_INT_RX);
  IntRegister(INT_UART0,UART0IntHandler);
  MAP_IntEnable(INT_UART0);
  MAP_UARTEnable(UART0_BASE);
}


// Writes one byte to the TX serial buffer. Called by main program.
void serial_write(uint8_t data) {
  // Calculate next head
  uint8_t next_head = serial_tx_buffer_head + 1;
  if (next_head == TX_RING_BUFFER) { next_head = 0; }

  // Wait until there is space in the buffer
  while (next_head == serial_tx_buffer_tail) {
    // TODO: Restructure st_prep_buffer() calls to be executed here during a long print.
    if (sys_rt_exec_state & EXEC_RESET) { return; } // Only check for abort to avoid an endless loop.
  }

  // Store data and advance head
  serial_tx_buffer[serial_tx_buffer_head] = data;
  serial_tx_buffer_head = next_head;

  // Enable Data Register Empty Interrupt to make sure tx-streaming is running
  // UCSR0B |=  (1 << UDRIE0);
  SERIAL_TX_ISR();
}


// Data Register Empty Interrupt handler
void SERIAL_TX_ISR()
// ISR(SERIAL_UDRE)
{
  
  uint8_t tail = serial_tx_buffer_tail; // Temporary serial_tx_buffer_tail (to optimize for volatile)
  while(1)
  {
    // Turn off Data Register Empty Interrupt to stop tx-streaming if this concludes the transfer
    if (tail == serial_tx_buffer_head) { 
      // UCSR0B &= ~(1 << UDRIE0); 
      return;
      }
    // Send a byte from the buffer
    // UDR0 = serial_tx_buffer[tail];
    if(!UARTCharPutNonBlocking(UART0_BASE,serial_tx_buffer[tail])) return;
    // Update tail position
    tail++;
    if (tail == TX_RING_BUFFER) { tail = 0; }

    serial_tx_buffer_tail = tail;

    
  }
}


// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t serial_read()
{
  uint8_t tail = serial_rx_buffer_tail; // Temporary serial_rx_buffer_tail (to optimize for volatile)
  if (serial_rx_buffer_head == tail)
  {
    return SERIAL_NO_DATA;
  }
  else
  {
    uint8_t data = serial_rx_buffer[tail];

    tail++;
    if (tail == RX_RING_BUFFER)
    {
      tail = 0;
    }
    serial_rx_buffer_tail = tail;

    return data;
  }
}

void SERIAL_RX_ISR()
{
  int32_t data;
  uint8_t next_head;
  while (1)
  {
    data = UARTCharGetNonBlocking(UART0_BASE);
    if (data == -1)
      return;
    // Pick off realtime command characters directly from the serial stream. These characters are
    // not passed into the main buffer, but these set system state flag bits for realtime execution.
    switch (data)
    {
    case CMD_RESET:
      mc_reset();
      break; // Call motion control reset routine.
    case CMD_STATUS_REPORT:
      system_set_exec_state_flag(EXEC_STATUS_REPORT);
      break; // Set as true
    case CMD_CYCLE_START:
      system_set_exec_state_flag(EXEC_CYCLE_START);
      break; // Set as true
    case CMD_FEED_HOLD:
      system_set_exec_state_flag(EXEC_FEED_HOLD);
      break; // Set as true
    default:
      if (data > 0x7F)
      { // Real-time control characters are extended ACSII only.
        switch (data)
        {
        case CMD_SAFETY_DOOR:
          system_set_exec_state_flag(EXEC_SAFETY_DOOR);
          break; // Set as true
        case CMD_JOG_CANCEL:
          if (sys.state & STATE_JOG)
          { // Block all other states from invoking motion cancel.
            system_set_exec_state_flag(EXEC_MOTION_CANCEL);
          }
          break;
#ifdef DEBUG
        case CMD_DEBUG_REPORT:
        {
          uint8_t sreg = SREG;
          cli();
          bit_true(sys_rt_exec_debug, EXEC_DEBUG_REPORT);
          SREG = sreg;
        }
        break;
#endif
        case CMD_FEED_OVR_RESET:
          system_set_exec_motion_override_flag(EXEC_FEED_OVR_RESET);
          break;
        case CMD_FEED_OVR_COARSE_PLUS:
          system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_PLUS);
          break;
        case CMD_FEED_OVR_COARSE_MINUS:
          system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_MINUS);
          break;
        case CMD_FEED_OVR_FINE_PLUS:
          system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_PLUS);
          break;
        case CMD_FEED_OVR_FINE_MINUS:
          system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_MINUS);
          break;
        case CMD_RAPID_OVR_RESET:
          system_set_exec_motion_override_flag(EXEC_RAPID_OVR_RESET);
          break;
        case CMD_RAPID_OVR_MEDIUM:
          system_set_exec_motion_override_flag(EXEC_RAPID_OVR_MEDIUM);
          break;
        case CMD_RAPID_OVR_LOW:
          system_set_exec_motion_override_flag(EXEC_RAPID_OVR_LOW);
          break;
        case CMD_SPINDLE_OVR_RESET:
          system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_RESET);
          break;
        case CMD_SPINDLE_OVR_COARSE_PLUS:
          system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_PLUS);
          break;
        case CMD_SPINDLE_OVR_COARSE_MINUS:
          system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_MINUS);
          break;
        case CMD_SPINDLE_OVR_FINE_PLUS:
          system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_PLUS);
          break;
        case CMD_SPINDLE_OVR_FINE_MINUS:
          system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_MINUS);
          break;
        case CMD_SPINDLE_OVR_STOP:
          system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_STOP);
          break;
        case CMD_COOLANT_FLOOD_OVR_TOGGLE:
          system_set_exec_accessory_override_flag(EXEC_COOLANT_FLOOD_OVR_TOGGLE);
          break;
#ifdef ENABLE_M7
        case CMD_COOLANT_MIST_OVR_TOGGLE:
          system_set_exec_accessory_override_flag(EXEC_COOLANT_MIST_OVR_TOGGLE);
          break;
#endif
        }
        // Throw away any unfound extended-ASCII character by not passing it to the serial buffer.
      }
      else
      { // Write character to buffer
        next_head = serial_rx_buffer_head + 1;
        if (next_head == RX_RING_BUFFER)
        {
          next_head = 0;
        }

        // Write data to buffer unless it is full.
        if (next_head != serial_rx_buffer_tail)
        {
          serial_rx_buffer[serial_rx_buffer_head] = data;
          serial_rx_buffer_head = next_head;
        }
      }
    }
  }
}

void
UART0IntHandler(void)
{
    uint32_t ui32Status;
    uint32_t ui32Mode;

    //
    // Read the interrupt status of the UART.
    //
    ui32Status = MAP_UARTIntStatus(UART0_BASE, 1);

    //
    // Clear any pending status, even though there should be none since no UART
    // interrupts were enabled.  If UART error interrupts were enabled, then
    // those interrupts could occur here and should be handled.  Since uDMA is
    // used for both the RX and TX, then neither of those interrupts should be
    // enabled.
    //
    MAP_UARTIntClear(UART0_BASE, ui32Status);
    if(ui32Status & UART_INT_TX) SERIAL_TX_ISR();
    if(ui32Status & UART_INT_RX) SERIAL_RX_ISR();
}
void serial_reset_read_buffer()
{
  serial_rx_buffer_tail = serial_rx_buffer_head;
}

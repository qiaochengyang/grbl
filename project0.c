
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "grbl.h"


// Declare system global variable structure
system_t sys;
int32_t sys_position[N_AXIS];      // Real-time machine (aka home) position vector in steps.
int32_t sys_probe_position[N_AXIS]; // Last probe position in machine coordinates and steps.
volatile uint8_t sys_probe_state;   // Probing state value.  Used to coordinate the probing cycle with stepper ISR.
volatile uint8_t sys_rt_exec_state;   // Global realtime executor bitflag variable for state management. See EXEC bitmasks.
volatile uint8_t sys_rt_exec_alarm;   // Global realtime executor bitflag variable for setting various alarms.
volatile uint8_t sys_rt_exec_motion_override; // Global realtime executor bitflag variable for motion-based overrides.
volatile uint8_t sys_rt_exec_accessory_override; // Global realtime executor bitflag variable for spindle/coolant overrides.
#ifdef DEBUG
  volatile uint8_t sys_rt_exec_debug;
#endif
//*****************************************************************************
//
// This example application demonstrates the use of the timers to generate
// periodic interrupts.
//
//*****************************************************************************
int
main(void)
{
    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    MAP_FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Enable the GPIO pins for the LED (PF1 & PF2).
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_1);


    //
    // Enable processor interrupts.
    //
    MAP_IntMasterEnable();//important

      // Initialize system upon power-up.
    serial_init();   // Setup serial baud rate and interrupts
    settings_init(); // Load Grbl settings from EEPROM
    stepper_init();  // Configure stepper pins and interrupt timers
    system_init();   // Configure pinout pins and pin-change interrupt

    memset(sys_position,0,sizeof(sys_position)); // Clear machine position.
    sei(); // Enable interrupts

    // Initialize system state.
    #ifdef FORCE_INITIALIZATION_ALARM
      // Force Grbl into an ALARM state upon a power-cycle or hard reset.
      sys.state = STATE_ALARM;
    #else
      sys.state = STATE_IDLE;
    #endif
    
    // Check for power-up and set system alarm if homing is enabled to force homing cycle
    // by setting Grbl's alarm state. Alarm locks out all g-code commands, including the
    // startup scripts, but allows access to settings and internal commands. Only a homing
    // cycle '$H' or kill alarm locks '$X' will disable the alarm.
    // NOTE: The startup script will run after successful completion of the homing cycle, but
    // not after disabling the alarm locks. Prevents motion startup blocks from crashing into
    // things uncontrollably. Very bad.
    #ifdef HOMING_INIT_LOCK
      if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { sys.state = STATE_ALARM; }
    #endif

    // Grbl initialization loop upon power-up or a system abort. For the latter, all processes
    // will return to this loop to be cleanly re-initialized.
    for(;;) {

      // Reset system variables.
      uint8_t prior_state = sys.state;
      memset(&sys, 0, sizeof(system_t)); // Clear system struct variable.
      sys.state = prior_state;
      sys.f_override = DEFAULT_FEED_OVERRIDE;  // Set to 100%
      sys.r_override = DEFAULT_RAPID_OVERRIDE; // Set to 100%
      sys.spindle_speed_ovr = DEFAULT_SPINDLE_SPEED_OVERRIDE; // Set to 100%
      memset(sys_probe_position,0,sizeof(sys_probe_position)); // Clear probe position.
      sys_probe_state = 0;
      sys_rt_exec_state = 0;
      sys_rt_exec_alarm = 0;
      sys_rt_exec_motion_override = 0;
      sys_rt_exec_accessory_override = 0;

      // Reset Grbl primary systems.
      serial_reset_read_buffer(); // Clear serial read buffer
      gc_init(); // Set g-code parser to default state
      spindle_init();
      coolant_init();
      limits_init();
      probe_init();
      plan_reset(); // Clear block buffer and planner variables
      st_reset(); // Clear stepper subsystem variables.

      // Sync cleared gcode and planner positions to current system position.
      plan_sync_position();
      gc_sync_position();

      // Print welcome message. Indicates an initialization has occured at power-up or with a reset.
      report_init_message();

      // Start Grbl main loop. Processes program inputs and executes them.
      protocol_main_loop();

    }
    return 0;   /* Never reached */
}


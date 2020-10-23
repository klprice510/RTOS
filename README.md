# RTOS
Real Time Operating System

This is C code for a real time operating system using an ATMEGA32 8 bit microcontroller on an ATSTK500 development board. The tasks assigned in the RTOS code blink different LEDs on the development board.  This particular code demonstrates the scheduling of 3 user tasks plus an idle task and uses preemptive multitasking.  The code implements a counting semaphore, a timer event handler, and an aging scheduler.

The code uses embedded assembly to switch task stacks.

This code was done for a computer science graduate class at the University of Idaho, CS 552 Real Time Operating Systems.

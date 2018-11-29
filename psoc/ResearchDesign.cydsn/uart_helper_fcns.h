/* ========================================
 *
 * UART Helper Functions for 2D Spine Control Test
 * Header file
 *
 * Copyright Berkeley Emergent Space Tensegrities Lab, 2018
 * (insert license later.)
 *
 * ========================================
*/

/**
 * uart_helper_fcns.h
 * A collection of helper functions for the UART communication,
 * when interrupt-driven (for multitasking.)
 * 
 * How do I use these functions?
 * Write #include "uart_helper_fcns.h" at the top of your main.c file,
 * and you'll be able to call every function here.
 * 
 * IMPORTANT: this assumes you'll use the interrupt handler below, Interrupt_Handler_UART_Receive.
 */

// This is called an include guard. See https://en.wikipedia.org/wiki/Include_guard for more info.
// This is a macro also.
#ifndef UART_HELPER_FCNS_H
#define UART_HELPER_FCNS_H
    
// Cypress' project.h already has its own include guard, so we can safely
// do nested #include's here.
#include <project.h>

// Handler for receiving UART data. Replace with your ISR if you've changed the name.
// This function is the command parser for UART data.
CY_ISR( Interrupt_Handler_UART_Receive );

// The command parser itself. Does the writing to and from variables and whatnot.
void UART_Command_Parser();

#endif //UART_HELPER_FCNS_H

/* [] END OF FILE */

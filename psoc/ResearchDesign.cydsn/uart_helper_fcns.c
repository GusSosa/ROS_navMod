/* ========================================
 *
 * UART Helper Functions for 2D Spine Control Test
 * Code implementation (.c file)
 *
 * Copyright Berkeley Emergent Space Tensegrities Lab, 2018
 * (insert license later.)
 *
 * ========================================
*/

// Adapted from Cypress' example CE95277 ADC and UART,
// and from Andrew Sabelhaus' example code for ME235 S'18 at UC Berkeley.

// As one aside: floating point numbers don't seem to work on the PSoC unless
// specifically enabled. Follow https://community.cypress.com/docs/DOC-9389
// for enabling floats (newlib_nano = False, and heap size change to 0x1000.)

// Include Cypress' libraries, which we need for various things.
// These have their own include guards so are "safe" without an ifndef here.
#include <project.h>
// stdio provides the sprintf and sscanf for parsing UART input and producing output
#include "stdio.h"
// and we include the corresponding header file just for consistency in definitions.
#include "uart_helper_fcns.h"
// Also need the global data storage variables for writing to and from
#include "data_storage.h"

// Local variables which do not have scope outside this file:
// Transmit and receive buffers for the PSoC's UART will be strings,
// with the following number of characters:
#define TRANSMIT_LENGTH 128
#define RECEIVE_LENGTH 128
// Since these buffers will only be used in this helper function,
// we can declare them here.
char transmit_buffer[TRANSMIT_LENGTH];
char receive_buffer[RECEIVE_LENGTH];

// We need to keep track of the number of characters received as they come in over UART,
// so that we can properly place them into the buffer.
// Needs to be global since used in a few different functions.
static uint8 num_chars_received = 0;

// Definition of the UART interrupt service routine.
// This stores the incoming data in a buffer, then calls the parser when command is done (newline.)
CY_ISR( Interrupt_Handler_UART_Receive ){
    // We assume this ISR is called when a byte is received.
    uint8 received_byte = UART_GetChar();
    
    // C allows us to "switch" on uint8s, since characters are also numbers via the ASCII table.
    // very convenient.
    // The switch-case statement makes it easy to do single-character commands.
    switch( received_byte )
    {
        case '\r':
            // flow downward, no specific lines of code for carriage return
        
        case '\n':
            // newline or carriage return received, so finally set the PWM parameters
            // First, terminate the string. This is for the use of sscanf below.
            receive_buffer[num_chars_received] = '\0';
            // Print back the newline/carriage return, to complete the "respond back to the terminal" code
            // Note: for files, linux needs \n whereas windows needs \r\n, but for terminals,
            // it seems that both want \r\n (CR+LF).
            UART_PutString("\r\n");
            // Call the helper function to actually set the PWM
            UART_Command_Parser();
            break;

        default:
            // The "default" case is "anything else", which is "store another character."
            // Add to the received buffer.
            receive_buffer[num_chars_received] = received_byte;
            // Respond back to the terminal
            UART_PutChar( received_byte );
            // We need to increment the counter. i++ does this without an equals sign for assignment
            num_chars_received++;
            break;
        // end of case statement.
    }
}

/**
 * The command parser itself. Currently supports the following:
 * e = enable the PWM
 * x = disable the PWM
 * u float float float float = set the current control input (write to the global variable.)
 * q = query. Return the current control input that's stored in memory.
 * s = hard stop. "The Big Red Button."
 */
void UART_Command_Parser() {
    // First, get the command, and switch on it.
    char cmd = 0;
    // this should only read the first character 
    sscanf(receive_buffer, "%c", &cmd);
    // When parsing a more complicated command below, we'll need to check
    // if the correct number of arguments were specified, and otherwise throw an error.
    uint8 num_filled;
    
    // switch on the command.
    switch(cmd) {
        case 'x':
            // Added functionality: if the user types an x, then the PWM stops.
            sprintf(transmit_buffer, "Stopping PWM.\r\n");
            //PWM_Servo_Stop();
            break;
            
        case 'e':
            // Similarly, type e to enable.
            sprintf(transmit_buffer, "Enabling PWM.\r\n");
            //PWM_Servo_Start();
            break;
            
        case 'u':
            // The most important one! 
            // The floats can be put directly into the global variable.
            num_filled = sscanf(receive_buffer, "u %f %f %f %f", &current_control[0],
                   &current_control[1], &current_control[2], &current_control[3]);
            // Print out a message according to how much was parsed.
            if( num_filled == 4 ){
                // Return the resulting data that was stored.
                sprintf(transmit_buffer, "Stored an input of %f, %f, %f, %f\r\n", current_control[0],
                    current_control[1], current_control[2], current_control[3]);
                controller_status = 1;
                first_loop_1 = 1;
                first_loop_2 = 1;
            }
            else {
                // did not receive exactly 4 control inputs.
                sprintf(transmit_buffer, "Error!! You typed %s, which gave %i control inputs when 4 were expected.\r\n", receive_buffer, num_filled);
            }
            

       
            break;
            
        case 'q':
            // query the state of the store control commands.
            sprintf(transmit_buffer, "Current control inputs are %f, %f, %f, %f\r\n", current_control[0],
                    current_control[1], current_control[2], current_control[3]);
            break;
            
        case 's':
            // hard shutdown, kill everything. "The Big Red Button."
            // To-do: implement this
            sprintf(transmit_buffer, "Hard Stop. TO-DO: IMPLEMENT THIS.\r\n");
            break;
            
        case 'w':
            // echo back the welcome message.
            UART_Welcome_Message();
            // and push an empty string to the transmit buffer.
            sprintf(transmit_buffer, "");
            break;
            
        case 'c':
            // Clear the UART's buffers. This shouldn't ever need to be used,
            // but is provided in case "something bad happens"
            UART_ClearRxBuffer();
            UART_ClearTxBuffer();
            // still need to specify some message to send back to the terminal.
            sprintf(transmit_buffer, "Cleared RX and TX buffers for the UART on the PSoC.\r\n");
            break;
            
        default:
            // In any other case, report an error.
            sprintf(transmit_buffer, "Error! Command not recognized!\r\n");
            break;
        
    }
    // Write the resulting message.
    UART_PutString(transmit_buffer);
    // and reset the counter into the receive buffer so that 
    // the ISR overwrites the last command.
    num_chars_received = 0;
}

// The welcome message.
// UPDATE THIS when new functionality is added.
void UART_Welcome_Message(){
    
    UART_PutString("\r\n2D Spine Controller Test.\r\n");
    UART_PutString("Copyright 2018 Berkeley Emergent Space Tensegrities Lab.\r\n");
    UART_PutString("Usage: send strings of the form (char) (optional_args). Currently supported:\r\n");
    UART_PutString("(NOTE: THESE MUST BE FOLLOWED EXACTLY, with exact spacing.)\r\n\n");
    UART_PutString("e = enable PWM\r\n");
    UART_PutString("x = disable PWM\r\n");
    UART_PutString("q = query currently-stored control input\r\n");
    UART_PutString("s = hard stop. The Big Red Button. (hopefully.)\r\n");
    UART_PutString("w = echo back this welcome message\r\n");
    UART_PutString("c = clear all UART tx/rx buffers on the PSoC \r\n");
    UART_PutString("u float float float float = assign control input\r\n\n");
    //UART_PutString("Remember to set your terminal's newline to LF or automatic detection. (TeraTerm: Setup -> Terminal -> New-line).\n\n");
}

/* [] END OF FILE */

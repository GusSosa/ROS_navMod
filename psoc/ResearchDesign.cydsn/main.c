/* ========================================
 *
 * 2D Spine Controller Test Embedded Software
 *
 * Copyright Berkeley Emergent Space Tensegrities Lab, 2018
 * (insert license later.)
 *
 * ========================================
*/

// Include both the UART helper functions and the header
// that has the global variables we need.
// note that both of these should have include guards in them already
// so it's safe to include them directly here.
#include <project.h>
#include "stdio.h"
#include "uart_helper_fcns.h"
#include "data_storage.h"

// Some constants that are only used in the main function here
#define fact 10

// Move any of the following variables (needed across functions)
// to the data_storage files.
// Any variables which are only needed within one file should not be
// made global, though. See uart_helper_fcns.c for an example, and compare
// the transmit/received buffers with the control input array.
uint16 flag = 0;
uint16 count = 0;
uint16 pwm = 0;

uint16 prev_count = 0;
double dT = 0.001;

double Kp = 0.1;
double Ki = 0.5;
double Kd = 0.1;

double proportional = 0;
double integral = 0;
double derivative = 0;

double output = 0;
double CUR_ERROR = 0;
double LAST_ERROR = 0;
double INTEGRAL_ACUM = 0;
double TOTAL_TICKS = 200;

// Move all the following interrupt handlers to their own .c files. 
// Use uart_helper_fcns as a template.c and .h as a template.

//CY_ISR(timer_handler) { 
//    LED_Pin_Write( ! LED_Pin_Read() );
//    CUR_ERROR = TOTAL_TICKS - count;
//    proportional = CUR_ERROR * Kp;
//    integral = (CUR_ERROR * dT * Ki) + INTEGRAL_ACUM;
//    derivative = ((CUR_ERROR - LAST_ERROR) * Kd)/dT;
//    output = proportional + integral + derivative; 
//    
//    
//    pwm = PWM_ReadCompare();
//    char buf[6];
//    sprintf(buf,"%hu",pwm);
//    /* UART_PutString(buf); */
//    /* UART_PutString(" "); */
//   
//    LAST_ERROR = CUR_ERROR;
//    INTEGRAL_ACUM = integral; 
//    
//    if (CUR_ERROR <0) {
//        PWM_WriteCompare(1000);
//    }
//    
//    Timer_ReadStatusRegister();
//}

//CY_ISR(encoder_interrupt_handler) {
//    Pin_Encoder_ClearInterrupt();
//    /* UART_PutString(" Interrupt "); */ 
//    count++;
//    
//    char buf[6];
//    sprintf(buf,"%hu",count);
//    UART_PutString(buf);
//    UART_PutString(" ");
//}

//CY_ISR(uart_handler) {
//    uint8 rx = 0;
//    rx = UART_ReadRxData();
//    uint16 ch = 0;
//    ch = rx*100;
//    UART_PutChar(rx*fact);
//    PWM_WriteCompare(ch);
//
//    UART_PutString("UART ");
//}

int main(void) {
    
    // Enable interrupts for the chip
    CyGlobalIntEnable;
    
    // Start the interrupt handlers / service routines for each interrupt:
    // UART, main control loop, encoder counting.
    // These are found in the corresponding helper files (declarations in .h, implementations in .c)
    isr_UART_StartEx(Interrupt_Handler_UART_Receive);
    //isr_Timer_StartEx(timer_handler);
    //isr_Encoder_StartEx(encoder_interrupt_handler);
    
    // Start the components themselves.
    // PSM for motor output, timer for main control loop calc's, UART for serial comm.
//    PWM_Start();
//    Timer_Start();
    UART_Start();
    
    // A welcome message
    UART_PutString("\n2D Spine Controller Test.\n");
    UART_PutString("Copyright 2018 Berkeley Emergent Space Tensegrities Lab.\n");
    UART_PutString("Usage: send strings of the form (char) (optional_args). Currently supported:\n");
    UART_PutString("**NOTE: THESE MUST BE FOLLOWED EXACTLY, with exact spacing.\n\n");
    UART_PutString("e = enable PWM\n");
    UART_PutString("x = disable PWM\n");
    UART_PutString("q = query currently-stored control input\n");
    UART_PutString("s = hard stop. The Big Red Button. (hopefully.)\n");
    UART_PutString("u float float float float = assign control input\n\n");
    //Pin_High_Write(1);
    //Pin_Low_Write(0);
    for(;;)
    {       
        // nothing here. 
        // We're entirely interrupt-driven!
        
//        if (flag == 1)
//        {
//            /* flag = 0; */
//            /* count++; */ 
//            /* char buf[6]; */
//            /* sprintf(buf,"%hu",count); */
//            /* UART_PutString(buf); */
//        }
    }
}

/* [] END OF FILE */

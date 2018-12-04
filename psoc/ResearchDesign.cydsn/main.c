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

int8 controller_status = 0;
int8 motor_2 = 0;

int count_1 = 0;
int count_2 = 0;

// Move any of the following variables (needed across functions)
// to the data_storage files.
// Any variables which are only needed within one file should not be
// made global, though. See uart_helper_fcns.c for an example, and compare
// the transmit/received buffers with the control input array.

uint16 pwm_1 = 0;
uint16 pwm_2 = 0;

double Kp = 0.2;

double proportional_1 = 0;
double proportional_2 = 0;

int CUR_ERROR_1 = 0;
int CUR_ERROR_2 = 0;


uint16 TICKS_1 = 0;
uint16 TICKS_2 = 0;

int first_loop = 1;

CY_ISR(timer_handler) { 
    if (controller_status == 1) {
        
        CUR_ERROR_1 = TICKS_1 - count_1;
        proportional_1 = (double)CUR_ERROR_1 * Kp;
        
        CUR_ERROR_2 = TICKS_2 - count_2;
        proportional_2 = (double)CUR_ERROR_2 * Kp;
        
        if (first_loop == 1) {
            PWM_2_WriteCompare(300); 
            first_loop = 0; }
        else if (CUR_ERROR_2 < 25){
            PWM_2_WriteCompare(0); 
            motor_2 = 0; }
        else if (proportional_2 > 1000) { 
            PWM_2_WriteCompare(300); }
        else if (proportional_2 < 1000) {
            if (proportional_2 > 70) {
                PWM_2_WriteCompare((uint16)proportional_2); }
           else {
               PWM_2_WriteCompare(70); } }
        
        //if (motor_2 == 0) {
        //    controller_status = 0; }
    }
    
    Timer_ReadStatusRegister();
}
CY_ISR(encoder_interrupt_handler_1) {
    Pin_Encoder_1_ClearInterrupt();
    count_1++;
    
    char buf[6];
    sprintf(buf,"%d",count_1);
    UART_PutString(buf);
    UART_PutString("Encoder 1: ");
}
CY_ISR(encoder_interrupt_handler_2) {
    Pin_Encoder_2_ClearInterrupt();
    count_2++;
    
    char buf[6];
    sprintf(buf,"%d",count_2);
    UART_PutString(buf);
    UART_PutString("Encoder 2: ");
}


// CY_ISR(uart_handler) {
//     controller_status = 1;
//     first_loop = 1;
//     TICKS_2 = 500;
//     count_2 = 0;
// }
// Move all the following interrupt handlers to their own .c files. 
// Use uart_helper_fcns as a template.c and .h as a template.



int main(void) {
    
    // Enable interrupts for the chip
    CyGlobalIntEnable;

    isr_Encoder_1_StartEx(encoder_interrupt_handler_1);
    isr_Encoder_2_StartEx(encoder_interrupt_handler_2);
    
    // Start the interrupt handlers / service routines for each interrupt:
    // UART, main control loop, encoder counting.
    // These are found in the corresponding helper files (declarations in .h, implementations in .c)
    isr_UART_StartEx(Interrupt_Handler_UART_Receive);
    isr_Timer_StartEx(timer_handler);
        
    PWM_1_Start();
    PWM_1_WriteCompare(400);
    PWM_2_Start();
    PWM_2_WriteCompare(0);
    
    Timer_Start();
    UART_Start();
    
    // Print a welcome message. Comes from uart_helper_fcns.
    UART_Welcome_Message();
    
    Pin_High_1_Write(1);
    Pin_Low_1_Write(0);
    Pin_High_2_Write(1);
    Pin_Low_2_Write(0);
    
    for(;;)
    {    
        PWM_1_WriteCompare(700);
    }
}

/* [] END OF FILE */

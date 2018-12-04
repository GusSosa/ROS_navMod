/* ========================================
 *
 * 2D Spine Controller Test Embedded Software
 *
 * Copyright Berkeley Emergent Space Tensegrities Lab, 2018
 * (insert license later.)
 *
 * ========================================
*/
#define PWM_MAX 300
#define PWM_INIT 300
#define PWM_MIN 80

// Include both the UART helper functions and the header
// that has the global variables we need.
// note that both of these should have include guards in them already
// so it's safe to include them directly here.
#include <project.h>
#include <math.h>
#include "stdio.h"
#include "uart_helper_fcns.h"
#include "data_storage.h"

int8 motor_2 = 0;

int16 count_1 = 0;
int16 count_2 = 0;

int controller_status = 0;
int first_loop = 0;


// Move any of the following variables (needed across functions)
// to the data_storage files.
// Any variables which are only needed within one file should not be
// made global, though. See uart_helper_fcns.c for an example, and compare
// the transmit/received buffers with the control input array.

float Kp = 0.2;

float proportional_1 = 0;
float proportional_2 = 0;

float CUR_ERROR_1 = 0;
float CUR_ERROR_2 = 0;

_Bool pin2B = 0;

int first_loop_1 = 0;
int first_loop_2 = 0;
CY_ISR(timer_handler) { 
    if (controller_status == 1) {
        
        
        // MOTOR 1 
        float TICKS_1 = current_control[0];        
        CUR_ERROR_1 = TICKS_1 - count_1;
        // Determine direction of rotation
        if (CUR_ERROR_1 > 0) {
            Pin_High_1_Write(1);
            Pin_Low_1_Write(0);
        }
        else {
            Pin_High_1_Write(0);
            Pin_Low_1_Write(1);
        }
        // Calculate proportional control 1
        proportional_1 = fabs(CUR_ERROR_1) * Kp;
        // Set PWM
        if (first_loop_1 == 1) {
            if (fabs(CUR_ERROR_1) > 15) {
                PWM_1_WriteCompare(PWM_INIT);
                first_loop_1 = 0;
            }
        }
        else if (fabs(CUR_ERROR_1) < 25){
            PWM_1_WriteCompare(0); 
            motor_2 = 0; 
        }
        else if (proportional_1 > 1000) { 
            PWM_1_WriteCompare(PWM_MAX); 
        }
        else if (proportional_1 < 1000) {
            if (proportional_1 > PWM_MIN) {
                PWM_1_WriteCompare(proportional_1); 
            }
           else {
               PWM_1_WriteCompare(PWM_MIN); } 
        }
        
        // MOTOR 2 
        float TICKS_2 = current_control[1];
        CUR_ERROR_2 = TICKS_2 - count_2;
        // Determine direction of rotation
        if (CUR_ERROR_2 > 0) {
            Pin_High_2_Write(1);
            Pin_Low_2_Write(0);
        }
        else {
            Pin_High_2_Write(0);
            Pin_Low_2_Write(1);
        }
        // Calculate proportional control 2
        proportional_2 = fabs(CUR_ERROR_2) * Kp;
        
        
        // Set PWM 2
        if (first_loop_2 == 1) {
            if (fabs(CUR_ERROR_2) > 15) {
                PWM_2_WriteCompare(PWM_INIT);
                first_loop_2 = 0;
            }
        }
        else if (fabs(CUR_ERROR_2) < 25){
            PWM_2_WriteCompare(0); 
            motor_2 = 0; 
        }
        else if (proportional_2 > 1000) { 
            PWM_2_WriteCompare(PWM_MAX); 
        }
        else if (proportional_2 < 1000) {
            if (proportional_2 > PWM_MIN) {
                PWM_2_WriteCompare(proportional_2); 
            }
           else {
               PWM_2_WriteCompare(PWM_MIN); } 
        }
        
    }
    
    Timer_ReadStatusRegister();
}
CY_ISR(encoder_interrupt_handler_1A) {
    Pin_Encoder_1A_ClearInterrupt();
    
    if (Pin_High_1_Read() == 1 && Pin_Low_1_Read() == 0) {
        count_1++;
    }
    else {
        count_1--;
    }
    
    char buf[6];
    sprintf(buf,"%d",count_1);
    UART_PutString(buf);
    UART_PutString("E1: ");
}

CY_ISR(encoder_interrupt_handler_2A) {
    Pin_Encoder_2A_ClearInterrupt();
    
    if (Pin_High_2_Read() == 1 && Pin_Low_2_Read() == 0) {
        count_2++;
    }
    else {
        count_2--;
    }
    
    char buf[6];
    sprintf(buf,"%d",count_2);
    UART_PutString(buf);
    UART_PutString("E2: ");
}
int main(void) {
    
    // Enable interrupts for the chip
    CyGlobalIntEnable;

    isr_Encoder_1A_StartEx(encoder_interrupt_handler_1A);
    
    isr_Encoder_2A_StartEx(encoder_interrupt_handler_2A);
    
    // Start the interrupt handlers / service routines for each interrupt:
    // UART, main control loop, encoder counting.
    // These are found in the corresponding helper files (declarations in .h, implementations in .c)
    isr_UART_StartEx(Interrupt_Handler_UART_Receive);
    isr_Timer_StartEx(timer_handler);
        
    PWM_1_Start();
    PWM_1_WriteCompare(0);
    PWM_2_Start();
    PWM_2_WriteCompare(0);
    
    Timer_Start();
    UART_Start();
    
    // Print a welcome message. Comes from uart_helper_fcns.
    UART_Welcome_Message();
    
    for(;;)
    {    
    }
}

/* [] END OF FILE */

/* ========================================
 *
 * 2D Spine Controller Test Embedded Software
 *
 * Copyright Berkeley Emergent Space Tensegrities Lab, 2018
 * (insert license later.)
 *
 * ========================================
*/
#define PWM_MAX 1000
#define PWM_INIT 500
#define PWM_MIN 200

// Include both the UART helper functions and the header
// that has the global variables we need.
// note that both of these should have include guards in them already
// so it's safe to include them directly here.
#include <project.h>
#include <math.h>
#include "stdio.h"
#include "uart_helper_fcns.h"
#include "data_storage.h"


int16 count_1 = 0;
int16 count_2 = 0;
int16 count_3 = 0;
int16 count_4 = 0;

int controller_status = 0;
int tensioning = 0;
float tension_control; 

// Move any of the following variables (needed across functions)
// to the data_storage files.
// Any variables which are only needed within one file should not be
// made global, though. See uart_helper_fcns.c for an example, and compare
// the transmit/received buffers with the control input array.

float Kp = 0.2;

float proportional_1 = 0;
float proportional_2 = 0;
float proportional_3 = 0;
float proportional_4 = 0;

float CUR_ERROR_1 = 0;
float CUR_ERROR_2 = 0;
float CUR_ERROR_3 = 0;
float CUR_ERROR_4 = 0;

int first_loop_1 = 0;
int first_loop_2 = 0;
int first_loop_3 = 0;
int first_loop_4 = 0;


int motor_1 = 0;
int motor_2 = 0;
int motor_3 = 0;
int motor_4 = 0;

int print = 1;

CY_ISR(timer_handler) { 
    if (tensioning == 1) {
        // Write tensioning function 
        // positive equals a little more tension
        // negative equals a little less tension
        if (tension_control == 1) {
            controller_status = 1;
            current_control[0] = 20;
            first_loop_1 = 1;
            motor_1 = 1; 
        }
        else if (tension_control == -1) {
            controller_status = 1;
            current_control[0] = -20;
            first_loop_1 = 1;
            motor_1 = 1; 
        }
        else if (tension_control == 2) {
            controller_status = 1;
            current_control[1] = 20;
            first_loop_2 = 1;
            motor_2 = 1; 
        }
        else if (tension_control == -2) {
            controller_status = 1;
            current_control[1] = -20;
            first_loop_2 = 1;
            motor_2 = 1; 
        }
        else if (tension_control == 3) {
            controller_status = 1;
            current_control[2] = 20;
            first_loop_3 = 1;
            motor_3 = 1; 
        }
        else if (tension_control == -3) {
            controller_status = 1;
            current_control[2] = -20;
            first_loop_3 = 1;
            motor_3 = 1; 
        }
        else if (tension_control == 4) {
            controller_status = 1;
            current_control[3] = 20;
            first_loop_4 = 1;
            motor_4 = 1; 
        }
        else if (tension_control == -4) {
            controller_status = 1;
            current_control[3] = -20;
            first_loop_4 = 1;
            motor_4 = 1; 
        }
    }
    
    
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
        // Set PWM 1
        if (first_loop_1 == 1) {
            if (fabs(CUR_ERROR_1) > 15) {
                PWM_1_WriteCompare(PWM_INIT);
                first_loop_1 = 0;
            }
            else if (fabs(CUR_ERROR_1) <= 15) {
                motor_1 = 0;
            }            
        }
        else if (fabs(CUR_ERROR_1) < 25){
            PWM_1_WriteCompare(0); 
            motor_1 = 0;
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
            else if (fabs(CUR_ERROR_2) <= 15) {
                motor_2 = 0;
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
         // MOTOR 3 
        float TICKS_3 = current_control[2];
        CUR_ERROR_3 = TICKS_3 - count_3;
        // Determine direction of rotation
        if (CUR_ERROR_3 > 0) {
            Pin_High_3_Write(1);
            Pin_Low_3_Write(0);
        }
        else {
            Pin_High_3_Write(0);
            Pin_Low_3_Write(1);
        }
        // Calculate proportional control 3
        proportional_3 = fabs(CUR_ERROR_3) * Kp;

        // Set PWM 3
        if (first_loop_3 == 1) {
            if (fabs(CUR_ERROR_3) > 15) {
                PWM_3_WriteCompare(PWM_INIT);
                first_loop_3 = 0;
            }
            else if (fabs(CUR_ERROR_3) <= 15) {
                motor_3 = 0;
            }
        }
        else if (fabs(CUR_ERROR_3) < 25){
            PWM_3_WriteCompare(0);   
            motor_3 = 0;
        }
        else if (proportional_3 > 1000) { 
            PWM_3_WriteCompare(PWM_MAX); 
        }
        else if (proportional_3 < 1000) {
            if (proportional_3 > PWM_MIN) {
                PWM_3_WriteCompare(proportional_3); 
            }
           else {
               PWM_3_WriteCompare(PWM_MIN); } 
        }
        
        
        // MOTOR 4 
        float TICKS_4 = current_control[3];
        CUR_ERROR_4 = TICKS_4 - count_4;
        // Determine direction of rotation
        if (CUR_ERROR_4 > 0) {
            Pin_High_4_Write(1);
            Pin_Low_4_Write(0);
        }
        else {
            Pin_High_4_Write(0);
            Pin_Low_4_Write(1);
        }
        // Calculate proportional control 4
        proportional_4 = fabs(CUR_ERROR_4) * Kp;

        // Set PWM 4
        if (first_loop_4 == 1) {
            if (fabs(CUR_ERROR_4) > 15) {
                PWM_4_WriteCompare(PWM_INIT);
                first_loop_4 = 0;
            }
            else if (fabs(CUR_ERROR_4) <= 15) {
                motor_4 = 0;
            }
        }
        else if (fabs(CUR_ERROR_4) < 25){
            PWM_4_WriteCompare(0);    
            motor_4 = 0;
        }
        else if (proportional_4 > 1000) { 
            PWM_4_WriteCompare(PWM_MAX); 
        }
        else if (proportional_4 < 1000) {
            if (proportional_4 > PWM_MIN) {
                PWM_4_WriteCompare(proportional_4); 
            }
           else {
               PWM_4_WriteCompare(PWM_MIN); } 
        }
        
//    if ((motor_1==0) && (motor_2==0) && (motor_3==0) && (motor_4==0)) {
//        if (print == 1) {
//            print = 0;
//            char buf1[6];
//            UART_PutString("  E1: ");
//            sprintf(buf1,"%d",count_1);
//            UART_PutString(buf1);
//
//            char buf2[6];
//            UART_PutString("  E2: ");
//            sprintf(buf2,"%d",count_2);
//            UART_PutString(buf2);
//
//            char buf3[6];
//            UART_PutString("  E3: ");
//            sprintf(buf3,"%d",count_3);
//            UART_PutString(buf3);
//
//            char buf4[6];
//            UART_PutString("  E4: ");
//            sprintf(buf4,"%d",count_4);
//            UART_PutString(buf4);
//           
//        }
//    }       
        if (tensioning == 1) {
            count_1 = 0;
            count_2 = 0;
            count_3 = 0;
            count_4 = 0;
            tensioning = 0; 
        }
    }

    
    
    Timer_ReadStatusRegister();
}
CY_ISR(encoder_interrupt_handler_1) {
    Pin_Encoder_1_ClearInterrupt();
    
    if (Pin_High_1_Read() == 1 && Pin_Low_1_Read() == 0) {
        count_1++;
    }
    else {
        count_1--;
    }
    
//    char buf[6];
//    sprintf(buf,"%d",count_1);
//    UART_PutString(buf);
//    UART_PutString("E1: ");
}

CY_ISR(encoder_interrupt_handler_2) {
    Pin_Encoder_2_ClearInterrupt();
    
    if (Pin_High_2_Read() == 1 && Pin_Low_2_Read() == 0) {
        count_2++;
    }
    else {
        count_2--;
    }
    
//    char buf[6];
//    sprintf(buf,"%d",count_2);
//    UART_PutString(buf);
//    UART_PutString("E2: ");
}
CY_ISR(encoder_interrupt_handler_3) {
    Pin_Encoder_3_ClearInterrupt();
    
    if (Pin_High_3_Read() == 1 && Pin_Low_3_Read() == 0) {
        count_3++;
    }
    else {
        count_3--;
    }
    
//    char buf[6];
//    sprintf(buf,"%d",count_3);
//    UART_PutString(buf);
//    UART_PutString("E3: ");
}
CY_ISR(encoder_interrupt_handler_4) {
    Pin_Encoder_4_ClearInterrupt();
    
    if (Pin_High_4_Read() == 1 && Pin_Low_4_Read() == 0) {
        count_4++;
    }
    else {
        count_4--;
    }
    
//    char buf[6];
//    sprintf(buf,"%d",count_4);
//    UART_PutString(buf);
//    UART_PutString("E4: ");
}
int main(void) {
    
    // Enable interrupts for the chip
    CyGlobalIntEnable;

    isr_Encoder_1_StartEx(encoder_interrupt_handler_1);
    isr_Encoder_2_StartEx(encoder_interrupt_handler_2);
    isr_Encoder_3_StartEx(encoder_interrupt_handler_3);
    isr_Encoder_4_StartEx(encoder_interrupt_handler_4);
    
    // Start the interrupt handlers / service routines for each interrupt:
    // UART, main control loop, encoder counting.
    // These are found in the corresponding helper files (declarations in .h, implementations in .c)
    isr_UART_StartEx(Interrupt_Handler_UART_Receive);
    isr_Timer_StartEx(timer_handler);
        
    PWM_1_Start();
    PWM_1_WriteCompare(0);
    PWM_2_Start();
    PWM_2_WriteCompare(0);
    PWM_3_Start();
    PWM_3_WriteCompare(0);    
    PWM_4_Start();
    PWM_4_WriteCompare(0);
    
    Timer_Start();
    UART_Start();
    
    // Print a welcome message. Comes from uart_helper_fcns.
    UART_Welcome_Message();
    
    for(;;)
    {    
    }
}

/* [] END OF FILE */

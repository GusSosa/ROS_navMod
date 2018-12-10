/* ========================================
 *
 * 2D Spine Controller Test Embedded Software
 *
 * Copyright Berkeley Emergent Space Tensegrities Lab, 2018
 * (insert license later.)
 *
 * ========================================
*/
// between 300 and 1000
#define PWM_MAX 500
// initial should be around 300
#define PWM_INIT 400
// min of 70 if at 12V 
#define PWM_MIN 90

#define TICKS_MIN 10
#define TICKS_STOP 50

// For the motors that are using the quadrature decoder component, the above
// should be multiplied by a factor of 4 (since it's 4 times as precise as our counting.)
#define TICKS_MIN_QD 40
#define TICKS_STOP_QD 200

// Include both the UART helper functions and the header
// that has the global variables we need.
// note that both of these should have include guards in them already
// so it's safe to include them directly here.
#include <project.h>
#include <math.h>
#include <stdlib.h>
#include "stdio.h"
#include "uart_helper_fcns.h"
#include "data_storage.h"

// For transmitting strings with other variables substituted in,
// (note: re-using variable names since out-of-scope of uart_helper_fcns.)
#define TRANSMIT_LENGTH 128
char transmit_buffer[TRANSMIT_LENGTH];

// EXTERNS FROM DATA_STORAGE.H
// now moved to data_storage.c
//int count_1 = 0;
//int count_2 = 0;
//int count_3 = 0;
//int count_4 = 0;
//
//int first_loop_1 = 0;
//int first_loop_2 = 0;
//int first_loop_3 = 0;
//int first_loop_4 = 0;
//
//int motor_1 = 0;
//int motor_2 = 0;
//int motor_3 = 0;
//int motor_4 = 0;
//
//int controller_status = 0;
//int tensioning = 0;
//float tension_control; 
//int print = 1;

// Move any of the following variables (needed across functions)
// to the data_storage files.
// Any variables which are only needed within one file should not be
// made global, though. See uart_helper_fcns.c for an example, and compare
// the transmit/received buffers with the control input array.

// constants of proportionality are integers.
// Manual counting gives about 1/4 the resultion of the quaddec component.
int16 Kp_man = 25;
// when using the quadrature decoder, seems we need a Kp smaller than 1.
float Kp_qd = 0.5;
//float Kp = 25;

// integer multiplication (error * Kp) is always an integer,
// so these are now ints also.
//float proportional_1 = 0;
//float proportional_2 = 0;
//float proportional_3 = 0;
//float proportional_4 = 0;

int16 proportional_1 = 0;
//int16 proportional_2 = 0;
float proportional_2 = 0.0;
int16 proportional_3 = 0;
int16 proportional_4 = 0;

// these moved to global variables now.
//float CUR_ERROR_1 = 0;
float CUR_ERROR_2 = 0;
float CUR_ERROR_3 = 0;
float CUR_ERROR_4 = 0;

int direction_1 = 1;

void move_motor_1() {
    // MOTOR 1 
    //float TICKS_1 = current_control[0];
    // Replacing with global int16's
   
    //CUR_ERROR_1 = TICKS_1 - count_1;
    error[0] = current_control[0] - count_1;
    
    // debugging
    //sprintf(transmit_buffer, "Motor 1 manual calc of encoder val: %i\r\n", count_1);
    //UART_PutString(transmit_buffer);
    //sprintf(transmit_buffer, "Motor 1 quadrature hardware readout: %i, status: %i\r\n", QuadDec_Motor1_GetCounter(), QuadDec_Motor1_GetEvents());
    //UART_PutString(transmit_buffer);
    
    // Determine direction of rotation
    if (error[0] > 0) {
        Pin_High_1_Write(1);
        Pin_Low_1_Write(0);
        direction_1 = 1;
        //UART_PutString("Motor 1, forward");
    }
    else {
        Pin_High_1_Write(0);
        Pin_Low_1_Write(1);
        direction_1 = 0;
        //UART_PutString("Motor 1, backward");
    }
    
    // Calculate proportional control 1
    // don't need the absolute values anymore!
    //proportional_1 = fabs(error[0]) * Kp;
    proportional_1 = abs(error[0]) * Kp_man;
   
    
    // Set PWM 1
    if (first_loop_1 == 1) {
        if (abs(error[0]) >= TICKS_MIN) {
            PWM_1_WriteCompare(PWM_INIT);
            first_loop_1 = 0;
        }
//            else if (abs(error[0]) <= 15) {
//                motor_1 = 0;
//            }            
    }
    else if (abs(error[0]) < TICKS_STOP){
        PWM_1_WriteCompare(0); 
        motor_1 = 0;
    }
    else if (proportional_1 > 1000) { 
        PWM_1_WriteCompare(PWM_MAX); 
    }
    else if (proportional_1 < 1000) {
        if (proportional_1 > PWM_MIN) {
            // we still need the abs() here, since proportional input is +/-
            // whereas PWM compare value is always > 0.
            PWM_1_WriteCompare(abs(proportional_1)); 
        }
       else {
           PWM_1_WriteCompare(PWM_MIN); } 
    }
}

void move_motor_2() {
    // MOTOR 2 
    //float TICKS_2 = current_control[1];
    
    //CUR_ERROR_2 = TICKS_2 - count_2;
    //error[1] = current_control[1] - count_2;
    error[1] = current_control[1] - QuadDec_Motor2_GetCounter();
    
    // debugging
//    sprintf(transmit_buffer, "Motor 2 manual calc of encoder val: %i\r\n", count_2);
//    UART_PutString(transmit_buffer);
//    sprintf(transmit_buffer, "Motor 2 quadrature hardware readout: %i, status: %i\r\n", QuadDec_Motor2_GetCounter(), QuadDec_Motor2_GetEvents());
//    UART_PutString(transmit_buffer);
//    sprintf(transmit_buffer, "Error signal for motor 2 (quadrature): %i,\r\n", error[1]);
//    UART_PutString(transmit_buffer);
    
    
    // Determine direction of rotation
    if (error[1] > 0) {
        Pin_High_2_Write(1);
        Pin_Low_2_Write(0);
    }
    else {
        Pin_High_2_Write(0);
        Pin_Low_2_Write(1);
    }
    
    // Calculate proportional control 2
    proportional_2 = abs(error[1]) * Kp_qd;
    
    // Set PWM 2
    // Changed to new limits for quaddec use
    if (first_loop_2 == 1) {
        if (abs(error[1]) >= TICKS_MIN_QD) {
            PWM_2_WriteCompare(PWM_INIT);
            first_loop_2 = 0;
        }
//            else if (fabs(error[1]) <= 15) {
//                motor_2 = 0;
//            }            
    }
    else if (abs(error[1]) < TICKS_STOP_QD){
        PWM_2_WriteCompare(0);    
        motor_2 = 0;
    }
    else if (proportional_2 > 1000) { 
        PWM_2_WriteCompare(PWM_MAX); 
    }
    else if (proportional_2 < 1000) {
        if (proportional_2 > PWM_MIN) {
            PWM_2_WriteCompare(fabs(proportional_2)); 
        }
       else {
           PWM_2_WriteCompare(PWM_MIN); } 
    }    
}
void move_motor_3() {
     // MOTOR 3 
    //float TICKS_3 = current_control[2];
    error[2] = current_control[2] - count_3;
    
    //CUR_ERROR_3 = TICKS_3 - count_3;
    
    // Determine direction of rotation
    if (error[2] > 0) {
        Pin_High_3_Write(1);
        Pin_Low_3_Write(0);
    }
    else {
        Pin_High_3_Write(0);
        Pin_Low_3_Write(1);
    }
    
    // Calculate proportional control 3
    proportional_3 = abs(error[2]) * Kp_man;

    // Set PWM 3
    if (first_loop_3 == 1) {
        if (abs(error[2]) >= TICKS_MIN) {
            PWM_3_WriteCompare(PWM_INIT);
            first_loop_3 = 0;
        }
//            else if (fabs(error[2]) <= 15) {
//                motor_3 = 0;
//            }
    }
    else if (abs(error[2]) < TICKS_STOP){
        PWM_3_WriteCompare(0);   
        motor_3 = 0;
    }
    else if (proportional_3 > 1000) { 
        PWM_3_WriteCompare(PWM_MAX); 
    }
    else if (proportional_3 < 1000) {
        if (proportional_3 > PWM_MIN) {
            PWM_3_WriteCompare(abs(proportional_3)); 
        }
       else {
           PWM_3_WriteCompare(PWM_MIN); } 
    }    
}

void move_motor_4() {
    // MOTOR 4 
    //float TICKS_4 = current_control[3];
    error[3] = current_control[3] - count_4;
    
    //CUR_ERROR_4 = TICKS_4 - count_4;
    
    // Determine direction of rotation
    if (error[3] > 0) {
        Pin_High_4_Write(1);
        Pin_Low_4_Write(0);
    }
    else {
        Pin_High_4_Write(0);
        Pin_Low_4_Write(1);
    }
    
    // Calculate proportional control 4
    proportional_4 = abs(error[3]) * Kp_man;

    // Set PWM 4
    if (first_loop_4 == 1) {
        if (abs(error[3]) > TICKS_MIN) {
            PWM_4_WriteCompare(PWM_INIT);
            first_loop_4 = 0;
        }
//            else if (fabs(error[3]) <= 15) {
//                motor_4 = 0;
//            }
    }
    else if (abs(error[3]) < TICKS_STOP){
        PWM_4_WriteCompare(0);    
        motor_4 = 0;
    }
    else if (proportional_4 > 1000) { 
        PWM_4_WriteCompare(PWM_MAX); 
    }
    else if (proportional_4 < 1000) {
        if (proportional_4 > PWM_MIN) {
            PWM_4_WriteCompare(abs(proportional_4)); 
        }
       else {
           PWM_4_WriteCompare(PWM_MIN); } 
    }    
}

CY_ISR(timer_handler) { 
    if (tensioning == 1) {
        if (fabs(tension_control) == 1) {
            move_motor_1();
        }
        else if (fabs(tension_control) == 2) {
            move_motor_2();
        }
        else if (fabs(tension_control) == 3) {
            move_motor_3();
        }
        else if (fabs(tension_control) == 4) {
            move_motor_4();
        }
    }
      
    if (controller_status == 1) {
        move_motor_1();
        move_motor_2();
        move_motor_3();
        move_motor_4();
        
    }    
    Timer_ReadStatusRegister();
}


CY_ISR(encoder_interrupt_handler_1) {
    Pin_Encoder_1_ClearInterrupt();
    
    if (Pin_High_1_Read() == 1 && Pin_Low_1_Read() == 0) {
    count_1++;
    }
    else if (direction_1 == 0) {
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
    __enable_irq();
    
    isr_Encoder_1_StartEx(encoder_interrupt_handler_1);
    //isr_Encoder_2_StartEx(encoder_interrupt_handler_2);
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
    
    // For the quadrature (encoder) hardware components
    QuadDec_Motor1_Start();
    QuadDec_Motor2_Start();
    //QuadDec_Motor3_Start();
    //QuadDec_Motor4_Start();
    
    // Print a welcome message. Comes from uart_helper_fcns.
    UART_Welcome_Message();
    
    for(;;)
    {    
    }
}

/* [] END OF FILE */

/* ========================================
 *
 * 2D Spine Controller Test Embedded Software
 *
 * Copyright Berkeley Emergent Space Tensegrities Lab, 2018
 * (insert license later.)
 *
 * ========================================
*/

// Constants and defines for the motors.
// On 2018-12-10, changed the PWM frequency so the output is scaled.
// Now, outputs are up to 10,000, so everything is times 10.
// This is to get higher resolution for (e.g.) the integral control,
// so our control constants don't have to be as small / overflow-y.
// between 3000 and 1000. Works-ish with 5000.
#define PWM_MAX 3000
// initial should be around 4000
#define PWM_INIT 400
// min of 90 if at 12V 
#define PWM_MIN 400

#define TICKS_MIN 10
#define TICKS_STOP 50

// For the motors that are using the quadrature decoder component, the above
// should be multiplied by a factor of 4 (since it's 4 times as precise as our counting.)
#define TICKS_MIN_QD 40
#define TICKS_STOP_QD 50

// For transmitting strings with other variables substituted in,
// (note: re-using variable names since out-of-scope of uart_helper_fcns.)
#define TRANSMIT_LENGTH 128

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

// for send some debugging messages
char transmit_buffer[TRANSMIT_LENGTH];

// constants of proportionality are integers.
// Manual counting gives about 1/4 the resultion of the quaddec component.
int16 Kp_man = 25;
// when using the quadrature decoder, seems we need a Kp smaller than 1.
float Kp_qd = 10;
float Ki_qd = 1;
int Kd_qd = 20;

int16 proportional_1 = 0;
int16 proportional_3 = 0;
int16 proportional_4 = 0;

// A set of local variables for the calculated PWM signals.
// These are *signed*, so cannot be directly used with WriteCompare.
// Gotta set the direction pins, then abs() this.
static int16 pwm_controls[NUM_MOTORS] = {0, 0, 0, 0};

void move_motor_1() {
    // MOTOR 1 
   
    // Proportional term
    error[0] = current_control[0] - QuadDec_Motor1_GetCounter();
    
    // Integral term: discretized integration = addition (scaled.)
    // Note that we have to prevent integer overflow here.
    if((integral_error[0] + error[0] >= INT16_LOWERBOUND) && (integral_error[0] + error[0] <= INT16_UPPERBOUND)){
        integral_error[0] += error[0];
    }
    
    // Derivative term. discretized derivative = subtraction.
    deriv_error[0] = error[0] - prev_error[0];
    
    // Calculate the control input.
    // This automatically casts the integral control input to an int from a float.
    pwm_controls[0] = error[0] * Kp_qd + integral_error[0] * Ki_qd + deriv_error[0] * Kd_qd;
    // store an absolute-value version for actual application.
    int16 pwm_control_0_abs = abs(pwm_controls[0]);
    
    // Determine direction of rotation
    if (pwm_controls[0] > 0) {
        Pin_High_1_Write(1);
        Pin_Low_1_Write(0);
    }
    else {
        Pin_High_1_Write(0);
        Pin_Low_1_Write(1);
    }   
    
    // Apply the PWM value. Four options:
    // 1) If we're within tolerance of the target, turn off the PWM.
    // 2) If not within tolerance, lower bound with PWM_MIN.
    // 3) If not within tolerance and input less than max, apply the calculated input.
    // 4) If not within tolerance, upper bound with PWM_MAX.
    
    // 1) Is absolute encoder value within tolerance?
    if (abs(error[0]) < TICKS_STOP_QD){
        PWM_1_WriteCompare(0);    
        motor_1 = 0;
        // minor hack for now:
        // reset the integral terms, so this is a "stopping point"
        integral_error[0] = 0;
    }
    // Otherwise, do 2-4.
    else {
        // 4) Check if upper bounded.
        if (pwm_control_0_abs > PWM_MAX) { 
            PWM_1_WriteCompare(PWM_MAX); 
        }
        // 2) Check if lower bounded.
        else if (pwm_control_0_abs < PWM_MIN) {
            PWM_1_WriteCompare(PWM_MIN); 
        }
        // 3) otherwise, we know we're within the min to max.
        else {
             // This, right here, is the actual application of our control signal.
            PWM_1_WriteCompare(pwm_control_0_abs); 
        }
    }
    // Finally, set the stored value for the next iteration's error term.
    // It's safest to do this all the way at the end.
    prev_error[0] = error[0];
}

//        sprintf(transmit_buffer, "Writing max to PWM1.\r\n");
//        UART_PutString(transmit_buffer);
//           sprintf(transmit_buffer, "Writing min to PWM1\r\n");
//           UART_PutString(transmit_buffer);
//            sprintf(transmit_buffer, "Writing %i to PWM1\r\n", pwm_control_0_abs);
//            UART_PutString(transmit_buffer);

//    else if (pwm_control_0_abs < PWM_MAX) {
//        if (pwm_control_0_abs > PWM_MIN) {
//            // This, right here, is the actual application of our control signal.
//            PWM_1_WriteCompare(pwm_control_0_abs); 
//        }
//       else {
//           PWM_1_WriteCompare(PWM_MIN); 
//        } 
//    }    


void move_motor_2() {
    // MOTOR 2 

    // Proportional term
    error[1] = current_control[1] - QuadDec_Motor2_GetCounter();
    
    // Integral term: discretized integration = addition to sum (scaled.)
    // Note that we have to prevent buffer overflow here.
    if((integral_error[1] + error[1] >= INT16_LOWERBOUND) && (integral_error[1] + error[1] <= INT16_UPPERBOUND)){
        integral_error[1] += error[1];
    }
    
    // Derivative term. discretized derivative = subtraction.
    deriv_error[1] = error[1] - prev_error[1];
    
    // Calculate the control input.
    // This automatically casts the integral control input to an int from a float.
    pwm_controls[1] = error[1] * Kp_qd + integral_error[1] * Ki_qd + deriv_error[1] * Kd_qd;
    // store an absolute-value version for actual application.
    int16 pwm_control_1_abs = abs(pwm_controls[1]);
    
    // Determine direction of rotation
    if (pwm_controls[1] > 0) {
        Pin_High_2_Write(1);
        Pin_Low_2_Write(0);
    }
    else {
        Pin_High_2_Write(0);
        Pin_Low_2_Write(1);
    }

    // Apply the PWM value. Some checking to see if within tolerance for stopping:
    if (abs(error[1]) < TICKS_STOP_QD){
        PWM_2_WriteCompare(0);    
        motor_2 = 0;
        // minor hack for now:
        // reset the integral terms, so this is a "stopping point"
        integral_error[1] = 0;
    }
    else if (pwm_control_1_abs > PWM_MAX) { 
        PWM_2_WriteCompare(PWM_MAX); 
    }
    else if (pwm_control_1_abs < PWM_MAX) {
        if (pwm_control_1_abs > PWM_MIN) {
            // This, right here, is the actual application of our control signal.
            PWM_2_WriteCompare(pwm_control_1_abs); 
        }
       else {
           PWM_2_WriteCompare(PWM_MIN); } 
    }    
    // Finally, set the stored value for the next iteration's error term.
    // It's safest to do this all the way at the end.
    prev_error[1] = error[1];
}

void move_motor_3() {
     // MOTOR 3 
    error[2] = current_control[2] - count_3;
    
    // Integral term: discretized integration = addition (scaled.)
    // Note that we have to prevent buffer overflow here.
    if((integral_error[2] + error[2] >= INT16_LOWERBOUND) && (integral_error[2] + error[2] <= INT16_UPPERBOUND)){
        integral_error[2] += error[2];
    }
    
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
    }
    else if (abs(error[2]) < TICKS_STOP){
        PWM_3_WriteCompare(0);   
        motor_3 = 0;
    }
    else if (proportional_3 > PWM_MAX) { 
        PWM_3_WriteCompare(PWM_MAX); 
    }
    else if (proportional_3 < PWM_MAX) {
        if (proportional_3 > PWM_MIN) {
            PWM_3_WriteCompare(abs(proportional_3)); 
        }
       else {
           PWM_3_WriteCompare(PWM_MIN); } 
    }    
}

void move_motor_4() {
    // MOTOR 4 
    error[3] = current_control[3] - count_4;
    
    // Integral term: discretized integration = addition (scaled.)
    // Note that we have to prevent buffer overflow here.
    if((integral_error[3] + error[3] >= INT16_LOWERBOUND) && (integral_error[3] + error[3] <= INT16_UPPERBOUND)){
        integral_error[3] += error[3];
    }
    
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
    }
    else if (abs(error[3]) < TICKS_STOP){
        PWM_4_WriteCompare(0);    
        motor_4 = 0;
    }
    else if (proportional_4 > PWM_MAX) { 
        PWM_4_WriteCompare(PWM_MAX); 
    }
    else if (proportional_4 < PWM_MAX) {
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


//CY_ISR(encoder_interrupt_handler_1) {
//    Pin_Encoder_1_ClearInterrupt();
//    
//    if (Pin_High_1_Read() == 1 && Pin_Low_1_Read() == 0) {
//    count_1++;
//    }
//    else {
//        count_1--;
//    }
//}

//CY_ISR(encoder_interrupt_handler_2) {
//    Pin_Encoder_2_ClearInterrupt();
//    
//    if (Pin_High_2_Read() == 1 && Pin_Low_2_Read() == 0) {
//        count_2++;
//    }
//    else {
//        count_2--;
//    }
//}

CY_ISR(encoder_interrupt_handler_3) {
    Pin_Encoder_3_ClearInterrupt();
    
    if (Pin_High_3_Read() == 1 && Pin_Low_3_Read() == 0) {
        count_3++;
    }
    else {
        count_3--;
    }
}
CY_ISR(encoder_interrupt_handler_4) {
    Pin_Encoder_4_ClearInterrupt();
    
    if (Pin_High_4_Read() == 1 && Pin_Low_4_Read() == 0) {
        count_4++;
    }
    else {
        count_4--;
    }
}
int main(void) {
    
    // Enable interrupts for the chip
    CyGlobalIntEnable;
    __enable_irq();
    
    //isr_Encoder_1_StartEx(encoder_interrupt_handler_1);
    //isr_Encoder_2_StartEx(encoder_interrupt_handler_2);
    isr_Encoder_3_StartEx(encoder_interrupt_handler_3);
    isr_Encoder_4_StartEx(encoder_interrupt_handler_4);
    
    // Start the interrupt handlers / service routines for each interrupt:
    // UART, main control loop, encoder counting.
    // These are found in the corresponding helper files (declarations in .h, implementations in .c)
    isr_UART_StartEx(Interrupt_Handler_UART_Receive);
    isr_Timer_StartEx(timer_handler);
    
    // For the quadrature (encoder) hardware components
    QuadDec_Motor1_Start();
    QuadDec_Motor2_Start();
    QuadDec_Motor3_Start();
    QuadDec_Motor4_Start();
        
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
        // Nothing to do. Entirely interrupt driven! Hooray!
    }
}

/* [] END OF FILE */

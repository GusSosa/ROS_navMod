/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include <stdio.h>

int8 controller_status = 0;
int8 motor_2 = 0;

int count_1 = 0;
int count_2 = 0;

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


CY_ISR(uart_handler) {
    controller_status = 1;
    first_loop = 1;
    TICKS_2 = 500;
    count_2 = 0;
}

int main(void)
    {
    
    CyGlobalIntEnable;
    isr_Encoder_1_StartEx(encoder_interrupt_handler_1);
    isr_Encoder_2_StartEx(encoder_interrupt_handler_2);
    
    isr_Timer_StartEx(timer_handler);
    
    isr_UART_StartEx(uart_handler);
    
    PWM_1_Start();
    PWM_1_WriteCompare(400);
    PWM_2_Start();
    PWM_2_WriteCompare(0);
    
    Timer_Start();
    UART_Start();
    
    UART_PutString("COM Port Open");
    
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

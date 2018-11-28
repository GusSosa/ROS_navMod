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

#define fact 10

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



CY_ISR(timer_handler) { 
    LED_Pin_Write( ! LED_Pin_Read() );
    CUR_ERROR = TOTAL_TICKS - count;
    proportional = CUR_ERROR * Kp;
    integral = (CUR_ERROR * dT * Ki) + INTEGRAL_ACUM;
    derivative = ((CUR_ERROR - LAST_ERROR) * Kd)/dT;
    output = proportional + integral + derivative; 
    
    
    pwm = PWM_ReadCompare();
    char buf[6];
    sprintf(buf,"%hu",pwm);
    /* UART_PutString(buf); */
    /* UART_PutString(" "); */
   
    LAST_ERROR = CUR_ERROR;
    INTEGRAL_ACUM = integral; 
    
    if (CUR_ERROR <0) {
        PWM_WriteCompare(1000);
    }
    
    Timer_ReadStatusRegister();
}

CY_ISR(encoder_interrupt_handler) {
    Pin_Encoder_ClearInterrupt();
    /* UART_PutString(" Interrupt "); */ 
    count++;
    
    char buf[6];
    sprintf(buf,"%hu",count);
    UART_PutString(buf);
    UART_PutString(" ");
}

CY_ISR(uart_handler) {
    uint8 rx = 0;
    rx = UART_ReadRxData();
    uint16 ch = 0;
    ch = rx*100;
    UART_PutChar(rx*fact);
    PWM_WriteCompare(ch);

    UART_PutString("UART ");
}

int main(void)
    {
    
    CyGlobalIntEnable;
    isr_Encoder_StartEx(encoder_interrupt_handler);
    
    isr_Timer_StartEx(timer_handler);
    
    isr_UART_StartEx(uart_handler);
    
    PWM_Start();
    Timer_Start();
    UART_Start();
    
    /* UART_PutString("COM Port Open"); */
    Pin_High_Write(1);
    Pin_Low_Write(0);
    for(;;)
    {       
        
        
        if (flag == 1)
        {
            /* flag = 0; */
            /* count++; */ 
            /* char buf[6]; */
            /* sprintf(buf,"%hu",count); */
            /* UART_PutString(buf); */
        }
    }
}

/* [] END OF FILE */

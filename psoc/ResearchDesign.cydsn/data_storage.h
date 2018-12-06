/* ========================================
 *
 * Data Storage / global variables for 2D Spine Control Test
 * Includes control commands, records of state, etc.
 * Header file (declarations only)
 *
 * Copyright Berkeley Emergent Space Tensegrities Lab, 2018
 * (insert license later.)
 *
 * ========================================
*/

// Here, we just declare the variables that need to be seen across files.
// Needs a correspoding c file to define these variables
// (we could do so here but that's bad practice.)
#ifndef DATA_STORAGE_H
#define DATA_STORAGE_H
    
// For the current control input,
// we need an array of 4 floating point numbers
#define NUM_MOTORS 4
    
extern float current_control[NUM_MOTORS];
extern float tension_control;

extern int controller_status;
extern int tensioning;


extern int first_loop_1;
extern int first_loop_2;
extern int first_loop_3;
extern int first_loop_4;
extern int motor_1;
extern int motor_2;
extern int motor_3;
extern int motor_4;
extern int print;

#endif // DATA_STORAGE_H

/* [] END OF FILE */

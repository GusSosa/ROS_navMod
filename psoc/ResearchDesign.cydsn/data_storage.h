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
    
// some dimensional parameters for use in various places
// the radius of the spool, and pi as a constant.
#define RADIUS 1.063087
#define PI 3.1415

// For the small-amount-change tensioning commant 't',
// specify a number of encoder ticks to increment.
// Need different ones for the manually-calc'd ticks vs. quaddec's.
#define T_TICKS_MAN 30
#define T_TICKS_QD 120
    
// including the Cypress project file here so that we have access
// to the int16 data type.
#include <project.h>

// For the controller: let's store what was originally set, as a float,
// but then really do all the math in terms of encoder ticks, as ints.
//extern float current_control[NUM_MOTORS];
extern int16 current_control[NUM_MOTORS];
extern float control_in_cm[NUM_MOTORS];
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

extern int count_1;
extern int count_2;
extern int count_3;
extern int count_4;

extern int print;

// For calculating control inputs, store
// some variable related to the total error in encoder ticks.
// used for prop, int, and deriv terms.
extern int16 error[NUM_MOTORS];
extern int16 integral_error[NUM_MOTORS];
extern int16 deriv_error[NUM_MOTORS];

#endif // DATA_STORAGE_H

/* [] END OF FILE */

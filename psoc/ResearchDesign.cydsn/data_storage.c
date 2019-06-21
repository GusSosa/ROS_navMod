/* ========================================
 *
 * Data Storage / global variables for 2D Spine Control Test
 * Includes control commands, records of state, etc.
 * Implementation (defines the variables.)
 *
 * Copyright Berkeley Emergent Space Tensegrities Lab, 2018
 * (insert license later.)
 *
 * ========================================
*/

// This file simply defines the variables that are present in the
// corresponding H file.
// Include the declarations
#include "data_storage.h"

// and then define the corresponding variables.
// Although this looks wrong, it's actually how you have to do it:
// one with "extern" which only declares, and one without that actually defines.
//float current_control[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0};
int32 current_control[NUM_MOTORS] = {0, 0, 0, 0};
float control_in_cm[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0};

// Kim's work:
float tension_control = 0.0;

int controller_status = 0;
int tensioning = 0;

// These flags are used to break static friction on motors.
// At startup, motors aren't moving, so yes, first loop of application of control.
int first_loop_1 = 1;
int first_loop_2 = 1;
int first_loop_3 = 1;
int first_loop_4 = 1;

int motor_1 = 0;
int motor_2 = 0;
int motor_3 = 0;
int motor_4 = 0;

int count_2 = 0;
int count_3 = 0;
int count_4 = 0;

int print = 1;

// For calculating control inputs, store
// some variable related to the total error in encoder ticks.
// used for prop, int, and deriv terms.
int32 error[NUM_MOTORS] = {0, 0, 0, 0};
int32 integral_error[NUM_MOTORS] = {0, 0, 0, 0};
int32 prev_error[NUM_MOTORS] = {0, 0, 0, 0};
int32 deriv_error[NUM_MOTORS] = {0, 0, 0, 0};


/* [] END OF FILE */

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
float current_control[NUM_MOTORS];

/* [] END OF FILE */

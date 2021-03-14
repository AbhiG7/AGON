#include "mission_constants.hh"
#include "moding.hh"

bool change_mode_coundown(Mode current_mode, bool cond)
{
    bool ret = false;
    if (current_mode == STARTUP_STABLE && cond)
    {
        ret = true;
    }
    return ret;
}

bool change_mode_to_final_countdown(Mode current_mode, bool cond)
{
    bool ret = false;
    if (current_mode == COUNTDOWN && cond)
    {
        ret = true;
    }
    return ret;
}

bool change_mode_to_prep_tvc(Mode current_mode, bool cond)
{
    bool ret = false;
    if (current_mode == FINAL_COUNTDOWN && cond)
    {
        ret = true;
    }
    return ret;
}

/* change_mode_to_burn_baby_burn
 * 
 * Determines whether to change the mode to BURN_BABY_BURN.
 * 
 * Conditions:
 *   - current mode is NAVIGATION
 *   - TODO: determine any remaining conditions
 */
bool change_mode_to_burn_baby_burn(Mode current_mode, bool cond)
{
    bool ret = false;
    if (current_mode == PREP_TVC && cond)
    {
        ret = true;
    }
    return ret;
}


/* change_mode_to_shutdown_stable
 * 
 * Determines whether to change the mode to SHUTDOWN_STABLE.
 * 
 * Conditions:
 *   - current mode is BURN_BABY_BURN
 *   - TODO: determine any remaining conditions
 */
bool change_mode_to_shutdown_stable(Mode current_mode, bool cond)
{
    bool ret = false;
    if (current_mode == BURN_BABY_BURN && cond)
    {
        ret = true;
    }
    return ret;
}



void transition_to_countdown()
{
    //TODO : change led
    //reset navigation
    //TODO: set final countdown point
    //TODO: change LED
}

void transition_to_final_countdown()
{
    //TODO: set prep tvc point
    //TODO: change LED
}

void transition_to_prep_tvc()
{
    //TODO: set burn time
    //TODO: change LED
}

/* transition_to_burn_baby_burn
 * 
 * Handles anything that needs to happen only once when
 * changing modes from NAVIGATION to BURN_BABY_BURN.
 */
void transition_to_burn_baby_burn()
{
    //set stop time
    //TODO: change LED
}

/* transition_to_shutdown_stable
 * 
 * Handles anything that needs to happen only once when
 * changing modes from BURN_BABY_BURN to SHUTDOWN_STABLE.
 */
void transition_to_shutdown_stable()
{
    //TODO: change LED
    //transfer data
    //TODO: change LED
}
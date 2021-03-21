#include "mission_constants.hh"
#include "moding.hh"

bool change_mode_to_countdown(bool cond)
{
    bool ret = false;
    if (cond)
    {
        ret = true;
    }
    return ret;
}

bool change_mode_to_final_countdown(bool cond)
{
    bool ret = false;
    if (cond)
    {
        ret = true;
    }
    return ret;
}

bool change_mode_to_prep_tvc(bool cond)
{
    bool ret = false;
    if (cond)
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
bool change_mode_to_burn_baby_burn(bool cond)
{
    bool ret = false;
    if (cond)
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
bool change_mode_to_shutdown_stable(bool cond)
{
    bool ret = false;
    if (cond)
    {
        ret = true;
    }
    return ret;
}



void transition_to_countdown()
{
    //TODO : change led
    //reset navigation
    //TODO: change LED
}

void transition_to_final_countdown()
{
    //TODO: change LED
}

void transition_to_prep_tvc()
{
    //TODO: change LED
}

/* transition_to_burn_baby_burn
 * 
 * Handles anything that needs to happen only once when
 * changing modes from NAVIGATION to BURN_BABY_BURN.
 */
void transition_to_burn_baby_burn()
{
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

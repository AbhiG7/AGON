#include "mission_constants.hh"
#include "moding.hh"


/* change_mode_to_navigation
 * 
 * Determines whether to change the mode to NAVIGATION.
 * 
 * Conditions:
 *   - current mode is STARTUP_STABLE
 *   - TODO: determine any remaining conditions
 */
bool change_mode_to_navigation(Mode current_mode, bool cond)
{
    bool ret = false;
    if (current_mode == STARTUP_STABLE && cond)
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
    if (current_mode == NAVIGATION && cond)
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


/* transition_to_navigation
 * 
 * Handles anything that needs to happen only once when
 * changing modes from STARTUP_STABLE to NAVIGATION.
 */
void transition_to_navigation()
{
}


/* transition_to_burn_baby_burn
 * 
 * Handles anything that needs to happen only once when
 * changing modes from NAVIGATION to BURN_BABY_BURN.
 */
void transition_to_burn_baby_burn()
{
    // TODO: engine ignition occurs here
}


/* transition_to_shutdown_stable
 * 
 * Handles anything that needs to happen only once when
 * changing modes from BURN_BABY_BURN to SHUTDOWN_STABLE.
 */
void transition_to_shutdown_stable()
{
    // TODO: engine shutdown and safing occurs here
}
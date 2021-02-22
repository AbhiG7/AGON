#ifndef __MODING_HH__
#define __MODING_HH__

#include <cstdbool>
#include <cstdint>


bool change_mode_to_navigation(bool);
bool change_mode_to_burn_baby_burn(bool);
bool change_mode_to_shutdown_stable(bool);

void transition_to_navigation();
void transition_to_burn_baby_burn();
void transition_to_shutdown_stable();

#endif  // __MODING_HH__
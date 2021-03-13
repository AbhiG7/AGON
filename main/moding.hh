#ifndef __MODING_HH__
#define __MODING_HH__

#include <cstdbool>
#include <cstdint>


bool change_mode_to_navigation(bool);
bool change_mode_to_countdown(bool);
bool change_mode_to_final_countdown(bool);
bool change_mode_to_prep_tvc(bool);
bool change_mode_to_burn_baby_burn(bool);
bool change_mode_to_transfer_data(bool);
bool change_mode_to_shutdown_stable(bool);

void transition_to_navigation();
void transition_to_countdown();
void transition_to_final_countdown();
void transition_to_prep_tvc();
void transition_to_burn_baby_burn();
void transition_to_transfer_data();
void transition_to_shutdown_stable();

#endif  // __MODING_HH__
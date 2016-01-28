/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"



/*
 * control_rtlpref.pde - init and run calls for RTL flight mode
 *
 * This variation of the RTL Controller will add Precision Landing using IRLock
 * as well as logic for abort/retry if the drone dips below the top of the landing basket, but looses the beacon.
 *
 * There are two parts to RTL, the high level decision making which controls which state we are in
 * and the lower implementation of the waypoint or landing controllers within those states
 */


static uint32_t rtlprec_beacon_lost_time;
static bool rtlprec_pause;

// rtlprec_init - initialise rtl controller
bool Copter::rtlprec_init(bool ignore_checks)
{
   
    if (position_ok() || ignore_checks) {
        rtl_build_path();
        rtl_climb_start();
        return true;
    }else{
        return false;
    }
    
    rtlprec_pause = false;  //initialize the pause value
    ap.land_repo_active = false;

}

// rtl_run - runs the return-to-launch controller
// Seeems to be called at 400hz in the "Fast Loop", via ArduCopter.cpp
void Copter::rtlprec_run()
{
    // check if we need to move to next state
    if (rtl_state_complete) {
        switch (rtl_state) {
        case RTL_InitialClimb:
            rtl_return_start();
            break;
        case RTL_ReturnHome:
            rtl_loiterathome_start();
            break;
        case RTL_LoiterAtHome:
            Log_Write_Event(DATA_RTLPREC_DESCENT_START);
            rtl_land_start();
            break;
        case RTL_FinalDescent:
            // do nothing
            break;
        case RTL_Land:
            // do nothing - rtl_land_run will take care of disarming motors
            break;
        }
    }

    // call the correct run function
    switch (rtl_state) {

    case RTL_InitialClimb:
        rtl_climb_return_run();
        break;

    case RTL_ReturnHome:
        rtl_climb_return_run();
        break;

    case RTL_LoiterAtHome:
        rtl_loiterathome_run();
        break;

    case RTL_Land:
        rtlprec_land_run();
        break;
    }
}


void Copter::rtlprec_land_run()
{
    int16_t roll_control = 0, pitch_control = 0;
    float target_yaw_rate = 0;


    static uint32_t last_sec;
    
    uint32_t tnow = AP_HAL::millis();
    
    float current_alt = inertial_nav.get_altitude();

    if (tnow - last_sec > 1000) {
        last_sec = tnow;
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Alt: %f | Beacon:%d | Timeout: %f",current_alt,precland.beacon_detected(),(millis() - rtlprec_beacon_lost_time));
    }

    // if not auto armed or landing completed or motor interlock not enabled set throttle to zero and exit immediately

    if (ap.land_complete) {
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        wp_nav.init_loiter_target();
        init_disarm_motors();
        rtl_state_complete = true;
        return;
     }

   
    // relax loiter target if we might be landed
    if (ap.land_complete_maybe) {
        wp_nav.loiter_soften_for_landing();
    }

    // process pilot inputs
    if (!failsafe.radio) {
        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // process pilot's roll and pitch input
            roll_control = channel_roll->control_in;
            pitch_control = channel_pitch->control_in;

            // record if pilot has overriden roll or pitch
            if (roll_control != 0 || pitch_control != 0) {
                ap.land_repo_active = true;
            }
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
    }

    // process roll, pitch inputs
    wp_nav.set_pilot_desired_acceleration(roll_control, pitch_control);

#if PRECISION_LANDING == ENABLED
    // run precision landing
    if (!ap.land_repo_active) {
        wp_nav.shift_loiter_target(precland.get_target_shift(wp_nav.get_loiter_target()));
    }
#endif

    // run loiter controller
    wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

    // call attitude controller
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);

    if (!precland.beacon_detected()){                // If the beacon isn't detected
        if (!rtlprec_pause){                            // and the timer wasn't started
            rtlprec_beacon_lost_time = millis();        // Initialize timer
            rtlprec_pause = true;                       // Flag that we should pause
        } else {
            if( millis() - rtlprec_beacon_lost_time >= g.rtlprec_timeout ) {     // If the beacon hasn't been detected during our timeout period
               rtl_state = RTL_InitialClimb;                                    // Timeout and restart climb
            }     
          }
        
    } else {
        rtlprec_pause = false;
      }


    float cmb_rate;
    if(rtlprec_pause) {
        cmb_rate = 0;
    } else {
        cmb_rate = get_land_descent_speed();
    }

    // record desired climb rate for logging
    desired_climb_rate = cmb_rate;

    // update altitude target and call position controller
    pos_control.set_alt_target_from_climb_rate(cmb_rate, G_Dt, true);
    pos_control.update_z_controller();

}



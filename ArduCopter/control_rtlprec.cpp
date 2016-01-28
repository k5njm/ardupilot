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
static uint32_t rtlprec_beacon_acquired_time;

static bool rtlprec_fail;

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
    
    rtlprec_fail = false;  //initialize the pause value
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

    if (tnow - last_sec > 500) {
        last_sec = tnow;
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Alt: %f | Beacon:%d | Timeout: %f",current_alt,precland.beacon_detected(),(millis() - rtlprec_beacon_lost_time));
    }

    // if not auto armed or landing completed or motor interlock not enabled set throttle to zero and exit immediately

    if (ap.land_complete) {
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        wp_nav.init_loiter_target();
        init_disarm_motors();
        rtl_state_complete = true;
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Alt: %f | Beacon:%d | Landed!!",current_alt,precland.beacon_detected());
        return;
     }

   
    // relax loiter target if we might be landed
    if (ap.land_complete_maybe) {
        wp_nav.loiter_soften_for_landing();
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Alt: %f | Beacon:%d | Maybe Landed?!?",current_alt,precland.beacon_detected());
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

static float cmb_rate;

if (current_alt > g.rtlprec_alt){   //encompass all my custom code here


    if (!precland.beacon_detected()){                // If the beacon isn't detected
        if (!rtlprec_fail){                             // And failure state was set to 'false'
            rtlprec_beacon_lost_time = tnow;            // Initialize timer
            rtlprec_fail = true;                        // And set failure state to 'true'
        } else {                                                            // If failure state was set to "true"
            if( tnow - rtlprec_beacon_lost_time >= g.rtlprec_timeout ) {        // If the beacon hasn't been detected during our timeout period
               rtl_state = RTL_InitialClimb;                                    // Restart climb
            }     
          }
        
    } else {                                        //If the beacon is detected
        if (rtlprec_fail) {                             //And the failure state was previously 'true'
            rtlprec_beacon_acquired_time = tnow;        //Initialize the timer
            rtlprec_fail = false;                       // and set failure state to 'false'
        }
      }

    
    
    bool rtlprec_timeout_flag = tnow - rtlprec_beacon_lost_time > g.rtlprec_timeout/10;   //Returns true if I've been in fail for than 1/10 of rtlprec_timeout. 
                                                                                            //This prevents halting descent for short beacon failure durations
    
    bool rtlprec_acquisition_flag = !rtlprec_fail && tnow - rtlprec_beacon_acquired_time < g.rtlprec_timeout;  //Returns true if I've reacquired the beacon recently

    if( ((rtlprec_fail && rtlprec_timeout_flag) || rtlprec_acquisition_flag) && !ap.land_complete_maybe){       //If the beacon isn't detected, or has only been recently redetected, don't descend yet
        
        if (cmb_rate != 0){
            gcs_send_text_fmt(MAV_SEVERITY_INFO, "Alt: %f | Beacon:%d | Descent Halted (was %f)",current_alt,precland.beacon_detected(),cmb_rate);    
        }
        cmb_rate = 0;
    } else {
        if (cmb_rate == 0){
            cmb_rate = get_land_descent_speed();
            gcs_send_text_fmt(MAV_SEVERITY_INFO, "Alt: %f | Beacon:%d | Descent Resumed to: %f",current_alt,precland.beacon_detected(),cmb_rate);    
        } else {
            cmb_rate = get_land_descent_speed();
          }
    }

} else {  //If we're not running my custom code, use the default land_descent speed
    cmb_rate = get_land_descent_speed();
}
    // record desired climb rate for logging
    desired_climb_rate = cmb_rate;

    // update altitude target and call position controller
    pos_control.set_alt_target_from_climb_rate(cmb_rate, G_Dt, true);
    pos_control.update_z_controller();

}


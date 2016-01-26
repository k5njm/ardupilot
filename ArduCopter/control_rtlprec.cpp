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

// rtlprec_init - initialise rtl controller
bool Copter::rtlprec_init(bool ignore_checks)
{
    
    //initialize beacon_failure_counter
    beacon_failure_counter = 0;
    beacon_hop_retry_counter = 0;

    if (position_ok() || ignore_checks) {
        rtl_build_path();
        rtl_climb_start();
        return true;
    }else{
        return false;
    }
}

// rtl_run - runs the return-to-launch controller
// should be called at 100hz or more
void Copter::rtlprec_run()
{
   // Test code to spam the GCS
   // gcs_send_text_fmt(MAV_SEVERITY_INFO, "V:%f | R:%f | A:%f",sonar.voltage_mv(),sonar.distance_cm(),inertial_nav.get_altitude());   //also, just sonar_alt;

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
        case RTLPREC_HOP:
            rtlprec_land_run();
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
    case RTLPREC_HOP:
        rtlprec_hop_run();
        break;
    }
}


void Copter::rtlprec_land_run()
{
    int16_t roll_control = 0, pitch_control = 0;
    float target_yaw_rate = 0;
    // if not auto armed or landing completed or motor interlock not enabled set throttle to zero and exit immediately
    if(!ap.auto_armed || ap.land_complete || !motors.get_interlock()) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(0, 0, 0, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        // set target to current position
        wp_nav.init_loiter_target();

#if LAND_REQUIRE_MIN_THROTTLE_TO_DISARM == ENABLED
        // disarm when the landing detector says we've landed and throttle is at minimum
        if (ap.land_complete && (ap.throttle_zero || failsafe.radio)) {
            init_disarm_motors();
        }
#else
        // disarm when the landing detector says we've landed
        if (ap.land_complete) {
            init_disarm_motors();
        }
#endif

        // check if we've completed this stage of RTL
        rtl_state_complete = ap.land_complete;
        return;
    }

    // relax loiter target if we might be landed
    if (ap.land_complete_maybe) {
        wp_nav.loiter_soften_for_landing();
    }

    // process pilot's input
    if (!failsafe.radio) {
        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // process pilot's roll and pitch input
            roll_control = channel_roll->control_in;
            pitch_control = channel_pitch->control_in;
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
    }

     // process pilot's roll and pitch input
    wp_nav.set_pilot_desired_acceleration(roll_control, pitch_control);

#if PRECISION_LANDING == ENABLED
    // run precision landing
    if (!ap.land_repo_active) {
        wp_nav.shift_loiter_target(precland.get_target_shift(wp_nav.get_loiter_target()));
    }
#endif


    // run loiter controller
    wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

    float cmb_rate = get_land_descent_speed();

    if (!precland.beacon_detected()) {          // If the beacon isn't detected
        if (beacon_failure_counter >= 500) { //g.rtlprec_lostwait) {        // We've hit our max number of cycle failures,
           if (beacon_hop_retry_counter < g.rtlprec_hopretry) {       // We're below our max number of short hop retries, do a Hop
                beacon_hop_retry_counter++;                                 //Increment Hop Counter
                Vector3f target_pos = inertial_nav.get_position();          //Get My current position
                target_pos.z = target_pos.z + g.rtlprec_hopalt;              //Set Z to current altitude + rtlprec_hopalt
                wp_nav.set_wp_destination(target_pos);
                rtl_state = RTLPREC_HOP;
                return;  
            }
            else {                                       //We've done the max number of hops, let's do a full abort/retry to rtl_alt!
                 rtl_state = RTL_InitialClimb;           // Set the state back to Initial climb
                beacon_failure_counter = 0;             // Zero out the cycle counter
                beacon_hop_retry_counter = 0;           // Zero out hop_retry counter
                Log_Write_Event(DATA_RTLPREC_RETRY);
                return;  
            }
        }
       else {                                   // If we're at < 100 failures, keep our climb rate halted, and increment our failure counter
            cmb_rate = 0;                           // Halt descent
            beacon_failure_counter++;               // Increment failure counter
            }
    }
    else{                                       // If we see the beacon
        beacon_failure_counter = 0;                 // Clear the counter, descent will resume
        }

    // call z-axis position controller
    //float cmb_rate = get_land_descent_speed(); //Doing this above
    pos_control.set_alt_target_from_climb_rate(cmb_rate, G_Dt, true);
    pos_control.update_z_controller();

    // record desired climb rate for logging
    desired_climb_rate = cmb_rate;

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);

    // check if we've completed this stage of RTL
    rtl_state_complete = ap.land_complete;

}


void Copter::rtlprec_hop_run()      //This function runs the waypoint controller while running a "Short hop"
{
      // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if(!ap.auto_armed || !motors.get_interlock()) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(0, 0, 0, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
        // reset attitude control targets
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        // To-Do: re-initialise wpnav targets
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // run waypoint controller
    wp_nav.update_wpnav();

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control.input_euler_angle_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), get_auto_heading(),true);
    }

    // check if we've completed this stage of RTL
    rtl_state_complete = wp_nav.reached_wp_destination();
}


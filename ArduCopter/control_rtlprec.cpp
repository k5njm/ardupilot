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
    if (position_ok() || ignore_checks) {
        rtlprec_build_path();
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
            rtl_descent_start();
            break;
        case RTL_FinalDescent:
            rtl_land_start();
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

    case RTL_FinalDescent:
        rtlprec_descent_run();  //Descends using precision landing down to the altitude of land_basket_alt. If we see the beacon, continue by setting rtl_path.land to true, if not then retry the landing by jumping back to RTL_Climb_Start
        break;

    case RTL_Land:
        rtlprec_land_run();
        break;
    }
}


// rtlprec_descent_run - implements the final descent to the RTL_ALT
//      called by rtl_run at 100hz or more
void Copter::rtlprec_descent_run()
{
    int16_t roll_control = 0, pitch_control = 0;
    float target_yaw_rate = 0;

    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if(!ap.auto_armed || !motors.get_interlock()) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(0, 0, 0, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        // set target to current position
        wp_nav.init_loiter_target();
        return;
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

    // call z-axis position controller
    pos_control.set_alt_target_with_slew(rtl_path.descent_target.z, G_Dt);
    pos_control.update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);

// if (fabsf(g.land_basket_alt - inertial_nav.get_altitude()) < 10.0f) 


    float current_alt = inertial_nav.get_altitude();

    //For iidon drone, Need to compensate for bottom of drone being several cm below Lidar sensor
    //float current_alt = (inertial_nav.get_altitude() - 6.0f;


    if (current_alt < (rtl_path.descent_target.z + 10.0f)) {  //If I've descended 10cm above rtl_path.descent_target.z = pv_alt_above_origin(g.land_basket_alt
        if (precland.beacon_detected()) {
            rtl_state_complete = true;  //Move on to landing if I have a beacon
        } else {
            //It's just going to loiter here if we don't tell it to try again
            rtl_state = RTL_InitialClimb;
        }
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

    // call z-axis position controller
    float cmb_rate = get_land_descent_speed();
    pos_control.set_alt_target_from_climb_rate(cmb_rate, G_Dt, true);
    pos_control.update_z_controller();

    // record desired climb rate for logging
    desired_climb_rate = cmb_rate;

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);

    // check if we've completed this stage of RTL
    rtl_state_complete = ap.land_complete;

    float current_alt = inertial_nav.get_altitude();

    //For iidon drone, Need to compensate for bottom of drone being several cm below Lidar sensor
    //float current_alt = (inertial_nav.get_altitude() - 6.0f;

    if ((current_alt < (g.land_beacon_alt - 5.0f)) && (!precland.beacon_detected()) && (!ap.land_complete)) {  //If I've descended 5 cm below the beacon and the beacon is no longer in view and I'm not landed
        rtl_state = RTL_InitialClimb;  //Abort and retry
        } 
}




void Copter::rtlprec_build_path()
{
    // origin point is our stopping point
    pos_control.get_stopping_point_xy(rtl_path.origin_point);
    pos_control.get_stopping_point_z(rtl_path.origin_point);

    // set return target to nearest rally point or home position
#if AC_RALLY == ENABLED
    Location rally_point = rally.calc_best_rally_or_home_location(current_loc, 0);
    rtl_path.return_target = pv_location_to_vector(rally_point);
#else
    rtl_path.return_target = pv_location_to_vector(ahrs.get_home());
#endif

    Vector3f return_vector = rtl_path.return_target-rtl_path.origin_point;

    float rtl_return_dist = pythagorous2(return_vector.x, return_vector.y);

    // compute return altitude
    rtl_path.return_target.z = rtl_compute_return_alt_above_origin(rtl_return_dist);

    // climb target is above our origin point at the return altitude
    rtl_path.climb_target.x = rtl_path.origin_point.x;
    rtl_path.climb_target.y = rtl_path.origin_point.y;
    rtl_path.climb_target.z = rtl_path.return_target.z;

    // descent target is below return target at rtl_alt_final
    rtl_path.descent_target.x = rtl_path.return_target.x;
    rtl_path.descent_target.y = rtl_path.return_target.y;
    //rtl_path.descent_target.z = pv_alt_above_origin(g.rtl_alt_final);
    //Instead of descending to parameter rtl_alt_final, descend to land_basket_alt
    rtl_path.descent_target.z = pv_alt_above_origin(g.land_basket_alt);  //Not totally sure if I should keep pv_alt_above_origin, from position_vector.cpp
    

    // set land flag
    //rtl_path.land = g.rtl_alt_final <= 0;  //We'll set the landing flag inside of descent_run
}

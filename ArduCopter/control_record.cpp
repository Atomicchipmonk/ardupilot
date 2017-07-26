#include "Copter.h"


// throw_init - initialise throw controller
bool Copter::record_init(bool ignore_checks)
{
#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to use throw to start
    return false;
#endif

    // do not enter the mode when already armed or when flying
    if (motors->armed()) {
        return false;
    }

    //Copter::log_init();

    // init state
    throw_state.stage = Throw_Detecting;

    return true;
}

// runs the throw to start controller
// should be called at 100hz or more
void Copter::record_run()
{
    /* Throw State Machine
    Throw_Disarmed - motors are off
    Throw_Detecting -  motors are on and we are waiting for the throw
    Throw_Uprighting - the throw has been detected and the copter is being uprighted
    Throw_HgtStabilise - the copter is kept level and  height is stabilised about the target height
    Throw_PosHold - the copter is kept at a constant position and height
    */

    if (throw_state.stage == Throw_Detecting && throw_detected()){
        gcs_send_text(MAV_SEVERITY_INFO,"throw detected - recording!");
        throw_state.stage = Throw_Uprighting;

        Copter::set_land_complete(false);
        //motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);

        // Cancel the waiting for throw tone sequence
        AP_Notify::flags.waiting_for_throw = false;
	if(ap.land_complete){
            gcs_send_text(MAV_SEVERITY_INFO,"LandComplete:True");
        } else {
            gcs_send_text(MAV_SEVERITY_INFO,"LandComplete:False");
        }


        Copter::init_arm_motors(true);
/*
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // reset attitude control targets
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);

        // clear i term when we're taking off
        set_throttle_takeoff();

   	//Copter::start_logging();
*/
    }

    if (throw_state.stage == Throw_Uprighting && ap.land_complete){
        throw_state.stage = Throw_Detecting;
   	gcs_send_text(MAV_SEVERITY_INFO,"Landing detected - stop recording!");


    }
}

bool Copter::record_detected()
{
    // Check that we have a valid navigation solution
    nav_filter_status filt_status = inertial_nav.get_filter_status();
    if (!filt_status.flags.attitude || !filt_status.flags.horiz_pos_abs || !filt_status.flags.vert_pos) {
        return false;
    }

    // Check for high speed (>500 cm/s)
    bool high_speed = inertial_nav.get_velocity().length() > THROW_HIGH_SPEED;

    // check for upwards or downwards trajectory (airdrop) of 50cm/s
    bool changing_height;
    if (g2.throw_type == ThrowType_Drop) {
        changing_height = inertial_nav.get_velocity().z < -THROW_VERTICAL_SPEED;
    } else {
        changing_height = inertial_nav.get_velocity().z > THROW_VERTICAL_SPEED;
    }

    // Check the vertical acceleraton is greater than 0.25g
    bool free_falling = ahrs.get_accel_ef().z > -0.25 * GRAVITY_MSS;

    // Check if the accel length is < 1.0g indicating that any throw action is complete and the copter has been released
    //bool no_throw_action = ins.get_accel().length() < 1.0f * GRAVITY_MSS;

    // High velocity or free-fall combined with increasing height indicate a possible air-drop or throw release
    bool possible_throw_detected = (free_falling || high_speed) && changing_height;

    // Record time and vertical velocity when we detect the possible throw
    if (possible_throw_detected && ((AP_HAL::millis() - throw_state.free_fall_start_ms) > 500)) {
        throw_state.free_fall_start_ms = AP_HAL::millis();
        throw_state.free_fall_start_velz = inertial_nav.get_velocity().z;
    }

    // Once a possible throw condition has been detected, we check for 2.5 m/s of downwards velocity change in less than 0.5 seconds to confirm
    //bool throw_condition_confirmed = ((AP_HAL::millis() - throw_state.free_fall_start_ms < 500) && ((inertial_nav.get_velocity().z - throw_state.free_fall_start_velz) < -250.0f));

    // start motors and enter the control mode if we are in continuous freefall
    if (possible_throw_detected) {
        return true;
    } else {
        return false;
    }
}

bool Copter::record_attitude_good()
{
    // Check that we have uprighted the copter
    const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
    return (rotMat.c.z > 0.866f); // is_upright
}

bool Copter::record_height_good()
{
    // Check that we are no more than 0.5m below the demanded height
    return (pos_control->get_alt_error() < 50.0f);
}

bool Copter::record_position_good()
{
    // check that our horizontal position error is within 50cm
    return (pos_control->get_horizontal_error() < 50.0f);
}

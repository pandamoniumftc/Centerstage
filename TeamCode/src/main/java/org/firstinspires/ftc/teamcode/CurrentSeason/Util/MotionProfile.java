package org.firstinspires.ftc.teamcode.CurrentSeason.Util;
//TODO: input time and get an x and a y value
//TODO: find the derivative of the variant degree Bezier curve (once to find velocity, twice to find acceleration)
//TODO: make functions to get reference pos, velocity, and acceleration based on time
//TODO: make velocity and acceleration component with PID
//accel is the derivative of velocity
//velocity is the derivative of position, integrate for position with respected time
@SuppressWarnings("FieldCanBeLocal")
public class MotionProfile {
    private double accel, acceleration_dt, acceleration_distance, deceleration_dt, deceleration_time, cruise_distance, cruise_current_dt, cruise_dt, entire_dt, halfway_distance;

    double motionProfile(double max_acceleration, double max_velocity, double distance, double elapsed_time) {

        acceleration_dt = max_velocity / max_acceleration;

        halfway_distance = distance / 2;
        acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);

        if (acceleration_distance > halfway_distance) {
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration));
        }

        acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);

        max_velocity = max_acceleration * acceleration_dt;

        deceleration_dt = acceleration_dt;

        cruise_distance = distance - 2 * acceleration_distance;
        cruise_dt = cruise_distance / max_velocity;
        deceleration_time = acceleration_dt + cruise_dt;

        entire_dt = acceleration_dt + cruise_dt + deceleration_dt;

        if (elapsed_time > entire_dt) {
            return distance;
        }

        //acceleration
        if (elapsed_time < acceleration_dt) {
            return accel = 0.5 * max_acceleration * Math.pow(elapsed_time, 2);
        }

        //cruse
        else if (elapsed_time < deceleration_time) {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            cruise_current_dt = elapsed_time - acceleration_dt;

            return acceleration_distance + max_velocity * cruise_current_dt;
        }

        // deceleration
        else {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            cruise_distance = max_velocity * cruise_dt;
            deceleration_time = elapsed_time - deceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            return acceleration_distance + cruise_distance + max_velocity * deceleration_time - 0.5 * max_acceleration * Math.pow(deceleration_time, 2);
        }
    }
    /*public double MotionProfilePosition(){
        return 0.00;
    }*/
}
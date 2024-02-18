package org.firstinspires.ftc.teamcode.CurrentSeason.Util;

import org.firstinspires.ftc.teamcode.CurvesPort.VariantDegreeBezier;

//TODO: input time and get an x and a y value
//TODO: find the derivative of the variant degree Bezier curve (once to find velocity, twice to find acceleration)
//TODO: make functions to get reference pos, velocity, and acceleration based on time
//TODO: make velocity and acceleration component with PID
//accel is the derivative of velocity
//velocity is the derivative of position, integrate for position with respected time
@SuppressWarnings("FieldCanBeLocal")
public class MotionProfile {
    private static double accel;
    private static double acceleration_dt;
    private static double acceleration_distance;
    private static double deceleration_dt;
    private static double deceleration_time;
    private static double cruise_distance;
    private static double cruise_current_dt;
    private static double cruise_dt;
    private static double entire_dt;
    private static double halfway_distance;

    public static double motionProfile(double max_acceleration, double max_velocity, double distance, double elapsed_time) {

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

            return acceleration_distance + cruise_distance + max_velocity * deceleration_time - 0.5 * max_acceleration * Math.pow(deceleration_time, 2);
        }
    }
    public static double motionProfile_position() {
        //TODO: make position part and find out how to make it
        return 0;
    }
    public static double motionProfile_velocity() {
        //TODO: take derivative of position
        return 0;
    }
    public static double motionProfile_acceleration() {
        //TODO: take derivative of velocity
        return 0;
    }
}
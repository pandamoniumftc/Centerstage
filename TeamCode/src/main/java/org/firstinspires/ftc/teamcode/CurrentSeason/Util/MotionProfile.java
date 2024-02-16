package org.firstinspires.ftc.teamcode.CurrentSeason.Util;

public class MotionProfile {
  double motionProfile(double max_acceleration, double max_velocity, double distance, double elapsed_time) {
    
    acceleration_dt = max_velocity / max_acceleration;
  
    halfway_distance = distance / 2;
    acceleration_distance = 0.5 * max_acceleration * acceleration_dt ^ 2;
  
    if (acceleration_distance > halfway_distance) {
      acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration));
    }
  
    acceleration_distance = 0.5 * max_acceleration * acceleration_dt ^ 2;
  
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
      return 0.5 * max_acceleration * pow(elapsed_time, 2);
    }
  
    //cruse
    else if (elapsed_time < deceleration_time) {
      acceleration_distance = 0.5 * max_acceleration * acceleration_dt ^ 2;
      cruise_current_dt = elapsed_time - acceleration_dt;
  
      return acceleration_distance + max_velocity * cruise_current_dt;
    }
  
    // deceleration
    else {
      acceleration_distance = 0.5 * max_acceleration * acceleration_dt ^ 2;
      cruise_distance = max_velocity * cruise_dt;
      deceleration_time = elapsed_time - deceleration_time;
  
      // use the kinematic equations to calculate the instantaneous desired position
      return acceleration_distance + cruise_distance + max_velocity * deceleration_time - 0.5 * max_acceleration * math.pow(deceleration_time, 2);
    }
  }
}
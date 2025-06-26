#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/gyro.h>
#include <webots/camera.h>
#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include "pid_controller.h"

#define FLYING_ALTITUDE 3.0
#define PATROL_SPEED 0.5
#define ARENA_SIZE 10.0
#define GRID_SPACING 2.0
#define WAYPOINT_TOLERANCE 0.2
#define STABILIZATION_TIME 2.0

typedef struct {
    double x;
    double y;
} Waypoint;

// Function to generate grid waypoints
void generate_grid_waypoints(Waypoint *waypoints, int *count, double size, double spacing) {
    int points_per_side = (int)(size / spacing) + 1;
    *count = points_per_side * points_per_side;
    
    double start = -(size/2);
    int index = 0;
    
    for (int i = 0; i < points_per_side; i++) {
        double x = start + i * spacing;
        
        // Alternate direction for each row (snake pattern)
        if (i % 2 == 0) {
            for (int j = 0; j < points_per_side; j++) {
                waypoints[index].x = x;
                waypoints[index].y = start + j * spacing;
                index++;
            }
        } else {
            for (int j = points_per_side - 1; j >= 0; j--) {
                waypoints[index].x = x;
                waypoints[index].y = start + j * spacing;
                index++;
            }
        }
    }
}

int main() {
  wb_robot_init();
  int timestep = (int)wb_robot_get_basic_time_step();

  // Initialize motors
  WbDeviceTag m1 = wb_robot_get_device("m1_motor");
  wb_motor_set_position(m1, INFINITY);
  wb_motor_set_velocity(m1, -1.0);
  WbDeviceTag m2 = wb_robot_get_device("m2_motor");
  wb_motor_set_position(m2, INFINITY);
  wb_motor_set_velocity(m2, 1.0);
  WbDeviceTag m3 = wb_robot_get_device("m3_motor");
  wb_motor_set_position(m3, INFINITY);
  wb_motor_set_velocity(m3, -1.0);
  WbDeviceTag m4 = wb_robot_get_device("m4_motor");
  wb_motor_set_position(m4, INFINITY);
  wb_motor_set_velocity(m4, 1.0);

  // Initialize sensors
  WbDeviceTag imu = wb_robot_get_device("inertial_unit");
  wb_inertial_unit_enable(imu, timestep);

  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);

  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);

  WbDeviceTag camera = wb_robot_get_device("camera(1)");
  wb_camera_enable(camera, timestep);

  // Wait for stabilization
  while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 2.0)
      break;
  }

  // Initialize variables
  actual_state_t actual_state = {0};
  desired_state_t desired_state = {0};

  double past_x = 0.0;
  double past_y = 0.0;
  double past_time = wb_robot_get_time();
  double height_desired = FLYING_ALTITUDE;

  // Initialize PID gains.
  gains_pid_t gains_pid = {
    .kp_att_y = 1.0,
    .kd_att_y = 0.5,
    .kp_att_rp = 0.5,
    .kd_att_rp = 0.1,
    .kp_vel_xy = 2.0,
    .kd_vel_xy = 0.5,
    .kp_z = 4.0,
    .ki_z = 0.5,
    .kd_z = 2.0
  };
  init_pid_attitude_fixed_height_controller();

  // Initialize struct for motor power
  motor_power_t motor_power;

  // Generate patrol waypoints
  Waypoint waypoints[100];
  int waypoint_count;
  generate_grid_waypoints(waypoints, &waypoint_count, ARENA_SIZE - 2.0, GRID_SPACING);
  
  // Add return to home as last waypoint
  waypoints[waypoint_count].x = 0.0;
  waypoints[waypoint_count].y = 0.0;
  waypoint_count++;
  
  printf("Generated %d waypoints for patrol\n", waypoint_count);
  for (int i = 0; i < waypoint_count; i++) {
      printf("Waypoint %d: (%.1f, %.1f)\n", i, waypoints[i].x, waypoints[i].y);
  }

  int current_waypoint = 0;
  int patrolling = 0;

  while (wb_robot_step(timestep) != -1) {
    double now = wb_robot_get_time();
    double dt = now - past_time;

    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
    const double *gyro_values = wb_gyro_get_values(gyro);
    const double *gps_values = wb_gps_get_values(gps);

    double x = gps_values[0];
    double y = gps_values[1];
    double z = gps_values[2];

    double vx_global = (x - past_x) / dt;
    double vy_global = (y - past_y) / dt;

    double yaw = rpy[2];
    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);

    actual_state.roll = rpy[0];
    actual_state.pitch = rpy[1];
    actual_state.yaw_rate = gyro_values[2];
    actual_state.altitude = z;
    actual_state.vx = vx_global * cos_yaw + vy_global * sin_yaw;
    actual_state.vy = -vx_global * sin_yaw + vy_global * cos_yaw;

    // === STABILIZATION CHECK ===
    double speed = sqrt(actual_state.vx * actual_state.vx + actual_state.vy * actual_state.vy);
    if (!patrolling) {
      if (speed < 0.05 && now > STABILIZATION_TIME) {
        patrolling = 1;
        printf("Starting patrol pattern\n");
      } else {
        desired_state.vx = 0.0;
        desired_state.vy = 0.0;
      }
    } else {
      // Patrol control logic
      Waypoint target = waypoints[current_waypoint];
      double dx = target.x - x;
      double dy = target.y - y;
      double distance = sqrt(dx*dx + dy*dy);

      if (distance < WAYPOINT_TOLERANCE) {
        current_waypoint = (current_waypoint + 1) % waypoint_count;
        printf("Moving to waypoint %d: (%.1f, %.1f)\n", current_waypoint, waypoints[current_waypoint].x, waypoints[current_waypoint].y);
      }

      // Normalize direction and scale by patrol speed
      double vx_desired = PATROL_SPEED * dx / distance;
      double vy_desired = PATROL_SPEED * dy / distance;
      
      // Convert to body frame
      desired_state.vx = vx_desired * cos_yaw + vy_desired * sin_yaw;
      desired_state.vy = -vx_desired * sin_yaw + vy_desired * cos_yaw;
    }

    // Desired state
    desired_state.altitude = height_desired;
    desired_state.yaw_rate = 0.0;
    desired_state.roll = 0.0;
    desired_state.pitch = 0.0;

    // Run PID controller
    pid_velocity_fixed_height_controller(actual_state, &desired_state, gains_pid, dt, &motor_power);

    // printf("t=%.2f | x=%.2f y=%.2f vx=%.2f vy=%.2f | dx=%.2f dy=%.2f | patrolling=%d | altitude=%.2f\n",
           // now, x, y, vx_global, vy_global, desired_state.vx, desired_state.vy, patrolling, z);

    // Apply motor powers
    wb_motor_set_velocity(m1, -motor_power.m1);
    wb_motor_set_velocity(m2, motor_power.m2);
    wb_motor_set_velocity(m3, -motor_power.m3);
    wb_motor_set_velocity(m4, motor_power.m4);

    past_time = now;
    past_x = x;
    past_y = y;
  }

  wb_robot_cleanup();
  return 0;
}
#ifndef __TUNNEL_H
#define __TUNNEL_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef enum {
  DRONE_STATE_INACTIVE = 0, // Drone is not in the selected drones allowed to fly
  DRONE_STATE_IDLE,         // Drone waiting to be armed by it's leader
  DRONE_STATE_ARMED,        // Drone waiting for the signal with its leader to get low
  DRONE_STATE_FLYING,       // Drone flying and applying movement behaviors
  DRONE_STATE_CRASHED,      // Have a beeper to make noise or something... :(
} DroneState;

typedef enum {
  DRONE_ROLE_HEAD = 0,      // Applies the HeadMode requests by the operator
  DRONE_ROLE_RELAY          // Autonomous flight for relaying the head<->operator link
} DroneRole;

// Get the current drone state
DroneState tunnelGetDroneState(); 

// Set the global drone state
void tunnelSetDroneState(DroneState newState);

// Get the current drone role
DroneRole tunnelGetDroneRole();

// Set the global drone role
void tunnelSetDroneRole(DroneRole newRole);

void tunnelInit(void);

bool tunnelTest(void);

#endif
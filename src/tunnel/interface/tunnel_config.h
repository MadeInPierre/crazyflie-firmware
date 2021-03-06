#ifndef __TUNNELCONFIG_H
#define __TUNNELCONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/****************
      MAIN
*****************/

// Default number of active drones, can be changed at runtime
#define DEFAULT_N_DRONES 4

// Main tunnel loop frequency rate
#define TUNNEL_TASK_RATE_HZ 10

// Battery voltage range
#define BATTERY_MIN 3.2f
#define BATTERY_MAX 4.3f
#define BATTERY_RES 16 // number of values available to encode the battery level

// Print the drone's status information to the console at ~5Hz
#define PRINT_TUNNEL_STATUS

/****************
      DEBUG
*****************/

// Uncomment this line to run the normal tunnel code without ever turning the motors on (for tests)
// #define TUNNEL_MOTORS_DISABLE

// Disable the motors when putting a hand over the drone
#define TUNNEL_RED_SWITCH

// Default height used by all the movement behaviors
#define TUNNEL_DEFAULT_HEIGHT 0.3f

// Even when centering in the tunnel, constrain the height (e.g. for the flow sensor)
#define TUNNEL_MIN_HEIGHT 0.f
#define TUNNEL_MAX_HEIGHT 0.5f

/****************
 FLIGHT SETTINGS
*****************/

// Choose one quad direction configuration
// #define TUNNEL_QUAD_SHAPE_X    // Normal orientation, uses 2 avoiding sensors
#define TUNNEL_QUAD_SHAPE_PLUS // Used to avoid obstacles with 4 distance sensors, M1 goes forward

// Max drone speed in m/s
#define TUNNEL_MAX_SPEED 0.4f

// Default drone speed in m/s used in behaviors 
#define TUNNEL_DEFAULT_SPEED 0.2f

// Max drone yaw speed in deg/s
#define TUNNEL_MAX_TURN_SPEED 30.f

// Take off and landing speeds (m/s)
#define TAKE_OFF_VELOCITY 0.2f

// If we loose sight of the two walls, cut the motors (used when testing)
#define TUNNEL_STOP_ON_WALLS_LOST

// If ratios are close on both sides, take the closest side to a wall
#define TUNNEL_RANGE_RATIO_MAX_DIFF .4f

// If we see two walls on the sides, consider we're in a tunnel if the walls are this far appart or less (cm)
// (otherwise we're in an open space)
#define TUNNEL_MAX_WIDTH 100

/****************
OBSTACLE AVOIDING
*****************/

// Choose which directions to avoid 
#define TUNNEL_AVOID_LEFTRIGHT
// #define TUNNEL_AVOID_FRONTBACK

// Distance at wich we should start avoiding obstacles (cm)
#define TUNNEL_RANGER_TRIGGER_DIST 200

// Maximum distance before avoiding at all costs (cm)
#define TUNNEL_RANGER_DANGER_DIST 20

// Default distance to a wall when the other side does not exist
#define TUNNEL_DEFAULT_WALL_DIST 20.f

// When avoiding walls, push the drone away with this force (m/sec per ranging m)
#define TUNNEL_RANGER_AVOID_FORCE .2f

// When avoiding walls, turn the drone based on the left-right distance difference (deg/s according to ratio)
#define TUNNEL_RANGER_TURN_FORCE 30.f

/****************
      SIGNAL
*****************/

// RSSI Thresholds used in movement behaviors and state decisions
#define TUNNEL_RSSI_BEST    40 // Best quality considered (used to avoid collisions with another drone)
#define TUNNEL_RSSI_ARMED   50 // Quality to reach before taking off as a new relay drone
#define TUNNEL_RSSI_DANGER  65 // Threshold before considering the connection will soon be lost

// Time before a follower arms itself once its leader took off
#define TUNNEL_ARM_DELAY 3000

// RSSI is better in the air than when idle on the ground, compensate here
#define TUNNEL_RSSI_GROUND_PENALTY 10

// Process noise in the RSSI kalman filtering
#define TUNNEL_SIGNAL_KALMAN_R 0.5f

// Measurement noise in the RSSI kalman filtering
#define TUNNEL_SIGNAL_KALMAN_Q 5.f

// Control vector, predict an evolution based on the drone's current speed
#define TUNNEL_SIGNAL_KALMAN_B 3.f

// Maximum age of the last message by the other agent to be considered near
#define TUNNEL_DISCONNECT_TIMEOUT 1000

// Frequency at which a global chain ping should be sent
#define TUNNEL_PING_FREQ 3

// Which ID corresponds to a broadcast (should not be changed for 16 agents)
#define TUNNEL_BROADCAST_ID 0x0F

/****************
     RUNTIME
*****************/

// Says is the drone has a follower or a leader or not
bool isPeerIDValid(uint8_t id);

// Returns the current follower's ID
uint8_t getFollowerID();

// Sets the current follower's ID
void setFollowerID(uint8_t newID);

// Returns the current leader's ID
uint8_t getLeaderID();

// Sets the current leader's ID
void setLeaderID(uint8_t newID);

// Returns the ID of the drone that is currently connected to the base via CRTP
uint8_t getBaseDroneID();

// Sets the ID of the drone that is currently connected to the base via CRTP
void setBaseDroneID(uint8_t id);

// Sets the drone's follower and leader to ID-1 and ID+1
void tunnelAutoSetFollowerLeader();

// Sets the drone inactive if its outside of the active chain
void tunnelAutoSetIdleInactive();

// Returns the number of drones to be considered in the chain
uint8_t getNDrones();

// Sets the number of drones in the chain
void setNDrones(uint8_t ndrones);

// Get how much time we have been flying since the last take off
uint32_t getTunnelFlightTime();

// Get the last 4 bits of this drone's address
uint8_t getDroneId();

#endif
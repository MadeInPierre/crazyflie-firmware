/**
 * LARSEN Research team - INRIA
 * Multi-agent tunnel exploration module
 * 
 * author: Pierre Laclau <pierre.laclau@etu.utc.fr>
 * maintainer: LARSEN, INRIA Nancy Grand-Est, France
 *
 * tunnel_behavior.c - Submodule that applies the correct movement policy 
 *                     and manages behavior changes. Returns the desired movement.
 * 
 * Is part of the navigation stack (commander, avoider & behavior).
 */

#include "tunnel_behavior.h"
#include "tunnel_config.h"
#include "tunnel_commander.h"
#include "tunnel_signal.h"
#include "tunnel_avoider.h"
#include "tunnel.h"

#define DEBUG_MODULE "BEH"
#include "debug.h"

#include "FreeRTOS.h"
#include "task.h"
#include "led.h"

#include "estimator_kalman.h"

static TunnelBehavior currentBehavior;
static TunnelBehavior previousBehavior;

static uint32_t takeOffTime = 0;

// Wait for reconnect behavior

static void tunnelBehaviorReconnectUpdate(TunnelHover *vel, bool *enableCollisions) {
  tunnelSetPreviousBehavior(); //TODO implement
}

// Rollback behavior

static void tunnelBehaviorRollbackUpdate(TunnelHover *vel, bool *enableCollisions) {
  tunnelSetPreviousBehavior(); //TODO implement
}

// Positioning Behavior

#define TUNNEL_SIGNAL_DIFF_TOLERANCE 3

static void tunnelBehaviorPositioningUpdate(TunnelHover *vel, bool *enableCollisions) {
  // Don't move on other axis
  vel->vy = 0;
  vel->yawrate = 0;
  vel->zDistance = TUNNEL_DEFAULT_HEIGHT;

  SignalLog *followerSignal = (getDroneId() >= getNDrones() - 1) ? tunnelGetFollowerSignal() : tunnelGetBaseSignal();

  //TODO If the last RSSI value is too old, consider connection lost
  // if(isPeerIDValid(getLeaderID()) && !tunnelIsDroneConnected(getLeaderID()))
  //   tunnelSetBehavior(TUNNEL_BEHAVIOR_RECONNECT);
  // if(isPeerIDValid(getFollowerID()) && !tunnelIsDroneConnected(getFollowerID()))
  //   tunnelSetBehavior(TUNNEL_BEHAVIOR_ROLLBACK);

  // Don't go too close to another drone
  if(tunnelGetLeaderSignal()->rssi < TUNNEL_RSSI_BEST)
    vel->vx = -TUNNEL_DEFAULT_SPEED;
  else if(followerSignal->rssi < TUNNEL_RSSI_BEST)
    vel->vx = TUNNEL_DEFAULT_SPEED;
  else {
    // Move forward or backward to reach the destination
    float signalDiff = tunnelGetLeaderSignal()->rssi - followerSignal->rssi;
    if(signalDiff > TUNNEL_SIGNAL_DIFF_TOLERANCE / 2.f)
      vel->vx = TUNNEL_DEFAULT_SPEED;
    else if(signalDiff < -TUNNEL_SIGNAL_DIFF_TOLERANCE / 2.f)
      vel->vx = -TUNNEL_DEFAULT_SPEED;
    else vel->vx = 0;
  }

  // Status LED, green if we consider ourselves in the middle
  // ledSet(LED_GREEN_R, vel->vx == 0);
}

// Goto Behavior

static float gotoGoal = 0;

void setBehaviorGotoGoal(float goal) {
  gotoGoal = goal;
}

static void tunnelBehaviorGotoUpdate(TunnelHover *vel, bool *enableCollisions) {
  // Don't move on other axis
  vel->vy = 0;
  vel->yawrate = 0;
  vel->zDistance = TUNNEL_DEFAULT_HEIGHT;

  // Move forward or backward to reach the destination
  float tunnelDistance = tunnelGetDistance();
  if(gotoGoal - tunnelDistance > 0.05f)
    vel->vx = TUNNEL_DEFAULT_SPEED;
  else if(gotoGoal - tunnelDistance < -0.05f)
    vel->vx = -TUNNEL_DEFAULT_SPEED;
  else tunnelSetBehavior(TUNNEL_BEHAVIOR_HOVER);
}

// Take off & Land Behaviors

static float zTarget = 0.1f;
static uint32_t prevTime = 0;

static void verticalMotionUpdate(TunnelHover *vel, bool *enableCollisions, float direction) {
  // Don't move on other axis
  vel->vx = 0;
  vel->vy = 0;
  vel->yawrate = 0;

  // Handle resets
  if(prevTime == 0)
    prevTime = xTaskGetTickCount();

  // Disable collisions during takeoff
  *enableCollisions = false;

  // Slowly increase the height
  zTarget += direction * TAKE_OFF_VELOCITY * (float)(xTaskGetTickCount() - prevTime) / 1000.f;
  prevTime = xTaskGetTickCount();
  vel->zDistance = zTarget;
}

static void tunnelBehaviorTakeOffUpdate(TunnelHover *vel, bool *enableCollisions) {
  verticalMotionUpdate(vel, enableCollisions, 1.0);

  // End the behavior when the default height is reached
  if(zTarget >= TUNNEL_DEFAULT_HEIGHT) {
    vel->zDistance = TUNNEL_DEFAULT_HEIGHT;
    tunnelSetDistance(0); // Reset distance estimation
    takeOffTime = xTaskGetTickCount();

    switch(tunnelGetDroneRole()) {
      case DRONE_ROLE_HEAD:
        tunnelSetBehavior(TUNNEL_BEHAVIOR_HOVER);
        break;
      case DRONE_ROLE_RELAY:
        tunnelSetBehavior(TUNNEL_BEHAVIOR_POSITIONING);
        break;
      case DRONE_ROLE_BASE:
        tunnelSetBehavior(TUNNEL_BEHAVIOR_HOVER);
        break;
    }
  }
}

static void tunnelBehaviorLandUpdate(TunnelHover *vel, bool *enableCollisions) {
  verticalMotionUpdate(vel, enableCollisions, -1.0);

  // End the behavior when the ground is reached
  if(zTarget <= 0.05) {
    zTarget = 0;
    sendSetpointStop();
    tunnelSetBehavior(TUNNEL_BEHAVIOR_IDLE);
  }
}

// Main update function

void tunnelBehaviorUpdate(TunnelHover *vel, bool *enableCollisions) {
  switch (currentBehavior) {
    case TUNNEL_BEHAVIOR_IDLE:
      vel->vx = 0;
      vel->vy = 0;
      vel->yawrate = 0;
      vel->zDistance = 0;

      *enableCollisions = false;
      break;
    case TUNNEL_BEHAVIOR_TAKE_OFF:
      tunnelBehaviorTakeOffUpdate(vel, enableCollisions);
      break;
    case TUNNEL_BEHAVIOR_HOVER:
      vel->vx = 0;
      vel->vy = 0;
      vel->yawrate = 0;
      vel->zDistance = TUNNEL_DEFAULT_HEIGHT;

      *enableCollisions = true;
      break;
    case TUNNEL_BEHAVIOR_GOTO:
      tunnelBehaviorGotoUpdate(vel, enableCollisions);
      break;
    case TUNNEL_BEHAVIOR_SCAN:
      tunnelBehaviorScanUpdate(vel, enableCollisions);
      break;
    case TUNNEL_BEHAVIOR_POSITIONING:
      tunnelBehaviorPositioningUpdate(vel, enableCollisions);
      break;
    case TUNNEL_BEHAVIOR_RECONNECT:
      tunnelBehaviorReconnectUpdate(vel, enableCollisions);
      break;
    case TUNNEL_BEHAVIOR_ROLLBACK:
      tunnelBehaviorRollbackUpdate(vel, enableCollisions);
      break;
    case TUNNEL_BEHAVIOR_LAND:
      tunnelBehaviorLandUpdate(vel, enableCollisions);
      break;
  }
}

// Current behavior management

TunnelBehavior tunnelGetCurrentBehavior() {
  return currentBehavior;
}

static void setBehavior(TunnelBehavior newBehavior) {
  if(newBehavior != currentBehavior) {
    DEBUG_PRINT("%i->%i\n", currentBehavior, newBehavior);

    currentBehavior = newBehavior;

    switch (newBehavior) {
      case TUNNEL_BEHAVIOR_IDLE: {
        tunnelSetDroneState(DRONE_STATE_IDLE);
        break;
      } 
      case TUNNEL_BEHAVIOR_TAKE_OFF: {
        tunnelSetDroneState(DRONE_STATE_FLYING);
        zTarget = 0.1f;
        prevTime = 0;
        estimatorKalmanInit();
        break;
      }
      case TUNNEL_BEHAVIOR_SCAN: {
        tunnelBehaviorScanEnable();
        break;
      }
      case TUNNEL_BEHAVIOR_LAND: {
        zTarget = TUNNEL_DEFAULT_HEIGHT;
        prevTime = 0;
        break;
      }
    }
  }
}

void tunnelSetBehavior(TunnelBehavior newBehavior) {
  if(newBehavior != currentBehavior) {
    previousBehavior = currentBehavior;
    setBehavior(newBehavior);
  }
}

void tunnelSetPreviousBehavior() {
  setBehavior(previousBehavior);
}

uint32_t tunnelGetTakeOffTime() { return takeOffTime; }

// Submodule initialization

void tunnelBehaviorInit() {
  currentBehavior = TUNNEL_BEHAVIOR_IDLE;
}

bool tunnelBehaviorTest() {
  return true;
}
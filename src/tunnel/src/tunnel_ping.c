/**
 * LARSEN Research team - INRIA
 * Multi-agent tunnel exploration module
 * 
 * author: Pierre Laclau <pierre.laclau@etu.utc.fr>
 * maintainer: LARSEN, INRIA Nancy Grand-Est, France
 *
 * tunnel_ping.c - Sends, processes and propagates ping requests
 * accross the drones chain. Used for reporting the chain status
 * to the operator and informing all drones of each other's status.
 */

#include "tunnel_ping.h"
#include "tunnel_config.h"
#include "tunnel_relay.h"
#include "tunnel.h"
#include "tunnel_behavior.h"
#include "tunnel_helpers.h"
#include "tunnel_comm.h"
#include "tunnel_signal.h"

#define DEBUG_MODULE "PING"
#include "debug.h"

#include "FreeRTOS.h"
#include "task.h"
#include "config.h"

#include "system.h"
#include "led.h"
#include "ledseq.h"

#include "crtp.h"
#include "p2p.h"

typedef enum {
  PING_MODE_DIRECT    = 0x00,
  PING_MODE_PROPAGATE = 0x01,
} TunnelPingMode;

static P2PPacket reply;
static unsigned long pingPrevTime = 0;

// Private functions

static void sendCRTPPingReport(P2PPacket *p) {
  uint16_t pingTime = xTaskGetTickCount() - pingPrevTime;
  // DEBUG_PRINT("Ping returned in %ims.\n", pingTime);

  CRTPTunnelPacket p_log;
  p_log.port = CRTP_PORT_TUNNEL;
  p_log.channel = 0;
  memcpy(p_log.basedata, &p->rxdata[1], p->size - 1);
  p_log.size = p->size - 1;

  // Add our RSSI+status if there's room
  if(p_log.size < CRTP_MAX_DATA_SIZE - 2) {
    p_log.basedata[p_log.size++] = tunnelGetSignal(p->origin)->rssi;
    p_log.size += appendStatusMessage(&p_log.basedata[p_log.size]);
  }

  // Add the ping time if there's room
  if(p_log.size < CRTP_MAX_DATA_SIZE)
    p_log.basedata[p_log.size++] = (pingTime > 255) ? 255 : pingTime;

  // Send the packet to the PC through the chain
  tunnelSendCRTPPacketToBase(&p_log);
}

static void detectBaseDrone(P2PPacket* p) {
  for(uint8_t i = 1; i < p->size; i += 3) {
    bool isConnected = p->rxdata[i] & 0x01;

    if(isConnected) {
      uint8_t baseDroneID = i / 3;
      if(baseDroneID >= getNDrones())
        baseDroneID -= (getNDrones() - 1) * 3;

      setBaseDroneID(baseDroneID);
      return;
    }
  }

  // If no one is connected, default to the tail drone
  setBaseDroneID(getNDrones() - 1);
}

static void processFinishedPing(P2PPacket *p) {
  // Send a ping report to the PC (containes RSSI values between each drone)
  sendCRTPPingReport(p);

  // Detect if a drone is connected to the base and update our drone estimate ID
  detectBaseDrone(p);
}

static void p2pPingHandler(P2PPacket *p) {
  if(p->rxdest == getDroneId()) {
    // Detect if a drone is connected to the base and update our drone estimate ID
    detectBaseDrone(p);

    // The first drone doesn't reply to propagating pings
    if(getDroneId() == 0 && p->rxdata[0] == PING_MODE_PROPAGATE) 
      processFinishedPing(p);

    // Other drones reply or propagate the ping
    else {
      // first byte tells if the ping has to propagate through the chain
      if(p->size >= 1 && p->rxdata[0] == PING_MODE_PROPAGATE) {
        reply.txdest = getDroneId();
        if(getDroneId() == 0) // first drone replies to second
          reply.txdest = 1;
        else if(getDroneId() == getNDrones() - 1)
          processFinishedPing(p); //reply.txdest = getDroneId() - 1; // Go back through the chain 
        else // middle drone propagates the ping
          reply.txdest += (p->rxdest - p->origin > 0) ? 1 : -1;
      }

      // If not, reply back directly
      else reply.txdest = p->origin;
    
      // Copy the previous data
      memcpy(reply.txdata, p->rxdata, p->size);
      reply.size = p->size;
      reply.port = P2P_PORT_PING;

      // Add our RSSI+status if there's room
      if(reply.size < P2P_MAX_DATA_SIZE - 2) {
        reply.txdata[reply.size++] = tunnelGetSignal(p->origin)->rssi;
        reply.size += appendStatusMessage(&reply.txdata[reply.size]);
      }

      p2pSendPacket(&reply);
    }
  }
}

static void sendPing(TunnelPingMode mode) {
  P2PPacket p2p_p;
  p2p_p.txdest = (getDroneId() == 0) ? 1 : (getDroneId() + 1); 
  p2p_p.port = P2P_PORT_PING;
  p2p_p.txdata[0] = mode;
  p2p_p.size = 1 + appendStatusMessage(&p2p_p.txdata[1]);
  p2pSendPacket(&p2p_p);
}

// Public functions

void tunnelPingUpdate() {
  // Drone-by-drone ping that adds the RSSI values
  if(getDroneId() == 0 && timerElapsed(&pingPrevTime, M2T(1000 / TUNNEL_PING_FREQ)))
    sendPing(PING_MODE_PROPAGATE);
}

void crtpTunnelPingHandler(CRTPTunnelPacket *p) {
  sendPing(p->dronedata[0]);
}

void tunnelPingInit() {
  p2pRegisterPortCB(P2P_PORT_PING, p2pPingHandler);
}

bool tunnelPingTest() {
  return true;
}


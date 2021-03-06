/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * p2p.c - CrazyRealtimeTransferProtocol stack
 */

#include <stdbool.h>
#include <errno.h>

#define DEBUG_MODULE "P2P"
#include "debug.h"

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "config.h"

#include "p2p.h"
#include "info.h"
#include "cfassert.h"
#include "queuemonitor.h"

#include "log.h"
#include "system.h"

static bool isInit;

static int nopFunc(void);
static struct p2pLinkOperations nopLink = {
  .setEnable         = (void*) nopFunc,
  .sendPacket        = (void*) nopFunc,
  .receivePacket     = (void*) nopFunc,
}; 

static struct p2pLinkOperations *link = &nopLink;

#define STATS_INTERVAL 500
static struct {
  uint32_t rxCount;
  uint32_t txCount;

  uint16_t rxRate;
  uint16_t txRate;

  uint32_t nextStatisticsTime;
  uint32_t previousStatisticsTime;
} stats;

static xQueueHandle txQueue;

#define P2P_NBR_OF_PORTS 16
#define P2P_TX_QUEUE_SIZE 16
#define P2P_RX_QUEUE_SIZE 16

static void p2pTxTask(void *param);
static void p2pRxTask(void *param);

static xQueueHandle queues[P2P_NBR_OF_PORTS];
static volatile P2pCallback callbacks[P2P_NBR_OF_PORTS];
static volatile P2pCallback rssiCallback;
static void updateStats();

void p2pInit(void)
{
  if(isInit)
    return;

  txQueue = xQueueCreate(P2P_TX_QUEUE_SIZE, sizeof(P2PPacket));
  DEBUG_QUEUE_MONITOR_REGISTER(txQueue);

  xTaskCreate(p2pTxTask, P2P_TX_TASK_NAME,
              P2P_TX_TASK_STACKSIZE, NULL, P2P_TX_TASK_PRI, NULL);
  xTaskCreate(p2pRxTask, P2P_RX_TASK_NAME,
              P2P_RX_TASK_STACKSIZE, NULL, P2P_RX_TASK_PRI, NULL);

  /* Start Rx/Tx tasks */


  isInit = true;
}

bool p2pTest(void)
{
  return isInit;
}

void p2pInitTaskQueue(P2PPort portId)
{
  ASSERT(queues[portId] == NULL);
  
  queues[portId] = xQueueCreate(P2P_RX_QUEUE_SIZE, sizeof(P2PPacket));
  DEBUG_QUEUE_MONITOR_REGISTER(queues[portId]);
}

int p2pReceivePacket(P2PPort portId, P2PPacket *p)
{
  ASSERT(queues[portId]);
  ASSERT(p);
    
  return xQueueReceive(queues[portId], p, 0);
}

int p2pReceivePacketBlock(P2PPort portId, P2PPacket *p)
{
  ASSERT(queues[portId]);
  ASSERT(p);
  
  return xQueueReceive(queues[portId], p, portMAX_DELAY);
}


int p2pReceivePacketWait(P2PPort portId, P2PPacket *p, int wait)
{
  ASSERT(queues[portId]);
  ASSERT(p);
  
  return xQueueReceive(queues[portId], p, M2T(wait));
}

int p2pGetFreeTxQueuePackets(void)
{
  return (P2P_TX_QUEUE_SIZE - uxQueueMessagesWaiting(txQueue));
}

void p2pTxTask(void *param)
{
  systemWaitStart();

  P2PPacket p;

  while (true)
  {
    if (link != &nopLink)
    {
      if (xQueueReceive(txQueue, &p, portMAX_DELAY) == pdTRUE)
      {
        // Keep testing, if the link changes to USB it will go though
        while (link->sendPacket(&p) == false)
        {
          // Relaxation time
          vTaskDelay(M2T(10));
        }
        stats.txCount++;
        updateStats();
      }
    }
    else
    {
      vTaskDelay(M2T(10));
    }
  }
}

void p2pRxTask(void *param)
{
  systemWaitStart();

  P2PPacket p;

  while (true)
  {
    if (link != &nopLink)
    {
      if (!link->receivePacket(&p))
      {
        // Send the message to the polling port queue if it exists
        if (queues[p.port])
        {
          if (xQueueSend(queues[p.port], &p, 0) == errQUEUE_FULL)
          {
            // We should never drop packet
            ASSERT(0);
          }          
        }

        // Call the right port callback if it has been subscribed
        if (callbacks[p.port])
        {
          callbacks[p.port](&p);
        }

        // Call the RSSI callback if it has been subscribed
        if(rssiCallback)
        {
          rssiCallback(&p);
        }

        stats.rxCount++;
        updateStats();
      }
    }
    else
    {
      vTaskDelay(M2T(10));
    }
  }
}

void p2pRegisterPortCB(int port, P2pCallback cb)
{
  if (port>P2P_NBR_OF_PORTS)
    return;
  
  callbacks[port] = cb;
}

void p2pRegisterRssiCB(P2pCallback cb)
{
  rssiCallback = cb;
}

int p2pSendPacket(P2PPacket *p)
{
  ASSERT(p);
  ASSERT(p->size <= P2P_MAX_DATA_SIZE);

  return xQueueSend(txQueue, p, 0);
}

int p2pSendPacketBlock(P2PPacket *p)
{
  ASSERT(p); 
  ASSERT(p->size <= P2P_MAX_DATA_SIZE);

  return xQueueSend(txQueue, p, portMAX_DELAY);
}

void p2pPrintPacket(P2PPacket *p, bool format) { //TODO print format for tx, rx and raw
  if(!format)
    DEBUG_PRINT("P2P s=%i d=%02x%02x%02x%02x%02x%02x\n", p->size, p->raw[0], p->raw[1], p->raw[2], p->raw[3], p->raw[4], p->raw[5]);
  else {
    DEBUG_PRINT("P2P s=%i p=%i %i->%i r=%u\n", p->size, p->port, p->origin, p->rxdest, p->rssi);

    DEBUG_PRINT("d=%02X%02X%02X%02X%02X%02X%02X%02X%02X\n", (p->size > 0) ? p->rxdata[0] : 0xAA,
                                        (p->size > 1) ? p->rxdata[1] : 0xAA,
                                        (p->size > 2) ? p->rxdata[2] : 0xAA,
                                        (p->size > 3) ? p->rxdata[3] : 0xAA,
                                        (p->size > 4) ? p->rxdata[4] : 0xAA,
                                        (p->size > 5) ? p->rxdata[5] : 0xAA,
                                        (p->size > 6) ? p->rxdata[6] : 0xAA,
                                        (p->size > 7) ? p->rxdata[7] : 0xAA,
                                        (p->size > 8) ? p->rxdata[8] : 0xAA,
                                        (p->size > 9) ? p->rxdata[9] : 0xAA);
  }
}

int p2pReset(void)
{
  xQueueReset(txQueue);
  if (link->reset) {
    link->reset();
  }

  return 0;
}

bool p2pIsConnected(void)
{
  if (link->isConnected)
    return link->isConnected();
  return true;
}

void p2pSetLink(struct p2pLinkOperations * lk)
{
  if(link)
    link->setEnable(false);

  if (lk)
    link = lk;
  else
    link = &nopLink;

  link->setEnable(true);
}

static int nopFunc(void)
{
  return ENETDOWN;
}

static void clearStats()
{
  stats.rxCount = 0;
  stats.txCount = 0;
}

static void updateStats()
{
  uint32_t now = xTaskGetTickCount();
  if (now > stats.nextStatisticsTime) {
    float interval = now - stats.previousStatisticsTime;
    stats.rxRate = (uint16_t)(1000.0f * stats.rxCount / interval);
    stats.txRate = (uint16_t)(1000.0f * stats.txCount / interval);

    clearStats();
    stats.previousStatisticsTime = now;
    stats.nextStatisticsTime = now + STATS_INTERVAL;
  }
}

LOG_GROUP_START(p2p)
LOG_ADD(LOG_UINT16, rxRate, &stats.rxRate)
LOG_ADD(LOG_UINT16, txRate, &stats.txRate)
LOG_GROUP_STOP(p2p)

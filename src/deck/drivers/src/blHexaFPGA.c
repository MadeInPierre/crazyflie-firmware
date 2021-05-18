#define DEBUG_MODULE "BLHexaFPGA_Deck"
#include "debug.h"
#include "deck.h"
#include "log.h"
#include "param.h"
#include "i2cdev.h"

#include "system.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdlib.h>

static bool isInit = false;

#define Addr_FPGA 0b0000101 //I2C addr of the FPGA 0x05
#define regLowRPM 0b00010000
#define regHIGHRPM 0b00010001
#define nbMotors 4

static I2C_Dev *I2Cx;
static bool bus_inited = false;
uint8_t val;

// Parameters
uint16_t RPM[nbMotors];
uint8_t L_RPM[nbMotors];
uint8_t H_RPM[nbMotors];
uint16_t setpoint[nbMotors];
uint8_t L_setpoint[nbMotors];
uint8_t H_setpoint[nbMotors];
uint8_t highTelemetry[nbMotors];
uint8_t errorDshot[nbMotors];
uint16_t PIDout[nbMotors];
uint8_t L_PIDout[nbMotors];
uint8_t H_PIDout[nbMotors];
uint8_t config[4];
bool PID_en[nbMotors];  //bit 0 from config register
bool I2C_control_en[nbMotors];  //bit 1 from config register
uint8_t kp[nbMotors];
uint8_t ki[nbMotors];
uint8_t kd[nbMotors];
uint16_t PIDerror[nbMotors];
uint8_t L_PIDerror[nbMotors];
uint8_t H_PIDerror[nbMotors];
// address register parameters (8bit : 4High = num motor, 4Low = addr)
const uint8_t L_RPM_addr = 0x00;
const uint8_t H_RPM_addr = 0x01;
const uint8_t L_setpoint_addr = 0x02;
const uint8_t H_setpoint_addr = 0x03;
const uint8_t H_telemetry_addr = 0x04;
const uint8_t errorDshot_addr = 0x05;
const uint8_t L_PIDout_addr = 0x06;
const uint8_t H_PIDout_addr = 0x07;
const uint8_t config_addr = 0x0a;
const uint8_t kp_addr = 0x0b;
const uint8_t ki_addr = 0x0c;
const uint8_t kd_addr = 0x0d;
const uint8_t L_PIDerror_addr = 0x0e;
const uint8_t H_PIDerror_addr = 0x0f;

static void task(void *param)
{
    systemWaitStart();
    TickType_t lastWakeTime = xTaskGetTickCount();

    while(1)
    {
        vTaskDelayUntil(&lastWakeTime, M2T(200));
        // Update all the registers
        for(uint8_t i = 0; i < nbMotors; i++)
        {

            i2cdevReadByte(I2Cx, Addr_FPGA, ((i+1) << 4) | L_RPM_addr, &L_RPM[i]);
            i2cdevReadByte(I2Cx, Addr_FPGA, ((i+1) << 4) | H_RPM_addr, &H_RPM[i]);
            RPM[i] = (H_RPM[i] << 8) | L_RPM[i];

            i2cdevReadByte(I2Cx, Addr_FPGA, ((i+1) << 4) | L_RPM_addr, &L_RPM[i]);
            i2cdevReadByte(I2Cx, Addr_FPGA, ((i+1) << 4) | H_RPM_addr, &H_RPM[i]);
            RPM[i] = (H_RPM[i] << 8) | L_RPM[i];

            i2cdevReadByte(I2Cx, Addr_FPGA, ((i+1) << 4) | L_setpoint_addr, &L_setpoint[i]);
            i2cdevReadByte(I2Cx, Addr_FPGA, ((i+1) << 4) | H_setpoint_addr, &H_setpoint[i]);
            setpoint[i] = H_setpoint[i] << 8 | L_setpoint[i];

            i2cdevReadByte(I2Cx, Addr_FPGA,  ((i+1) << 4) | H_telemetry_addr, &highTelemetry[i]);
            i2cdevReadByte(I2Cx, Addr_FPGA,  ((i+1) << 4) | errorDshot_addr, &errorDshot[i]);

            i2cdevReadByte(I2Cx, Addr_FPGA, ((i+1) << 4) | L_PIDout_addr, &L_PIDout[i]);
            i2cdevReadByte(I2Cx, Addr_FPGA, ((i+1) << 4) | H_PIDout_addr, &H_PIDout[i]);
            PIDout[i] = H_PIDout[i] << 8 | L_PIDout[i];

            i2cdevReadByte(I2Cx, Addr_FPGA,  ((i+1) << 4) | config_addr, &config[i]);

            i2cdevReadByte(I2Cx, Addr_FPGA,  ((i+1) << 4) | kp_addr, &kp[i]);
            i2cdevReadByte(I2Cx, Addr_FPGA,  ((i+1) << 4) | ki_addr, &ki[i]);
            i2cdevReadByte(I2Cx, Addr_FPGA,  ((i+1) << 4) | kd_addr, &kd[i]);

            i2cdevReadByte(I2Cx, Addr_FPGA, ((i+1) << 4) | L_PIDerror_addr, &L_PIDerror[i]);
            i2cdevReadByte(I2Cx, Addr_FPGA, ((i+1) << 4) | H_PIDerror_addr, &H_PIDerror[i]);
            PIDerror[i] = H_PIDerror[i] << 8 | L_PIDerror[i];
        }
    }
}

static void init()
{
    if(isInit)
        return;
    
    if (!bus_inited) {
        i2cdevInit(I2C1_DEV);
        bus_inited = true;
        I2Cx = I2C1_DEV;
    }

    isInit = true;
    errorDshot[1] = 0xee;

    xTaskCreate(task, BLHEXAFPGA_TASK_NAME, BLHEXAFPGA_TASK_STACKSIZE, NULL, BLHEXAFPGA_TASK_PRI, NULL);

    DEBUG_PRINT("FPGA deck Init\n");
}

static bool test()
{
    if(i2cdevReadByte(I2Cx, Addr_FPGA, regLowRPM, &val))
    {
        DEBUG_PRINT("I2C FPGA ok\n");
        if(i2cdevWriteByte(I2Cx, Addr_FPGA, (0x0 | config_addr), 0x00)){
            DEBUG_PRINT("write FPGA config ok\n");
            return true;
        }
        else 
        {
            DEBUG_PRINT("write FPGA config error\n");
            return false;
        }
    }
    else
    {
        DEBUG_PRINT("I2C FPGA error\n");
        return false;
    }
}

static const DeckDriver BLHexaFPGA_deck = {
    .name = "BLHexaFPGA",
    .init = init,
    .test = test,
};

DECK_DRIVER(BLHexaFPGA_deck);

LOG_GROUP_START(FPGA)
// for (uint8_t i = 0; i < nbMotors; i++)   // would be better with a preprocessor for loop ...
// {
//     LOG_ADD(LOG_UINT16, RPM_1, &RPM[i])
//     LOG_ADD(LOG_UINT8, config_1, &config[i])
//     LOG_ADD(LOG_UINT8, errorDshot_1, &errorDshot[i])
// }

LOG_ADD(LOG_UINT16, RPM_1, &RPM[1])
LOG_ADD(LOG_UINT16, RPM_2, &RPM[2])
LOG_ADD(LOG_UINT16, RPM_3, &RPM[3])
LOG_ADD(LOG_UINT16, RPM_4, &RPM[4])
#if nbMotors==6
LOG_ADD(LOG_UINT16, RPM_5, &RPM[5])
LOG_ADD(LOG_UINT16, RPM_6, &RPM[6])
#endif
LOG_ADD(LOG_UINT8, config_1, &config[1])
LOG_ADD(LOG_UINT8, config_2, &config[2])
LOG_ADD(LOG_UINT8, config_3, &config[3])
LOG_ADD(LOG_UINT8, config_4, &config[4])
#if nbMotors==6
LOG_ADD(LOG_UINT8, config_5, &config[5])
LOG_ADD(LOG_UINT8, config_6, &config[6])
#endif

LOG_ADD(LOG_UINT8, errorDshot_1, &errorDshot[1])
LOG_ADD(LOG_UINT8, errorDshot_2, &errorDshot[2])
LOG_ADD(LOG_UINT8, errorDshot_3, &errorDshot[3])
LOG_ADD(LOG_UINT8, errorDshot_4, &errorDshot[4])
#if nbMotors==6
LOG_ADD(LOG_UINT8, errorDshot_5, &errorDshot[5])
LOG_ADD(LOG_UINT8, errorDshot_6, &errorDshot[6])
#endif

LOG_GROUP_STOP(FPGA)

PARAM_GROUP_START(FPGA)
PARAM_ADD(PARAM_UINT8, config1, &config[1])
PARAM_ADD(PARAM_UINT8, config2, &config[2])
PARAM_ADD(PARAM_UINT8, config3, &config[3])
PARAM_ADD(PARAM_UINT8, config4, &config[4])
#if nbMotors==6
PARAM_ADD(PARAM_UINT8, config5, &config[5])
PARAM_ADD(PARAM_UINT8, config6, &config[6])
#endif
PARAM_GROUP_STOP(FPGA)
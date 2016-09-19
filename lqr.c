#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"

#include "stabilizer.h"

#include "motors.h"

#include "sensors.h"
#include "commander.h"
#include "sitaw.h"
#include "controller.h"
#include "power_distribution.h"

#ifdef ESTIMATOR_TYPE_kalman
#include "estimator_kalman.h"
#else
#include "estimator.h"
#endif

static bool isInit;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;

static void lqrTask(void* param);

float sqrt_value(float value);
void lqr_control(setpoint_t *setpoint, const sensorData_t *sensorData, const state_t *state);

void lqrInit(void)
{
  if(isInit)
    return;

  sensorsInit();
  stateEstimatorInit();
  stateControllerInit();
  powerDistributionInit();
#if defined(SITAW_ENABLED)
  sitAwInit();
#endif

  /*xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME,
              STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);*/

  xTaskCreate(lqrTask, LQR_TASK_NAME,
              LQR_TASK_STACKSIZE, NULL, LQR_TASK_PRI, NULL);

  isInit = true;
}

bool lqrTest(void)
{
  bool pass = true;

  pass &= sensorsTest();
  pass &= stateEstimatorTest();
  pass &= stateControllerTest();
  pass &= powerDistributionTest();

  return pass;
}

/* The stabilizer loop runs at 1kHz (stock) or 500Hz (kalman). It is the
 * responsibility of the different functions to run slower by skipping call
 * (ie. returning without modifying the output structure).
 */

static void lqrTask(void* param)
{
  uint32_t tick = 0;
  uint32_t lastWakeTime;

  /*uint32_t ut=0;

  float w1=0;
  float w2=0;
  float w3=0;
  float w4=0;

  float w1_sq=0; 
  float w2_sq=0;
  float w3_sq=0;
  float w4_sq=0;

  float ux=0;
  float uy=0;
  float uz=0;*/


  //vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);
  vTaskSetApplicationTaskTag(0, (void*)TASK_LQR_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  // Wait for sensors to be calibrated
  lastWakeTime = xTaskGetTickCount ();
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }

  while(1) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));

#ifdef ESTIMATOR_TYPE_kalman
    stateEstimatorUpdate(&state, &sensorData, &control);
#else
    sensorsAcquire(&sensorData, tick);
    stateEstimator(&state, &sensorData, tick);
#endif

    commanderGetSetpoint(&setpoint, &state);

    sitAwUpdateSetpoint(&setpoint, &sensorData, &state);
    
    //ut=mult_ut*setpoint.thrust;

  /*ux = -gain_wx*sensorData.gyro.x-angle_gain_x*(state.attitude.roll - setpoint.attitude.roll);
    uy = gain_wy*sensorData.gyro.y-angle_gain_y*(state.attitude.pitch- setpoint.attitude.pitch);
    uz = gain_wz*sensorData.gyro.z-angle_gain_z*(state.attitude.yaw - setpoint.attitude.yaw);

    w1_sq= inv_dct*ut-inv_dct*ux+inv_dct*uy-inv_cq*uz;
    w2_sq= inv_dct*ut-inv_dct*ux-inv_dct*uy+inv_cq*uz;
    w3_sq= inv_dct*ut+inv_dct*ux-inv_dct*uy-inv_cq*uz;
    w4_sq= inv_dct*ut+inv_dct*ux+inv_dct*uy+inv_cq*uz;

    w1=sqrt_value(w1_sq);
    w2=sqrt_value(w2_sq);
    w3=sqrt_value(w3_sq);
    w4=sqrt_value(w4_sq);

    motorsSetRatio(MOTOR_M1, (uint16_t)w1);  //se implemente un casteo en las variables
    motorsSetRatio(MOTOR_M2, (uint16_t)w2);
    motorsSetRatio(MOTOR_M3, (uint16_t)w3);
    motorsSetRatio(MOTOR_M4, (uint16_t)w4);*/

    lqr_control(&setpoint, &sensorData, &state);    

    tick++;
  }
}

void lqr_control(setpoint_t *setpoint, const sensorData_t *sensorData, const state_t *state)
{

  volatile float w1=0;
  volatile float w2=0;
  volatile float w3=0;
  volatile float w4=0;

  volatile float w1_sq=0; 
  volatile float w2_sq=0;
  volatile float w3_sq=0;
  volatile float w4_sq=0;

  volatile uint32_t ut=0;

  volatile float ux =0;
  volatile float uy =0;
  volatile float uz =0;

  ut=mult_ut*setpoint->thrust;

  ux = -gain_wx*sensorData->gyro.x-angle_gain_x*(state->attitude.roll - setpoint->attitude.roll);
  uy = gain_wy*sensorData->gyro.y-angle_gain_y*(state->attitude.pitch- setpoint->attitude.pitch);
  uz = gain_wz*sensorData->gyro.z-angle_gain_z*(state->attitude.yaw - setpoint->attitude.yaw);

  w1_sq= inv_dct*ut-inv_dct*ux+inv_dct*uy-inv_cq*uz;
  w2_sq= inv_dct*ut-inv_dct*ux-inv_dct*uy+inv_cq*uz;
  w3_sq= inv_dct*ut+inv_dct*ux-inv_dct*uy-inv_cq*uz;
  w4_sq= inv_dct*ut+inv_dct*ux+inv_dct*uy+inv_cq*uz;

  w1=sqrt_value(w1_sq);
  w2=sqrt_value(w2_sq);
  w3=sqrt_value(w3_sq);
  w4=sqrt_value(w4_sq);

  motorsSetRatio(MOTOR_M1, (uint16_t)w1);
  motorsSetRatio(MOTOR_M2, (uint16_t)w2);
  motorsSetRatio(MOTOR_M3, (uint16_t)w3);
  motorsSetRatio(MOTOR_M4, (uint16_t)w4);

}

float sqrt_value(float value){
  float sqrut_value=0;
  if (value<0)
  {
    return 0;
  }
  else
  {
    sqrut_value=sqrt(value);
    if (sqrut_value>Max_Value)
    {
      return Max_Value;
    }
    else
    {
      return sqrut_value;
    };
  }
}

LOG_GROUP_START(ctrltarget)
LOG_ADD(LOG_FLOAT, roll, &setpoint.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &setpoint.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &setpoint.attitudeRate.yaw)
LOG_GROUP_STOP(ctrltarget)

LOG_GROUP_START(lqr)
LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)
LOG_ADD(LOG_FLOAT, wx, &sensorData.gyro.x)
LOG_ADD(LOG_FLOAT, wy, &sensorData.gyro.y)
LOG_ADD(LOG_FLOAT, wz, &sensorData.gyro.z)
LOG_GROUP_STOP(lqr)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &sensorData.acc.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.acc.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.acc.z)
LOG_GROUP_STOP(acc)

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &sensorData.baro.asl)
LOG_ADD(LOG_FLOAT, temp, &sensorData.baro.temperature)
LOG_ADD(LOG_FLOAT, pressure, &sensorData.baro.pressure)
LOG_GROUP_STOP(baro)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyro.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyro.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyro.z)
LOG_GROUP_STOP(gyro)

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &sensorData.mag.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.mag.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(controller)
LOG_ADD(LOG_INT16, ctr_yaw, &control.yaw)
LOG_GROUP_STOP(controller)

#include <time.h>
#include <math.h>
#include <pthread.h>
#include "generated/flight_plan.h"
#include "uwb_follower_control.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/autopilot_guided.h"
#include "navigation.h"

#define NDI_METHOD 1
// method 0 is first order approximation, no acceleration or yaw rate used
// method 1 is first order approximation, acceleration and yaw rate used, but yaw rate not taken into account in integral

#define NDI_MOST_RECENT ndihandle.data_entries-1

#ifndef UWB_NDI_DELAY
#define UWB_NDI_DELAY 4 // Delay of trajectory with respect to the leader
#endif

#define TRAJ_EPS 0.5
#define TRAJ_LENGTH 4

bool ndi_following_leader = false;
bool ndi_run_computation = true;
uint8_t traj_targetindex = 0;

static pthread_mutex_t uwb_ndi_mutex;
static uint8_t trajectory[TRAJ_LENGTH] = {WP_t3, WP_t2, WP_t1, WP_t0};

void cleanNdiValues(float tcur);
float accessCircularFloatArrElement(float *arr, int index);
float computeNdiFloatIntegral(float *ndiarr, float curtime);
void bindNorm(void);
void findNearestWaypoint(void);
void getNextTargetWaypoint(void);

// Tuned gains, as used in experiments from paper described in pre-amble comments
ndihandler ndihandle = {.delay = UWB_NDI_DELAY,
                        .tau_x = 3,
                        .tau_y = 3,
                        .wn_x = 0.9,
                        .wn_y = 0.9,
                        .eps_x = 0.28,
                        .eps_y = 0.28,
                        .Kp = -1.5,
                        .Ki = 0,
                        .Kd = -3,
                        .data_start = 0,
                        .data_end = 0,
                        .data_entries = 0,
                        .maxcommand = 1.5
                       };

static abi_event relative_localization_ev;
extern void uwb_follower_control_init(void)
{
  AbiBindMsgRELEATIVE_LOCALIZATION(ABI_BROADCAST, &relative_localization_ev, relative_localization_callback);
}

void relative_localization_callback(uint8_t sender_id __attribute__((unused)),uint8_t ac_id, float time, float range, float trackedAx, float trackedAy, float trackedYawr, float xin, float yin, float zin __attribute__((unused)), float u1in, float v1in, float u2in, float v2in, float gammain){
  if (ac_id != 0) {
    return;
  }
  
  // Store data from leader's position estimate
  pthread_mutex_lock(&uwb_ndi_mutex);
  if (ndihandle.data_entries == NDI_PAST_VALS) {
    ndihandle.data_entries--;
    ndihandle.data_start = (ndihandle.data_start + 1) % NDI_PAST_VALS;
  }
  ndihandle.xarr[ndihandle.data_end] = xin;
  ndihandle.yarr[ndihandle.data_end] = yin;
  ndihandle.u1arr[ndihandle.data_end] = u1in;
  ndihandle.v1arr[ndihandle.data_end] = v1in;
  ndihandle.u2arr[ndihandle.data_end] = u2in;
  ndihandle.v2arr[ndihandle.data_end] = v2in;
  ndihandle.r1arr[ndihandle.data_end] = update_butterworth_2_low_pass(&uwb_butter_yawr,stateGetBodyRates_f()->r);
  ndihandle.r2arr[ndihandle.data_end] = trackedYawr;
  ndihandle.ax1arr[ndihandle.data_end] = stateGetAccelNed_f()->x;
  ndihandle.ay1arr[ndihandle.data_end] = stateGetAccelNed_f()->y;
  ndihandle.ax2arr[ndihandle.data_end] = trackedAx;
  ndihandle.ay2arr[ndihandle.data_end] = trackedAy;
  ndihandle.gamarr[ndihandle.data_end] = gammain;
  ndihandle.tarr[ndihandle.data_end] = time;
  ndihandle.data_end = (ndihandle.data_end + 1) % NDI_PAST_VALS;
  ndihandle.data_entries++;
  pthread_mutex_unlock(&uwb_ndi_mutex);
}

void cleanNdiValues(float tcur)
{
  pthread_mutex_lock(&uwb_ndi_mutex);
  int curentries = ndihandle.data_entries;
  for (int i = 0; i < curentries; i++) {
    if ((tcur - ndihandle.tarr[ndihandle.data_start]) > ndihandle.delay) {
      ndihandle.data_start = (ndihandle.data_start + 1) % NDI_PAST_VALS;
      ndihandle.data_entries--;
    }
  }
  pthread_mutex_unlock(&uwb_ndi_mutex);
  return;
}

float accessCircularFloatArrElement(float *arr, int index)
{
  float value;
  pthread_mutex_lock(&uwb_ndi_mutex);
  int realindex = (ndihandle.data_start + index) % NDI_PAST_VALS;
  value = arr[realindex];
  pthread_mutex_unlock(&uwb_ndi_mutex);
  return value;
}

bool startNdiTracking(void)
{
  ndi_following_leader = true;
  return false;
}

bool stopNdiTracking(void)
{
  ndi_following_leader = false;
  return false;
}

// TODO: Check behavior when array is empty!
void uwb_follower_control_periodic(void)
{
  if (ndi_run_computation) {
    float curtime = get_sys_time_usec() / pow(10, 6);
    cleanNdiValues(curtime);
    if (ndihandle.data_entries > 0) {
#if(NDI_METHOD==0)
      float oldx = accessCircularFloatArrElement(ndihandle.xarr, 0);
      float oldy = accessCircularFloatArrElement(ndihandle.yarr, 0);

      float newu1 = accessCircularFloatArrElement(ndihandle.u1arr, NDI_MOST_RECENT);
      float newv1 = accessCircularFloatArrElement(ndihandle.v1arr, NDI_MOST_RECENT);
      float oldu2 = accessCircularFloatArrElement(ndihandle.u2arr, 0);
      float oldv2 = accessCircularFloatArrElement(ndihandle.v2arr, 0);
      oldx = oldx - computeNdiFloatIntegral(ndihandle.u1arr, curtime);
      oldy = oldy - computeNdiFloatIntegral(ndihandle.v1arr, curtime);

      float Minv[4];
      fmat_make_zeros(Minv, 2, 2);
      fmat_assign(0, 0, 2, Minv, -ndihandle.tau_x);
      fmat_assign(1, 1, 2, Minv, -ndihandle.tau_y);
      
      float l[2];
      l[0] = newu1 / ndihandle.tau_x;
      l[1] = newv1 / ndihandle.tau_y;

      float oldxed = oldu2 - newu1;
      float oldyed = oldv2 - newv1;

      float v[2];
      v[0] = ndihandle.Kp * oldx + ndihandle.Kd * oldxed;
      v[1] = ndihandle.Kp * oldy + ndihandle.Kd * oldyed;

      float sig[2];
      sig[0] = v[0] - l[0];
      sig[1] = v[1] - l[1];

      fmat_mult(2, 2, 1, ndihandle.commands, Minv, sig);
      bindNorm();

#elif(NDI_METHOD == 1)
      float oldx = accessCircularFloatArrElement(ndihandle.xarr, 0);
      float oldy = accessCircularFloatArrElement(ndihandle.yarr, 0);
      float newu1 = accessCircularFloatArrElement(ndihandle.u1arr, NDI_MOST_RECENT);
      float newv1 = accessCircularFloatArrElement(ndihandle.v1arr, NDI_MOST_RECENT);
      float newr1 = accessCircularFloatArrElement(ndihandle.r1arr, NDI_MOST_RECENT);
      float oldu2 = accessCircularFloatArrElement(ndihandle.u2arr, 0);
      float oldv2 = accessCircularFloatArrElement(ndihandle.v2arr, 0);
      float oldax2 = accessCircularFloatArrElement(ndihandle.ax2arr, 0);
      float olday2 = accessCircularFloatArrElement(ndihandle.ay2arr, 0);
      oldx = oldx - computeNdiFloatIntegral(ndihandle.u1arr, curtime);
      oldy = oldy - computeNdiFloatIntegral(ndihandle.v1arr, curtime);

      float Minv[4];
      fmat_make_zeros(Minv, 2, 2);
      fmat_assign(0, 0, 2, Minv, -ndihandle.tau_x);
      fmat_assign(1, 1, 2, Minv, -ndihandle.tau_y);

      float l[2];
      l[0] = (newu1 - newr1 * newr1 * oldx - newr1 * ndihandle.tau_x * newv1 + oldax2 * ndihandle.tau_x + 2 * newr1 *
              ndihandle.tau_x * oldv2) / ndihandle.tau_x;
      l[1] = (newv1 - newr1 * newr1 * oldy + newr1 * ndihandle.tau_y * newu1 + olday2 * ndihandle.tau_y - 2 * newr1 *
              ndihandle.tau_y * oldu2) / ndihandle.tau_y;

      float oldxed = oldu2 - newu1 + newr1 * oldy;
      float oldyed = oldv2 - newv1 - newr1 * oldx;

      float v[2];
      v[0] = ndihandle.Kp * oldx + ndihandle.Kd * oldxed;
      v[1] = ndihandle.Kp * oldy + ndihandle.Kd * oldyed;

      float sig[2];
      sig[0] = v[0] - l[0];
      sig[1] = v[1] - l[1];

      fmat_mult(2, 2, 1, ndihandle.commands, Minv, sig);
      bindNorm();
#endif
    } else {
      pthread_mutex_lock(&uwb_ndi_mutex);
      ndihandle.commands[0] = 0;
      ndihandle.commands[1] = 0;
      pthread_mutex_unlock(&uwb_ndi_mutex);
    }

  } else {
    pthread_mutex_lock(&uwb_ndi_mutex);
    ndihandle.commands[0] = 0;
    ndihandle.commands[1] = 0;
    pthread_mutex_unlock(&uwb_ndi_mutex);
  }
}

// TODO: Trapezoidal integration, and not simply discarding values before tcur-delay, but linearly interpolating to tcur-delay
float computeNdiFloatIntegral(float *ndiarr, float curtime)
{
  float integral = 0;
  float dt;
  for (int i = 0; i < ndihandle.data_entries - 1; i++) {
    dt = accessCircularFloatArrElement(ndihandle.tarr, i + 1) - accessCircularFloatArrElement(ndihandle.tarr, i);
    integral += dt * accessCircularFloatArrElement(ndiarr, i);
  }
  dt = curtime - accessCircularFloatArrElement(ndihandle.tarr, NDI_MOST_RECENT);
  integral += dt * accessCircularFloatArrElement(ndiarr, NDI_MOST_RECENT);

  return integral;
}

void bindNorm(void)
{
  pthread_mutex_lock(&uwb_ndi_mutex);
  float normcom = sqrt(ndihandle.commands[1] * ndihandle.commands[1] + ndihandle.commands[0] * ndihandle.commands[0]);
  if (normcom > ndihandle.maxcommand) {
    ndihandle.commandscap[0] = ndihandle.commands[0] * ndihandle.maxcommand / normcom;
    ndihandle.commandscap[1] = ndihandle.commands[1] * ndihandle.maxcommand / normcom;
  } else {
    ndihandle.commandscap[0] = ndihandle.commands[0];
    ndihandle.commandscap[1] = ndihandle.commands[1];
  }
  pthread_mutex_unlock(&uwb_ndi_mutex);

}

bool ndi_follow_leader(void)
{
  bool temp = true;
  temp &= guidance_v_set_guided_z(-NDI_FLIGHT_HEIGHT);
  temp &= guidance_h_set_guided_vel(ndihandle.commands[0], ndihandle.commands[1]);
  return !temp;
}
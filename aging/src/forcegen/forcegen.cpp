#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
#include <future>
#include <mutex>
#include <condition_variable>
#include <unordered_map>

#include "Eigen/Eigen"
using namespace Eigen;

#include "dhdc.h"
#include "drdc.h"

#include <chrono>
#include <thread>

#define PI 3.1415926535897

int divv = 1;
double block = 0.0, interval_in = 46;
double pmp[3] = {1,1,1};

/**
 * @brief		hand controller force generator
 * @details		supported types of forces:
 *              - exponential, parabola, sigmoid
 *              bugs:
 *              - fadd(force added) not functioning properly
 * @author		yong(yonglee0612@gmail.com)
 * @version		0.0
 */

#define REFRESH_INTERVAL 0.005 // sec default: 5ms or 5 배수

bool bRegOn = false;
bool bctrl  = false;
char ctrl[2];

enum {
  // formula
  NONE = 0,
  EXPONENTIAL,
  PARABOLA,
  SIGMOID_2010,
  SIGMOID_4035,

  // position
  X = 0,
  Y,
  Z
};

/***  for debugging ***/
double max_v[3] = {0.0};
double fadd = 0.0;
double vx, vy, vz;

void inline autoRegulatePos() { if(!bRegOn) drdRegulatePos(false); }

int drdInit()
{
  // message
  printf("\n+======================================================================+\n");
    printf("|                                                                      |\n");
    printf("|  Hand Controller Force Generator Debugger v0.0                       |\n");
    printf("|  Last updated: December 16th, 2022 by Yongkuk Lee                    |\n");
    printf("|  All Rights Reserved.                                                |\n");
    printf("|                                                                      |\n");
    printf("+======================================================================+\n\n");

  // open the first available device
  if (drdOpen() < 0)                                                  
  {
    printf("error: cannot open device (%s)\n", dhdErrorGetLastStr());
    dhdSleep(2.0);
    return -1;
  }

  // print out device identifier
  if (!drdIsSupported())
  {
    printf("unsupported device\n");
    printf("exiting...\n");
    dhdSleep(2.0);
    drdClose();
    return -1;

  }

  // print out list of devices
  printf ("<< devices >>\n");
  if(dhdGetDeviceCount() == 2)
  {
    int l = 0, r = 0;
    l = dhdOpenID(0);
    r = dhdOpenID(1);
    
    if(l < 0 || r < 0) 
        printf("\n error: initializing both ctrls failed (%s)\n", dhdErrorGetLastStr());
    
    ctrl[0] = static_cast<char>(l);
    ctrl[1] = static_cast<char>(r);
    bctrl = true;
    printf("%s haptic device detected\n", dhdGetSystemName(0));
    printf("%s haptic device detected\n\n", dhdGetSystemName(1));
  }
  else
    printf("%s haptic device detected\n\n", dhdGetSystemName());




  // perform auto-initialization
  if (!drdIsInitialized() && drdAutoInit() < 0)
  {
    printf("error: auto-initialization failed (%s)\n", dhdErrorGetLastStr());
    dhdSleep(2.0);
    return -1;
  }
  else if (drdStart() < 0)
  {
    printf("error: regulation thread failed to start (%s)\n", dhdErrorGetLastStr());
    dhdSleep(2.0);
    return -1;
  } else {}

  double nullPose[DHD_MAX_DOF] =
  {
    0.0, 0.0, 0.0,      // base  (translations)
    0.0, 0.0, 0.0,      // wrist (rotations)
    0.0                 // gripper
  };                       

  // move to center
  drdMoveTo(nullPose);

  // disable pos regulation (but leave regulation thread running)
  drdRegulatePos(false);
  return 1;
}

void dhdInit()
{
  // open the first available device
  if (dhdOpen () < 0) {
    printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr());
    dhdSleep (2.0);
    return;
  }
  
  // identify device
  printf ("<< Devices >>\n");
  printf ("%s device detected\n\n", dhdGetSystemName());
}

void shutdown_HC()
{
  // close the connection
  printf("cleaning up...                                                                                \n");
  drdClose();

  // safely exit
  printf("\nsafely terminated\n");
}

void onRegulate_HC()
{
  drdRegulatePos(true);
  bRegOn = true;
  printf("\n** enabled regulation\n\n");
}

void offRegulate_HC()
{
  drdRegulatePos(false);
  bRegOn = false;
  printf("\n** disabled regulation\n\n");
}

// return the current Cartesian coordinate
double *getPos_HC()
{
  double p[3];
  double *pp = p;
  dhdGetPosition(&p[0], &p[1], &p[2]);
  for (int i = 0; i < 3; ++i)
    p[i] /= 0.0001;
  printf("\ncurrent pos: ( x = %+0.03f mm, y = %+0.03f mm, z = %+0.03f mm )\n\n",p[0],p[1],p[2]);
  return pp;
}

void moveCenter_HC()
{
  drdRegulatePos(true);
  drdMoveToPos(0.0, 0.0, 0.0); 
  printf("\nmoved to center...");
  getPos_HC();
  autoRegulatePos();
}

void moveTo(double *pos)
{
  drdRegulatePos(true);
  drdMoveToPos(pos[X], pos[Y], pos[Z]); 
  printf("\nmoved to (%+0.03f, %+0.03f, %+0.03f) ...", pos[X], pos[Y], pos[Z]);
  getPos_HC();
  autoRegulatePos();
}
  
void setting()
{
  std::cout << "\nInput a, v, jerk: ";
  double tmp[3];
  std::cin >> tmp[0] >> tmp[1] >> tmp[2];
  if(tmp[0] != -1 && tmp[1] != -1 && tmp[2] != -1)
  {
    pmp[0] = tmp[0];
    pmp[1] = tmp[1];
    pmp[2] = tmp[2];
  }
  
  std::cout << "\nblock status(0,1) & delay: ";
  double tmp1, tmp2;
  std::cin >> tmp1 >> tmp2;
  if(tmp1 != -1 && tmp2 != -1)
  {
    block = tmp1;
    interval_in = tmp2;
  }

  std::cout << "\ncircle points: ";
  int tmp3;
  std::cin >> tmp3;
  if(tmp3 != -1)
  {
    divv = 360/tmp3;
  }
}

double getRad(double _deg){
    return _deg * (PI / 180);
};

void drawCircle(double block, double interval_in) // false, 46
{
  const double radius = 0.08;
  double addx = 0.066/(double)divv;
  double x = -0.045;

  drdRegulatePos(true);
  drdMoveToPos(x, radius * cos(getRad(0)), radius * sin(getRad(0)));
  drdSetPosMoveParam(pmp[0], pmp[1], pmp[2]);
  drdSetEncTrackParam(pmp[0], pmp[1], pmp[2]); // 100, ...
  std::chrono::milliseconds interval((int)interval_in);
  auto nextTime = std::chrono::system_clock::now() + interval;

  for(int deg = 0; deg <= 360/divv; ++deg) // 1 deg = 46ms
  {
    std::cout << "deg: " << deg << ' ';
    drdMoveToPos(x, radius * cos(getRad(deg*divv)), radius * sin(getRad(deg*divv)), bool(block));
    nextTime += interval;
    std::this_thread::sleep_until(nextTime);
    std::cout <<"(x,y,z): " << x << ' ' << radius * cos(getRad(deg)) << ' ' << radius * sin(getRad(deg)) << std::endl;
    x = x + addx;
  }
  
  printf("\n finished drawing a circle ... ");
  getPos_HC();
  autoRegulatePos();
}

void moveByForce(double* f)
{
  drdSetForceAndTorqueAndGripperForce(f[0], f[1], f[2],
                                      0, 0, 0,
                                      0);
}

void setForce(bool force = false, double dummyLength = 0.0)
{
    int res;
    double f = 0.0;
    double px, py, pz;
    
    if(force)
    {
        dhdGetPosition(&px, &py, &py);
        double x = px - dummyLength;
        double const coeff = 125.0;
        f = -x * coeff;
        f = 10.0;
    }
    res = drdSetForceAndTorqueAndGripperForce(f, 0, 0,  // force
                                              0, 0, 0,  // torque
                                              0);       // gripper force
    if (res < DHD_NO_ERROR) 
    {
      printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr ());
      drdClose();
    }
}

double exponential_logic (const double x, const double vmax) { return pow(1000000, ((x / vmax) - 1));       } 
double parabola_logic    (const double x, const double vmax) { return pow((x / vmax), 2);                   } 
double sigmoid_logic     (const double x, const double vmax, const double k, const double a) { return 1 / (1 + exp(-k * (x / vmax) + a)); }     // (k, a) = {(20, 10), (40,35)}

void setForceMode(const int mode, int hid)
{
    // handctrl
    char id = static_cast<char>(hid);
    const int axis_size = 6;

    // velocity vars
    double vabs           = 0.0;                   // absolute current velocity  (m/s)      receommened setting: = 0.0      
    double v[DHD_MAX_DOF] = {0.0};                 // applied velocity           (m/s)      receommened setting: = {0.0}
    const double vmax     = 0.3;                   // max allowed velocity       (m/s)      receommened setting: = 0.3
    
    // force vars       
    double fcoeff         = 0.0;                   // force coefficient                     recommended setting: = 0.0
    double f[DHD_MAX_DOF] = {0.0};                 // applied force              (N)        recommended setting: = {0.0}
    const double fmax     = 5.0;                   // max force before vmax      (N)        recommended setting: = 5.0
    const double fstop    = 98.0;                  // stopping force beyond vmax (N)        recommended setting: = 98.0
    
    // sigmoid var
    const double sig_ka[2][2] = {{20.0,10.0} , {40.0,35.0}};    // {20,10} = free motion: ~ 15.0% vmax , half fmax activation: 50.0% vmax
                                                                // {40,35} = free motion: ~ 66.6% vmax , half fmax activation: 87.5% vmax
                  
    /*** for debugging ***/
    static bool once   = false; 
    static int sigtype = 0;    
    
    // get velocity
    dhdGetLinearVelocity        (&(v[0]), &(v[1]), &(v[2]), id);     // position                   (m)
    dhdGetAngularVelocityRad    (&(v[3]), &(v[4]), &(v[5]), id);     // angular                    (m)
    dhdGetGripperLinearVelocity (&(v[6]), id);                       // gripper                    (m)

    std::function<double(const double, const double)> parabola    = parabola_logic;
    std::function<double(const double, const double)> exponential = exponential_logic;
    std::function<double(const double, const double, const double, const double)> sigmoid = sigmoid_logic;

    for(int i = 0 ; i < axis_size ; ++i)
    {
      if((vabs = fabs(v[i])) < vmax)    // within vmax
      {      
        switch(mode)
        {
            case EXPONENTIAL:
                fcoeff = exponential(vabs, vmax); 
                break;
            case PARABOLA:
                fcoeff = parabola(vabs, vmax); 
                break;
            case SIGMOID_2010:
                fcoeff = sigmoid(vabs, vmax, sig_ka[0][0], sig_ka[0][1]);
                break;
            case SIGMOID_4035:
                fcoeff = sigmoid(vabs, vmax, sig_ka[1][0], sig_ka[1][1]);
                break;
            default:
                break;
        }
      }
      else                              // beyond vmax
      {
        fcoeff = 1.0; 
        f[i] += (vabs - vmax) * fstop;
        /***  for debugging ***/  
        // printf("\n over vmax %d: v[i] = %+0.3f, vabs = %+0.3f, vmax = %+0.3f\n", i, v[i], vabs, vmax);
      }
      
      f[i] += (fcoeff * fmax);                        
      if(v[i] > 0) f[i] =  -(f[i]);     // adjust force direction

      /***  for debugging ***/
      max_v[i] = std::max(max_v[i], fabs(v[i]));
      if(i < 3)  fadd = std::max(fadd, (vabs-vmax)*fstop);
    }

    /***  for debugging pos ***/
    printf("fadd, f[0-2], f[6] (%0.01f, %+0.01f %+0.01f %+0.01f, %+0.01f) N  | v[0-2], v[6] (%+0.03f %+0.03f %+0.03f, %+0.03f) m/s      \r", 
                                fadd, f[0], f[1], f[2], f[6],                                v[0], v[1], v[2], v[6]);


    // set force
    drdSetForceAndTorqueAndGripperForce(f[0], f[1], f[2],    // position
                                        f[3], f[4], f[5],    // angular
                                        f[6],
                                        id);               // gripper
}

/* 
 *  functions from here on are for debugging purposes
 */

void printInstruction()
{
  // display instructions
  printf("<< instruction >>\n");
  printf("+---------------------------------"); printf("-------------------------------------------+\n");
  printf("| key |      general command      "); printf(" | key |         general command           |\n");
  printf("|---------------------------------"); printf("-------------------------------------------|\n");
  printf("| 'q' | quit                      "); printf(" | 'd' | display instruction               |\n"); 
  printf("| 'i' | initialize regulation     "); printf(" | 'j' | end regulation                    |\n");
  printf("| 'g' | get current position      "); printf(" | 'c' | re-center end-effector (all axis) |\n");
  printf("|---------------------------------"); printf("-------------------------------------------|\n");
  printf("| key |       force command       "); printf(" | key |          force command            |\n");
  printf("|---------------------------------"); printf("-------------------------------------------|\n");
  printf("| '1' | set force by sigmoid_2010 "); printf(" | 'a' | set position for aging            |\n"); 
  printf("| '3' | set force by exponential  "); printf(" | '2' | set force by sigmoid_4035         |\n"); 
  printf("| 'r' | reset force               "); printf(" | '4' | set force by parabola             |\n"); 
  printf("+---------------------------------"); printf("-------------------------------------------+\n\n");
}

void debugMode_HC()
{ 
  int done = 0;
  double t1, t0 = dhdGetTime();
  double freq = 0.0;
  double px, py, pz;
  double fx, fy, fz;
  double max_vx, max_vy, max_vz = 0.0;
  int forceType = 0;

  // dhdInit();
  drdInit();
  
  printInstruction();
  printf("<< input status >>\n");

  // haptic loop
  while (!done)
  {    
    // synchronize with regulation thread
    drdWaitForTick();
    
    if (forceType) setForceMode(forceType,-1);
    
    // display refresh rate and position at 10Hz
    t1 = dhdGetTime();
    if ((t1 - t0) > REFRESH_INTERVAL)
    {

      // retrieve information to display
      freq = dhdGetComFreq();
      t0 = t1;

      // write down position
      if (dhdGetPosition(&px, &py, &pz) < 0)
      {
        printf("error: cannot read position (%s)\n", dhdErrorGetLastStr());
        done = 1;
      }
      if (dhdGetForce(&fx, &fy, &fz) < 0)
      {
        printf("error: cannot read force (%s)\n", dhdErrorGetLastStr());
        done = 1;
      }

      dhdGetLinearVelocity (&vx, &vy, &vz);
      
      max_vx = std::max(max_vx, vx);
      max_vy = std::max(max_vy, vy);
      max_vz = std::max(max_vz, vz);

      if(!forceType) printf("p (%+0.03f %+0.03f %+0.03f) m  |  f (%+0.01f %+0.01f %+0.01f) N  | v (%+0.03f %+0.03f %+0.03f) m/s  |  freq [%0.02f kHz]       \r", px, py, pz, fx, fy, fz, vx, vy, vz, freq);

      // user input
      if (dhdKbHit())
      {
        switch (dhdKbGet())
        {
        case 'q': 
            done = 1;                               
            break;
        case 'd':                    
            printInstruction();                      
            break;
        case 'i':                        
            onRegulate_HC();                         
            break;
        case 'j':                        
            offRegulate_HC();                        
            break;
        case 'c':                    
            moveCenter_HC();                         
            break;
        case 'g':
            getPos_HC();                             
            break;
        case '1':
            forceType = SIGMOID_2010;
            if(bctrl) {setForceMode(SIGMOID_2010,0); setForceMode(SIGMOID_2010,1);}
            else      setForceMode(SIGMOID_2010,-1);         
            break;
        case '2':
            forceType = SIGMOID_4035;
            if(bctrl) {setForceMode(SIGMOID_4035,0); setForceMode(SIGMOID_4035,1);} 
            else      setForceMode(SIGMOID_4035,-1);         
            break;
        case '3':
            forceType = EXPONENTIAL;         
            if(bctrl) {setForceMode(EXPONENTIAL,0); setForceMode(EXPONENTIAL,1);}
            else      setForceMode(EXPONENTIAL,-1);     
            break;
        case '4':
            forceType = PARABOLA;                    
            if(bctrl) {setForceMode(PARABOLA,0); setForceMode(PARABOLA,1);}
            else      setForceMode(PARABOLA,-1);        
            break;
        case 'r':
            forceType = NONE;          
            setForce(false); 
            break;
        case 'a':
          {
            double in_pos[3] = {};
            std::cout << "\n\ninput desired x,y,z position(m) sequentially: ";
            std::cin >> in_pos[0] >> in_pos[1] >> in_pos[2];
            
            moveTo(in_pos);
            break;
          }
        case 'f':
          {
            double in_f[3] = {};
            std::cout << "\n\ninput desired x,y,z force(N) sequentially: ";
            std::cin >> in_f[0] >> in_f[1] >> in_f[2];
            
            moveByForce(in_f);
            break;
          }
        case 'o':
        {
          // double block, interval;
          // std::cout << "Input block status and interval(s): ";
          // std::cin >> block >> interval;
          drawCircle(block, interval_in);
          break;
        }
        case 's':
          setting();
          break;
        default:                                     
            break;
        }

      }
    }
  }
  if(fadd> 0) fadd += 5.0; // fmax
  printf("\n\n<< result (scalar) >>\n");
  printf("max velocity reached: (%0.03f, %0.03f, %0.03f) m/s\n", max_v[0], max_v[1], max_v[2]);
  printf("max force applied:    (%0.03f) N\n\n", fadd);

  shutdown_HC();
}

/*** TOP LEVEL ***/

int main(int argc,
         char **argv)
{
  debugMode_HC();
  return 0;
}

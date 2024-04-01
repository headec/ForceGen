/**
 * @brief		  R/W rate checker
 * @details		checks the rate of write/read that runs asynchronously
 * @author		yong(yonglee0612@gmail.com)
 * @version		NA
 **/

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <chrono>
#include <future>
#include <fstream>
#include <string>

#include "dhdc.h"
#include "drdc.h"

#define _USE_MATH_DEFINES
#define MAX_COUNT 999999

using namespace std::chrono;

unsigned int rmin = 99999, rmax = 0, rtotal = 0;
unsigned int wmin = 4294967295, wmax = 0, wtotal = 0; 
unsigned int relem_start[MAX_COUNT],relem_end[MAX_COUNT], welem_start[MAX_COUNT], welem_end[MAX_COUNT];
double p[3];
double ravg, wavg;
int cycle = 1000;
int addBy = 0;

int drdInit()
{
  // message
  printf("\n+======================================================================+\n");
  printf("|                                                                      |\n");
  printf("|  R/W-rate Checker v0.0                                               |\n");
  printf("|  Last updated: April 11th, 2022 by Yongkuk Lee                       |\n");
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
  printf("<< devices >>\n");
  if (dhdGetDeviceCount() == 2)
  {
    int l = 0, r = 0;
    l = dhdOpenID(0);
    r = dhdOpenID(1);

    if (l < 0 || r < 0)
      printf("\n error: initializing both ctrls failed (%s)\n", dhdErrorGetLastStr());

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
  }
  else
  {
  }

  double nullPose[DHD_MAX_DOF] =
      {
          0.0, 0.0, 0.0, // base  (translations)
          0.0, 0.0, 0.0, // wrist (rotations)
          0.0            // gripper
      };

  // move to center
  drdMoveTo(nullPose);

  // disable pos regulation (but leave regulation thread running)
  drdRegulatePos(false);
  return 1;
}

/*
// deprecated
uint64_t timeSinceEpochMillisec() {
  return duration_cast<nanoseconds>(high_resolution_clock::now().time_since_epoch()).count();
}
*/

void read_fd()
{
  for(int i = 0; i < cycle; ++i)
  {
    auto start = steady_clock::now();
    dhdGetPosition(&p[0], &p[1], &p[2]);
    auto end = steady_clock::now();
    auto delay = duration_cast<microseconds>(end - start);
    relem_start[i] = duration_cast<microseconds>(start.time_since_epoch()).count();
    relem_end[i] = duration_cast<microseconds>(end.time_since_epoch()).count();
    rmin = std::min(rmin, (unsigned int)delay.count()); 
    rmax = std::max(rmax, (unsigned int)delay.count()); 
    rtotal += (unsigned int)delay.count();
  }
}

void write_fd()
{
  // static bool symbol = false;
  double a[3] = {0,};
  for(int i = 0; i < cycle; ++i)
  {
    auto start = steady_clock::now();
    drdMoveToPos(a[0], a[1], a[2], true); 
    auto end = steady_clock::now();
    auto delay = duration_cast<microseconds>(end - start);
    welem_start[i] = duration_cast<microseconds>(start.time_since_epoch()).count();
    welem_end[i] = duration_cast<microseconds>(end.time_since_epoch()).count();
    wmin = std::min(wmin, (unsigned int)delay.count()); 
    wmax = std::max(wmax, (unsigned int)delay.count()); 
    wtotal += (unsigned int)delay.count();

    // testing outbound
    // a[0] = 1.0;
    // a[1] = 1.0;
    // a[2] = 1.0;
    
    // testing inbound
    // a[0] = 0.005 - (double)symbol;
    // a[1] = 0.005 - (double)symbol;
    // a[2] = 0.005 - (double)symbol;
    // symbol = !symbol; 
  }
}

// Example usage
int main()
{
  drdInit();
  drdRegulatePos(true);
  drdMoveToPos(0, 0, 0);
  drdSetEncTrackParam(2, 2, 2);

  std::cout << "<< input >>\n";
  std::cout << "input number of cycles: ";
  std::cin >> cycle;

  std::future<void> write_thread = std::async(std::launch::async, write_fd);
  std::future<void> read_thread = std::async(std::launch::async, read_fd);  

  write_thread.wait();
  read_thread.wait();

  ravg = (double)(rtotal / cycle);
  wavg = (double)(wtotal / cycle);

  int prev = 0, curr = 0;

  std::cout << "\n<< result >>\n";
  while(curr < cycle) //for(int i = 0 ; i < cycle ; i++)
  {
    if(welem_start[prev] < relem_start[curr])
    {
      if(relem_start[curr] < welem_end[prev]) {
        std::cout << "write index: " << prev << " | read index: " << curr <<
        "\nwrite: " << welem_start[prev] << ' ' << welem_end[prev] <<
        "\nread:  " << relem_start[curr] << ' ' << relem_end[curr] << std::endl;  
      }
      ++prev;
      continue;
    }
    ++curr;
  }

  std::cout << "\nRead(ns)  >> min: " << rmin << "  | max: " << rmax << " | avg: " << ravg <<std::endl; 
  std::cout << "Write(ns) >> min: " << wmin << " | max: " << wmax << " | avg: " << wavg << std::endl;

  // output csv file
  std::ofstream outfile;
  outfile.open("RW_test_result.csv");
  outfile << "write_start,write_end,read_start,read_end" << std::endl;
  for(int i = 0 ; i < cycle ; i++)
  {
    outfile << welem_start[i] << ',' << welem_end[i] <<
      "," << relem_start[i] << ',' << relem_end[i] << std::endl;  
  }
  outfile << "write_min,write_max,write_avg" << std::endl;
  outfile << wmin << ',' << wmax << ',' << wavg << std::endl;
  outfile << "read_min,read_max,read_avg" << std::endl;
  outfile << rmin << ',' << rmax << ',' << ravg << std::endl;
  
  outfile.close();
  return 0;
}
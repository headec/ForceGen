/*
 * temporary file to check errors from position and button
 */

#include "header/handctrl.h"
#include "header/cycle.h"

#define REFRESH_INTERVAL 1 // ms

std::chrono::milliseconds cycle(REFRESH_INTERVAL);

int main()
{
    handctrl omega;

    while(true){
        auto start_time = std::chrono::steady_clock::now();
        {
            /* dhdGetPosition test */
            // omega.runPositionTest();
            
            
            /* dhdGetButton test for index 0 & 1*/
            // omega.runButtonTest();
        }
        auto end_time = std::chrono::steady_clock::now();
        common::runCyclic(start_time, end_time, cycle);
    }
}
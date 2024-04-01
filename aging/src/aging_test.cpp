/**
 * @brief		aging test
 * @details		runs aging test with designated functions
 * @author		yong(yonglee0612@gmail.com)
 * @version		NA
 **/

#include "header/cycle.h"
#include "header/testunit.h"

#define REFRESH_INTERVAL 5  // ms

std::chrono::milliseconds cycle(REFRESH_INTERVAL);

int main()
{
    handctrl omega;
    testunit aging(omega);
    
    TestFuncContainer testing_functions = {
        {testunit::function::spiral,   {4}},
        {testunit::function::random,   {12}},
        {testunit::function::formula,  {testunit::formula::sine}},
        {testunit::function::formula,  {testunit::formula::parabola}},
        {testunit::function::circle,   {360}},
        {testunit::function::straight, {testunit::axis::x, 4}},
        {testunit::function::straight, {testunit::axis::y, 4}},
        {testunit::function::straight, {testunit::axis::z, 4}},
    };

    // default settings
    omega
        .setBlock(false)
        .setPosMoveParam(1,1,1);

    aging
        .setTable(testing_functions)
        .setTableIterative(true);

    // run
    while(true){
        auto start_time = std::chrono::steady_clock::now();

        aging.runTable();

        auto end_time = std::chrono::steady_clock::now();
        common::runCyclic(start_time, end_time, cycle);
    }
}
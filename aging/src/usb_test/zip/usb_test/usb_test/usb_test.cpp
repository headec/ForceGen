/**
 * @brief		test case in 1 ms cycle for usb error
 * @details		all the dhd functions that are in handctrl.h from epsilon are used here
 * @author	    yong(yonglee0612@gmail.com)
 * @version		NA
 */

#include "header/handctrl.h"
#include "header/cycle.h"
#include <future>

#define REFRESH_INTERVAL 1 // ms

std::chrono::milliseconds cycle(REFRESH_INTERVAL);

void passed(std::string s)
{
    std::cout << "passed '" << s << "'" <<std::endl;
}
                                                                                                                                            
void getButton()
{
    int input = -1; // for a single device
    while(true){
        auto start_time = std::chrono::steady_clock::now();
        {
            dhdGetButton(1, input);
            printf("dhdGetButton(%d,%d) is running asynchronously... \r",1 ,input);
        }
        auto end_time = std::chrono::steady_clock::now();
        common::runCyclic(start_time, end_time, cycle);
    }
}

int main()
{
    //handctrl omega;    // drd by default
    handctrl omega(mode::dhd);

    int input = omega._hid[0];
    double x;
    int stat[2][DHD_MAX_STATUS];
    std::string enter;

    printf("hid[0]: %d\n",omega._hid[0]);
    std::cout << "starting the test ... "<<std::endl <<std::endl;

    // std::async(std::launch::async, getButton); 

    while(true){
        auto start_time = std::chrono::steady_clock::now();
        {
            // press enter to stop the test
            // dhdGetButton(1, input);
            // passed("dhdGetButton");
            // std::getline(std::cin, enter);
            // return 0;

            // dhdGetGripperAngleRad(&x, input);  
            // passed("dhdGetGripperAngleRad");

            // dhdGetPositionAndOrientationRad(&x, &x, &x, &x, &x, &x, input);
            // passed("dhdGetPositionAndOrientationRad");

            // dhdGetStatus(stat[0], input);
            // passed("dhdGetStatus");
            
            // int b_idx = 1;
            // dhdGetButton(b_idx, input);
            // printf("dhdGetButton(%d,%d)\n",b_idx ,input);
            
            // dhdHasGripper(input);
            // passed("dhdHasGripper");
            
            // dhdOpenID(input);
            // passed("dhdOpenID");
            
            // dhdGetSystemType(input);
            // passed("dhdGetSystemType");

            // dhdGetSystemName(input);
            // passed("dhdGetSystemName");

            // dhdEnableForce(static_cast<unsigned char>(DHD_ON), input);
            // passed("dhdGetSystemName");

        }
        auto end_time = std::chrono::steady_clock::now();
        common::runCyclic(start_time, end_time, cycle);
    }
    dhdClose(input);
    passed("dhdClose");
}
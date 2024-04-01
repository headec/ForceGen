/**
 * @brief		temporary hand control device class for testing purposes
 * @details		omega.6 and lambda.7 are capable for this class.
 *              this class will be used for aging test mainly.
 * @author	    yong(yonglee0612@gmail.com)
 * @version		NA
 */

#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>

#include "dhdc.h"
#include "drdc.h"
#include "Eigen/Eigen"

#define _USE_MATH_DEFINES

using namespace Eigen;

#define PI 3.1415926535897

enum mode {
    dhd,
    drd
};

class handctrl
{
public:
    handctrl(mode init_mode = mode::drd) {
        if(!initiate(init_mode)) shutdown(); 
    };
    ~handctrl(){
        shutdown();
    };

    int initiate(mode mode)
    {
        // message
        printf("\n+======================================================================+\n");
        printf("|                                                                      |\n");
        printf("|  Force Dimension for Aging Test                                      |\n");
        printf("|  Last updated: April 12th, 2023 by Yongkuk Lee                       |\n");
        printf("|  All Rights Reserved.                                                |\n");
        printf("|                                                                      |\n");
        printf("+======================================================================+\n\n");

        auto open_e = [](){
            printf("error: cannot open device (%s)\n", dhdErrorGetLastStr());
            dhdSleep(2.0);
        };

        //open dhd
        if(mode == mode::dhd) {
            if(dhdOpen() < 0) {
                open_e;
                return 0;
            }   
        }
        // open drd
        else {
            if(drdOpen() < 0) {
                open_e;
                return 0;
            }
            // print out device identifier
            if (!drdIsSupported())
            {
                printf("unsupported device\n");
                printf("exiting...\n");
                dhdSleep(2.0);
                drdClose();
                return 0;
            }
        }

        // print out list of devices
        printf ("<< devices >>\n");
        if(dhdGetDeviceCount() == 2)
        {
            printf("detected two devices ... \n");
            
            // _hid[0] = dhdOpenType(DHD_DEVICE_LAMBDA331_LEFT);
            _hid[0] = 0;
            // _hid[1] = dhdOpenType(DHD_DEVICE_LAMBDA331);
            _hid[1] = 1;

            _rid[0] = dhdOpenID(_hid[0]);
            _rid[1] = dhdOpenID(_hid[1]);
            printf("_hid[0]: %d | _hid[1]: %d \n", _hid[0], _hid[1]);
            printf("_rid[0]: %d | _rid[1]: %d \n", _rid[0], _rid[1]);

            if(_hid[0] < 0 || _hid[1] < 0) {
                printf("\n error: initializing both ctrls failed (%s)\n", dhdErrorGetLastStr());
                return 0;
            }
            printf("%s haptic device detected\n", dhdGetSystemName(_hid[0]));
            printf("%s haptic device detected\n\n", dhdGetSystemName(_hid[1]));
        }
        else {
            _hid[0] = dhdOpenID(0);
            printf("%s haptic device detected\n\n", dhdGetSystemName());
        }

        if(mode == mode::dhd) return 1;
        
        // perform auto-initialization
        if (!drdIsInitialized() && drdAutoInit() < 0)
        {
            printf("error: auto-initialization failed (%s)\n", dhdErrorGetLastStr());
            dhdSleep(2.0);
            return 0;
        }
        
        else if (drdStart() < 0)
        {
            printf("error: regulation thread failed to start (%s)\n", dhdErrorGetLastStr());
            dhdSleep(2.0);
            return 0;
        } else { }

        double nullPose[DHD_MAX_DOF] =
        {
            0.0, 0.0, 0.0,      // base  (translations)
            0.0, 0.0, 0.0,      // wrist (rotations)
            0.0                 // gripper
        };                       

        // move to center
        drdMoveTo(nullPose);

        // enable pos regulation
        drdRegulatePos(true);

        return 1;
    }

    void shutdown()
    {
        printf("\n[device] cleaning up... \n");
        drdClose();
        printf("[device] safely terminated\n");
    }
    
    void moveToPosition(bool mode = false)
    {
        if(mode) drdTrackPos(_p[X], _p[Y], _p[Z]);
        else     drdMoveToPos(_p[X], _p[Y], _p[Z], _block);
    };
    
    bool isReached()
    {
        double* curr = this->getPosition();
        double* target = this->getTargetPosition();
        int isInPos = 0;
        std::cout<<"curr: " <<curr[0]<<' '<<curr[1]<<' '<<curr[2];
        std::cout<<" | target: " <<target[0]<<' '<<target[1]<<' '<<target[2] << "\n";
        for(int i = 0; i < ALL; i++) {
            if(target[i] - 0.001 < curr[i] && curr[i] < target[i] + 0.001) isInPos += pow(2,i);
        } 
        return (isInPos == 0b111);
    };

    inline double getTime()
    {
        return dhdGetTime();
    };

    double toRadian(double _deg)
    {
        return _deg * (PI / 180);
    };

    double *getTargetPosition()
    {
        return _p;
    };

    double *getPosition()
    {
        dhdGetPosition(&p[0], &p[1], &p[2]);
        return p;
    };
    
    handctrl& setBlock(bool block)
    {
        _block = block;
        return *this;
    };

    handctrl& setPosition(double* pa)
    {
        for(int i = 0; i < 3; i++) {
            _p[i] = pa[i];
        }
        return *this;
    };

    handctrl& setPosition(int which, double p)
    {
        _p[which] = p;
        return *this;
    }; 

    handctrl& setPositionCircleInYZ(double radius, double deg)
    {
        _p[Y] = radius * cos(toRadian(deg));
        _p[Z] = radius * sin(toRadian(deg));
        return *this;
    };
    
    handctrl& setPosMoveParam(double amax = 0.1, double vmax = 0.1, double jerk = 0.1)
    {
        drdSetPosMoveParam(amax, vmax, jerk);
        return *this;
    };

// USB Test starts (depreacated)
    void runPositionTest()
    {
        double p[2][3];
        dhdGetPosition(&p[0][0], &p[0][1], &p[0][2], _hid[0]);
        std::cout << "position(left):  " << p[0][0] << ' ' << p[0][1] << ' ' << p[0][2] << std::endl;
        if(dhdGetDeviceCount() == 2) {
            dhdGetPosition(&p[1][0], &p[1][1], &p[1][2], _hid[1]);
            std::cout << "position(right): " << p[1][0] << ' ' << p[1][1] << ' ' << p[1][2] << std::endl;
        }
        std::cout << std::endl;
    };

    void runButtonTest()
    {
        int b[2][2] = {{-2,},};

        for(int i = 0; i < 2; ++i)
        {
            b[0][i] = dhdGetButton(i, _hid[0]);

            if(_prev[i] != b[0][i]) std::cout << std::endl;
            std::cout << "b_l_" << i << ": " << b[0][i] << " ";
            _prev[i] = b[0][i];

            if(dhdGetDeviceCount() == 2) {
                b[1][i] = dhdGetButton(i, _hid[1]);

                if(_prev[2+i] != b[1][i]) std::cout << std::endl;
                std::cout << "b_r_" << i << ": " << b[1][i] << ' ';
                _prev[i+2] = b[1][i];
            }
        }
        std::cout << " | _hid[0]: "<< _hid[0] <<' '<<"_hid[1]: " << _hid[1] << "        \r";
    };
// USB Test ends

// variables
public:
    enum : int {
        X = 0,
        Y,
        Z,
        ALL
    };
    int _hid[2];
    int _rid[2];
protected:
    const double _x_min = -0.045, _x_max = 0.073;
    const double _y_min = -0.08,  _y_max = 0.08;
    const double _z_min = -0.08,  _z_max = 0.08;
    const double _x_circle_min = -0.047, _x_circle_max = 0.021;
private:
    double _p[3] = {0,};
    double p[3] = {0,};
    bool _block;

    double _prev[4];
};


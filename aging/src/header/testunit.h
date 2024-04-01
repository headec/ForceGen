/**
 * @brief		testing unit for aging
 * @details		build a table of functions to implement for aging test
 * @author		yong(yonglee0612@gmail.com)
 * @version		NA
 **/

#include <unordered_map>
#include <vector>
#include <random>

#include "handctrl.h"

typedef std::vector<int>                         TestFuncArg;
typedef std::vector<std::pair<int, TestFuncArg>> TestFuncContainer;

class testunit
{
public:
    // check available functions here
    explicit testunit(handctrl &arg_target) : _target(arg_target)
    {
        _table[testunit::function::circle]   = std::bind(&testunit::test_circle,   this, std::placeholders::_1);
        _table[testunit::function::straight] = std::bind(&testunit::test_straight, this, std::placeholders::_1);
        _table[testunit::function::formula]  = std::bind(&testunit::test_formula,  this, std::placeholders::_1);
        _table[testunit::function::random]   = std::bind(&testunit::test_random,   this, std::placeholders::_1);
        _table[testunit::function::spiral]   = std::bind(&testunit::test_spiral,   this, std::placeholders::_1);
        _table[testunit::function::eom]      = std::bind(&testunit::test_eom,      this, std::placeholders::_1);
    };

    // 1st argument defines the axis to implement; use axis enum
    // 2nd argument defines the number of iteration
    bool test_straight(TestFuncArg axis_iter)
    {
        static bool changeDirection = false;
        char what[3] = {'x', 'y', 'z'};

        if(!_once) {
            std::cout << "\nstarted straight in " << what[axis_iter.at(0)] << "-axis" << std::endl;
            _target
                .setPositionToCenter()
                .moveToPosition();
            _once = true;
        }

        if(_target.isReached()) {
            // terminate
            if(_counter >= axis_iter.at(1)) {
                changeDirection = 0;
                next();
                return status::stop; 
            }

            if(changeDirection)
                _target.setPosition(axis_iter.at(0), _target.getMin(axis_iter.at(0)));
            else
                _target.setPosition(axis_iter.at(0), _target.getMax(axis_iter.at(0)));
            _target.moveToPosition();
            changeDirection = !changeDirection;
            _counter++;
        }
        return status::repeat;
    };

    // input number of vertex you want to visit drawing a circle
    bool test_circle(TestFuncArg vertex)
    {
        static double deg = 0;

        if(!_once) {
            std::cout << "\nstarted circle in (y,z)-plane" << std::endl;
            _target
                .setPosTrackParam(true, 10, 10, 10)
                .setPositionToCenter()
                .moveToPosition();
            _once = true;
        }

        if(_target.isReached()) {
            if(vertex.at(0) < 1)
                vertex.at(0) = 1;
            
            // terminate
            if(deg > 360) {
                _target.setPosTrackParam(false);
                deg = 0;
                next();
                return status::stop;
            }

            std::cout << "\nheading to deg: " << deg << "    " << std::endl;
            _target
                .setPositionCircleInYZ(0.08, deg)
                .trackPosition();
            deg += 360 / vertex.at(0);
        }
        return status::repeat;
    };

    bool test_formula(TestFuncArg formulaName)
    {
        static double py      = _target.getMin(handctrl::Y);
        const double  py_max  = _target.getMax(handctrl::Y);
        const double  py_inc  = 2 * py_max / 360; 

        static std::unordered_map<int, std::function<double(const double, const double)>> formulaTable;
        std::function<double(const double, const double)> parabola = [](const double x, const double xmax) -> double { 
            return pow((x / xmax), 2); 
        };
        std::function<double(const double, const double)> sine = [](const double x, const double xmax) -> double { 
            return sin(x / xmax * 2 * PI); 
        };

        if(!_once) {
            std::string title[max_formula];
            title[formula::parabola] = "parabola";
            title[formula::sine]     = "sine";
            std::cout << "\nstarted " << title[formulaName.at(0)] <<" in (y,z)-plane" << std::endl;
            _target
                .setPosTrackParam(true, 10, 10, 10)
                .setPositionToCenter()
                .moveToPosition();
            formulaTable.emplace(formula::parabola, parabola);
            formulaTable.emplace(formula::sine,     sine);
            _once = true;
        }

        double pz = formulaTable[formulaName.at(0)](py, py_max) * _target.getMax(handctrl::Z);
        double outbound = std::sqrt(pow(0.08, 2) - pow(py, 2));

        if(std::abs(pz) > outbound && py < py_max) {
            py += py_inc;
            return status::repeat;
        }

        if(_target.isReached()) {
            _target
                .setPosition(handctrl::Y, py)
                .setPosition(handctrl::Z, pz)
                .trackPosition();

            py += py_inc;

            // terminate
            if(py > py_max) {
                _target.setPosTrackParam(_once = false);
                py = _target.getMin(handctrl::Y);
                next();
                return status::stop;
            }
        }
        return status::repeat;
    };

    // randomly generates vectors with given number of iterations
    bool test_random(TestFuncArg iter)
    {
        std::random_device rd;                          // random device to seed the generator
        std::mt19937 gen(rd());                         // Mersenne Twister pseudo-random generator with rd as seed
        std::uniform_int_distribution<> dist(1, 100);   // uniform distribution between 1 and 100

        if(!_once) {
            std::cout << "\nstarted random in (y,z)-plane" << std::endl;
            _target
                .setPositionToCenter()
                .moveToPosition();
            _once = true;
        }

        if(_target.isReached()) {
            double rradius = 0.08 * dist(gen) / 100;
            double rdegree = 360  * dist(gen) / 100;
        
            _target
                .setPositionCircleInYZ(rradius, rdegree)
                .moveToPosition();
            std::cout << "\nrandom vector[" << _counter << "] reached" << std::endl;
            
            // terminate
            if(iter.at(0) == ++_counter) {
                next();
                return status::stop;
            }
        }
        return status::repeat;
    };

    bool test_spiral(TestFuncArg nPeriod)
    {
        const double
            inc = 0.08 / (360 * nPeriod.at(0));
        static double 
            r = 0,
            t = 0;

        if(!_once) {
            std::cout << "\nstarted spiral pattern in (y,z)-plane" << std::endl;
            _target
                .setPosTrackParam(true, 5, 5, 5)
                .setPositionToCenter()
                .moveToPosition();
            _once = true;
        }

        if(_target.isReached()) {
            _target
                .setPositionCircleInYZ(r, t)
                .trackPosition();
            
            r += inc;
            ++t;

            // terminate
            if(t >= (360 * nPeriod.at(0)) || r > 0.08){
                r = t = 0;
                _target
                    .setPosTrackParam(false);
                next();
                return status::stop;
            }
        }
        return status::repeat;
    };

    bool test_eom(TestFuncArg amp)
    {
        const double 
            inc = 2 * PI / 360, 
            radius = 0.08,
            m = amp.at(0),
            n = amp.at(1);
        static double t = 0;
        double a, r;
     

        if(!_once) {
            std::cout << "\nstarted eom pattern in (y,z)-plane" << std::endl;
            _target
                .setPosTrackParam(true, 5, 5, 5)
                .setPositionToCenter()
                .moveToPosition();
            _once = true;
        }

        if(_target.isReached()) {
            a = 360 * sin(t*m);
            r = radius * sin(t*n);
            _target
                .setPositionCircleInYZ(r, a)
                .trackPosition();

            t += inc;

            // terminate
            if(t >= 2*PI){
                t = 0;
                _target
                    .setPosTrackParam(false);next();
                return status::stop;
            }
        }
        return status::repeat;
    };

    testunit& setTable(TestFuncContainer set)
    {
        _testset  = set;
        _max_test = _testset.size();
        return *this;
    };

    // true to run table iterative
    testunit& setTableIterative(bool on = false)
    {
        if(on) _iter = true;
        return *this;
    };

    void runTable()
    {
        if (_iter) 
            _which_test %= _max_test;
        if (_which_test < _max_test)
            _table[_testset[_which_test].first](_testset[_which_test].second);
    };

    int getStatus()
    {
        return _which_test;
    };

    void next()
    {
        _once = _counter = 0;
        ++_which_test;
    };
private:
    handctrl &_target;
    TestFuncContainer _testset;
    
    std::unordered_map<int, std::function<bool(TestFuncArg)>> _table;
    
    int _which_test = 0;
    int _max_test   = 0;
    bool _iter = false;

    // recommended to be reintialized in next()
    int  _counter = 0;
    bool _once    = false;
public:
    // check the constructor for available functions
    enum function : int
    {
        straight = 0,
        circle,
        formula,
        random,
        eom,
        spiral,
        max_function    // unused
    };

    enum formula : int {
        parabola,
        sine,
        cosine,     // unavailable yet
        tangent,    // unavailable yet
        max_formula
    };

    enum axis : int
    {
        x = 0,
        y,
        z,
        max_axis
    };

    enum status : bool 
    {
        stop   = false,
        repeat = true
    };
};
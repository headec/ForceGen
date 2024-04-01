/*
 * temporary file to make code cyclic 
 */

#include <chrono>
#include <thread>

namespace common
{
template <class T>
void inline runCyclic(T start_time, T end_time, std::chrono::milliseconds cycle)
{    
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    if(elapsed_time < cycle) std::this_thread::sleep_for(cycle - elapsed_time);
    // std::cout << " cycle(ms): " <<std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -start_time).count() << " \r";
}
}
#include "Timer.h"

using namespace std::chrono;

void Timer::startTimer(std::string timerName)
{
    if (timers.find(timerName) == timers.end())
    {
        timers.insert(std::pair<std::string, fntimer>(timerName, fntimer()));
    }
    currentTimer = timerName;
    tStart = high_resolution_clock::now();
}

void Timer::stopTimer()
{
    tStop  = high_resolution_clock::now();

    auto duration = duration_cast<microseconds>( tStop - tStart ).count();

    fntimer fn = timers[currentTimer];

    if (duration > fn.maxVal)
        fn.maxVal = duration;

    if (duration < fn.minVal || fn.minVal < 0)
        fn.minVal = duration;

    fn.calls++;    

    float avg_inc = (float)(duration - fn.avgVal)/fn.calls;
    fn.avgVal += avg_inc;
    timers[currentTimer] = fn;
}

void Timer::outputAll()
{
    std::map<std::string, fntimer>::iterator it;
    for (it = timers.begin(); it != timers.end(); ++it)
    {        
        printf("Timer: %s\n", it->first.c_str());
        printf("Min call val: %i\n", it->second.minVal);
        printf("Max call val: %i\n", it->second.maxVal);
        printf("Avg call val: %i\n", it->second.avgVal);
        printf("Num calls   : %i\n", it->second.calls);        
    }
}
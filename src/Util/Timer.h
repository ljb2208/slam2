#pragma once

#include <vector>
#include <chrono>
#include <string>
#include <map>

using namespace std::chrono;

class Timer
{
     // bucketing parameters
    struct fntimer { 
        int32_t calls;  // maximal number of features per bucket 
        int32_t minVal;
        int32_t maxVal;
        int32_t avgVal;
        fntimer() { calls = 0 ; minVal = -1; maxVal = 0; avgVal=0;};
    };

    public:
        void startTimer(std::string timerName);
        void stopTimer();
        void outputAll();

    private:
        std::map<std::string, fntimer> timers;

        high_resolution_clock::time_point tStart;
        high_resolution_clock::time_point tStop;

        std::string currentTimer;


};
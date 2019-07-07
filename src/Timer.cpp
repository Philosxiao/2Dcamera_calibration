#include "Timer.h"

Timer::Timer(std::string _info) {
    t_start = {0,0};
    t_end = {0,0};
    info = _info;
}

Timer::~Timer() {
    
}

void Timer::setInfo(std::string _info) {
    info = _info;
}

void Timer::start() {
    clock_gettime(CLOCK_MONOTONIC, &t_start);
   // std::cout << info << " has start" << std::endl;
}

double Timer::stop() {
    clock_gettime(CLOCK_MONOTONIC, &t_end);
    //std::cout << info << " has stop " << std::endl;
    double deltaT = calcTime(t_start, t_end);
    std::cout << info <<"Cost time: " <<std::endl<<"  "<< deltaT << "ms" << std::endl;
    return deltaT;
}

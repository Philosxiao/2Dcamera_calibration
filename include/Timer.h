#include <time.h>
#include <string>
#include <iostream>

class Timer {
private:
    timespec t_start, t_end;
    std::string info;

public:
    Timer(std::string _info = "");
    ~Timer();
    void setInfo(std::string _info);
    void start();
    double stop();

    static double calcTime(timespec t1, timespec t2) {
        double deltaT = (t2.tv_sec - t1.tv_sec) * 1e6 + (t2.tv_nsec - t1.tv_nsec)/1e3;
        return deltaT / 1e3;
    }
};

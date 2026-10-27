#include "vision_pid.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
void close(double actual,double expected) {assert(std::abs(actual-expected)<1e-9);}
int main() {
    VisionPID pid;
    close(pid.step(1,.1,.3,0,0,0),.3);
    close(pid.step(-1,.1,.3,0,0,0),-.3);
    pid.reset();
    close(pid.step(1,.1,0,1,0,0),.1);
    close(pid.step(1,.1,0,1,0,0),.2);
    pid.reset();
    close(pid.step(1,.1,0,0,.1,0),0);
    close(pid.step(.5,.1,0,0,.1,0),-.5);
    pid.reset();
    close(pid.step(10,.1,1,0,0,.25),.25);
    close(pid.step(-10,.1,1,0,0,.25),-.25);
    close(pid.step(10,.1,1,0,0,0),10);
    std::cout<<"PASS: P/I/D, reset, sign, user speed amplitude and unbounded zero\n";
}

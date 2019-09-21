//
// Created by tb on 19-8-22.
//

#include "gtest/gtest.h"
#include "utils/Time.hpp"

TEST(TimeTest, TimeAndSteadyTime)
{
utils::Time curTime = utils::Time::now();
utils::SteadyTime curSteadyTime = utils::SteadyTime::now();
std::cout << "Current wall clock time: " << curTime
          << " Second: " << curTime.toSec()
          << " Nano second: "<< curTime.toNSec() << std::endl;
std::cout << "Current steay clock time: " << curSteadyTime
          << " Second: " << curSteadyTime.toSec()
          << " Nano second: "<< curSteadyTime.toNSec() << std::endl;

usleep(1000000);    ///sleep 1s
utils::Duration d = utils::Time::now() - curTime;
utils::WallDuration wd = utils::SteadyTime::now() - curSteadyTime;
std::cout << "Duration for Time: " << d << std::endl;
std::cout << "Wall Duration for Steady Time: " << wd << std::endl;


}
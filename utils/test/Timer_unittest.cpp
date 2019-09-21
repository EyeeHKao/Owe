//
// Created by tb on 19-8-21.
//

#include "gtest/gtest.h"
#include "utils/Timer.hpp"

TEST(TimerTest, Timers)
{
    //timer1:
    {
        utils::Timer timer1("local_timer1");
        usleep(1000);
        timer1.Stop();
    }


    //timer2:
    utils::Timer timer2("timer2", false);
    timer2.Start();
    usleep(1000);
    timer2.Stop();
    ASSERT_EQ(false, timer2.IsTiming());

    //timer3:
    utils::Timer timer3("timer3", false);
    timer3.Start();
    usleep(1000);
    timer3.Stop();
    timer3.Start();
    usleep(2000);
    timer3.Stop();
    timer3.Start();
    usleep(3000);
    timer3.Stop();

    //Test Timing to statistic
    ASSERT_EQ(1 ,utils::Timing::GetHandle("timer2"));
    ASSERT_EQ("local_timer1", utils::Timing::GetTag(0));


    //Print all timers
    utils::Timing::Print(std::cout);

    //Print timer2's statistic
    std::cout << "Hz:" <<  utils::Timing::GetHz("timer3") << std::endl;
    std::cout << "Min: " << utils::Timing::GetMinSeconds(2) << std::endl;
    std::cout << "Max: " << utils::Timing::GetMaxSeconds(2) << std::endl;
    std::cout << "Mean: " << utils::Timing::GetMeanSeconds(2) << std::endl;
    std::cout << "Var: " << utils::Timing::GetVarianceSeconds(2) << std::endl;
    std::cout << "Recent: " << utils::Timing::GetRecentSeconds("timer3") << std::endl;
    std::cout << "TotalSecs: " << utils::Timing::GetTotalSeconds(2) << std::endl;
    std::cout << "Samples: " << utils::Timing::GetNumSamples(2) << std::endl;

    utils::Timing::GetHz(10);
    utils::DebugTimer timer("dummytimer");
    timer.Start();

}
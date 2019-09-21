/*
* Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
    * You can contact the author at <slynen at ethz dot ch>
    *
    * Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
    *
    * http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
    * limitations under the License.
*/


/*Modifed:1.system_clock change to steady_clock for a timer func, more accuarcy, because system clock may be change
 *          due to sync with network or other machine
 *        2.assert timing hanlder out of range
 *
 * */

#ifndef STARK_TIMING_HPP
#define STARK_TIMING_HPP


#include <algorithm>
#include <chrono>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace utils
{
/**
 * @brief 累积样本数据统计器，可统计所有历史数据的总和，均值，最大/小值， 指定窗口大小的均值/方差等
 * @param T 样本类型
 * @param Total 样本和类型
 * @param N 窗口大小
 *
 * */

template<typename T, typename Total, int N>
class Accumulator
{
public:
    Accumulator()
        : window_samples_(0),
          totalsamples_(0),
          window_sum_(0),
          sum_(0),
          min_(std::numeric_limits<T>::max()),
          max_(std::numeric_limits<T>::min())
    {
    }

    void Add(T sample)
    {
        recent_ = sample;

        if (window_samples_ < N) {
            samples_[window_samples_++] = sample;
            window_sum_ += sample;
        }
        else {
            T &oldest = samples_[window_samples_++ % N];
            window_sum_ += sample - oldest;
            oldest = sample;
        }
        sum_ += sample;
        ++totalsamples_;
        if (sample > max_) {
            max_ = sample;
        }
        if (sample < min_) {
            min_ = sample;
        }
    }

    int TotalSamples() const
    {
        return totalsamples_;
    }

    double Sum() const
    {
        return sum_;
    }

    double Mean() const
    {
        return sum_ / totalsamples_;
    }

    double RollingMean() const
    {
        return window_sum_ / std::min(window_samples_, N);
    }

    double Max() const
    {
        return max_;
    }

    double Min() const
    {
        return min_;
    }

    double Recent() const
    {
        return recent_;
    }

    double LazyVariance() const
    {
        if (window_samples_ == 0) {
            return 0.0;
        }
        double var = 0;
        double mean = RollingMean();
        for (int i = 0; i < std::min(window_samples_, N); ++i) {
            var += (samples_[i] - mean) * (samples_[i] - mean);
        }
        var /= std::min(window_samples_, N);
        return var;
    }

private:
    int window_samples_;    ///窗口中样本数
    int totalsamples_;      ///总样本数
    Total window_sum_;      ///窗口中样本总和
    Total sum_;             ///样本总和
    T min_;                 ///历史最小
    T max_;                 ///历史最大
    T recent_;              ///最近一次统计值
    T samples_[N];          ///窗口中样本
};

/**
 * @brief 计时器数据统计类，内含实例化的样本数据统计器
 *
 * */
struct TimerMapValue
{
    TimerMapValue()
    {}

    //Create an accumulator with specified window size.
    Accumulator<double, double, 100> acc_;
};


/**
 * @brief A class that has the timer interface but does nothing. Swapping this in in
          place of the Timer class (say with a typedef) should allow one to disable
          timing. Because all of the functions are inline, they should just disappear.
 * */
//
class DummyTimer
{
public:
    DummyTimer(size_t /*handle*/, bool /*constructStopped*/ = false)
    {}
    DummyTimer(std::string const & /*tag*/, bool /*constructStopped*/ = false)
    {}
    ~DummyTimer()
    {}

    void Start()
    {}
    void Stop()
    {}
    bool IsTiming()
    {
        return false;
    }
};

/**
 * @brief 计时器类
 * */
class Timer
{
public:
    Timer(size_t handle, bool constructStopped = false);
    Timer(std::string const &tag, bool constructStopped = false);
    ~Timer();

    void Start();
    void Stop();
    bool IsTiming() const;
private:
    // std::chrono::time_point<std::chrono::system_clock> time_;    ///system_clock's epoch point is 1970.1.1 00:00,and maybe change when change system utils or sync with machine/network utils
    std::chrono::steady_clock::time_point time_;    ///steady_clock's epoch point is the system startup time and never don't change
    bool timing_;   ///if timing or not
    size_t handle_;
};




/// Singleton design pattern
class Timing
{
public:
    typedef std::map<std::string, size_t> map_t;    ///计时器类目-编号对
    friend class Timer;
    // Definition of static functions to query the timers.
    static size_t GetHandle(std::string const &tag);
    static std::string GetTag(size_t handle);
    static double GetTotalSeconds(size_t handle);
    static double GetTotalSeconds(std::string const &tag);
    static double GetMeanSeconds(size_t handle);
    static double GetMeanSeconds(std::string const &tag);
    static size_t GetNumSamples(size_t handle);
    static size_t GetNumSamples(std::string const &tag);
    static double GetVarianceSeconds(size_t handle);
    static double GetVarianceSeconds(std::string const &tag);
    static double GetMinSeconds(size_t handle);
    static double GetMinSeconds(std::string const &tag);
    static double GetMaxSeconds(size_t handle);
    static double GetMaxSeconds(std::string const &tag);
    static double GetRecentSeconds(size_t handle);
    static double GetRecentSeconds(std::string const &tag);
    static double GetHz(size_t handle);
    static double GetHz(std::string const &tag);
    static void Print(std::ostream &out);
    static std::string Print();
    static std::string SecondsToTimeString(double seconds);
    static void Reset();
    static const map_t &GetTimers()
    {
        return Instance().tagMap_;
    }
private:
    void AddTime(size_t handle, double seconds);

    static Timing &Instance();

    //设置为私有， 单例设计——程序任何地方总是获取的相同的实例对象
    Timing();
    ~Timing();

    typedef std::vector<TimerMapValue> list_t;  ///一组带数据统计功能的计时器

    list_t timers_;
    map_t tagMap_;
    size_t maxTagLength_;
};


#ifdef ENABLE_TIMER     ///ENABLE_TIMER是在CMakelist.txt中定义的预处理变量
typedef Timer DebugTimer;
#else
typedef DummyTimer DebugTimer;
#endif


}///end of namespace utils

#endif //STARK_TIMING_HPP

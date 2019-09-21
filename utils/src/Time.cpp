/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#ifdef _MSC_VER
#ifndef NOMINMAX
    #define NOMINMAX
  #endif
#endif

#include "utils/Time.hpp"
#include <cmath>
#include <ctime>
#include <iomanip>
#include <limits>
#include <stdexcept>

// time related includes for macOS
#if defined(__APPLE__)
#include <mach/clock.h>
#include <mach/mach.h>
#endif  // defined(__APPLE__)

#ifdef _WINDOWS
#include <chrono>
#include <thread>
#include <windows.h>
#endif

/*********************************************************************
 ** Preprocessor
 *********************************************************************/

// Could probably do some better and more elaborate checking
// and definition here.
#define HAS_CLOCK_GETTIME (_POSIX_C_SOURCE >= 199309L)

/*********************************************************************
 ** Namespaces
 *********************************************************************/

namespace utils
{

/*********************************************************************
 ** Variables
 *********************************************************************/

const Duration DURATION_MAX(std::numeric_limits<int32_t>::max(), 999999999);
const Duration DURATION_MIN(std::numeric_limits<int32_t>::min(), 0);

const Time TIME_MAX(std::numeric_limits<uint32_t>::max(), 999999999);
const Time TIME_MIN(0, 1);

/*********************************************************************
 ** Cross Platform Functions
 *********************************************************************/
///Ps:CST（Central Standard Time),UTC(Universal Time Coordinated)约等于GMT(Greenwich Mean Time)
///CST可表示中国（UTC+8），美国（UTC-6），澳大利亚和古巴
///壁钟时间：从1970.1.1 00:00(UTC/GMT）起到当前系统时间，因为当前系统时间可能被同步更正，所以这个时间不是很准，而ros::Time默认使用的是这个时间
void walltime(uint32_t& sec, uint32_t& nsec)
{
#if !defined(_WIN32)    ///非win32平台
#if HAS_CLOCK_GETTIME   ///如果有clock_gettime,则使用clock_gettime
    timespec start;
    clock_gettime(CLOCK_REALTIME, &start);  ///CLOCK_REALTIME表示UTC时间
    if (start.tv_sec < 0 || start.tv_sec > std::numeric_limits<uint32_t>::max())
        throw std::runtime_error("Timespec is out of dual 32-bit range");
    sec  = start.tv_sec;
    nsec = start.tv_nsec;
#else   ///否则使用gettimeofday
    // std::cout << "wall_TIME" << std::endl;

    struct timeval timeofday;
    gettimeofday(&timeofday,NULL);  ///epoch:UNIX system time---1970.1.1 00:00（UTC）
    if (timeofday.tv_sec < 0 || timeofday.tv_sec > std::numeric_limits<uint32_t>::max())
      throw std::runtime_error("Timeofday is out of dual signed 32-bit range");
    sec  = timeofday.tv_sec;
    nsec = timeofday.tv_usec * 1000;
#endif
#else
    // Win32 implementation
    // unless I've missed something obvious, the only way to get high-precision
    // time on Windows is via the QueryPerformanceCounter() call. However,
    // this is somewhat problematic in Windows XP on some processors, especially
    // AMD, because the Windows implementation can freak out when the CPU clocks
    // down to save power. Time can jump or even go backwards. Microsoft has
    // fixed this bug for most systems now, but it can still show up if you have
    // not installed the latest CPU drivers (an oxymoron). They fixed all these
    // problems in Windows Vista, and this API is by far the most accurate that
    // I know of in Windows, so I'll use it here despite all these caveats
    static LARGE_INTEGER cpu_freq, init_cpu_time;
    static uint32_t start_sec = 0;
    static uint32_t start_nsec = 0;
    if ( ( start_sec == 0 ) && ( start_nsec == 0 ) )
      {
        QueryPerformanceFrequency(&cpu_freq);
        if (cpu_freq.QuadPart == 0) {
          throw NoHighPerformanceTimersException();
        }
        QueryPerformanceCounter(&init_cpu_time);
        // compute an offset from the Epoch using the lower-performance timer API
        FILETIME ft;
        GetSystemTimeAsFileTime(&ft);
        LARGE_INTEGER start_li;
        start_li.LowPart = ft.dwLowDateTime;
        start_li.HighPart = ft.dwHighDateTime;
        // why did they choose 1601 as the time zero, instead of 1970?
        // there were no outstanding hard rock bands in 1601.
#ifdef _MSC_VER
    	start_li.QuadPart -= 116444736000000000Ui64;
#else
    	start_li.QuadPart -= 116444736000000000ULL;
#endif
        int64_t start_sec64 = start_li.QuadPart / 10000000;  // 100-ns units
        if (start_sec64 < 0 || start_sec64 > std::numeric_limits<uint32_t>::max())
          throw std::runtime_error("SystemTime is out of dual 32-bit range");
        start_sec = (uint32_t)start_sec64;
        start_nsec = (start_li.LowPart % 10000000) * 100;
      }
    LARGE_INTEGER cur_time;
    QueryPerformanceCounter(&cur_time);
    LARGE_INTEGER delta_cpu_time;
    delta_cpu_time.QuadPart = cur_time.QuadPart - init_cpu_time.QuadPart;
    // todo: how to handle cpu clock drift. not sure it's a big deal for us.
    // also, think about clock wraparound. seems extremely unlikey, but possible
    double d_delta_cpu_time = delta_cpu_time.QuadPart / (double) cpu_freq.QuadPart;
    uint32_t delta_sec = (uint32_t) floor(d_delta_cpu_time);
    uint32_t delta_nsec = (uint32_t) boost::math::round((d_delta_cpu_time-delta_sec) * 1e9);

    int64_t sec_sum  = (int64_t)start_sec  + (int64_t)delta_sec;
    int64_t nsec_sum = (int64_t)start_nsec + (int64_t)delta_nsec;

    // Throws an exception if we go out of 32-bit range
    normalizeSecNSecUnsigned(sec_sum, nsec_sum);

    sec = sec_sum;
    nsec = nsec_sum;
#endif
}

//steady clock: 从系统启动时算起，根据cpu的tick数稳步向前递增，不随系统时间变化， 可用作精确时间戳
void steadytime(uint32_t& sec, uint32_t& nsec)
{
#if !defined(_WIN32)   ///非win32平台
    timespec start;
#if defined(__APPLE__)  ///Mac平台
    // On macOS use clock_get_time.
    clock_serv_t cclock;
    mach_timespec_t mts;
    host_get_clock_service(mach_host_self(), SYSTEM_CLOCK, &cclock);
    clock_get_time(cclock, &mts);
    mach_port_deallocate(mach_task_self(), cclock);
    start.tv_sec = mts.tv_sec;
    start.tv_nsec = mts.tv_nsec;
#else  // defined(__APPLE__)    ///非Mac平台
    // Otherwise use clock_gettime.
    // std::cout << "steady_TIME" << std::endl;

    clock_gettime(CLOCK_MONOTONIC, &start); ///epoch 从系统启动时间开始算起，CLOCK_MONOTONIC表示稳定递增
#endif  // defined(__APPLE__)
    sec  = start.tv_sec;
    nsec = start.tv_nsec;
#else   ///win32平台上的
    static LARGE_INTEGER cpu_frequency, performance_count;
    // These should not ever fail since XP is already end of life:
    // From https://msdn.microsoft.com/en-us/library/windows/desktop/ms644905(v=vs.85).aspx and
    //      https://msdn.microsoft.com/en-us/library/windows/desktop/ms644904(v=vs.85).aspx:
    // "On systems that run Windows XP or later, the function will always succeed and will
    //  thus never return zero."
    QueryPerformanceFrequency(&cpu_frequency);
    if (cpu_frequency.QuadPart == 0) {
      throw NoHighPerformanceTimersException();
    }
    QueryPerformanceCounter(&performance_count);
    double steady_time = performance_count.QuadPart / (double) cpu_frequency.QuadPart;
    int64_t steady_sec = floor(steady_time);
    int64_t steady_nsec = boost::math::round((steady_time - steady_sec) * 1e9);

    // Throws an exception if we go out of 32-bit range
    normalizeSecNSecUnsigned(steady_sec, steady_nsec);

    sec = steady_sec;
    nsec = steady_nsec;
#endif
}

/*
 * These have only internal linkage to this translation unit.
 * (i.e. not exposed to users of the time classes)
 */

/**
 * @brief Simple representation of the rt library nanosleep function.
 */
int utils_nanosleep(const uint32_t &sec, const uint32_t &nsec)
{
#if defined(_WIN32)
    std::this_thread::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(sec * 1e9 + nsec)));
    return 0;
#else
    timespec req = { sec, nsec };
    return nanosleep(&req, NULL);
#endif
}

/**
 * @brief Go to the wall!
 *
 * @todo Fully implement the win32 parts, currently just like a regular sleep.
 */
bool wallsleep(uint32_t sec, uint32_t nsec)
{
#if defined(_WIN32)
    utils_nanosleep(sec,nsec);
#else
    timespec req = { sec, nsec };
    timespec rem = {0, 0};
    while (nanosleep(&req, &rem))
    {
        req = rem;
    }
#endif
    return true;
}

/*********************************************************************
 ** Class Methods
 *********************************************************************/

std::ostream& operator<<(std::ostream& os, const Duration& rhs)
{
    if (rhs.sec >= 0 || rhs.nsec == 0)
    {
        os << rhs.sec << "." << std::setw(9) << std::setfill('0') << rhs.nsec;
    }
    else
    {
        os << (rhs.sec == -1 ? "-" : "") << (rhs.sec + 1) << "." << std::setw(9) << std::setfill('0') << (1000000000 - rhs.nsec);
    }
    return os;
}


bool Time::sleepUntil(const Time& end)
{
    Duration d(end - Time::now());
    if (d > Duration(0))
    {
        return d.sleep();
    }

    return true;
}

bool SteadyTime::sleepUntil(const SteadyTime& end)
{
    WallDuration d(end - SteadyTime::now());
    if (d > WallDuration(0))
    {
        return d.sleep();
    }

    return true;
}

bool Duration::sleep() const
{
    return wallsleep(sec, nsec);
}

std::ostream &operator<<(std::ostream& os, const Time &rhs)
{
    os << rhs.sec << "." << std::setw(9) << std::setfill('0') << rhs.nsec;
    return os;
}

std::ostream &operator<<(std::ostream& os, const SteadyTime &rhs)
{
    os << rhs.sec << "." << std::setw(9) << std::setfill('0') << rhs.nsec;
    return os;
}

Time Time::now()
{
    Time t;
    walltime(t.sec, t.nsec);

    return t;
}

SteadyTime SteadyTime::now()
{
    SteadyTime t;
    steadytime(t.sec, t.nsec);

    return t;
}

std::ostream &operator<<(std::ostream& os, const WallDuration& rhs)
{
    if (rhs.sec >= 0 || rhs.nsec == 0)
    {
        os << rhs.sec << "." << std::setw(9) << std::setfill('0') << rhs.nsec;
    }
    else
    {
        os << (rhs.sec == -1 ? "-" : "") << (rhs.sec + 1) << "." << std::setw(9) << std::setfill('0') << (1000000000 - rhs.nsec);
    }
    return os;
}

bool WallDuration::sleep() const
{
    return wallsleep(sec, nsec);
}

void normalizeSecNSec(uint64_t& sec, uint64_t& nsec)
{
    uint64_t nsec_part = nsec % 1000000000UL;
    uint64_t sec_part = nsec / 1000000000UL;

    if (sec + sec_part > std::numeric_limits<uint32_t>::max())
        throw std::runtime_error("Time is out of dual 32-bit range");

    sec += sec_part;
    nsec = nsec_part;
}

void normalizeSecNSec(uint32_t& sec, uint32_t& nsec)
{
    uint64_t sec64 = sec;
    uint64_t nsec64 = nsec;

    normalizeSecNSec(sec64, nsec64);

    sec = (uint32_t)sec64;
    nsec = (uint32_t)nsec64;
}

void normalizeSecNSecUnsigned(int64_t& sec, int64_t& nsec)
{
    int64_t nsec_part = nsec % 1000000000L;
    int64_t sec_part = sec + nsec / 1000000000L;
    if (nsec_part < 0)
    {
        nsec_part += 1000000000L;
        --sec_part;
    }

    if (sec_part < 0 || sec_part > std::numeric_limits<uint32_t>::max())
        throw std::runtime_error("Time is out of dual 32-bit range");

    sec = sec_part;
    nsec = nsec_part;
}

template class TimeBase<Time, Duration>;
template class TimeBase<SteadyTime, WallDuration>;
}
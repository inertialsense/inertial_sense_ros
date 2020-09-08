#pragma once

#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <array>
#include <iostream>
#include <thread>
#include <memory>
#include <mutex>
#include "hard_real_time_box.h"

static long diff_in_us(struct timespec t1, struct timespec t2)
{
    struct timespec diff;
    if (t2.tv_nsec - t1.tv_nsec < 0)
    {
        diff.tv_sec = t2.tv_sec - t1.tv_sec - 1;
        diff.tv_nsec = t2.tv_nsec - t1.tv_nsec + 1000000000;
    }
    else
    {
        diff.tv_sec = t2.tv_sec - t1.tv_sec;
        diff.tv_nsec = t2.tv_nsec - t1.tv_nsec;
    }
    return (diff.tv_sec * 1000000.0 + diff.tv_nsec / 1000.0);
}

struct RTStatistics
{
    timespec first, second;
    bool countperiod = true;
    long usecdiff = -1;
    long i = 0;
    bool initializating = true;

    const int TIMES = 30000; // 1 shot per minute
    const int SIGMA = 20;

    const int DEADLINE_us; // 1 shot per minute

    RTStatistics(int deadline_us = 10000, int LOG_UPDATE_COUNTER = 30000)
        : DEADLINE_us(deadline_us), TIMES(LOG_UPDATE_COUNTER)
    {
    }

    struct StatsData
    {
        double maxperiod = -1;
        long countSigma5OverAverage = 0;
        long countOutDeadline = 0;
        double avg = 0;
        double savedAverageUs = 2000;
        double savedStDevUs = 5;
        double devUs = 0;

        long savedmaxperiod = -1;
        long savedmaxudev = -1;
        long totalMissDeadlineCount = 0;
    };

    StatsData statdata;
    HardRealTimeDataBox<StatsData> rtbox;

    void init()
    {
        ROS_INFO("precall");
        clock_gettime(CLOCK_MONOTONIC, &first);
        ROS_INFO("postcall");
        second = first;
    }

    void preupdate()
    {
        clock_gettime(CLOCK_MONOTONIC, &first);
        usecdiff = diff_in_us(second, first);
    }

    void tryPrint()
    {
        StatsData nrtdata;
        if (this->rtbox.readFromNonRealTime(nrtdata))
        {
            std::cout << "------------------" << std::endl;
            std::cout << "[AVG RT dt] " << nrtdata.avg << " us" << std::endl;
            std::cout << "[STDEV]" << nrtdata.devUs << "us" << std::endl;
            std::cout << "[" << SIGMA << "sigma outliers (" << SIGMA * nrtdata.devUs << "us)]" << nrtdata.countSigma5OverAverage << " (" << 100.0 * (double)nrtdata.countSigma5OverAverage / (double)TIMES << "%)" << std::endl;
            std::cout << "[MAX RT dt] " << nrtdata.maxperiod << " us" << std::endl;
            std::cout << "[MISS DEADLINE (" << DEADLINE_us << "us)] " << nrtdata.countOutDeadline << std::endl;
            std::cout << "[ABS MAX RT dt] " << nrtdata.savedmaxperiod << " us" << std::endl;
            std::cout << "[ABS MAX STDEV] " << nrtdata.savedmaxudev << " us" << std::endl;
            std::cout << "[ABS MISS DEADLINE (" << DEADLINE_us << "us)] " << nrtdata.totalMissDeadlineCount << std::endl;
        }
    }

    void update()
    {
        if (i > TIMES * 0.1) /*ignore stabilized startup*/
            initializating = false;

        if (countperiod && !initializating)
        {
            if (usecdiff > statdata.maxperiod)
            {
                statdata.maxperiod = usecdiff;
            }

            statdata.avg += usecdiff;
            auto dev = fabs(statdata.savedAverageUs - usecdiff);
            statdata.devUs += dev;

            if (dev > SIGMA * statdata.savedStDevUs)
            {
                statdata.countSigma5OverAverage++;
            }

            if (usecdiff > DEADLINE_us)
            {
                statdata.countOutDeadline++;
            }
        }

        i++;
        second = first;

        // updated only once each x iterations
        if (i % TIMES == 0)
        {
            statdata.avg /= TIMES; // times
            statdata.devUs /= TIMES;

            if (statdata.maxperiod > statdata.savedmaxperiod)
                statdata.savedmaxperiod = statdata.maxperiod;

            if (statdata.savedStDevUs > statdata.savedmaxudev)
                statdata.savedmaxudev = statdata.savedStDevUs;

            statdata.totalMissDeadlineCount += statdata.countOutDeadline;

            rtbox.writeFromRT(statdata);

            statdata.savedStDevUs = statdata.devUs;
            statdata.countOutDeadline = 0;

            statdata.maxperiod = 0;

            statdata.avg = 0;

            statdata.devUs = 0;
            statdata.countSigma5OverAverage = 0;

            countperiod = false;
        }
        else
        {
            countperiod = true;
        }
    }
};
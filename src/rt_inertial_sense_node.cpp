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

#include "inertial_sense.h"
#include "rt_stats.h"

struct period_info
{
        struct timespec next_period;
        long period_ns;
};

std::shared_ptr<InertialSenseROS> thing;
RTStatistics rtstats;
std::atomic<bool> rtexit;

static void inc_period(struct period_info *pinfo);
static void periodic_task_init(struct period_info *pinfo);
static void wait_rest_of_period(struct period_info *pinfo);
static void do_rt_task();
static void alignToFrameStart();
static void *main_rt_loop(void *data);
int create_realtime_thread();
void main_non_rt_loop();

const double DESIRED_RT_LOOP_TIME_NS = 2000000; // 2ms
const double NON_RT_LOOP_FREQ = 500;

int main(int argc, char *argv[])
{
        ros::init(argc, argv, "inertial_sense_node");

        thing = std::make_shared<InertialSenseROS>();
        thing->IS_.SetLoggerEnabled(false);
        rtstats.init();

        int ret = create_realtime_thread();

        if (ret == 0)
        {
                main_non_rt_loop();
        }

        return ret;
}

void main_non_rt_loop()
{

        //pthread_join(rtThread, NULL);
        ROS_INFO("[inertial_sense_node] Starting ros loop");
        {
                ros::Rate r(NON_RT_LOOP_FREQ);
                while (ros::ok())
                {
                        ros::spinOnce();
                        thing->nonRTUpdate();
                        rtstats.tryPrint();
                        r.sleep();
                }

                rtexit = true; // exits rt thread
                ROS_INFO("[inertial_sense_node] Exiting application");
        }
}

void *main_rt_loop(void *data)
{
        ROS_INFO("[inertial_sense_node] RT THREAD");
        struct period_info pinfo;
        pinfo.period_ns = DESIRED_RT_LOOP_TIME_NS; // us

        periodic_task_init(&pinfo);

        ROS_INFO("[inertial_sense_node] rtstats post sinit");
        while (!rtexit)
        {
                rtstats.preupdate();
                thing->rtUpdate();
                wait_rest_of_period(&pinfo);
                rtstats.update();
        }

        return NULL;
}

int create_realtime_thread()
{
        struct sched_param param;
        pthread_attr_t attr;
        pthread_t rtThread;
        int ret = 0;

        ROS_INFO("[inertial_sense_node] Configuring real time thread");

        /* Lock memory */
        if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
        {
                printf("mlockall failed: %m\n");
                exit(-2);
        }

        /* Initialize pthread attributes (default values) */
        ret = pthread_attr_init(&attr);
        if (ret)
        {
                printf("init pthread attributes failed\n");
                goto out;
        }

        /* Set a specific stack size  */
        ret = pthread_attr_setstacksize(&attr, 100 * PTHREAD_STACK_MIN);
        if (ret)
        {
                printf("pthread setstacksize failed\n");
                goto out;
        }

        /* Set scheduler policy and priority of pthread */
        ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
        //ret = pthread_attr_setschedpolicy(&attr, SCHED_OTHER);

        if (ret)
        {
                printf("pthread setschedpolicy failed\n");
                goto out;
        }
        param.sched_priority = 99;
        //param.sched_priority = 0;
        ret = pthread_attr_setschedparam(&attr, &param);
        if (ret)
        {
                printf("pthread setschedparam failed\n");
                goto out;
        }
        /* Use scheduling parameters of attr */
        ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
        if (ret)
        {
                printf("pthread setinheritsched failed\n");
                goto out;
        }

        ROS_INFO("[inertial_sense_node] Creating real time thread");

        /* Create a pthread with specified attributes */
        ret = pthread_create(&rtThread, &attr, main_rt_loop, NULL);
        if (ret)
        {
                printf("create pthread failed\n");
                goto out;
        }

out:
        return ret;
}

static void inc_period(struct period_info *pinfo)
{
        pinfo->next_period.tv_nsec += pinfo->period_ns;

        while (pinfo->next_period.tv_nsec >= 1000000000)
        {
                /* timespec nsec overflow */
                pinfo->next_period.tv_sec++;
                pinfo->next_period.tv_nsec -= 1000000000;
        }
}

static void periodic_task_init(struct period_info *pinfo)
{
        /* for simplicity, hardcoding a 1ms period */
        pinfo->period_ns = DESIRED_RT_LOOP_TIME_NS;

        clock_gettime(CLOCK_MONOTONIC, &(pinfo->next_period));
}
static void wait_rest_of_period(struct period_info *pinfo)
{
        inc_period(pinfo);

        /* for simplicity, ignoring possibilities of signal wakes */
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, NULL);
}

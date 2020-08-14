#include "inertial_sense.h"
#include "rt_stats.h"

RTStatistics rtstats;
const double NON_RT_LOOP_FREQ = 500;

int main(int argc, char**argv)
{
  ros::init(argc, argv, "inertial_sense_node");
  InertialSenseROS thing;
  rtstats.init();
  ros::Rate r(NON_RT_LOOP_FREQ);
  while (ros::ok())
  {
    rtstats.preupdate();
    ros::spinOnce();
    thing.rtUpdate();
    thing.nonRTUpdate();
    rtstats.tryPrint();
    r.sleep();
    rtstats.update();
  }
  return 0;
}
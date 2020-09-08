#pragma once
#include <boost/lockfree/spsc_queue.hpp>

template <typename T>
class RTInputPort
{
    public:
       bool readFromRealTime(T& data)
       {
           return spsc_queue.pop(data);
       }

       void writeFromNonRealTime(const T& data)
       {
           spsc_queue.push(data);
       }

    private:
        boost::lockfree::spsc_queue<T, boost::lockfree::capacity<20> > spsc_queue;
};
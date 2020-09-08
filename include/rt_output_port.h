#pragma once
#include <boost/lockfree/spsc_queue.hpp>

template <typename T>
class RTOutputPort
{
    public:
       bool readFromNonRealTime(T& data)
       {
           return spsc_queue.pop(data);
       }

       void writeFromRealTime(const T& data)
       {
           spsc_queue.push(data);
       }
    
    private:
        boost::lockfree::spsc_queue<T, boost::lockfree::capacity<20> > spsc_queue;
};
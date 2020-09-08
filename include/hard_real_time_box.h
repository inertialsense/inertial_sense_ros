#pragma once

#include <boost/lockfree/spsc_queue.hpp>

template <typename T>
class HardRealTimeDataBox
{
public:
    HardRealTimeDataBox() 
    {

    }

    /*Returns true if databox data was successfully updated*/
    bool writeFromRT(const T &value)
    {
        data_ = value;
        auto res = spsc_queue.push(data_);
        availableDataRT_=true;
        return res;
    }

    /*Returns true if some new data was available*/
    bool readFromNonRealTime(T &outvalue)
    {
        auto res = spsc_queue.pop(lastNonRealTimeData_);
        outvalue = lastNonRealTimeData_;
        return res;
    }

    // can be safely executed if is used in the same NonRT thread than readFromNonRealTime
    inline T &getLastNonRTData() 
    { 
       return lastNonRealTimeData_; 
    }

    inline T& readFromRT()
    {
        availableDataRT_=false;
        return data_;
    }

    std::atomic<bool> availableDataRT_;

    private:
        T lastNonRealTimeData_;
        T data_;
        boost::lockfree::spsc_queue<T, boost::lockfree::capacity<20> > spsc_queue;
        
};
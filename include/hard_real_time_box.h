#pragma once

#include <mutex>
#include <thread>

template <typename T>
class HardRealTimeDataBox
{
public:
    HardRealTimeDataBox() 
      : dirty_(false)
    {
        // configure mutex priority inheritance
        pthread_mutexattr_t mta;
        pthread_mutexattr_init(&mta);
        pthread_mutexattr_setprotocol(&mta, PTHREAD_PRIO_INHERIT);
        pthread_mutex_init(&mutex_, &mta);
    }

    /*Returns true if databox data was successfully updated*/
    bool writeFromRT(const T &value)
    {
        if (pthread_mutex_trylock(&mutex_) == 0)
        {
            lastRTWriteData_ = value;
            dirty_ = true;
            pthread_mutex_unlock(&mutex_);
            
            return true;
        }
        else
        {
            return false;
        }
    }

    /*Returns true if some new data was available*/
    bool readFromNonRealTime(T &outvalue)
    {
        if (dirty_)
        {
            {
                pthread_mutex_lock(&mutex_);
                dirty_ = false;
                lastData_ = lastRTWriteData_;
                pthread_mutex_unlock(&mutex_);
            }
            outvalue = lastData_;
            return true;
        }
        else
        {
            outvalue = lastData_;
            return false;
        }
    }

    // can be safely executed if is used in the same NonRT thread than readFromNonRealTime
    inline T &getLastNonRTData() 
    { 
       return lastData_; 
    }

private:
    // updated from non-realtime thread when on "readFromNonRealTime"
    T lastData_;

    // storage for realtime data, only updated from the RT-Thread
    T lastRTWriteData_;

    // this variable balance the reads/writes, 
    // only locking if a new RT value is available
    std::atomic<bool> dirty_;

    pthread_mutex_t mutex_;
};
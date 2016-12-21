#ifndef SENSORFLOW_H
#define SENSORFLOW_H

#include "imu/imu.h"

#include <list>

typedef std::list<Imu> container_type;
typedef container_type::iterator iterator;
typedef container_type::const_iterator const_iterator;

class SensorFlow {
public:
    iterator begin();
    iterator end();
    const_iterator begin() const;
    const_iterator end() const;
    
    Imu& front();
    const Imu& front() const;
    Imu& back();
    const Imu& back() const;
    
    bool empty() const;
    std::size_t size() const;
    
    void push_back(const Imu& item);
    void push_back(Imu&& item);
    
    void truncateToMinimalInterpolationTime(double time);
    
    static Imu interpolate(double time, const Imu& earlier, const Imu& later);
    static Imu interpolateAnyTime(double time, iterator hint);
    static void shiftToInterpolationInterval(double time, iterator& it);
private:
    container_type buffer_;
};

#endif //SENSORFLOW_H
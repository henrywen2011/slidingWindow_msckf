#include "imu/sensorflow.h"

SensorFlow::iterator SensorFlow::begin() {
    return std::begin(buffer_);
}

SensorFlow::iterator SensorFlow::end() {
    return std::end(buffer_);
}

SensorFlow::const_iterator SensorFlow::begin() const {
    return std::begin(buffer_);
}

SensorFlow::const_iterator SensorFlow::end() const {
    return std::end(buffer_);
}

Imu& SensorFlow::front() {
    return buffer_.front();
}

const Imu& SensorFlow::front() const {
    return buffer_.front();
}

Imu& SensorFlow::back() {
    return buffer_.back();
}

const Imu& SensorFlow::back() const {
    return buffer_.back();
}

bool SensorFlow::empty() const {
    return buffer_.empty();
}

std::size_t SensorFlow::size() const {
    return buffer_.size();
}

void SensorFlow::push_back(const Imu &item) {
    buffer_.push_back(item);
}

void SensorFlow::push_back(Imu&& item) {
    buffer_.push_back(std::move(item));
}

void SensorFlow::truncateToMinimalInterpolationTime(double time) {
    while (std::next(std::begin(buffer_))->getTime() < time) {
        buffer_.pop_front();
    }
}

Imu SensorFlow::interpolate(double time, const Imu &earlier, const Imu &later) {
    double time_delta = later.getTime() - earlier.getTime();
    double t = (time - earlier.getTime()) / time_delta;
    ImuDevice device = later.getDevice();
    Eigen::Vector3d data = t * earlier.getVector() + (1-t) * later.getVector();
    Imu interpolated = Imu::fromVector3d(time, device, data);
    
    assert(interpolated.getVector().norm() > 1e-100);
    assert(interpolated.getVector().norm() < 1e100);
    
    return interpolated;
}

Imu SensorFlow::interpolateAnyTime(double time, SensorFlow::iterator hint) {
    // Hint might be ok.
    double hint_time = hint->getTime();
    if (time == hint_time) {
        return *hint;
    }
    
    SensorFlow::iterator it = hint;
    shiftToInterpolationInterval(time, it);
    return interpolate(time, *it, *std::next(it));
}

void SensorFlow::shiftToInterpolationInterval(double time, SensorFlow::iterator &it) {
    int diff = it->getTime() < time ? 1 : -1;
    while (it->getTime() > time || time >= std::next(it)->getTime()) {
        std::advance(it, diff);
    }
}
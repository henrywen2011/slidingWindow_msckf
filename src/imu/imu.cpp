#include "imu/imu.h"
#include "utils/calibration.h"

#include "exceptions/general_exception.h"
#include "exceptions/impossible_exception.h"

Imu Imu::fromVector3d(double time, const ImuDevice& device, const Eigen::Vector3d &data) {
    assert(!std::isnan(data.maxCoeff()));
    Imu imu_;
    imu_.time_ = time;
    imu_.device_ = device;
    imu_.data_ = data;
    return imu_;
}

ImuDevice Imu::getDevice() const {
    return device_;
}

double Imu::getTime() const {
    return time_;
}

double Imu::getX() const {
    return data_(0, 0);
}

double Imu::getY() const {
    return data_(1, 0);
}

double Imu::getZ() const {
    return data_(2, 0);
}

Eigen::Vector3d Imu::getVector() const {
    return data_;
}

std::ostream& operator<< (std::ostream& out, const ImuDevice& device) {
    switch (device) {
        case ImuDevice::ACCELEROMETER:
            out << "IMU::Accelerometer";
            break;
        case ImuDevice::GYROSCOPE:
            out << "IMU::Gyroscope";
            break;
        default:
            throw ImpossibleException("Unknown ImuDevice");
    }
    return out;
}
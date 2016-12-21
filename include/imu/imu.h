#ifndef IMU_H
#define IMU_H

#include <string>
#include <Eigen/Core>

enum class ImuDevice {
    ACCELEROMETER, GYROSCOPE
};

class Imu {
public:
    
    static Imu fromVector3d(double time, const ImuDevice& device, const Eigen::Vector3d& data);

    ImuDevice getDevice() const;

    double getTime() const;

    double getX() const;
    double getY() const;
    double getZ() const;
    Eigen::Vector3d getVector() const;
    
    std::ostream& operator<<(std::ostream& out, const ImuDevice& device);
private:
    ImuDevice device_;
    double time_;
    Eigen::Vector3d data_;
};

#endif //IMU_H
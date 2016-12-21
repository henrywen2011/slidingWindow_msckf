#ifndef JPLQUATERNION_H
#define JPLQUATERNION_H

#include <Eigen/Core>

/**
 * @brief JPL Quaternion, see 'Indirect Kalman Filter for 3D Attitude Estimation A Tutorial for Quaternion Algebra', University of Minnesota.
 *
 * @author Doom
 */
class Quaternion {
public:
    Quaternion(double x, double y, double z, double w);
    Quaternion(const Quaternion& other);
    
    Quaternion& operator=(const Quaternion& other);
    Quaternion operator*(const Quaternion& rhs) const;
    
    double norm() const;
    void normalize();
    Quaternion normalized() const;
    bool isUnitQuaternion(double eps = 1e-12) const;
    
    Quaternion conjugate() const;
    Eigen::Matrix3d toRotationMatrix() const;
    
    double x() const;
    double y() const;
    double z() const;
    double w() const;
    Eigen::Vector3d vec() const;
    Eigen::Vector4d coeffs() const;
    
    bool isApprox(const Quaternion& other, double eps = 1e-12) const;
    
    static Quaternion identity();
    static Quaternion fromRotationMatrix(const Eigen::Matrix3d& m);
    
    static Eigen::Matrix3d crossMatrix(const Eigen::Vector3d& v);
    static Eigen::Matrix4d bigOmegaMatrix(const Eigen::Vector3d& v);
private:
    double x_;
    double y_;
    double z_;
    double w_;
};

#endif //JPLQUATERNION_H
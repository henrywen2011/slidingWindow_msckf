#ifndef RESIDUAL_H
#define RESIDUAL_H

#include <Eigen/Core>

/**
 * @brief The residual function used in update process.
 */

class Residual {
public:
    Residual(std::size_t track_length, std::size_t poses_in_state);
    
    operator bool() const;
    
    void setIsInvalid();
    
    /** @brief \f$r_{i,j}\f$ */
    void setPoseResidual(std::size_t pose, const Eigen::Vector2d& rezidual);
    
    /** @brief \f$H_{c_{i,j}}\f$ */
    void setJacobianByCameraParameters(std::size_t pose, const Eigen::Matrix<double, 2, 14>& jacobian);
    
    /** @brief \f$H_{x_{i,B_j}}\f$ */
    void setJacobianByCameraPose(std::size_t pose, const Eigen::Matrix<double, 2, 9>& jacobian);
    
    /** @brief \f$H_{f_{i,j}}\f$ */
    void setJacobianByFeaturePosition(std::size_t pose, const Eigen::Matrix<double, 2, 3>& jacobian);
    
    Eigen::Vector3d getGlobalFeaturePosition() const;
    void setGlobalFeaturePosition(const Eigen::Vector3d& global_position);
    
    Eigen::VectorXd getResiduals() const;
    Eigen::MatrixXd getJacobianByState() const;
    Eigen::Matrix<double, Eigen::Dynamic, 3> getJacobianByFeaturePosition() const;
    
private:
    bool is_valid_;
    std::size_t track_length_;
    std::size_t poses_in_state_;
    Eigen::Vector3d global_position_;
    Eigen::VectorXd r_;
    Eigen::MatrixXd H_x_;
    Eigen::Matrix<double, Eigen::Dynamic, 3> H_f_;
};

#endif //RESIDUAL_H
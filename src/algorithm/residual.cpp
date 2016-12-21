#include "algorithm/residual.h"

Residual::Residual(
        std::size_t track_length, std::size_t poses_in_state)
: track_length_(track_length), poses_in_state_(poses_in_state),
    r_(Eigen::VectorXd::Zero(2*track_length)),
    H_x_(Eigen::MatrixXd::Zero(2*track_length, 56+9*poses_in_state)),
    H_f_(Eigen::MatrixXd::Zero(2*track_length, 3)) {
    is_valid_ = true;
}

Residual::operator bool() const {
    return is_valid_;
}

void Residual::setIsInvalid() {
    is_valid_ = false;
}

void Residual::setPoseResidual(std::size_t pose, const Eigen::Vector2d& rezidual) {
    r_.segment<2>(2*pose) = rezidual;
}

void Residual::setJacobianByCameraParameters(
        std::size_t pose, const Eigen::Matrix<double, 2, 14>& jacobian) {
    H_x_.block<2, 14>(2*pose, 15+27) = jacobian;
}

void Residual::setJacobianByCameraPose(
        std::size_t pose, const Eigen::Matrix<double, 2, 9>& jacobian) {
    H_x_.block<2, 9>(2*pose, 56 + (poses_in_state_ - 1 - track_length_ + pose)*9) = jacobian;
}

void Residual::setJacobianByFeaturePosition(
        std::size_t pose, const Eigen::Matrix<double, 2, 3>& jacobian) {
    H_f_.block<2, 3>(2*pose, 0) = jacobian;
}

Eigen::Vector3d Residual::getGlobalFeaturePosition() const {
    return global_position_;
}

void Residual::setGlobalFeaturePosition(const Eigen::Vector3d& global_position) {
    global_position_ = global_position;
}

Eigen::VectorXd Residual::getResiduals() const {
    return r_;
}

Eigen::MatrixXd Residual::getJacobianByState() const {
    return H_x_;
}

Eigen::Matrix<double, Eigen::Dynamic, 3> Residual::getJacobianByFeaturePosition() const {
    return H_f_;
}
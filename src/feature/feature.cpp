#include "feature/feature.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "exceptions/general_exception.h"

Feature::Feature(cv::Mat& image, cv::Ptr<cv::ORB> detector) {
    static int feature_id = 0;
    feature_id_ = feature_id++;
    
    is_out_of_view_ = false;
    was_used_for_residualization_ = false;
}

const Eigen::Vector2d& Feature::operator[](std::size_t i) const {
    assert(i < positions_.size());
    return positions_[i];
}

void Feature::addFeaturePosition(double x, double y) {
    Eigen::Vector2d position;
    position << x, y;
    positions_.push_back(std::move(position));
}

void Feature::revertLastPosition() {
    assert(positions_.size() > 0);
    positions_.pop_back();
}

std::size_t Feature::posesTrackedCount() const {
    return positions_.size();
}

bool Feature::isOutOfView() const {
    return is_out_of_view_;
}

void Feature::setOutOfView() {
    is_out_of_view_ = true;
}

bool Feature::wasUsedForResidualization() const {
    return was_used_for_residualization_;
}

void Feature::setWasUsedForResidualization() {
    assert(!was_used_for_residualization_);
    was_used_for_residualization_ = true;
}

int Feature::getFeatureId() const {
    return feature_id_;
}
#include "frame/frame.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "exceptions/general_exception.h"

Frame::Frame() : is_valid_(false) {
}

Frame::Frame(cv::Mat& image, cv::Ptr<cv::ORB> detector) {
    assert(image.channels() == 3 || image.channels() == 1);
    cv::Mat gray = toGray(image);

    detectKeypointsAndDescripts(detector, gray);
}

Frame::Frame(double time, const cv::Mat& image) : is_valid_(true) {
    time_ = time;
    image_ = toGray(image);
    was_processed_ = false;
}

Frame::operator bool() const {
    return is_valid_;
}

void Frame::setIsProcessed() {
    assert(was_processed_ == false);
    was_processed_ = true;
}

bool Frame::wasProcessed() const {
    return was_processed_;
}

double Frame::getTime() const {
    return time_;
}

cv::Mat& Frame::getImage() {
    return image_;
}

const cv::Mat& Frame::cgetImage() const {
    return image_;
}

void Frame::detectKeypointsAndDescripts(cv::Ptr< cv::ORB > detector, cv::Mat gray)
{
    detector->detectAndCompute(gray, cv::noArray(), keypoints_, descriptors_);
}

cv::Mat Frame::toGray(const cv::Mat& image) {
    switch (image.channels()) {
        case 1:
            return image;
        case 3:
        {
            cv::Mat gray;
            cv::cvtColor(image, gray, CV_BGR2GRAY);
            return gray;
        }
        default:
            throw GeneralException("Unknown image format");
    }
}

std::vector<cv::DMatch> Frame::match(cv::Ptr<cv::DescriptorMatcher> matcher, const Frame &other, float threshold) {
    if (other.descriptors_.rows == 0) {
        return std::vector<cv::DMatch>();
    }
    
    std::vector<cv::DMatch> matches;
    matcher->match(descriptors_, other.descriptors_, matches);
    
    bool use_homography_filter = true;
    if (use_homography_filter) {
        std::vector<cv::Point2f> this_pts;
        std::vector<cv::Point2f> other_pts;
        this_pts.reserve(matches.size());
        other_pts.reserve(matches.size());
        for (std::size_t i = 0; i < matches.size(); ++i) {
            this_pts.push_back(keypoints_[matches[i].queryIdx].pt);
            other_pts.push_back(other.keypoints_[matches[i].trainIdx].pt);
        }
        cv::Mat good_features_mask;
        cv::Mat H = cv::findHomography(this_pts, other_pts, CV_RANSAC, 3, good_features_mask);
        
        std::vector<cv::DMatch> good_matches;
        for (std::size_t i = 0; i < matches.size(); ++i) {
            if (good_features_mask.at<bool>(i, 0)) {
                good_matches.push_back(matches[i]);
            }
        }
        return good_matches;
    } else {
        return matches;
    }
}

double Frame::computeDistanceLimitForMatch(const std::vector<cv::DMatch>& matches) const {
    double min_distance = 100;
    double max_distance = 0;
    double mean_distance = 0;
    for (std::size_t i = 0; i < matches.size(); ++i) {
        const cv::DMatch& match = matches[i];
        mean_distance += match.distance;
        if (match.distance < min_distance) {
            min_distance = match.distance;
        }
        if (match.distance > max_distance) {
            max_distance = match.distance;
        }
    }
    mean_distance /= matches.size();
    return std::max(2*min_distance, 5.0);
}
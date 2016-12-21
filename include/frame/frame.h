#ifndef FRAME_H
#define FRAME_H

#include <iostream>
#include <vector>

#include <Eigen/Core>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

/**
 * @brief Frame class, include its keypoints, pose and so on.
 * 
 * @author Doom
 */
class Frame {
public:
    Frame(cv::Mat& image, cv::Ptr<cv::ORB> detector);
    
    std::vector<cv::DMatch> match(cv::Ptr<cv::DescriptorMatcher> matcher, const Feature& other,
            float threshold = 0.5);
    
    std::vector<cv::KeyPoint>& keypoints();
    const std::vector<cv::KeyPoint>& keypoints() const;
    
    double computeDistanceLimitForMatch(const std::vector<cv::DMatch>& matches) const;

protected:
    std::vector<cv::KeyPoint> keypoints_;
    cv::Mat descriptors_;

    void detectKeypointsAndDescripts(cv::Ptr<cv::ORB> detector, cv::Mat gray);

    static cv::Mat toGray(const cv::Mat& image);
};

#endif //FRAME_H
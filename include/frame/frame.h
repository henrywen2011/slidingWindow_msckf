#ifndef FRAME_H
#define FRAME_H

#include <iostream>
#include <string>
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
    /** @brief default construction method */
    Frame();
    
    /** @brief used in feature tracker */
    Frame(cv::Mat& image, cv::Ptr<cv::ORB> detector);
    
    /** @brief used in filter */
    Frame(double time, const cv::Mat& image);
    
    Frame(const Frame& other) = default;
    
    Frame& operator=(const Frame& oher) = default;
    
    operator bool() const;
    
    std::vector<cv::DMatch> match(cv::Ptr<cv::DescriptorMatcher> matcher, const Feature& other,
            float threshold = 0.5);
    
    std::vector<cv::KeyPoint>& keypoints();
    const std::vector<cv::KeyPoint>& keypoints() const;
    
    double computeDistanceLimitForMatch(const std::vector<cv::DMatch>& matches) const;
    
    void setIsProcessed();
    bool wasProcessed() const;

    double getTime() const;
    cv::Mat& getImage();
    const cv::Mat& cgetImage() const;

protected:
    bool is_valid_;
    double time_;
    cv::Mat image_;
    bool was_processed_;
  
    std::vector<cv::KeyPoint> keypoints_;
    cv::Mat descriptors_;

    void detectKeypointsAndDescripts(cv::Ptr<cv::ORB> detector, cv::Mat gray);

    static cv::Mat toGray(const cv::Mat& image);
};

#endif //FRAME_H
#ifndef TRACKER_H
#define TRACKER_H

#include "frame/frame.h"
#include "feature/feature.h"

#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

typedef std::vector<std::shared_ptr<Feature>> feature_track_list;

/**
 * @brief Tracking features in the sliding window, as a user, you should only communicate with this class.
 * 
 * @author Doom
 */
class Tracker {
public:
    Tracker(int nfeatures_to_track);
      
    feature_track_list processImage(feature_track_list& previous_tracks, cv::Mat& image);

private:
    cv::Ptr<cv::ORB> detector_;
    cv::Ptr<cv::DescriptorMatcher> matcher_;
    Frame previous_frame_;

    double computeDistanceLimitForMatch(const std::vector<cv::DMatch>& matches) const;
    void markOutOfViewFeatures(std::vector<double>& feature_matched, feature_track_list& feature_tracks) const;
    void createNewFeatureTracks(std::vector<bool>& feature_matched, feature_track_list& feature_tracks,
        const Frame& frame, double scale_factor) const;
};


#endif //TRACKER_H

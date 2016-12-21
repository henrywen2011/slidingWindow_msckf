#include "frame/frame.h"
#include "feature/feature.h"
#include "feature/tracker.h"

#include "exceptions/general_exception.h"
#include "exceptions/impossible_exception.h"

#include <iostream>
#include <cmath>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

Tracker::Tracker(int nfeatures_to_track) {
    detector_ = cv::ORB::create();
    detector_->setMaxFeatures(nfeatures_to_track);
    matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");
}

feature_track_list Tracker::processImage(feature_track_list& previous_tracks, cv::Mat& image) {
    double scale_factor = 1.0;
    
    cv::Mat working_image;
    cv::resize(image, working_image, cv::Size(), scale_factor, scale_factor, cv::INTER_LINEAR);
    
    Frame frame = Frame(detector_, working_image);
    
    if (previous_frame_.keypoints().size() == 0) {
        feature_track_list current_features;
        for (std::size_t i = 0; i < frame.keypoints().size(); ++i) {
            const cv::KeyPoint& keypoint = frame.keypoints()[i];
            current_features.emplace_back(new Feature);
            current_features.back()->addFeaturePosition(keypoint.pt.x/scale_factor, keypoint.pt.y/scale_factor);
        }

        previous_frame_ = frame;
        return current_features;
    }

    std::vector<cv::DMatch> matches = frame.match(matcher_, previous_frame_);

    std::vector<double> previous_feature_matched(previous_tracks.size(), INFINITY);
    std::vector<std::size_t> matched_feature_assigned(previous_tracks.size(), std::numeric_limits<std::size_t>::max());
    std::vector<bool> current_feature_matched(frame.keypoints().size(), false);
    feature_track_list current_tracks(frame.keypoints().size());

    for (std::size_t i = 0; i < matches.size(); ++i) {
        const cv::DMatch& match = matches[i];

        int query_idx = match.queryIdx;
        int train_idx = match.trainIdx;

        if (current_feature_matched[query_idx]) {
            throw ImpossibleException("Current feature should match only once.");
        }

        if (match.distance < previous_feature_matched[train_idx]) {
            if (!std::isinf(previous_feature_matched[train_idx])) {
                // Revert previous match
                previous_tracks[train_idx]->revertLastPosition();
                
                std::size_t previous_matched_idx = matched_feature_assigned[train_idx];
                current_tracks[previous_matched_idx].reset(new Feature);
                
                current_feature_matched[previous_matched_idx] = false;
            }

            // Track feature
            const cv::KeyPoint& current_keypoint = frame.keypoints()[query_idx];
            current_tracks[query_idx] = previous_tracks[train_idx];
            matched_feature_assigned[train_idx] = query_idx;
            current_tracks[query_idx]->addFeaturePosition(
                current_keypoint.pt.x/scale_factor, current_keypoint.pt.y/scale_factor);

            previous_feature_matched[train_idx] = match.distance;
            current_feature_matched[query_idx] = true;
        }
    }

    markOutOfViewFeatures(previous_feature_matched, previous_tracks);
    createNewFeatureTracks(current_feature_matched, current_tracks, frame, scale_factor);

    previous_frame_ = frame;

    return current_tracks;
}

void Tracker::markOutOfViewFeatures(std::vector<double>& feature_matched, feature_track_list& feature_tracks)
        const {
    assert(feature_matched.size() == feature_tracks.size());

    for (std::size_t i = 0; i < feature_tracks.size(); ++i) {
        if (std::isinf(feature_matched[i])) {
            feature_tracks[i]->setOutOfView();
        }
    }
}

void Tracker::createNewFeatureTracks(std::vector<bool> &feature_matched,
        feature_track_list &feature_tracks, const Frame& frame, double scale_factor) const {
    for (std::size_t i = 0; i < feature_tracks.size(); ++i) {
        if (!feature_matched[i]) {
            // This is new feature
            const cv::KeyPoint& current_keypoint = frame.keypoints()[i];
            std::shared_ptr<Feature> feature(new Feature);
            feature->addFeaturePosition(current_keypoint.pt.x/scale_factor, current_keypoint.pt.y/scale_factor);
            feature_tracks[i] = feature;
        }
    }
}












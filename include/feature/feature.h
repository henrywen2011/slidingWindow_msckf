#ifndef FEATURE_H
#define FEATURE_H

#include <iostream>
#include <vector>

#include <Eigen/Core>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

/**
 * @brief Image feature class, include its status.
 * 
 * @author Doom
 */
class Feature {
public:
    Feature();

    const Eigen::Vector2d& operator[] (std::size_t i) const;
    
    void addFeaturePosition(double x, double y);
    void revertLastPosition();
    
    std::size_t posesTrackedCount() const;
    
    bool isOutOfView() const;
    void setOutOfView();
    
    bool wasUsedForResidualization() const;
    void setWasUsedForResidualization();
    
    int getFeatureId() const;

protected:
    int feature_id_;
    bool is_out_of_view_;
    bool was_used_for_residualization_;
    std::vector<Eigen::Vector2d> positions_;
  
};

#endif //FEATURE_H
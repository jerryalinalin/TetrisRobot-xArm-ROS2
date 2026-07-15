#ifndef COLOR_DETECTOR_CLASSIFICATION_H
#define COLOR_DETECTOR_CLASSIFICATION_H

#include "color_detector/includeFiles.h"

class ColorClassiFication {
private:
    std::map<COLORS, cv::Vec3b> standard_colors;
    cv::Mat src;
    std::map<COLORS, std::pair<cv::Scalar, cv::Scalar>> HSVThresholds;
    bool check();

public:
    static std::unordered_set<COLORS> COLORS_SET;

    ColorClassiFication();
    ~ColorClassiFication();

    void setStandard_colors(std::map<COLORS, cv::Vec3b>& standard_colors);
    void setSrc(cv::Mat& src);
    void Classify(std::map<COLORS, cv::Mat>& mats);
    std::map<COLORS, cv::Mat> threshold_Classify();
    void setThreshold_HSV(std::map<COLORS, std::pair<cv::Scalar, cv::Scalar>>& HSVThresholds);
};

#endif  // COLOR_DETECTOR_CLASSIFICATION_H

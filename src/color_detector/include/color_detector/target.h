#ifndef COLOR_DETECTOR_TARGET_H
#define COLOR_DETECTOR_TARGET_H

#include "color_detector/includeFiles.h"

class target {
private:
    cv::RotatedRect rect;
    COLORS type;
    float height;
    float width;
    cv::Mat codeMat;
    cv::Mat srcMat_to_codeMat;
    cv::Mat codeMat_to_srcMat;

public:
    int sta;
    static float maxArea;
    static float minArea;
    static std::unordered_map<int, COLORS> color_status;
    static std::unordered_map<int, bool> status;

    cv::Point position;
    bool updown;
    double Yaw;

    target(cv::RotatedRect rect, COLORS type);
    target();
    ~target();

    cv::RotatedRect getRect();
    cv::Mat getCodeMat();
    COLORS getType() { return type; }

    static void findTarget(cv::Mat src, std::map<COLORS, std::vector<target>>& targetSet);
    static void DrawRect(cv::Mat& Src, cv::RotatedRect rect, cv::Scalar color);
    static void setCodeMat(cv::Mat Src, target& Target);
    static void getStatus(target& Target);
    static void getPosition(target& Target);

    void operator=(const target& t) {
        rect = t.rect;
        type = t.type;
        height = t.height;
        width = t.width;
        codeMat = t.codeMat;
        srcMat_to_codeMat = t.srcMat_to_codeMat;
        codeMat_to_srcMat = t.codeMat_to_srcMat;
        sta = t.sta;
        updown = t.updown;
        Yaw = t.Yaw;
    }
};

#endif  // COLOR_DETECTOR_TARGET_H

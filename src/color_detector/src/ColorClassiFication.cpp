#include "color_detector/ColorClassiFication.h"

ColorClassiFication::ColorClassiFication() {}
ColorClassiFication::~ColorClassiFication() {}

void ColorClassiFication::setStandard_colors(std::map<COLORS, cv::Vec3b> &standard_colors){
    this->standard_colors = standard_colors;
}

void ColorClassiFication::setThreshold_HSV(std::map<COLORS, std::pair<cv::Scalar, cv::Scalar> > &HSVThresholds){
    this->HSVThresholds = HSVThresholds;
}

bool ColorClassiFication::check(){
    if(! standard_colors.empty()|| !src.data)
        return false;
    return true;
}

void ColorClassiFication::setSrc(cv::Mat &src){
    this->src = src;
}

void ColorClassiFication::Classify(std::map<COLORS, cv::Mat> &mats){
    if(check()){
        std::cout << "ColorClassiFication set error" << std::endl;
    }
    int minn_M = 0;
    cv::cvtColor(src, src, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> channel;
    auto cmp = [](const std::pair<COLORS, int> &a, const std::pair<COLORS, int> &b){
        return a.second < b.second;
    };
    std::map<COLORS, std::vector<cv::Point> > re;

    for(auto &color:COLORS_SET){
        re[color];
        mats[color] = cv::Mat(src.rows, src.cols, CV_8UC1, cv::Scalar(0, 0, 0));
    }
    int pixCnt = src.rows* src.cols * src.channels();
    uchar* Data = src.data;
    for(int Index = 0; Index < pixCnt; Index++){
        int H = Data[Index++];
        int S = Data[Index++];
        int V = Data[Index];
        int x = (Index / 3 ) % src.cols;
        int y = Index / 3 / src.cols;
        COLORS ty;
        int minDis = 0x3f3f3f;
        for(auto &[k, v]: standard_colors){
            int Dis = (int)std::sqrt((H - v(0))*(H - v(0)) + (S - v(1))*(S - v(1)));
            if(minDis > Dis){
                minDis = Dis;
                ty = k;
            }
        }
        mats[ty].at<uchar>(y, x) = 255;
        re[ty].push_back(cv::Point(x, y));
        if(x % 50 == 0 && y % 50 == 0){
            minn_M = 0x3f3f3f;
            for(auto &[k, v]:re){
                minn_M = std::min(minn_M, (int)v.size());
            }
        }
    }
    cv::Mat kernel1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7), cv::Point(-1, -1));
    cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5), cv::Point(-1, -1));
    cv::Mat kernel3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
    for(auto &color:COLORS_SET){
        cv::morphologyEx(mats[color], mats[color], cv::MORPH_ERODE, kernel3);
        cv::morphologyEx(mats[color], mats[color], cv::MORPH_CLOSE, kernel2);
        cv::morphologyEx(mats[color], mats[color], cv::MORPH_OPEN, kernel2);
        cv::morphologyEx(mats[color], mats[color], cv::MORPH_DILATE, kernel1);
    }
    return;
}

std::map<COLORS, cv::Mat> ColorClassiFication::threshold_Classify(){
    if(HSVThresholds.empty()){
        while(1){
            std::cout<<"threshold_Classify error"<<std::endl;
        }
    }
    std::map<COLORS, cv::Mat> re;
    for(auto &colors:COLORS_SET){
        cv::Mat binary;
        cv::inRange(src, HSVThresholds[colors].first, HSVThresholds[colors].second, binary);
        re[colors] = binary.clone();
    }
    return re;
}

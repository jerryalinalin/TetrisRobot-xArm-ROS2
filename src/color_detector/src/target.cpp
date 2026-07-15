#include "color_detector/target.h"

target::target(cv::RotatedRect rect, COLORS type) {
    this->rect = rect;
    this->type = type;
    this->height = rect.size.height;
    this->width = rect.size.width;
}

target::~target() {}
target::target() {}

void target::getStatus(target &Target){
    int cnt = 0;
    Target.sta = 0;
    for(int y = 0; y < 200; y+=100){
        for(int x = 0; x < 300; x+=100){
            cv::Rect ROI(x+40, y+40, 20, 20);
            if(cv::sum(cv::Mat(Target.codeMat, ROI))[0]/255 >= 200){
                Target.sta |= 1<<cnt;
            }
            ++cnt;
        }
    }
    if(color_status.find(Target.sta)!=color_status.end()){
        Target.type = color_status[Target.sta];
    }
    return ;
}

void target::getPosition(target &Target){
    cv::Mat pos;
    if(Target.type == PURPLE){
        if(Target.updown == 0){
            pos = Target.codeMat_to_srcMat * cv::Mat(cv::Vec3d(249, 149, 1), CV_64F);
            cv::circle(Target.codeMat, cv::Point2d(249, 149), 3, cv::Scalar(0), -1);
        }else{
            pos = Target.codeMat_to_srcMat * cv::Mat(cv::Vec3d(52, 52, 1), CV_64F);
            cv::circle(Target.codeMat, cv::Point2d(52, 52), 3, cv::Scalar(0), -1);
        }
    }else if(Target.type == YELLOW){
        if(Target.updown == 0){
            pos = Target.codeMat_to_srcMat * cv::Mat(cv::Vec3d(53, 147, 1), CV_64F);
        }else{
            pos = Target.codeMat_to_srcMat * cv::Mat(cv::Vec3d(247, 53, 1), CV_64F);
        }
    }else if(Target.type == BLUE){
        pos = Target.codeMat_to_srcMat * cv::Mat(cv::Vec3d(150, 50, 1), CV_64F);
        cv::circle(Target.codeMat, cv::Point2d(148, 49), 3, cv::Scalar(0), -1);
    }else if(Target.type == GREEN){
        pos = Target.codeMat_to_srcMat * cv::Mat(cv::Vec3d(148, 52, 1), CV_64F);
        cv::circle(Target.codeMat, cv::Point2d(148, 52), 3, cv::Scalar(0), -1);
    }else if(Target.type == ORANGE){
        pos = Target.codeMat_to_srcMat * cv::Mat(cv::Vec3d(50, 50, 1), CV_64F);
        cv::circle(Target.codeMat, cv::Point2d(50, 50), 3, cv::Scalar(0), -1);
    }else if(Target.type == BROWN){
        if(Target.updown == 0){
            pos = Target.codeMat_to_srcMat * cv::Mat(cv::Vec3d(150, 150, 1), CV_64F);
            cv::circle(Target.codeMat, cv::Point2d(150, 150), 3, cv::Scalar(0), -1);
        }else{
            pos = Target.codeMat_to_srcMat * cv::Mat(cv::Vec3d(150, 50, 1), CV_64F);
            cv::circle(Target.codeMat, cv::Point2d(150, 50), 3, cv::Scalar(0), -1);
        }
    }else if(Target.type == RED){
        pos = Target.codeMat_to_srcMat * cv::Mat(cv::Vec3d(150, 50, 1), CV_64F);
        cv::circle(Target.codeMat, cv::Point2d(150, 50), 3, cv::Scalar(0), -1);
    }
    Target.position = cv::Point(pos.at<double>(0, 0), pos.at<double>(0, 1));
}

void target::setCodeMat(cv::Mat Src, target &Target){
    int height, width;
    if(Target.type == RED){
        height = 100; width = 400;
    }else if(Target.type == ORANGE){
        height = 200; width = 200;
    }else{
        height = 200; width = 300;
    }
    cv::Point2f point[4];
    Target.rect.points(point);
    cv::Point2f srcPoints[3];
    if(Target.height < Target.width){
        Target.Yaw = - std::abs(Target.rect.angle) * M_PI / 180;
        srcPoints[0] = point[0];
        srcPoints[1] = point[1];
        srcPoints[2] = point[2];
    }else{
        Target.Yaw = (90 - std::abs(Target.rect.angle)) * M_PI / 180;
        srcPoints[0] = point[1];
        srcPoints[1] = point[2];
        srcPoints[2] = point[3];
    }
    cv::Point2f codeMatPoints[] = {cv::Point2f(0, height), cv::Point2f(0, 0), cv::Point2f(width, 0)};
    Target.srcMat_to_codeMat = cv::getAffineTransform(srcPoints, codeMatPoints);
    Target.codeMat_to_srcMat = cv::getAffineTransform(codeMatPoints, srcPoints);
    cv::warpAffine(Src, Target.codeMat, Target.srcMat_to_codeMat, cv::Size(width, height));
}

void target::findTarget(cv::Mat src, std::map<COLORS, std::vector<target>> &targetSet){
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(src, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
    for(auto &contour : contours){
        if(contour.size() <= 6) continue;
        cv::RotatedRect rect = cv::minAreaRect(contour);
        double p = std::max(rect.size.height, rect.size.width) / std::min(rect.size.height, rect.size.width);
        float area = rect.size.area();
        if(area > maxArea || area < minArea) continue;
        target::DrawRect(src, rect, cv::Scalar(255, 255, 255));
        target t;
        if(p < 1.3 && p > 0.7){
            t = target(rect, ORANGE);
            target::setCodeMat(src, t);
        }else if(p < 6 && p > 3.5){
            t = target(rect, RED);
            target::setCodeMat(src, t);
        }else if(p < 1.8 && p > 1.2){
            t = target(rect, WHITE);
            target::setCodeMat(src, t);
            target::getStatus(t);
        }else{
            continue;
        }
        if(t.type == WHITE) continue;
        if(target::status[t.sta])
            t.Yaw += M_PI;
        t.updown = target::status[t.sta];
        target::getPosition(t);
        targetSet[t.type].push_back(t);
    }
}

void target::DrawRect(cv::Mat &Src, cv::RotatedRect rect, cv::Scalar color) {
    cv::Point2f point[4];
    rect.points(point);
    for (int i = 0; i < 4; i++)
        cv::line(Src, point[i], point[(i + 1) % 4], color, 2);
}

cv::RotatedRect target::getRect() { return rect; }
cv::Mat target::getCodeMat() { return codeMat; }

#ifndef KALMANTEST_H
#define KALMANTEST_H

#include "configure.h"
#include "control.h"

class RM_kalmanfilter
{
public:
    RM_kalmanfilter();
    ~RM_kalmanfilter();
    Point2f predict_point(double _t, Point _p);
    void reset();

    float anti_range = 1.5;

private:
    cv::KalmanFilter KF_;
    Mat measurement_matrix;
    int k=5;
    double t = 0.005;
    float x = CAMERA_RESOLUTION_COLS*0.5;
    float y = CAMERA_RESOLUTION_ROWS*0.5;
};

class text{
    public:
    text(int size){
        data.resize(size);
        index = 0;
    }
    ~text() = default;
    void push(Point2f num){
        data[index++] = num;
        index %= data.size();
    }
    Point2f print(int x){
        return data[x];
    }
    int re(){
        length++;
        return length;
    }
    private:
        vector<Point2f> data;
    int index;
    int sum=0;
    int length=0; 
};

#endif // RM_KALMANFILTER_H

// class RM_kalmanfilter
// {
// public:
//     RM_kalmanfilter();
//     ~RM_kalmanfilter();
//     Point2f point_predict(double _t1, Point2d _p1);
//     void reset();
//     int n=2;
//     float anti_range = 1.5;

// private:
//     cv::KalmanFilter S_kalman;
//     Mat measurement_img;
//     Point2f last_point;
//     double last_v=0;
//     double t =(1e-2) + 0.005666666f;
//     float x = 640;
//     float y = 400;
// };


// #endif 

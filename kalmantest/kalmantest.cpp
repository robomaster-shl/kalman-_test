#include "kalmantest.h"


RM_kalmanfilter::RM_kalmanfilter()
    :KF_(4,2)
{
    measurement_matrix = Mat::zeros(2,1,CV_32F);
    t = 0.005;
    KF_.transitionMatrix = (Mat_<float>(4, 4) << 1, 0,t*t/2, 0,
                                                 0, 1, 0, t*t/2,
                                                 0, 0, 1, 0,
                                                 0, 0, 0, 1);

    setIdentity(KF_.measurementMatrix, Scalar::all(1));//H_k观测矩阵，为1就可以
    setIdentity(KF_.processNoiseCov, Scalar::all(1e-3));//Q_k过程噪声协方差矩阵
    setIdentity(KF_.measurementNoiseCov, Scalar::all(1e-2));//测量协方差矩阵R，数值越大回归越慢
    setIdentity(KF_.errorCovPost, Scalar::all(1e-5));//P_k后验误差估计协方差矩阵

    KF_.statePost = (Mat_<float>(4,1)<< x, y, 0, 0);//初始值-后验更新值(迭代起点)
}

RM_kalmanfilter::~RM_kalmanfilter(){

}

Point2f RM_kalmanfilter::predict_point(double _t, Point _p)
{
    t = _t;
    Mat prediction = KF_.predict();//如果影响较小，可以考虑加入一阶滤波器直接取这里算出的点
    Point2f predict_pt = Point2f(prediction.at<float>(0),prediction.at<float>(1));

    measurement_matrix.at<float>(0,0) = _p.x;
    measurement_matrix.at<float>(1,0) = _p.y;

    KF_.correct(measurement_matrix);

    Point2f anti_kalmanPoint;
    if((_p.x + anti_range*(_p.x - predict_pt.x))<=CAMERA_RESOLUTION_COLS
        || (_p.x + anti_range*(_p.x - predict_pt.x))>=0)
    {
        if(abs(_p.x - predict_pt.x) > 10)
            anti_kalmanPoint.x = _p.x + anti_range*(_p.x - predict_pt.x);
        else
            anti_kalmanPoint.x = _p.x;
    }
    else
    {
        anti_kalmanPoint.x = _p.x;
    }
    anti_kalmanPoint.y = _p.y;

    return anti_kalmanPoint;
}

void RM_kalmanfilter::reset()
{
    measurement_matrix = Mat::zeros(2,1,CV_32F);
    t = 0.005; 
}

// RM_kalmanfilter::RM_kalmanfilter()
//     :KF_(4,2)
// {
//     measurement_matrix = Mat::zeros(2,1,CV_32F);
//     t = 0.005;
//     KF_.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, t, 0,
//                                                  0, 1, 0, t,
//                                                  0, 0, 1, 0,
//                                                  0, 0, 0, 1);

//     setIdentity(KF_.measurementMatrix, Scalar::all(1));
//     setIdentity(KF_.processNoiseCov, Scalar::all(1e-3));//
//     setIdentity(KF_.measurementNoiseCov, Scalar::all(1e-2));//测量协方差矩阵R，数值越大回归越慢
//     setIdentity(KF_.errorCovPost, Scalar::all(1));

//     KF_.statePost = (Mat_<float>(4,1)<< x, y, 0, 0);//初始值
// }

// RM_kalmanfilter::~RM_kalmanfilter(){

// }

// Point2f RM_kalmanfilter::predict_point(double _t, Point _p)
// {
//     t = _t;
//     Mat prediction = KF_.predict();
//     Point2f predict_pt = Point2f(prediction.at<float>(0),prediction.at<float>(1));

//     measurement_matrix.at<float>(0,0) = _p.x;
//     measurement_matrix.at<float>(1,0) = _p.y;

//     KF_.correct(measurement_matrix);

//     Point2f anti_kalmanPoint;
//     if((_p.x + anti_range*(_p.x - predict_pt.x))<=CAMERA_RESOLUTION_COLS
//         || (_p.x + anti_range*(_p.x - predict_pt.x))>=0)
//     {
//         if(abs(_p.x - predict_pt.x) > 10)
//             anti_kalmanPoint.x = _p.x + anti_range*(_p.x - predict_pt.x);
//         else
//             anti_kalmanPoint.x = _p.x;
//     }
//     else
//     {
//         anti_kalmanPoint.x = _p.x;
//     }
//     anti_kalmanPoint.y = _p.y;

//     return anti_kalmanPoint;
// }

// void RM_kalmanfilter::reset()
// {
//     measurement_matrix = Mat::zeros(2,1,CV_32F);
//     t = 0.005; // FPS 200
// }

// RM_kalmanfilter::RM_kalmanfilter()
//     :S_kalman(4,2)
// {
//     measurement_img = Mat::zeros(2,1,CV_64F);
//     S_kalman.transitionMatrix = (Mat_<float>(4, 4) << 1, t, pow(t,2)/2,0,
//                                                       0, 1, t,pow(t,2)/2,
//                                                       0, 0, 1,0,
//                                                       0, 0, 0,1);

//     setIdentity(S_kalman.measurementMatrix, Scalar::all(1));//H_k观测矩阵，为1就可以
//     setIdentity(S_kalman.processNoiseCov, Scalar::all(0.0000000001f));//Q_k过程噪声协方差矩阵
//     setIdentity(S_kalman.measurementNoiseCov, Scalar::all(1e-2));//R_k测量噪声协方差矩阵
//     setIdentity(S_kalman.errorCovPost, Scalar::all(1e-5));//P_k后验误差估计协方差矩阵

//     S_kalman.statePost = (Mat_<float>(3,1)<< 0, 0, 0); //后验更新值(迭代起点)
//     last_point.x = 640;
//     last_point.y = 400;
//     t = (1e-2) + 0.005666666f;;
// }
 
// Point2f RM_kalmanfilter::point_predict(double t1, Point2d p1)
// {
//     t=t1;
//     double v = (p1.x - last_point.x) / t;
//     double a = v/ t;
//     measurement_img.at < double > (0, 0) = p1.x;
//     measurement_img.at < double > (1, 0) = v;
//     measurement_img.at < double > (2, 0) = a;

//     Mat prediction2 = S_kalman.predict(); //如果影响较小，可以考虑加入一阶滤波器直接取这里算出的点
//     Mat prediction = S_kalman.correct(measurement_img); 
//     Point2f temp_point = Point2f(prediction.at < double > (0, 0), p1.y);

//     double temp_x = p1.x + pow(anti_range,1.6) * (p1.x - temp_point.x);
//     Point2f anti_kalman_point;

//     if(a<0.02||a>0.02){
//         n++;
//     }
//     else{n=2;}
//     if(temp_x <= 640 && temp_x >= 0)
//     {
//         if (abs(p1.x - temp_point.x) > 0)
//         {
//             anti_kalman_point.x = temp_x;
//         }
//         else
//         {
//             anti_kalman_point.x = p1.x;
//         }
//     }
//     else
//     {
//         anti_kalman_point.x = p1.x;
//     }


//     anti_kalman_point.y = temp_point.y;

//     last_v = v;
//     last_point = p1;
//     return anti_kalman_point;
// }
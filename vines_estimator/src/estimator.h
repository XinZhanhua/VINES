#pragma once

#include "parameters.h"
#include "feature_manager.h"
#include "utility/utility.h"
#include "utility/tic_toc.h"
#include "initial/solve_5pts.h"
#include "initial/initial_sfm.h"
#include "initial/initial_alignment.h"
#include "initial/initial_ex_rotation.h"
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>

#include <ceres/ceres.h>
#include "factor/imu_factor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/axis_local_paramenterization.h"
#include "factor/projection_factor.h"
#include "factor/projection_td_factor.h"
#include "factor/projection_encoder_factor.h"
#include "factor/marginalization_factor.h"

#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>
#include <cmath>


class Estimator
{
  public:
    Estimator();

    void setParameter();

    // interface
    void processIMU(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::Header &header);
    void setReloFrame(double _frame_stamp, int _frame_index, vector<Vector3d> &_match_points, Vector3d _relo_t, Matrix3d _relo_r);

    // internal
    void clearState();
    bool initialStructure();
    bool visualInitialAlign();
    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    void slideWindow();
    void solveOdometry();
    void slideWindowNew();
    void slideWindowOld();
    void optimization();
    void vector2double();
    void double2vector();
    bool failureDetection();


    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    SolverFlag solver_flag;
    MarginalizationFlag  marginalization_flag;
    Vector3d g;
    MatrixXd Ap[2], backup_A;
    VectorXd bp[2], backup_b;

    Matrix3d ric[(WINDOW_SIZE + 1)];
    Matrix3d rie[NUM_OF_CAM];
    Vector3d tic[(WINDOW_SIZE + 1)];
    Vector3d tie[NUM_OF_CAM];
    Vector3d tec[NUM_OF_CAM];

    Vector3d axis_ce[NUM_OF_CAM];

    int encoder_data, last_encoder_data, continuous_encoder_data, rot_times, last_continuous_encoder_data;
    double encoder_angle, encoder_angle_velocity_original, encoder_angle_velocity, last_encoder_angle_velocity, last_time;

    Vector3d Ps[(WINDOW_SIZE + 1)];
    Vector3d Vs[(WINDOW_SIZE + 1)];
    Matrix3d Rs[(WINDOW_SIZE + 1)];
    Vector3d Bas[(WINDOW_SIZE + 1)];
    Vector3d Bgs[(WINDOW_SIZE + 1)];
    double Encoder_angle[(WINDOW_SIZE + 1)][1];
    double Encoder_angle_original[(WINDOW_SIZE + 1)];
    double td;

    Matrix3d back_R0, last_R, last_R0, back_ric;
    Vector3d back_P0, last_P, last_P0, back_tic;
    std_msgs::Header Headers[(WINDOW_SIZE + 1)];

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    Vector3d acc_0, gyr_0;

    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

    int frame_count;
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;

    FeatureManager f_manager;
    MotionEstimator m_estimator;
    InitialEXRotation initial_ex_rotation;

    bool first_imu;
    bool is_valid, is_key;
    bool failure_occur;

    vector<Vector3d> point_cloud;
    vector<Vector3d> margin_cloud;
    vector<Vector3d> key_poses;
    double initial_timestamp;


    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_angle[WINDOW_SIZE + 1][SIZE_ANGLE];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_Ex_Pose[NUM_OF_CAM][SIZE_POSE];
    double para_Retrive_Pose[SIZE_POSE];
    double para_Td[1][1];
    double para_Tr[1][1];

    int loop_window_index;

    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;

    map<double, ImageFrame> all_image_frame;
    IntegrationBase *tmp_pre_integration;

    //relocalization variable
    bool relocalization_info;
    double relo_frame_stamp;
    double relo_frame_index;
    int relo_frame_local_index;
    vector<Vector3d> match_points;
    double relo_Pose[SIZE_POSE];
    Matrix3d drift_correct_r;
    Vector3d drift_correct_t;
    Vector3d prev_relo_t;
    Matrix3d prev_relo_r;
    Vector3d relo_relative_t;
    Quaterniond relo_relative_q;
    double relo_relative_yaw;
};

class Observer : public ceres::IterationCallback
{
public:
    explicit Observer() {}

    ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary)
    {
        printf("iteration: %d, cost: %e, cost_change: %e \r\n", summary.iteration, summary.cost, summary.cost_change);
        return ceres::SOLVER_CONTINUE;
    }
};

class KalmanFilter 
{
public:
    KalmanFilter(double process_noise) 
    {
        this->process_noise = process_noise;
        // 初始化状态变量
        x_ = Vector2d::Zero();
        
        // 初始化状态协方差矩阵
        P_ = Matrix2d::Identity();
        P_ << 1, 0, 0, 100;
        
        // 状态转移矩阵
        F_ = Matrix2d::Identity();
        
        // 测量矩阵
        H_ = Matrix<double, 2, 2>::Identity();
        
        // 测量噪声协方差矩阵
        R_ = 0.1 * Matrix<double, 2, 2>::Identity();
        R_ << 0.3, 0, 0, 3;   
        
        // 系统噪声协方差矩阵
        Q_ = process_noise * Matrix2d::Identity();
        Q_ << 1, 0, 0, 0.5;
    }

    // 更新卡尔曼滤波器状态
    void update(double dt, double measurement, double angle_v) 
    {
        F_ << 1, dt, 0, 1;
        // x_ << measurement, angle_v;

        // 预测
        x_ = F_ * x_;
        P_ = F_ * P_ * F_.transpose() + Q_;

        // 更新
        Matrix<double, 2, 1> m_m;
        m_m << measurement, angle_v;
        Matrix<double, 2, 1> y = m_m - H_ * x_;
        Matrix<double, 2, 2> S = H_ * P_ * H_.transpose() + R_;
        Matrix<double, 2, 2> K = P_ * H_.transpose() * S.inverse();

        x_ = x_ + K * y;
        P_ = (Matrix2d::Identity() - K * H_) * P_;
    }

    // 获取当前状态的角度值
    double getAngle() const 
    {
        return x_[0];
    }

    // 获取当前状态的角速度值
    double getAngularVelocity() const 
    {
        return x_[1];
    }

private:
    double process_noise;
    Vector2d x_;  // 状态变量：[角度, 角速度]
    Matrix2d P_;  // 状态协方差矩阵
    Matrix2d F_;  // 状态转移矩阵
    Matrix<double, 2, 2> H_;  // 测量矩阵
    Matrix<double, 2, 2> R_;  // 测量噪声协方差矩阵
    Matrix2d Q_;  // 系统噪声协方差矩阵
};
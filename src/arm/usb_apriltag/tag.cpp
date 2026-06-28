#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include <cmath>

#include <opencv2/opencv.hpp>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

extern "C"
{
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
}

// ============================================================
//                      可配置参数
// ============================================================

// ---- USB 相机 ----
constexpr int   CAMERA_DEVICE_ID = 2;     // /dev/videoX
constexpr int   FRAME_WIDTH      = 640;
constexpr int   FRAME_HEIGHT     = 480;

// ---- 相机内参（标定结果）----
constexpr double FX = 325.53172475155594;
constexpr double FY = 325.5127347957183;
constexpr double CX = 323.4747330012186;
constexpr double CY = 251.68298556701887;

constexpr double K1 = -0.0011399200886537332;
constexpr double K2 = -0.09491210221445738;
constexpr double P1 = -0.0003002068636793419;
constexpr double P2 =  0.00322949710749793;
constexpr double K3 =  0.09408591579045257;

// ---- 窗口 ----
constexpr int   WINDOW_WIDTH  = 1280;
constexpr int   WINDOW_HEIGHT = 960;

// ---- AprilTag 检测器 ----
constexpr float QUAD_DECIMATE = 1.0f;
constexpr float QUAD_SIGMA    = 0.0f;
constexpr int   NTHREADS      = 4;
constexpr int   REFINE_EDGES  = 1;

// ---- 帧平均 ----
constexpr int   FRAME_AVG_COUNT = 2;      // 几帧取平均（≥1）

// ---- Tag 物理尺寸（米）----
constexpr float TAG_SIZE = 0.035f;

// ---- 坐标轴长度（米）----
constexpr float AXIS_LENGTH = 0.02f;

// ---- ROS TF ----
constexpr char  PARENT_FRAME[]      = "usb_camera";   // 父坐标系（相机）
constexpr char  MERGED_FRAME[]      = "tag_merged";   // 融合后的坐标系
constexpr int   TAG_ID_A = 0;                         // 要融合的 tag A
constexpr int   TAG_ID_B = 1;                         // 要融合的 tag B
constexpr double OFFSET_X = 0.0;                      // X 附加偏移量（米）
constexpr double OFFSET_Z = 0.0;                      // Z 附加偏移量（米）

// ============================================================
//                 线程共享数据结构
// ============================================================

struct TagPose
{
    int      id;
    double   t[3];       // 平移 X, Y, Z
    double   R[9];       // 旋转矩阵 3×3 (row-major)
};

// 融合后的均值（主线程写入，ROS 线程读取）
struct MergedPose
{
    bool    valid = false;
    double  x_avg = 0.0;    // tag0 和 tag1 的 X 平均值 + OFFSET_X
    double  z_avg = 0.0;    // tag0 和 tag1 的 Z 平均值 + OFFSET_Z
    double  t[3];           // 完整平移
    double  R[9];           // 完整旋转矩阵
};

// ============================================================
//                     ROS2 TF 发布线程
// ============================================================

static std::mutex              g_mutex;
static std::condition_variable g_cv;
static MergedPose              g_merged;
static bool                    g_new_data = false;
static std::atomic<bool>       g_running{true};

void ros_tf_thread()
{
    auto node = std::make_shared<rclcpp::Node>("apriltag_tf_publisher");
    tf2_ros::TransformBroadcaster broadcaster(node);
    rclcpp::Rate rate(30);  // 30Hz

    while (rclcpp::ok() && g_running.load())
    {
        MergedPose local;

        {
            std::unique_lock<std::mutex> lock(g_mutex);
            g_cv.wait_for(lock, std::chrono::milliseconds(33),
                          []{ return g_new_data; });
            if (g_new_data)
            {
                local      = g_merged;
                g_new_data = false;
            }
        }

        if (local.valid)
        {
            geometry_msgs::msg::TransformStamped ts;

            ts.header.stamp    = node->get_clock()->now();
            ts.header.frame_id = PARENT_FRAME;
            ts.child_frame_id  = MERGED_FRAME;

            ts.transform.translation.x = local.t[0];
            ts.transform.translation.y = local.t[1];
            ts.transform.translation.z = local.t[2];

            tf2::Matrix3x3 rot(
                local.R[0], local.R[1], local.R[2],
                local.R[3], local.R[4], local.R[5],
                local.R[6], local.R[7], local.R[8]);
            tf2::Quaternion q;
            rot.getRotation(q);

            ts.transform.rotation.x = q.x();
            ts.transform.rotation.y = q.y();
            ts.transform.rotation.z = q.z();
            ts.transform.rotation.w = q.w();

            broadcaster.sendTransform(ts);
        }

        rclcpp::spin_some(node);
        rate.sleep();
    }
}

// ============================================================

int main(int argc, char **argv)
{
    // =========================================
    // ROS2 初始化 (主线程)
    // =========================================
    rclcpp::init(argc, argv);

    // =========================================
    // USB 相机初始化
    // =========================================
    cv::VideoCapture cap(CAMERA_DEVICE_ID);

    if (!cap.isOpened())
    {
        std::cerr << "无法打开USB相机 /dev/video"
                  << CAMERA_DEVICE_ID << std::endl;
        return -1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);

    // =========================================
    // 相机内参矩阵
    // =========================================
    cv::Mat cameraMatrix =
        (cv::Mat_<double>(3,3) <<
            FX,  0,  CX,
            0,  FY,  CY,
            0,   0,  1);

    cv::Mat distCoeffs =
        (cv::Mat_<double>(5,1) << K1, K2, P1, P2, K3);

    std::cout << "Camera: fx=" << FX << " fy=" << FY << std::endl;

    // =========================================
    // AprilTag 初始化
    // =========================================
    apriltag_family_t *tf = tag36h11_create();

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    td->quad_decimate = QUAD_DECIMATE;
    td->quad_sigma    = QUAD_SIGMA;
    td->nthreads      = NTHREADS;
    td->debug         = 0;
    td->refine_edges  = REFINE_EDGES;

    // =========================================
    // Tag 角点（物体坐标系）
    // =========================================
    float s = TAG_SIZE / 2.0f;

    std::vector<cv::Point3f> objectPoints =
    {
        {-s, -s, 0},
        { s, -s, 0},
        { s,  s, 0},
        {-s,  s, 0}
    };

    // =========================================
    // 窗口
    // =========================================
    cv::namedWindow("AprilTag Pose", cv::WINDOW_NORMAL);
    cv::resizeWindow("AprilTag Pose", WINDOW_WIDTH, WINDOW_HEIGHT);

    // =========================================
    // 启动 ROS2 TF 发布线程
    // =========================================
    std::thread tf_publisher(ros_tf_thread);

    // =========================================
    // 帧平均累加器
    // =========================================
    cv::Mat accum_gray;
    int     acc_count = 0;

    while (rclcpp::ok())
    {
        // ---- 采集一帧 ----
        cv::Mat image;
        cap >> image;
        if (image.empty()) break;

        // ---- 转灰度 ----
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        // ---- 累加 ----
        if (accum_gray.empty())
        {
            gray.convertTo(accum_gray, CV_32F);
        }
        else
        {
            cv::Mat tmp;
            gray.convertTo(tmp, CV_32F);
            accum_gray += tmp;
        }
        acc_count++;

        // 未满 N 帧：只显示原图
        if (acc_count < FRAME_AVG_COUNT)
        {
            cv::imshow("AprilTag Pose", image);
            int key = cv::waitKey(1);
            if (key == 27) break;
            continue;
        }

        // ---- 求平均 ----
        accum_gray /= static_cast<float>(FRAME_AVG_COUNT);
        cv::Mat gray_avg;
        accum_gray.convertTo(gray_avg, CV_8U);

        // 重置累加器
        accum_gray.release();
        acc_count = 0;

        // ---- AprilTag 检测 ----
        image_u8_t img_header =
        {
            .width  = gray_avg.cols,
            .height = gray_avg.rows,
            .stride = gray_avg.cols,
            .buf    = gray_avg.data
        };

        zarray_t *detections =
            apriltag_detector_detect(td, &img_header);

        // ---- 收集本帧所有 pose ----
        std::vector<TagPose> frame_poses;
        TagPose *p0 = nullptr, *p1 = nullptr;

        for (int i = 0; i < zarray_size(detections); i++)
        {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            // 图像角点
            std::vector<cv::Point2f> imagePoints;
            for (int j = 0; j < 4; j++)
            {
                imagePoints.push_back(
                    cv::Point2f(det->p[j][0], det->p[j][1]));
            }

            // solvePnP
            cv::Mat rvec, tvec;
            bool ok = cv::solvePnP(objectPoints, imagePoints,
                                   cameraMatrix, distCoeffs,
                                   rvec, tvec);
            if (!ok) continue;

            // ---- 提取 pose ----
            cv::Mat R;
            cv::Rodrigues(rvec, R);

            TagPose pose;
            pose.id   = det->id;
            pose.t[0] = tvec.at<double>(0);
            pose.t[1] = tvec.at<double>(1);
            pose.t[2] = tvec.at<double>(2);
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    pose.R[r*3 + c] = R.at<double>(r, c);

            frame_poses.push_back(pose);

            if (det->id == TAG_ID_A) p0 = &frame_poses.back();
            if (det->id == TAG_ID_B) p1 = &frame_poses.back();

            // ---- 控制台输出 ----
            std::cout << std::fixed << std::setprecision(4);
            std::cout << "=== TF tag:" << det->id << " ===\n";
            std::cout << "t: [" << pose.t[0]
                      << ", "    << pose.t[1]
                      << ", "    << pose.t[2] << "]\n";
            std::cout << "R: ["
                      << pose.R[0] << ", " << pose.R[1] << ", " << pose.R[2] << " | "
                      << pose.R[3] << ", " << pose.R[4] << ", " << pose.R[5] << " | "
                      << pose.R[6] << ", " << pose.R[7] << ", " << pose.R[8] << "]\n";
            std::cout << std::endl;

            // ---- 画框 ----
            for (int j = 0; j < 4; j++)
            {
                cv::line(image, imagePoints[j],
                         imagePoints[(j + 1) % 4],
                         cv::Scalar(0, 255, 0), 2);
            }

            // ---- 画中心点 ----
            cv::circle(image, cv::Point(det->c[0], det->c[1]),
                       5, cv::Scalar(0, 0, 255), -1);

            // ---- 画坐标轴 ----
            cv::drawFrameAxes(image, cameraMatrix, distCoeffs,
                              rvec, tvec, AXIS_LENGTH);

            // ---- 显示 XYZ ----
            std::stringstream ss;
            ss << std::fixed << std::setprecision(3)
               << "X:" << pose.t[0]
               << " Y:" << pose.t[1]
               << " Z:" << pose.t[2];

            cv::putText(image, ss.str(),
                        cv::Point(det->c[0], det->c[1] - 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(255, 0, 0), 2);

            // ---- Tag ID ----
            cv::putText(image, "ID: " + std::to_string(det->id),
                        cv::Point(det->c[0], det->c[1] + 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(0, 255, 255), 2);
        }

        // ---- 融合 pose → ROS 线程 + 串口 + 控制台 ----
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            g_merged.valid = false;

            if (p0 && p1)
            {
                g_merged.x_avg = (p0->t[0] + p1->t[0]) * 0.5 + OFFSET_X;
                g_merged.z_avg = (p0->t[2] + p1->t[2]) * 0.5 + OFFSET_Z;
                g_merged.t[0]  = g_merged.x_avg;
                g_merged.t[1]  = (p0->t[1] + p1->t[1]) * 0.5;
                g_merged.t[2]  = g_merged.z_avg;
                g_merged.valid = true;

                // 旋转矩阵平均 + SVD 正交化
                for (int i = 0; i < 9; i++)
                    g_merged.R[i] = (p0->R[i] + p1->R[i]) * 0.5;
                cv::Mat R_avg = (cv::Mat_<double>(3,3) <<
                    g_merged.R[0], g_merged.R[1], g_merged.R[2],
                    g_merged.R[3], g_merged.R[4], g_merged.R[5],
                    g_merged.R[6], g_merged.R[7], g_merged.R[8]);
                cv::SVD svd(R_avg, cv::SVD::FULL_UV);
                cv::Mat R_ortho = svd.u * svd.vt;
                for (int r = 0; r < 3; r++)
                    for (int c = 0; c < 3; c++)
                        g_merged.R[r*3 + c] = R_ortho.at<double>(r, c);

                // ====== 控制台打印平均值 ======
                std::cout << " >>> MERGED  X_avg=" << g_merged.x_avg
                          << "  Z_avg=" << g_merged.z_avg << " <<<"
                          << std::endl;
            }
            g_new_data = true;
        }
        g_cv.notify_one();

        // ---- 在图像上也画平均值 ----
        if (p0 && p1)
        {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(3)
               << "AVG X:" << g_merged.x_avg
               << " Z:" << g_merged.z_avg;
            cv::putText(image, ss.str(),
                        cv::Point(10, image.rows - 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        cv::Scalar(0, 255, 255), 2);
        }

        // ---- 显示 ----
        cv::imshow("AprilTag Pose", image);

        if (cv::waitKey(1) == 27) break;

        apriltag_detections_destroy(detections);
    }

    // =========================================
    // 清理
    // =========================================
    g_running = false;
    g_cv.notify_all();
    if (tf_publisher.joinable())
        tf_publisher.join();

    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);

    rclcpp::shutdown();

    return 0;
}

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>
#include <iostream>

using namespace std;
using namespace cv;

class RedLightsDetector : public rclcpp::Node {
public:
    RedLightsDetector() : Node("red_lights_detector") {
        // Define camera parameters
        camera_matrix_ = (cv::Mat_<double>(3, 3) << 2328.99894, 0, 718.62016,
                                                     0, 2324.69204, 537.24441,
                                                     0, 0, 1);
        dist_coeffs_ = (cv::Mat_<double>(1, 5) << -0.122063, 0.282615, -0.000997, -0.000350, 0.0);
        
        // Open video file
        video_capture_ = cv::VideoCapture("/mnt/hgfs/share/Video_2024.avi");
        if (!video_capture_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open the video file.");
            return;
        }

        // HSV threshold initialization
        hmin_ = 0; smin_ = 100; vmin_ = 100;
        hmax_ = 22; smax_ = 255; vmax_ = 255;

        // Create trackbars for HSV threshold adjustments
        namedWindow("TrackBars", WINDOW_AUTOSIZE);
        createTrackbar("Hmin", "TrackBars", &hmin_, 180);
        createTrackbar("Hmax", "TrackBars", &hmax_, 180);
        createTrackbar("Smin", "TrackBars", &smin_, 255);
        createTrackbar("Smax", "TrackBars", &smax_, 255);
        createTrackbar("Vmin", "TrackBars", &vmin_, 255);
        createTrackbar("Vmax", "TrackBars", &vmax_, 255);

        // Process video frames
        process_video();
    }

private:
    void process_video() {
        cv::Mat frame;
        while (rclcpp::ok()) {
            video_capture_ >> frame;
            if (frame.empty()) {
                RCLCPP_WARN(this->get_logger(), "Empty frame, ending.");
                break;
            }

            cv::Mat undistorted_frame;
            cv::undistort(frame, undistorted_frame, camera_matrix_, dist_coeffs_);

            // Convert to HSV color space
            cv::Mat hsv_frame;
            cv::cvtColor(undistorted_frame, hsv_frame, cv::COLOR_BGR2HSV);

            // Apply HSV threshold
            cv::Mat mask1, mask2;
            cv::inRange(hsv_frame, cv::Scalar(hmin_, smin_, vmin_), cv::Scalar(hmax_, smax_, vmax_), mask1);
            cv::inRange(hsv_frame, cv::Scalar(145, 100, 100), cv::Scalar(180, 255, 255), mask2);
            cv::Mat mask = mask1 | mask2;

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            std::vector<cv::Rect> bounding_boxes;
            for (const auto& contour : contours) {
                cv::Rect bounding_box = cv::boundingRect(contour);
                if (bounding_box.height > 10 && bounding_box.width > 10) {
                    bounding_boxes.push_back(bounding_box);
                }
            }

            // Use the first two bounding boxes if they exist
            if (bounding_boxes.size() >= 2) {
                cv::Rect big_rectangle = bounding_boxes[0] | bounding_boxes[1]; // Combine boxes
                cv::rectangle(undistorted_frame, big_rectangle, cv::Scalar(0, 0, 255), 2);

                // Calculate center of the rectangle
                cv::Point center = (big_rectangle.tl() + big_rectangle.br()) * 0.5;

                // Known 3D points in the world frame (this will depend on your real-world setup)
                std::vector<cv::Point3f> object_points = {
                    cv::Point3f(0, 0, 0),  // Assuming the object is at origin
                    cv::Point3f(0, 1, 0),  // Adjust accordingly
                    cv::Point3f(1, 0, 0),
                    cv::Point3f(1, 1, 0),
                };

                // Corresponding 2D image points
                std::vector<cv::Point2f> image_points = {
                    cv::Point2f(big_rectangle.tl().x, big_rectangle.tl().y),
                    cv::Point2f(big_rectangle.tl().x, big_rectangle.br().y),
                    cv::Point2f(big_rectangle.br().x, big_rectangle.tl().y),
                    cv::Point2f(big_rectangle.br().x, big_rectangle.br().y)
                };

                // Pose estimation
                cv::Mat rvec, tvec;
                cv::solvePnP(object_points, image_points, camera_matrix_, dist_coeffs_, rvec, tvec);

                std::cout << "rvec: " << rvec << endl<<"tvec:" <<tvec<< std::endl;

                // Optionally also draw the center on the image
                cv::circle(undistorted_frame, center, 5, cv::Scalar(255, 255, 0), -1);
            }

            double scale_factor = 0.5;
            cv::Size new_size(scale_factor * undistorted_frame.cols,
                              scale_factor * undistorted_frame.rows);
            cv::Mat resized_frame;
            cv::resize(undistorted_frame, resized_frame, new_size);
            cv::imshow("Red Lights Detector", resized_frame);
            waitKey(1);
        }
    }

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::VideoCapture video_capture_;
    int hmin_, smin_, vmin_;
    int hmax_, smax_, vmax_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RedLightsDetector>());
    rclcpp::shutdown();
    return 0;
}

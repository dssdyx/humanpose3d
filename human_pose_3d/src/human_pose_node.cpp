#include <vector>
#include <chrono>
#include <ros/ros.h>
#include <inference_engine.hpp>

#include <monitors/presenter.h>
#include "human_pose_3d/ocv_common.hpp"

#include "human_pose_3d/human_pose_estimation_demo.hpp"
#include "human_pose_3d/human_pose_estimator.hpp"
#include "human_pose_3d/render_human_pose.hpp"
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseArray.h>

//Eigen
#include <Eigen/Eigen>
#include <Eigen/StdVector>

using namespace InferenceEngine;
using namespace human_pose_estimation;

cv::Mat Color_pic;
cv::Mat Depth_pic;
bool init_flag = false;
/*************************
 * 订阅/camera/aligned_depth_to_color/camera_info查询相机内参
 * **********************/
//step 1:
double fx = 610.4365234375;
double fy = 609.2212524414062;
double cx = 322.4535827636719;
double cy = 238.80897521972656;
//from camera_link to camera_optical
Eigen::Vector3d camera_pos(-0.000107598956674,0.0149245066568,0.000289224844892);
Eigen::Quaterniond camera_q = Eigen::Quaterniond(0.502717638126, -0.501220281032,
                                                 0.496576036996, -0.499465063011);

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
   cv_bridge::CvImagePtr cam_img;
   //std::cout<<"received pic!"<<std::endl;

   try {
       cam_img = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
   }catch(cv_bridge::Exception& e){
       ROS_ERROR("cv_bridge exception: %s",e.what());
       return;
   }
   if(cam_img)
   {
       Color_pic = cam_img->image.clone();

   }

}
void depthCallback(const sensor_msgs::ImageConstPtr& img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s",e.what());
        return;
    }
    cv_ptr->image.copyTo(Depth_pic);

}
int main(int argc, char** argv) {
    ros::init(argc, argv, "human_pose_node");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber img_sub = it.subscribe("/camera/color/image_raw",1,imageCallback);
    image_transport::Subscriber depth_sub = it.subscribe("/camera/aligned_depth_to_color/image_raw",1,depthCallback);
    image_transport::Publisher img_pub = it.advertise("human_pose_show",1);

    ros::Publisher poses_pub = nh.advertise<geometry_msgs::PoseArray>("/human/poses_3d",1);



    std::cout << "InferenceEngine: " << GetInferenceEngineVersion() << std::endl;

    HumanPoseEstimator estimator("/opt/intel/openvino_2020.4.287/deployment_tools/open_model_zoo/tools/downloader/intel/human-pose-estimation-0001/FP16/human-pose-estimation-0001.xml",
                                     "CPU", false);

    int delay = 33;
    cv::Mat curr_frame;
        //curr_frame = Color_pic.clone();
    cv::Mat next_frame;
    cv::Mat depth_frame;
    Eigen::Vector3d frame_pos;
    Eigen::Quaterniond frame_q;

        //estimator.reshape(curr_frame);  // Do not measure network reshape, if it happened

    std::cout << "To close the application, press 'CTRL+C' here";
    std::cout << std::endl;

    cv::Size graphSize{(int)640 / 4, 60};
    Presenter presenter("", (int)480 - graphSize.height - 10, graphSize);
    std::vector<HumanPose> poses;
    bool isLastFrame = false;
    bool isAsyncMode = false; // execution is always started in SYNC mode
    bool isModeChanged = false; // set to true when execution mode is changed (SYNC<->ASYNC)
    bool blackBackground = false;

    typedef std::chrono::duration<double, std::ratio<1, 1000>> ms;
    auto total_t0 = std::chrono::high_resolution_clock::now();
    auto wallclock = std::chrono::high_resolution_clock::now();
    double render_time = 0;
        //init_flag =true;
    ros::Rate loop_rate(30);
    cv::namedWindow("HUMAN_POSE");

    while (ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();
            if(!init_flag)
            {
                if(!Color_pic.empty()&&!Depth_pic.empty())
                {
                    curr_frame = Color_pic.clone();
                    estimator.reshape(curr_frame);  // Do not measure network reshape, if it happened
                    init_flag =true;
                }
            }
            else{
                auto t0 = std::chrono::high_resolution_clock::now();
                //here is the first asynchronus point:
                //in the async mode we capture frame to populate the NEXT infer request
                //in the regular mode we capture frame to the current infer request
                next_frame = Color_pic.clone();
                depth_frame = Depth_pic.clone();
                frame_pos = camera_pos;
                frame_q = camera_q;
                std::cout<<"received pic!"<<std::endl;


                if (isAsyncMode) {
                    if (isModeChanged) {
                        estimator.frameToBlobCurr(curr_frame);
                    }
                    if (!isLastFrame) {
                        estimator.frameToBlobNext(next_frame);
                    }
                } else if (!isModeChanged) {
                    estimator.frameToBlobCurr(curr_frame);
                }
                auto t1 = std::chrono::high_resolution_clock::now();
                double decode_time = std::chrono::duration_cast<ms>(t1 - t0).count();

                t0 = std::chrono::high_resolution_clock::now();
                // Main sync point:
                // in the trully Async mode we start the NEXT infer request, while waiting for the CURRENT to complete
                // in the regular mode we start the CURRENT request and immediately wait for it's completion
                if (isAsyncMode) {
                    if (isModeChanged) {
                        estimator.startCurr();
                    }
                    if (!isLastFrame) {
                        estimator.startNext();
                    }
                } else if (!isModeChanged) {
                    estimator.startCurr();
                }

                if (estimator.readyCurr()) {
                    t1 = std::chrono::high_resolution_clock::now();
                    ms detection = std::chrono::duration_cast<ms>(t1 - t0);
                    t0 = std::chrono::high_resolution_clock::now();
                    ms wall = std::chrono::duration_cast<ms>(t0 - wallclock);
                    wallclock = t0;

                    t0 = std::chrono::high_resolution_clock::now();

                    if (true) {
                        if (blackBackground) {
                            curr_frame = cv::Mat::zeros(curr_frame.size(), curr_frame.type());
                        }
                        std::ostringstream out;
                        out << "OpenCV cap/render time: " << std::fixed << std::setprecision(2)
                            << (decode_time + render_time) << " ms";

                        cv::putText(curr_frame, out.str(), cv::Point2f(0, 25),
                                    cv::FONT_HERSHEY_TRIPLEX, 0.6, cv::Scalar(0, 255, 0));
                        out.str("");
                        out << "Wallclock time " << (isAsyncMode ? "(TRUE ASYNC):      " : "(SYNC, press Tab): ");
                        out << std::fixed << std::setprecision(2) << wall.count()
                            << " ms (" << 1000.f / wall.count() << " fps)";
                        cv::putText(curr_frame, out.str(), cv::Point2f(0, 50),
                                    cv::FONT_HERSHEY_TRIPLEX, 0.6, cv::Scalar(0, 0, 255));
                        if (!isAsyncMode) {  // In the true async mode, there is no way to measure detection time directly
                            out.str("");
                            out << "Detection time  : " << std::fixed << std::setprecision(2) << detection.count()
                            << " ms ("
                            << 1000.f / detection.count() << " fps)";
                            cv::putText(curr_frame, out.str(), cv::Point2f(0, 75), cv::FONT_HERSHEY_TRIPLEX, 0.6,
                                cv::Scalar(255, 0, 0));
                        }
                    }

                    poses = estimator.postprocessCurr();

                    if (false) {
                        if (!poses.empty()) {
                            std::time_t result = std::time(nullptr);
                            char timeString[sizeof("2020-01-01 00:00:00: ")];
                            std::strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S: ", std::localtime(&result));
                            std::cout << timeString;
                         }

                        for (HumanPose const& pose : poses) {
                            std::stringstream rawPose;
                            rawPose << std::fixed << std::setprecision(0);
                            for (auto const& keypoint : pose.keypoints) {
                                rawPose << keypoint.x << "," << keypoint.y << " ";
                            }
                            rawPose << pose.score;
                            std::cout << rawPose.str() << std::endl;
                        }
                    }

                    if (true) {
                        presenter.drawGraphs(curr_frame);
                        renderHumanPose(poses, curr_frame);
                        const cv::Point2f absentKeypoint(-1.0f, -1.0f);

                        for (const auto& pose : poses) {
                            CV_Assert(pose.keypoints.size() == HumanPoseEstimator::keypointsNumber);
                            //poses_3d save one person pose without orientation

                            geometry_msgs::PoseArray poses_3d;
                            poses_3d.header.stamp = ros::Time::now();
                            poses_3d.header.frame_id ="camera_link";
                            poses_3d.poses.reserve(pose.keypoints.size());

                            for (size_t keypointIdx = 0; keypointIdx < pose.keypoints.size(); keypointIdx++) {
                                if (pose.keypoints[keypointIdx] != absentKeypoint) {
                                    int u = pose.keypoints[keypointIdx].x;
                                    int v = pose.keypoints[keypointIdx].y;
                                    double depth;
                                    depth = double(depth_frame.at<uint16_t>(v,u))/1000;
                                    Eigen::Matrix3d camera_r = frame_q.toRotationMatrix();
                                    //poses in camera optical frame
                                    Eigen::Vector3d pt_cur, pt_world;
                                    pt_cur(0) = (u - cx) * depth / fx;
                                    pt_cur(1) = (v - cy) * depth / fy;
                                    pt_cur(2) = depth;
                                    pt_world = camera_r * pt_cur + frame_pos;
                                    geometry_msgs::Pose pose_3d;
                                    pose_3d.position.x = pt_world(0);
                                    pose_3d.position.y = pt_world(1);
                                    pose_3d.position.z = pt_world(2);
                                    poses_3d.poses.push_back(pose_3d);
                                }
                            }
                            poses_pub.publish(poses_3d);
                        }
                        //curr_frame.resize(960,1280);
                        cv::resize(curr_frame,curr_frame,cv::Size(1920,1440),0,0,cv::INTER_LINEAR);
                        cv::imshow("Human Pose Estimation on CPU", curr_frame);
                        cv::waitKey(3);
                        //sensor_msgs::ImagePtr
                        //img_pub.publish();
                        t1 = std::chrono::high_resolution_clock::now();
                        render_time = std::chrono::duration_cast<ms>(t1 - t0).count();
                    }
                }

                if (isLastFrame) {
                    break;
                }

                if (isModeChanged) {
                    isModeChanged = false;
                }

                // Final point:
                // in the truly Async mode we swap the NEXT and CURRENT requests for the next iteration
                curr_frame = next_frame.clone();
                next_frame = cv::Mat();
                if (isAsyncMode) {
                    estimator.swapRequest();
                }
            }



     }




        return 0;

}

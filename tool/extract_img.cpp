#include "utility.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#define CAMERAEXTRINSICMAT "CameraExtrinsicMat"
#define CAMERAMAT "CameraMat"
#define DISTCOEFF "DistCoeff"
#define IMAGESIZE "ImageSize"

cv::Mat cameraExtrinsicMat_;    //camera laser calibration param
cv::Mat cameraMat_;             //camera internal parameter
cv::Mat distCoeff_;             //camera distortion parameter
cv::Size imageSize_;            //image size
cv::Mat R_;                     //lasr to image rotation matrix
cv::Mat T_;                     //lasr to image translation matrix



int main(int argc, char** argv){
    ros::init(argc, argv, "extract_img");
    
    std::string pathBag="/home/chamo/Documents/data/bag/2018-09-27-17-03-39_anticlockwise.bag";
    std::string out_img_root="/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/out/";

    std::vector<double> img_timestamps;
    rosbag::Bag bag;
    bag.open(pathBag, rosbag::bagmode::Read);
    rosbag::View view(bag);
    std::string image_topic="camera/image_color/compressed";
    ofstream outfile;
    outfile.open (out_img_root+ "traj.txt");
    if (!outfile.is_open())
    {
        std::cout<<"file not open"<<std::endl;
    }
    
    int img_count=100000;

    for (const rosbag::MessageInstance& m : view) {
        sensor_msgs::CompressedImagePtr simg = m.instantiate<sensor_msgs::CompressedImage>();
        if(simg!=NULL && m.getTopic() == image_topic)
        {
            cv_bridge::CvImagePtr cv_ptr;
            cv::Mat img_;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(simg, "bgra8");
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return 0;
            }
            img_count++; 
            cv_ptr->image.copyTo(img_);
            std::stringstream ss;
            ss<<img_count<<".jpg";
            cv::imwrite(out_img_root+ss.str(), img_);     
            std::stringstream ss1;
            ss1<<ss.str()<<",";
            ss1<<std::setprecision(20)<<simg->header.stamp.toSec()<<std::endl;
            outfile<<ss1.str();
        } 
    } 
    outfile.close();
    return 0;
}
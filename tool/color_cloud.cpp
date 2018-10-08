#include "utility.h"
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

std::vector<std::string> split(const std::string& str, const std::string& delim)
{
    std::vector<std::string> tokens;
    size_t prev = 0, pos = 0;
    do
    {
        pos = str.find(delim, prev);
        if (pos == std::string::npos) pos = str.length();
        std::string token = str.substr(prev, pos-prev);
        if (!token.empty()) tokens.push_back(token);
        prev = pos + delim.length();
    }
    while (pos < str.length() && prev < str.length());
    return tokens;
}

int readCalibPara(string filename)
{
    cv::FileStorage fs(filename,cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        std::cout<<"Invalid calibration filename."<<std::endl;
        return 0;
    }
    fs[CAMERAMAT]>>cameraMat_;
    fs[DISTCOEFF]>>distCoeff_;
    fs[CAMERAEXTRINSICMAT]>>cameraExtrinsicMat_;
    fs[IMAGESIZE]>>imageSize_;

    R_ = cameraExtrinsicMat_(cv::Rect(0,0,3,3));
    T_ = cameraExtrinsicMat_(cv::Rect(3,0,1,3));
    cout<<"cameraMat:"<<cameraMat_<<endl;
    cout<<"distCoeff:"<<distCoeff_<<endl;
    cout<<"cameraExtrinsicMat:"<<cameraExtrinsicMat_<<endl;
    cout<<"imageSize:"<<imageSize_<<endl;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "color_cloud");
    pcl::PointCloud<PointType>::Ptr laserCloudIn;
    laserCloudIn.reset(new pcl::PointCloud<PointType>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr laserCloudOut;
    laserCloudOut.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
    
    std::string img_root="/home/albert/Desktop/ParkingLot/img/";
    std::string traj_path="/home/albert/Desktop/ParkingLot/image_pose_clock_wise.txt";
    std::string point_cloud="/home/albert/Desktop/ParkingLot/point_cloud_merged.pcd";
    std::string root_save_folder="/home/albert/Desktop/ParkingLot/";
    
    //float fx, fy, cx, cy, k1, k2, p1, p2;
    
    //read camera intrinsic and extrinsic parameters
    readCalibPara("/home/albert/Desktop/ParkingLot/calib.yml");
    
    //read the poses
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
    std::ifstream infile(traj_path);
    while (true)
    {
        std::string transform_str;
        std::getline(infile, transform_str);
        if (transform_str==""){
            break;
        }
        std::vector<std::string> splited= split(transform_str, " ");
        PointTypePose pose_t;
        pose_t.time=std::stod(splited[1]);
        pose_t.x=std::stod(splited[2]);
        pose_t.y=std::stod(splited[3]);
        pose_t.z=std::stod(splited[4]);
        pose_t.roll=std::stod(splited[5]);
        pose_t.pitch=std::stod(splited[6]);
        pose_t.yaw=std::stod(splited[7]);
        pose_t.intensity = std::stod(splited[0]);
        cloudKeyPoses6D->push_back(pose_t);
    }
    
    //read point cloud
    if (pcl::io::loadPCDFile<PointType> (point_cloud, *laserCloudIn) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file pcd.pcd \n");
        return (-1);
    }
    std::stringstream img_filename;
    img_filename<<img_root<<"img_1.jpg";
    cv::Mat img = cv::imread(img_filename.str());
	
    //loop over all images to draw color into point cloud
    int cloudSize = laserCloudIn->points.size();
    for (int j = 0; j < cloudSize; ++j)    
    {
        pcl::PointXYZRGBA drawpoint;
        PointType *pointFrom;
	PointType pointTo;
	
	pointFrom = &laserCloudIn->points[j];
	
	drawpoint.x = laserCloudIn->points[j].x;
        drawpoint.y = laserCloudIn->points[j].y;
        drawpoint.z = laserCloudIn->points[j].z;
        drawpoint.b = 0;
        drawpoint.g = 0;
        drawpoint.r = 0;
        drawpoint.a = 0;
	    
	int r, g , b, observe_count;
	r = 0;
	g = 0;
	b = 0;
	observe_count = 0;
	//for (int i=0; i<cloudKeyPoses6D->size(); i++)
	for (int i=0; i<1; i++)  
        {
            //todo project the point into current frame, filter out the points outside the frame and get the color from the image

	    
	    //get image name
	    //std::stringstream img_filename;
	    //img_filename<<img_root<<"img_"<<std::to_string(int(cloudKeyPoses6D->points[i].intensity))<<".jpg";
	    //cv::Mat img = cv::imread(img_filename.str());
	
	    //std::cout<<img_filename.str()<<std::endl;	    
	    //transfer global to local
        
            float x1 = pointFrom->x - cloudKeyPoses6D->points[i].x;
            float y1 = pointFrom->y - cloudKeyPoses6D->points[i].y;
            float z1 = pointFrom->z - cloudKeyPoses6D->points[i].z;

	    float x2 = cos(-cloudKeyPoses6D->points[i].pitch) * x1 + sin(-cloudKeyPoses6D->points[i].pitch) * z1;
            float y2 = y1;
            float z2 = -sin(-cloudKeyPoses6D->points[i].pitch) * x1 + cos(-cloudKeyPoses6D->points[i].pitch) * z1;

            float x3 = x2;
            float y3 = cos(-cloudKeyPoses6D->points[i].roll) * y2 - sin(-cloudKeyPoses6D->points[i].roll) * z2;
            float z3 = sin(-cloudKeyPoses6D->points[i].roll) * y2 + cos(-cloudKeyPoses6D->points[i].roll)* z2;

            pointTo.x = cos(-cloudKeyPoses6D->points[i].yaw) * x3 - sin(-cloudKeyPoses6D->points[i].yaw) * y3;
            pointTo.y = sin(-cloudKeyPoses6D->points[i].yaw) * x3 + cos(-cloudKeyPoses6D->points[i].yaw) * y3;
            pointTo.z = z3;
            
	    //std::cout<<"Debug"<<pointTo.x<<"  "<<pointTo.y<<"  "<<pointTo.z<<std::endl;
 	    //transfer point cloud into camera frame
            cv::Mat point(3, 1, CV_64F);
            point.at<double>(0) =  pointTo.z;
            point.at<double>(1) = -pointTo.y;
            point.at<double>(2) =  pointTo.x;
	    //point.at<double>(0) =  pointFrom->z;
            //point.at<double>(1) =  -pointFrom->y;
            //point.at<double>(2) =  pointFrom->x; 
            point = R_ * point  + T_;
	    
	    //std::cout<<"Debug"<<point.at<double>(0)<<"  "<<point.at<double>(1)<<"  "<<point.at<double>(2)<<std::endl;
            if(point.at<double>(2)>0)
            {
                double tmpx = point.at<double>(0) / point.at<double>(2);
                double tmpy = point.at<double>(1)/point.at<double>(2);
                cv::Point2d imagepoint;
    //        double r2 = tmpx * tmpx + tmpy * tmpy;
    //        double tmpdist = 1 + distCoeff.at<double>(0) * r2
    //            + distCoeff.at<double>(1) * r2 * r2
    //            + distCoeff.at<double>(4) * r2 * r2 * r2;
    //        imagepoint.x = tmpx * tmpdist
    //            + 2 * distCoeff.at<double>(2) * tmpx * tmpy
    //            + distCoeff.at<double>(3) * (r2 + 2 * tmpx * tmpx);
    //        imagepoint.y = tmpy * tmpdist
    //            + distCoeff.at<double>(2) * (r2 + 2 * tmpy * tmpy)
    //            + 2 * distCoeff.at<double>(3) * tmpx * tmpy;

                imagepoint.x = tmpx;
                imagepoint.y = tmpy;

                imagepoint.x = cameraMat_.at<double>(0,0) * imagepoint.x + cameraMat_.at<double>(0,2);
                imagepoint.y = cameraMat_.at<double>(1,1) * imagepoint.y + cameraMat_.at<double>(1,2);
                int px = int(imagepoint.x + 0.5);
                int py = int(imagepoint.y + 0.5);
                if(0 <= px && px < imageSize_.width && 0 <= py && py < imageSize_.height)
                {
                    //int pid = py * w + px;
                    b += img.at<cv::Vec3b>(py,px)[0];
                    g += img.at<cv::Vec3b>(py,px)[1];
                    r += img.at<cv::Vec3b>(py,px)[2];
          	    ++observe_count;
		    std::cout<<"Debug  "<<observe_count<<std::endl;
		}//if cloud poind belongs to image                   
            }//if z>0
	}//itertor image
	if(observe_count > 0)
	{
	  drawpoint.b = (b / float(observe_count));
          drawpoint.g = (g / float(observe_count));
          drawpoint.r = (r / float(observe_count));
	  drawpoint.a = 255;
	}
	laserCloudOut->push_back(drawpoint);
    }//itertor cloud

    pcl::io::savePCDFile (root_save_folder+ "/point_cloud_colored.pcd", *laserCloudOut, true);
                
    return 0;
}

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
 #include "bgumodo_vision/GetDistance.h"

#include <iostream>


#define CALIBRATION

using namespace cv;
using namespace std;
using namespace pcl;

//ros::Publisher marker_point_pub;

int image_w=0,image_h=0;
pcl::PointCloud<pcl::PointXYZRGBA> cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

//vector<Point2f>  markers;

image_transport::Publisher image_pub_;
image_transport::Subscriber image_sub_;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud) ;

typedef pcl::PointXYZRGB rgbpoint;
typedef pcl::PointCloud<pcl::PointXYZRGB> cloudrgb;
typedef cloudrgb::Ptr cloudrgbptr;


bool get_distance_cb(bgumodo_vision::GetDistance::Request  &req, bgumodo_vision::GetDistance::Response &res)
 {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudp (new pcl::PointCloud<pcl::PointXYZRGBA> (cloud));

    if (!cloudp->empty()) {
        int h= req.u;
        int w= req.v;
        cout <<"u -   " << req.u <<endl;
        cout <<"v -   s" << req.v <<endl;
        int pcl_index = (h*image_w) + w; 

        // extract rgb and depth info from pixel
        pcl::PointXYZRGBA point = cloudp->at(pcl_index);

        cout <<"x----   " << point.x <<endl;
        cout <<"y----   " << point.y <<endl;
        cout <<"z----   " << point.z <<endl;
        res.x= point.x;
        res.y= point.y;
        res.z= point.z;
        

    }
    else cout <<"empty cloud"<<endl;

   
   return true;
 }


void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        Mat bgr=cv_ptr->image;

        if (image_w==0) image_w=bgr.cols;
        if (image_h==0) image_h=bgr.rows;


}


void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {

    if ((image_w==0)||(image_h==0)) return;
	

    pcl::fromROSMsg (*input, cloud);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudp (new pcl::PointCloud<pcl::PointXYZRGBA> (cloud));

    cv::Mat result;


    result = cv::Mat(image_h, image_w, CV_8UC3);

    if (!cloudp->empty()) {
        for (int h=0; h<image_h; h++) {
            for (int w=0; w<image_w; w++) {
		// convert 2d matrix index to 1d vector index
                int pcl_index = (h*image_w) + w;
		// extract rgb and depth info from pixel
                pcl::PointXYZRGBA point = cloudp->at(pcl_index);

                if (point.z>0.1) {// if pixel is not to close - put in rgb informatiom. 
                    Eigen::Vector3i rgb = point.getRGBVector3i();
                    result.at<cv::Vec3b>(h,w)[0] = point.r;
                    result.at<cv::Vec3b>(h,w)[1] = point.g;
                    result.at<cv::Vec3b>(h,w)[2] = point.b;
                }
                else {// if not make black
                    result.at<cv::Vec3b>(h,w)[0]=0;
                    result.at<cv::Vec3b>(h,w)[1]=0;
                    result.at<cv::Vec3b>(h,w)[2]=0;
                }
            }
        }

    }
    else cout <<"empty cloud"<<endl;

    cv_bridge::CvImage out_msg;
    out_msg.header.stamp =ros::Time::now(); // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image    = result; // Your cv::Mat


    // Output modified video stream
    image_pub_.publish(out_msg.toImageMsg());

}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "get_pcl");
    ros::NodeHandle n;
   image_transport::ImageTransport it_(n);

   image_sub_ = it_.subscribe("kinect2/hd/image_color", 1,imageCb);

    image_pub_ = it_.advertise("depth_test/output_video", 1);

    ros::Subscriber pcl_sub = n.subscribe("kinect2/hd/points", 1, cloud_cb);
  //  marker_point_pub = n.advertise<geometry_msgs::PointStamped>("marker_point", 10);
    ros::ServiceServer service = n.advertiseService("get_distance", get_distance_cb);
    ROS_INFO("Ready get_distance.");

   // ImageConverter ic;
    while (n.ok()) {
        char c=waitKey(1); //delay runnig
        if(c == 27) //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
        {
            cout << "esc key is pressed by user" << endl;
            break;
        }
        ros::spinOnce();
    }
    return 0;
}

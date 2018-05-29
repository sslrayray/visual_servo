#include "my_ms.h"
#include <iostream>

using namespace std;


// ROS image handling
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>


cv::Rect rect;
bool drawing_rect = false;
bool selected = false;
void mouse_cb(int event,int x,int y,int flag,void* param);

void rgb_to_yuv(cv::Mat& frame)
{
    // Y = R * .299 + G * .587 + B * .114;
    // U = R * -.169 + G * -.332 + B * .500 + 128.;
    // V = R * .500 + G * -.419 + B * -.0813 + 128.;

    int h = frame.rows;
    int w = frame.cols;

    float r, g, b;
    float y, u, v;
    cv::Vec3f curr_pixel_value;
    
    for (int i = 0; i < h; i++)
        for (int j = 0; j < w; j++)
        {
            curr_pixel_value = frame.at<cv::Vec3b>(i,j);
            r = curr_pixel_value[0];
            g = curr_pixel_value[1];
            b = curr_pixel_value[2];


            y = r * 0.299 + g * 0.587 + b * 0.114;
            u = r * -0.169 + g * -0.332 + b * 0.500 + 128.;
            v = r * 0.500 + g * -0.419 + b * -0.0813 + 128.;
            frame.at<cv::Vec3b>(i,j)[0] = y;
            frame.at<cv::Vec3b>(i,j)[1] = u;
            frame.at<cv::Vec3b>(i,j)[2] = v;
               
        }
}

void previous_main()
{
    // cv::VideoCapture frame_capture; /*!< Detailed description after the member */ 
    // frame_capture = cv::VideoCapture("test.webm");

    // cv::Mat frame,temp_img;
    // frame_capture.read(frame);
    
    // cv::namedWindow("image_show");
    // temp_img = frame.clone();
    // cv::setMouseCallback("image_show",mouse_cb,(void*) &temp_img);
    // while(selected == false)
    // {
    //     cv::Mat temp = temp_img.clone()  ;
    //     if( drawing_rect )
    //         cv::rectangle( temp, rect,cv::Scalar(0,0,255),2);
    //     cv::imshow("image_show", temp );

    //     if( cv::waitKey( 15 )==27 )
    //         break;
    // }

    // // creat meanshift obj
    // MeanShift ms;
    // // init the meanshift
    // // rgb_to_yuv(frame);
    // imshow("image_show", frame);
    
    // // cv::waitKey( 0);

    // ms.calc_target_pdf(frame,rect);

    // // return;

    // cv::VideoWriter writer("tracking_result.avi",CV_FOURCC('M','J','P','G'),20,cv::Size(frame.cols,frame.rows));

    // while(1)
    // {

    //     if(!frame_capture.read(frame))
    //         break;  
    //     temp_img = frame.clone();
    //     // rgb_to_yuv(frame);
    //     cv::Rect ms_rect =  ms.track(frame);

    //     //show the tracking reslut;
    //     cv::rectangle(temp_img,ms_rect,cv::Scalar(0,0,255),3);
    //     writer<< temp_img;
    //     cv::imshow("image_show",temp_img);
    //     cv::waitKey(25);

    // }

    // cv::waitKey( 0);
    // cv::destroyWindow("image_show");
}

int g_img_trigger = 1;

void img_callback(const sensor_msgs::ImageConstPtr& img_msg)
{

    if (g_img_trigger == 1)
    try
    {
        cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");

        // Display in a GUI
        // namedWindow("Display Image", WINDOW_AUTOSIZE );
        // imshow("Display Image", bridge_ptr->image);
        //waitKey(100);
        string filename("/home/ylei/image2.png");
        cout << "saved images" << endl;
        imwrite(filename, bridge_ptr->image);
        g_img_trigger = 0;


        // cv::namedWindow("image_show");
        // temp_img = frame.clone();
        // cv::setMouseCallback("image_show",mouse_cb,(void*) &temp_img);
        // while(selected == false)
        // {
        //     cv::Mat temp = temp_img.clone()  ;
        //     if( drawing_rect )
        //         cv::rectangle( temp, rect,cv::Scalar(0,0,255),2);
        // imshow("image_show", bridge_ptr->image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img_msg->encoding.c_str());
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "meanshift");
    ros::NodeHandle node_handle;

    ros::Subscriber sub_img = node_handle.subscribe("/vrep/image", 1, img_callback);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::waitForShutdown();

    return 0;
}

void mouse_cb(int event,int x,int y,int flag,void* param)
{
    // cv::Mat *image = (cv::Mat*) param;
    // switch( event ){
    //     case CV_EVENT_MOUSEMOVE:
    //         if( drawing_rect ){
    //             rect.width = x-rect.x;
    //             rect.height = y-rect.y;
    //         }
    //         break;

    //     case CV_EVENT_LBUTTONDOWN:
    //         drawing_rect = true;
    //         rect = cv::Rect( x, y, 0, 0 );
    //         break;

    //     case CV_EVENT_LBUTTONUP:
    //         drawing_rect = false;
    //         if( rect.width < 0 ){
    //             rect.x += rect.width;
    //             rect.width *= -1;
    //         }
    //         if( rect.height < 0 ){
    //             rect.y += rect.height;
    //             rect.height *= -1;
    //         }
    //         cv::rectangle(*image,rect,cv::Scalar(0),2);
    //         selected = true;
    //         break;
    // }

}

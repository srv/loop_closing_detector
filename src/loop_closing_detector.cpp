#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <signal.h>
#include <iostream>
#include <numeric>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>
#include <boost/filesystem.hpp>
#include "libhaloc/lc.h"

namespace enc = sensor_msgs::image_encodings;
namespace fs=boost::filesystem;

using namespace std;

class LoopClosingDetector
{
  public:

    /** \brief LoopClosingDetector class constructor
     */
    LoopClosingDetector(ros::NodeHandle nh, ros::NodeHandle nhp) : nh_(nh), nh_private_(nhp)
    {
      // Read the parameters
      readParams();

      // Initialize the class
      init();
    }

    /** \brief Finalize the node
      * @return
      */
    void finalize()
    {
      lc_.finalize();
    }

    /** \brief Messages callback. This function is called when synchronized image
      * messages are received.
      * @return
      * \param l_img left stereo image message of type sensor_msgs::Image
      * \param r_img right stereo image message of type sensor_msgs::Image
      * \param l_info left stereo info message of type sensor_msgs::CameraInfo
      * \param r_info right stereo info message of type sensor_msgs::CameraInfo
      */
    void msgsCallback(const sensor_msgs::ImageConstPtr& l_img_msg,
                      const sensor_msgs::ImageConstPtr& r_img_msg,
                      const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                      const sensor_msgs::CameraInfoConstPtr& r_info_msg)
    {
      // Get the time
      double timestamp = l_img_msg->header.stamp.toSec()*1000000000;

      // Take one image every n seconds
      if (time_hist_.size() > 0)
      {
        if (timestamp - time_hist_.back() < every_n_seconds_)
          return;
      }

      ROS_INFO_STREAM("Processing image " << cur_id_ << " with timestamp " << timestamp);

      // Save the time
      time_hist_.push_back(timestamp);

      // Get the images from message
      Mat l_img, r_img;
      getImages(*l_img_msg, *r_img_msg, *l_info_msg, *r_info_msg, l_img, r_img);

      // Save this node for loop closure detection
      lc_.setNode(l_img, r_img, boost::lexical_cast<string>(cur_id_));
      cur_id_++;

      // Detect loop closure
      int lc_img_id;
      string lc_img_name;
      tf::Transform edge;
      if (!lc_.getLoopClosure(lc_img_id, lc_img_name, edge)) return;

      // Save images
      std::stringstream l_ss, r_ss;
      l_ss << work_dir_ << "/img/l_" << to_string(timestamp) << ".jpg";
      r_ss << work_dir_ << "/img/r_" << to_string(timestamp) << ".jpg";
      string l_file = l_ss.str();
      string r_file = r_ss.str();
      imwrite(l_file, l_img);
      imwrite(r_file, r_img);

      // Save loop closure to file
      string out_file = work_dir_ + "/loop_closures.txt";

      // Open to append
      fstream f_out(out_file.c_str(), fstream::out | fstream::app);
      f_out << fixed << setprecision(0) <<
            timestamp << "," <<
            time_hist_[lc_img_id] << "," <<
            setprecision(6) <<
            edge.getOrigin().x() << "," <<
            edge.getOrigin().y() << "," <<
            edge.getOrigin().z() << "," <<
            edge.getRotation().x() << "," <<
            edge.getRotation().y() << "," <<
            edge.getRotation().z() << "," <<
            edge.getRotation().w() <<  endl;
      f_out.close();
  }

  protected:

    // Node handlers
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

  private:

    // Properties
    haloc::LoopClosure lc_;
    bool first_iter_;
    int cur_id_;
    double every_n_seconds_;
    vector<double> time_hist_;
    string work_dir_;

    // Topic properties
    image_transport::SubscriberFilter left_sub_, right_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_, right_info_sub_;

    // Topic sync properties
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
                                                      sensor_msgs::Image,
                                                      sensor_msgs::CameraInfo,
                                                      sensor_msgs::CameraInfo> ExactPolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    boost::shared_ptr<ExactSync> exact_sync_;


    /** \brief Init the class
     */
    void init()
    {
      first_iter_ = true;
      cur_id_ = 1;

      // Init Haloc
      lc_.init();

      // Callback synchronization
      exact_sync_.reset(new ExactSync(ExactPolicy(5),
                                      left_sub_,
                                      right_sub_,
                                      left_info_sub_,
                                      right_info_sub_) );
      exact_sync_->registerCallback(boost::bind(
          &LoopClosingDetector::msgsCallback,
          this, _1, _2, _3, _4));

      // Output file header
      string out_file = work_dir_ + "/loop_closures.txt";
      remove(out_file.c_str());
      fstream f_out(out_file.c_str(), fstream::out | fstream::app);
      f_out << "%timestamp_image_A,timestamp_image_B,x,y,z,qx,qy,qz,qw" <<  endl;
      f_out.close();
    }

    /** \brief Read the parameters from the ros parameter server
     */
    void readParams()
    {
      // Topic parameters
      string left_topic, right_topic, left_info_topic, right_info_topic;
      nh_private_.param("left_topic", left_topic, string("/left/image_rect_color"));
      nh_private_.param("right_topic", right_topic, string("/right/image_rect_color"));
      nh_private_.param("left_info_topic", left_info_topic, string("/left/camera_info"));
      nh_private_.param("right_info_topic", right_info_topic, string("/right/camera_info"));
      nh_private_.getParam("every_n_seconds", every_n_seconds_);

      // Working directory
      nh_private_.param("work_dir", work_dir_, string(""));

      // Working directory sanity check
      if (work_dir_[work_dir_.length()-1] == '/')
        work_dir_ = work_dir_.substr(0, work_dir_.size()-1);

      // Create the directory to store the keypoints and descriptors
      if (fs::is_directory(work_dir_ + "/img"))
        fs::remove_all(work_dir_ + "/img");
      fs::path dir(work_dir_ + "/img");
      if (!fs::create_directory(dir))
        ROS_ERROR("ERROR -> Impossible to create the execution directory.");

      // Loop closing parameters
      haloc::LoopClosure::Params lc_params;
      lc_params.num_proj = 2;
      lc_params.verbose = true;
      nh_private_.param("desc_type", lc_params.desc_type, string("SIFT"));
      nh_private_.getParam("desc_thresh_ratio", lc_params.desc_thresh_ratio);
      nh_private_.getParam("min_neighbour", lc_params.min_neighbour);
      nh_private_.getParam("n_candidates", lc_params.n_candidates);
      nh_private_.getParam("min_matches", lc_params.min_matches);
      nh_private_.getParam("min_inliers", lc_params.min_inliers);
      lc_params.work_dir = work_dir_;
      lc_.setParams(lc_params);

      // Topics subscriptions
      image_transport::ImageTransport it(nh_);
      left_sub_ .subscribe(it, left_topic, 1);
      right_sub_.subscribe(it, right_topic, 1);
      left_info_sub_.subscribe(nh_, left_info_topic, 1);
      right_info_sub_.subscribe(nh_, right_info_topic, 1);
    }

    template <typename T>
    std::string to_string(T const& value) {
        stringstream sstr;
        sstr << setprecision(19) << value;
        return sstr.str();
    }

    bool getImages(sensor_msgs::Image l_img_msg,
                   sensor_msgs::Image r_img_msg,
                   sensor_msgs::CameraInfo l_info_msg,
                   sensor_msgs::CameraInfo r_info_msg,
                   Mat &l_img, Mat &r_img)
    {
      // Convert message to Mat
      try
      {
        l_img = (cv_bridge::toCvCopy(l_img_msg, enc::BGR8))->image;
        r_img = (cv_bridge::toCvCopy(r_img_msg, enc::BGR8))->image;
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("[StereoSlam:] cv_bridge exception: %s", e.what());
        return false;
      }

      // Set camera model (only once)
      if (first_iter_)
      {
        // Get the stereo camera model
        image_geometry::StereoCameraModel stereo_camera_model;
        stereo_camera_model.fromCameraInfo(l_info_msg, r_info_msg);

        // Get the projection/camera matrix
        const Mat P(3,4, CV_64FC1, const_cast<double*>(l_info_msg.P.data()));
        Mat camera_matrix = P.colRange(Range(0,3)).clone();

        // Are the images scaled?
        int binning_x = l_info_msg.binning_x;
        int binning_y = l_info_msg.binning_y;
        if (binning_x > 1 || binning_y > 1)
        {
          camera_matrix.at<double>(0,0) = camera_matrix.at<double>(0,0) / binning_x;
          camera_matrix.at<double>(0,2) = camera_matrix.at<double>(0,2) / binning_x;
          camera_matrix.at<double>(1,1) = camera_matrix.at<double>(1,1) / binning_y;
          camera_matrix.at<double>(1,2) = camera_matrix.at<double>(1,2) / binning_y;
        }

        // Set all for the loop closure class
        lc_.setCameraModel(stereo_camera_model, camera_matrix);
        first_iter_ = false;
      }
      return true;
    }
};

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void sigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    g_request_shutdown = 1; // Set flag
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}

// Main entry point
int main(int argc, char **argv)
{
  // Override SIGINT handler
  ros::init(argc, argv, "loop_closing_detector", ros::init_options::NoSigintHandler);
  signal(SIGINT, sigIntHandler);

  // Override XMLRPC shutdown
  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

  // Stereo slam class
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  LoopClosingDetector loop_closing_detector(nh,nh_private);

  // Do our own spin loop
  while (!g_request_shutdown)
  {
    // Do non-callback stuff
    ros::spinOnce();
    usleep(100000);
  }

  // Finalize stereo slam
  loop_closing_detector.finalize();

  // Exit
  ros::shutdown();
  return 0;
}


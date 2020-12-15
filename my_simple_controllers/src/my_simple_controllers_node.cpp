#include "ros/ros.h"
#include "string.h"
#include "sensor_msgs/JointState.h"
#include "ros/publisher.h"
#include <sstream>
#include "ros/node_handle.h"
#include "joint_state_controller/joint_state_controller.h"
#include "position_controllers/joint_group_position_controller.h"
#include "eigen3/Eigen/Eigen"
#include "eigen3/Eigen/Dense"
#include "std_msgs/Float64.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "ros/timer.h"
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include "tf/transform_broadcaster.h"
#include <geometry_msgs/TransformStamped.h>
#include "tf_conversions/tf_eigen.h"
#include "eigen_conversions/eigen_msg.h"
#include <geometry_msgs/TransformStamped.h>
#include "tf/transform_datatypes.h"
#include <kdl_parser/kdl_parser.hpp>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include "kdl/chainiksolver.hpp"
#include "eigen_conversions/eigen_kdl.h"
#include "kdl/chainiksolvervel_pinv.hpp"
#include "tf/transform_listener.h"
#include "kdl/chainiksolverpos_nr.hpp"

#include "std_srvs/Empty.h"
using namespace KDL;
using std::cout;
using std::endl;
using namespace cv;
using namespace std;
class VisualServoing
{
public:
  VisualServoing():nh_("~") {


    pubs.resize(joints);
    qtarget << 0, 0.0, 0, -M_PI/2, 0, M_PI, 0;
    qcurr << 0, 0.0, 0, -M_PI/2, 0, M_PI, 0;
    for(int i=0 ; i<joints ; i++){
      std::string topic = prefix+std::to_string(i+1)+suffix;
      //cout<<"advertize: "<<topic<<endl;
      pubs[i] = nh_.advertise<std_msgs::Float64>(topic, 50);
    }


    ros::NodeHandle node;
    std::string robot_desc_string;
    node.param("robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
      ROS_ERROR("Failed to construct kdl tree - no model on parameter server");
      exit(0);
    }
    my_tree.getChain("panda_link0", "camera_link", chain);

    Eigen::Affine3d Tinit = create_rotation_matrix(0, 0, 0.0*M_PI/180.0);
    Tinit.translation()<<0.3, 0.1, 0.75;
    Ik(Tinit, qtarget);

    sub_jntstate = nh_.subscribe("/joint_states", 1000, &VisualServoing::jntCallback, this);
    timer_joint_publish = nh_.createTimer(ros::Duration(0.1), &VisualServoing::PublishCallback, this);
    cv::namedWindow("view");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh_);
    sub = it.subscribe("/rrbot/camera1/image_raw", 10, &VisualServoing::imageCallback, this);
    service = nh_.advertiseService("/enable_tracking", &VisualServoing::callback, this);
  }
  bool callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
  {
    enable_tracking =! enable_tracking;
    return true;
  }



  Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az) {
    Eigen::Affine3d rx =
        Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
    Eigen::Affine3d ry =
        Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
    Eigen::Affine3d rz =
        Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
    return rz * ry * rx;
  }
  void PixelRadToPose(double radius, double ey, double ez, Eigen::Affine3d& TargetPoseDiff){

    TargetPoseDiff = create_rotation_matrix(0, ey, ez);
    TargetPoseDiff.translation()<<0,0,0;
    //cout<<"radius: "<<radius<<", ez: "<<ez<<", ey"<<ey<<endl;

  }

  void jntCallback(const sensor_msgs::JointState::ConstPtr msg){
    m_curr.lock();
    for(int i=0;i<msg->position.size();i++){
      qcurr[i] = msg->position[i];
    }
    //cout<<"current: "<<qcurr.transpose()<<endl;
    m_curr.unlock();
  }
  bool FK( Eigen::Affine3d& Tend){

    // Create solver based on kinematic chain
    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);

    // Create joint array
    unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray jointpositions = JntArray(nj);


    // Assign some values to the joint positions
    m_curr.lock();
    for(unsigned int i=0;i<nj;i++)
      jointpositions(i) = qcurr(i);
    m_curr.unlock();


    // Create the frame that will contain the results
    KDL::Frame cartpos;

    // Calculate forward position kinematics
    bool kinematics_status;
    kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
    if(kinematics_status>=0){
      //std::cout << cartpos <<std::endl;
      printf("%s \n","Succes, thanks KDL!");
      tf::transformKDLToEigen( cartpos, Tend);
    }else{
      printf("%s \n","Error: could not calculate forward kinematics :(");
    }
  }
  int Ik(const Eigen::Affine3d& Tpose, Eigen::Matrix<double,7,1>& qsol){

    //cout<<"target: "<<Tpose.translation().transpose()<<"\n"<<Tpose.linear()<<endl;
    KDL::Frame target;

    tf::transformEigenToKDL(Tpose,target);
    KDL::ChainFkSolverPos_recursive fksolver1(chain);//Forward position solver
    ChainIkSolverVel_pinv iksolver1v(chain);//Inverse velocity solver
    KDL::ChainIkSolverPos_NR  iksolver1(chain, fksolver1, iksolver1v,100,1e-6);
    KDL::JntArray q(chain.getNrOfJoints());
    KDL::JntArray q_init(chain.getNrOfJoints());
    m_curr.lock();
    for(int i=0;i<7;i++){
      q_init(i) = qcurr(i);
      q(i) = qcurr(i);
    }
    m_curr.unlock();
    int ret = iksolver1.CartToJnt(q_init, target, q);

    //cout<<"res. "<<ret<<", problem size: "<<chain.getNrOfJoints()<<endl;
    m_curr.lock();
    for(int i=0;i<7;i++){
      qsol(i) = q(i);
      //cout<<"solution: "<<i<<"= "<<q(i)<<", prev: "<<qcurr(i)<<endl;
    }
    m_curr.unlock();
    if( ret !=KDL::ChainIkSolverPos_NR::E_NOERROR){
      cout<<"error: "<<iksolver1.strError(ret)<<endl;
    }
    return ret;
  }
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
      cv::Mat src = cv_bridge::toCvShare(msg, "bgr8")->image;

      cv::Mat gray;
      cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
      medianBlur(gray, gray, 5);

      std::vector<cv::Vec3f> circles;
      HoughCircles(gray, circles, HOUGH_GRADIENT, 1,
                   gray.rows/16,  // change this value to detect circles with different distances to each other
                   100, 30, 0, 0// change the last two parameters
                   // (min_radius & max_radius) to detect larger circles
                   );
      if(circles.size()==1){
        Vec3i c;
        Point center;
        double radius;
        for( size_t i = 0; i < circles.size(); i++ )
        {
          c = circles[i];
          center = Point(c[0], c[1]);
          // circle center
          circle( src, center, 1, Scalar(0,100,100), 3, LINE_AA);
          // circle outline
          radius = c[2];
          circle( src, center, radius, Scalar(255,0,255), 3, LINE_AA);
        }
        imshow("detected circles", src);
        //cout<<"center: "<<center.x<<", "<<center.y<<endl;
        Eigen::Affine3d Tobj_error = Eigen::Affine3d::Identity();
        double Fov_half = (M_PI/180.0)/2.0*45; // 10 deg
        Eigen::Vector2d bearing;
        double max_pixel = gray.rows/2.0;
        double z_normal = -(center.y - gray.rows/2.0)/(max_pixel);
        double y_normal = (center.x - gray.rows/2.0)/(max_pixel);
        //cout<<"z_normal: "<<z_normal<<", y_normal: "<<y_normal<<endl;
        double ey = -z_normal*Fov_half;
        double ez = -y_normal*Fov_half;
        cout<<"because z_normal "<<z_normal*max_pixel<<"rotate ey: "<<ey<<endl;
        cout<<"because y_normal "<<y_normal*max_pixel<<"rotate ez: "<<ez<<endl;

        PixelRadToPose(radius, ey,ez, Tobj_error);

        tf::Transform Tf; // publish
        tf::transformEigenToTF(Tobj_error, Tf);
        std::vector<tf::StampedTransform> trans_vek;
        trans_vek.push_back(tf::StampedTransform(Tf, msg->header.stamp, "/camera_link", "/taraget_pose")) ;
        Tb.sendTransform(trans_vek);
        if(enable_tracking){

          static tf::TransformListener listener;
          tf::StampedTransform transform;


          try{
            listener.lookupTransform("panda_link0","camera_link"
                                     ,msg->header.stamp, transform);
          }
          catch (tf::TransformException ex){
            return;
          }
          Eigen::Affine3d Tcurrent, Tgoal;
          tf::poseTFToEigen(transform, Tcurrent);
          //cout<<"tf: pose"<<Tcurrent.translation().transpose()<<endl;



          Tgoal = Tcurrent*Tobj_error;
          Eigen::Vector3d euler = Tgoal.linear().eulerAngles(0,1,2);
          static double first_orientation_x = Tcurrent.linear().eulerAngles(0,1,2)(0) ;
          static Eigen::Vector3d first = Tcurrent.translation();
          Eigen::Affine3d noroll = create_rotation_matrix(3.14,euler(1),euler(2));
          noroll.translation() = first;
          cout<<"orient: "<<noroll.linear().eulerAngles(0,1,2).transpose()<<endl;

          Eigen::Matrix<double,7,1> q;
          if(Ik(noroll, q)==KDL::ChainIkSolverPos_NR::E_NOERROR)
          {
            m_tar.lock();

            for(int i=0;i<7;i++){
              qtarget(i) = q(i);
            }

            m_tar.unlock();
          }
          else{

          }
        }


      }
      // cv::imshow("view", m);
      //cv::waitKey(30);

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
  }


  void PublishCallback(const ros::TimerEvent&){
    std_msgs::Float64 msg;
    m_tar.lock();
    for(int i=0;i<joints;i++){
      msg.data = qtarget(i);
      pubs[i].publish(msg);
    }
    m_tar.unlock();
  }
  /* sensor_msgs::JointState GetDefault(){
                                            Eigen::Matrix<double,6,1> q;
                                            std::vector<std::string> names = { "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4" , "panda_joint5" , "panda_joint6", "panda_joint7" };
                                            j.name = names;
                                            std::vector<double> positions(7,0.5); //= { 0.5, 0, 0, 0, 0, 0, 0  };
                                            j.position = positions;
                                            j.effort.resize(names.size(),20);
                                            j.velocity.resize(names.size(),1);


                                            j.header.stamp = ros::Time::now();


                                            return j;
                                          }*/



private:

  ros::NodeHandle nh_;
  KDL::Tree my_tree;
  KDL::Chain chain;
  std::string prefix="/joint", suffix="_position_controller/command";
  sensor_msgs::JointState J;
  std::vector<ros::Publisher> pubs;
  std::mutex m_tar, m_curr;
  Eigen::Matrix<double,7,1> qtarget;
  Eigen::Matrix<double,7,1> qcurr;
  int joints = 7;
  tf::TransformBroadcaster Tb;
  image_transport::Subscriber sub;
  ros::Timer timer_joint_publish;
  ros::Subscriber sub_jntstate;
  ros::ServiceServer service ;
  bool enable_tracking = false;
  Eigen::Affine3d Torigin;
  Eigen::Affine3d Tdiff = Eigen::Affine3d::Identity();
};


int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "talker");

  VisualServoing serv;
  ros::spin();

  cv::destroyWindow("view");
  return 0;
}

#include <iostream>
#include <pcl/point_types.h>
#include <std_msgs/Int32.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <visualization_msgs/Marker.h>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/filters/passthrough.h>
#include "kalman_filter.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


MatrixXd CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  Hj << 0,0,0,0,
        0,0,0,0,
        0,0,0,0;

  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  //check division by zero
  if(fabs(c1) < 0.0001){
    std::cout << "Function CalculateJacobian() has Error: Division by Zero" << std::endl;
    return Hj;
  }


  //compute the Jacobian matrix
  Hj << (px/c2),                (py/c2),                0,      0,
        -(py/c1),               (px/c1),                0,      0, 
        py*(vx*py - vy*px)/c3,  px*(px*vy - py*vx)/c3,  px/c2,  py/c2;

  return Hj;

}

pcl::PointCloud<pcl::PointXYZI>::Ptr passThroughFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (3.0, 11.0);
    pass.filter (*cloud);
    pass.setInputCloud (cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits (-3.0, 3.0);
    pass.filter (*cloud);

    return cloud;
}

std::vector<pcl::PointIndices> euclidean_cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){
 
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud (cloud);  

  std::vector<pcl::PointIndices> cluster_indices;

  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance (1); // 1m
  ec.setMinClusterSize (3);
  ec.setMaxClusterSize (10);//Declare Kalman Filter Parameters
KalmanFilter *ekf_;
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  ROS_INFO_STREAM ("No of clusters: " << cluster_indices.size () << std::endl); 

  return cluster_indices;
}

float prev_x = 1.f;
float prev_y = 1.f; 

visualization_msgs::Marker mark_cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster, std::string ns ,int id, float r, float g, float b, KalmanFilter *ekf_) 
{ 
  Eigen::Vector3f centroid; 
  Eigen::Vector3f min( std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()); 
  Eigen::Vector3f max( -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max()); 
  
  // pcl::compute3DCentroid (*cloud_cluster, centroid); 
  // pcl::getMinMax3D (*cloud_cluster, min, max); 
  
  for(int i=0; i<cloud_cluster->size(); i++){
    
    min[0] = std::min( cloud_cluster->points[i].x, min[0] );
    min[1] = std::min( cloud_cluster->points[i].y, min[1] );
    min[2] = std::min( cloud_cluster->points[i].z, min[2] );

    max[0] = std::max( cloud_cluster->points[i].x, max[0] );
    max[1] = std::max( cloud_cluster->points[i].y, max[1] );
    max[2] = std::max( cloud_cluster->points[i].z, max[2] ); 
  }

  centroid =( max + min) /2.f;

  // //kalman filter
  float dt = 1/30.f;
  VectorXd z = VectorXd(2);
  // ROS_INFO_STREAM("current state: " << centroid[0] <" " << prev_x << " " <<(centroid[0] - prev_x) / dt << std::endl);
  z << centroid[0], centroid[1]; // measurement x,y
  // ROS_INFO_STREAM( "current Measurement" << "\n" <<z << std::endl);

  ekf_->Predict();
  // ROS_INFO_STREAM( "\n Predict state: " << "\n" << ekf_->x_ << "\n" <<"Predict P: " << "\n" << ekf_->P_ << std::endl);
  // ekf_->H_ = CalculateJacobian(ekf_->x_);

  ekf_->Update(z);
  // std::cout << "ekf x: " << ekf_->x_ <<std::endl;

  ROS_INFO_STREAM( "update state: " << "\n" << ekf_->x_ << "\n" << "update P: " <<"\n" << ekf_->P_ << std::endl);

  uint32_t shape = visualization_msgs::Marker::CUBE; 
  visualization_msgs::Marker marker; 

  marker.header.frame_id = "/ti_mmwave";
  marker.header.stamp = ros::Time::now(); 
  
  marker.ns = ns; 
  marker.id = id; 
  marker.type = shape; 
  marker.action = visualization_msgs::Marker::ADD; 
  
  marker.pose.position.x = ekf_->x_[0]; 
  marker.pose.position.y = ekf_->x_[1]; 
  marker.pose.position.z = centroid[2]; 
  marker.pose.orientation.x = 0.0; 
  marker.pose.orientation.y = 0.0; 
  marker.pose.orientation.z = 0.0; 
  marker.pose.orientation.w = 1.0; 
  
  marker.scale.x = (max[0]-min[0]); 
  marker.scale.y = (max[1]-min[1]); 
  marker.scale.z = (max[2]-min[2]); 
  
  if (marker.scale.x ==0) 
      marker.scale.x=0.1; 

  if (marker.scale.y ==0) 
    marker.scale.y=0.1; 

  if (marker.scale.z ==0) 
    marker.scale.z=0.1; 
    
  marker.color.r = r; 
  marker.color.g = g; 
  marker.color.b = b; 
  marker.color.a = 0.5; 

  marker.lifetime = ros::Duration(); 
  marker.lifetime = ros::Duration(2.0); 
  return marker; 
}


class SubscribeAndPublish
{
public:
  //Declare Kalman Filter Parameters
  KalmanFilter *ekf_ = new KalmanFilter;

  SubscribeAndPublish()
  {

    //Topic you want to publish
    pub_ = n_.advertise<sensor_msgs::PointCloud2>("/cloud_cluster", 1);

    pub_marker_ = n_.advertise<visualization_msgs::Marker>("/bounding_box", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/mmWaveDataHdl/RScan", 1, &SubscribeAndPublish::callback, this);

    //kalman filter Initialization
    // set measurement noises
    float noise_ax = 9;
    float noise_ay = 9;
    float dt = 1/30.f;  // time elapsed 
    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;
    VectorXd x_in = VectorXd(4);
    MatrixXd P_in = MatrixXd(4, 4);
    MatrixXd F_in = MatrixXd(4, 4);
    MatrixXd H_in = MatrixXd(2, 4);
    MatrixXd R_in = MatrixXd(2, 2);
    MatrixXd Q_in = MatrixXd(4, 4); 

    x_in << 1,1,1,1;

    P_in << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1000, 0,
                0, 0, 0, 1000;
    
    F_in << 1, 0, dt, 0,
                0, 1, 0, dt,
                0, 0, 1, 0,
                0, 0, 0, 1;

    R_in << 0.0225, 0,
              0, 0.0225;

    H_in << 1, 0, 0, 0,
              0, 1, 0, 0;
  
    Q_in <<  dt_4/4*noise_ax,   0,                dt_3/2*noise_ax,  0,
                0,                 dt_4/4*noise_ay,  0,                dt_3/2*noise_ay,
                dt_3/2*noise_ax,   0,                dt_2*noise_ax,    0,
                0,                 dt_3/2*noise_ay,  0,                dt_2*noise_ay;
                dt_3/2*noise_ay,  0,                dt_2*noise_ay;
    
    ROS_INFO_STREAM("Initialize EKF"<<std::endl);
    ekf_->Init(x_in, P_in, F_in, H_in, R_in, Q_in);
    
  }

  void callback(const sensor_msgs::PointCloud2ConstPtr& input)
  {
 
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    
    int count_valid = 0;  

    // Publish the data.
    // ROS_INFO("+++++++++++++++++++++START+++++++++++++++++++++++++++ %s", "data");
    // for (uint j=0; j < cloud->points.size(); j++){
    //           float x = cloud->points[j].x;
    //           float y = cloud->points[j].y;
    //           float intensity = cloud->points[j].intensity;
    //           ROS_INFO("x:%f, y:%f", x, y);
    //           count_valid++;
    // }
    // ROS_INFO("+++++++++++++++++++++SUMMARY++++++++++++++++++++++++ %d", count_valid);



    // sorted out noise with passthrough filter
    // cloud = passThroughFilter(cloud);


    // Euclidean Clustering
    std::vector<pcl::PointIndices> cluster_indices;
    cluster_indices = euclidean_cluster(cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr drone_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    int max_points_cloud = 0; // record the number of most points cloud 

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);

      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (cloud->points[*pit]); //*


      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
      

      
      // define the drone cluster
      if(cloud_cluster->points.size() > max_points_cloud){
        max_points_cloud = cloud_cluster->points.size();
        drone_cloud = cloud_cluster;
      }

      // merge the clusters
      *clustered_cloud += *cloud_cluster;

      j++;
    }

    // publish the clusters
    sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2 pcl_pc2_2;
    
    pcl::toPCLPointCloud2(*clustered_cloud, pcl_pc2_2);
    pcl_conversions::fromPCL(pcl_pc2_2, output);
    output.header.frame_id = "/ti_mmwave";
    output.header.stamp = ros::Time::now();
    pub_.publish(output);
    ROS_INFO("POINTLENGTH: %u", cloud->points.size());
    std::cout <<  "VALID: " << count_valid << std::endl;
    ROS_INFO("---");

    // draw bounding box
    if (!drone_cloud->empty()){      
      visualization_msgs::Marker marker =  mark_cluster(drone_cloud, "drone", 1, 0, 255, 0, ekf_);
      pub_marker_.publish(marker);
    }

  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Publisher pub_marker_;
  ros::Subscriber sub_;


};//End of class SubscribeAndPublish


int main (int argc, char** argv)
{

    ros::init(argc, argv, "cluster_extraction");

  
    SubscribeAndPublish SAPObject;

    ros::spin();

    return (0);
}
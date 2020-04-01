//
// Created by vickylzy on 19-12-20.
//

#include <ros/ros.h>
#include <ros/timer.h>
// #include "time.h"
#include "std_msgs/String.h"
// ros_time

// ros_msg
#include <prm_localization/DriveInfo.h>
#include <nav_msgs/Odometry.h>
//ros srv
#include <prm_localization/locali.h>
#include <std_srvs/Trigger.h>
//pcl
#include <pclomp/ndt_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

//eigen
#include <Eigen/Dense>
//tf
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
//cpp
#include <ctime>
#include <limits.h>
#include <mutex>
#include <math.h>
#include <boost/circular_buffer.hpp>
//untility
#include <prm_localization/transform_utility.hpp>
#include <prm_localization/csv_transform_reader.hpp>


using namespace std;

class Localizer_global {

public:
    Localizer_global(){

    }
    virtual  ~Localizer_global(){
    }

    void onInit()  {
        //param
        p_nh = ros::NodeHandle("~");
        string global_pcd_path;
        string csv_path;
        p_nh.param("downsample_resolution",downsample_res,0.1f);
        p_nh.param("TransformationEpsilon",TransformationEpsilon,0.0001f);
        p_nh.param("ndt_resolution",ndt_resolution,0.5f);
        p_nh.param("search_radius",search_radius,60.0f);
        p_nh.param<string>("map_tf",map_tf,"map_tf");
        p_nh.param<string>("global_pcd_path",global_pcd_path,"/home/vickylzy/workspaceROS/MAP_BAG/wuhan/Transform_map_GNSS_3dbag_ls700b_1204_1/wuhan_ls_great.pcd");
        p_nh.param<string>("base_lidar_tf",lidar_tf,"base_lidar_tf");
        p_nh.param<string>("base_foot_tf",base_tf,"base_foot_tf");
        p_nh.param<string>("csv_path",csv_path,"/home/vickylzy/workspaceROS/MAP_BAG/wuhan/Transform_map_GNSS_3dbag_ls700b_1204_1/3dbag_ls700b_1204_1_M_map_GNSS.csv");

        //suber puber service
        reloca_service = nh.advertiseService("/localization/relocalization",&Localizer_global::relocalization_callback,this);
//        gnss_suber = nh.subscribe("/drive",2,&Localizer_global::location_retrans_callback,this);
        reloca_pc_puber = nh.advertise<sensor_msgs::PointCloud2>("reloca_pointcloud",2, true);
        //read transform
        prm_localization::CSV_reader csvReader(csv_path);
        Motion_mg = csvReader.getTransformMg();
        //map
        full_map.reset(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::io::loadPCDFile(global_pcd_path, *full_map);
        full_map->header.frame_id = map_tf;
        downSampler.setInputCloud(full_map);
        downSampler.setLeafSize(downsample_res,downsample_res,downsample_res);
        //boost::shared_ptr<pcl::VoxelGrid<pcl::PointXYZ>> voxelgrid(new pcl::VoxelGrid<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
        downSampler.filter(*filtered);
        full_map = filtered;
        kdtree.setInputCloud(full_map);
        //registrition ready
        pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt(
                new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
        ndt->setTransformationEpsilon(TransformationEpsilon);
        ndt->setResolution(ndt_resolution);
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
        registration = ndt;
        // fine alignment
        icp.setMaximumIterations(200);
        icp.setTransformationEpsilon(1e-5);
        icp.setRANSACOutlierRejectionThreshold( 0.04f);
        icp.setMaxCorrespondenceDistance( 100* 0.04f);
        map_pub = nh.advertise<sensor_msgs::PointCloud2>("map_cloud",1);
        //full_map->header.frame_id=map_tf;
        //map_pub_click = nh.advertiseService("/localiztion/map_pub_click",&Localizer_global::click_pub_map_callback,this);
        //odom_pub = nh.advertise<nav_msgs::Odometry>("trans_odom",3);


    }

private:


    bool relocalization_callback(prm_localization::localiRequest &req,prm_localization::localiResponse &res){
        prm_localization::DriveInfoConstPtr drive_msg =  ros::topic::waitForMessage<prm_localization::DriveInfo>("/drive");
        sensor_msgs::PointCloud2ConstPtr point_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/lslidar_point_cloud",ros::Duration(10));
        Vector4f gps_pos;
        if(drive_msg->gnss_flag!=1)
            ROS_WARN("Bad GPS_Singal, relocation maybe wrong!");
        gps_pos << drive_msg->gnss_x,drive_msg->gnss_y,0,1;
        Vector4f map_curr_pos =  Motion_mg * gps_pos;
        //trim localmap
        pcl::PointXYZ searchPoint(map_curr_pos(0),map_curr_pos(1),0);
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        pcl::PointCloud<pcl::PointXYZ>::Ptr trimmed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        if ( kdtree.radiusSearch (searchPoint, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ) {
//                NODELET_INFO("trimmed_cloud init points_num:%ld",trimmed_cloud->width);
//                NODELET_INFO("search points_num:%ld",pointIdxRadiusSearch.size());
            trimmed_cloud->points.reserve( size_t (full_map->width/3));
            for (int i : pointIdxRadiusSearch)
            {
                trimmed_cloud->points.push_back(full_map->points[i]);
            }
            trimmed_cloud->width=trimmed_cloud->points.size();
            trimmed_cloud->height = 1;
            cout<<"full_map.size()\t:"<<full_map->size()<<endl<<"trimmed_cloud->width:\t"<<trimmed_cloud->width<<endl;
        }
        //ready init transform
        int particle = 16;
        std::vector<Eigen::Matrix4f> init_motions;
        for (int i=0;i<particle;++i){
            Eigen::Matrix4f curr_m =Eigen::MatrixXf::Identity(4,4);
            curr_m.block(0,0,3,3)=euler2rot(0,0,i*2*M_PI/particle);
            curr_m.block(0,3,2,1)=map_curr_pos.block(0,0,2,1);
            init_motions.push_back(curr_m);
        }
        //regis
        std::vector<Eigen::Matrix4f> result_motions;
        VectorXd objfuns = VectorXd::Ones(particle); // mse store
        pcl::PointCloud<pcl::PointXYZ>::Ptr curr_cloud_wnan (new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr curr_cloud_dense (new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr curr_cloud (new pcl::PointCloud<pcl::PointXYZ>());

        pcl::fromROSMsg(*point_msg, *curr_cloud_wnan);
        //remove nan
        std::vector<int> indi_index;
        pcl::removeNaNFromPointCloud(*curr_cloud_wnan,*curr_cloud_dense,indi_index);
        downSampler.setInputCloud(curr_cloud_dense);
        downSampler.filter(*curr_cloud);

        registration->setInputTarget(trimmed_cloud);
        registration->setInputSource(curr_cloud);
        icp.setInputTarget(trimmed_cloud);
        icp.setInputSource(curr_cloud);

        {
            int i=0;
            for(auto & init_motion : init_motions){
                pcl::PointCloud<pcl::PointXYZ> result_cloud ;
                registration->align(result_cloud,init_motion);
                result_motions.push_back(registration->getFinalTransformation());
                objfuns(i) = registration->getFitnessScore();
                ++i;
            }

        }
        //store 3 potionel set
        VectorXd::Index  index;
        std::vector<int> interest_sets_index;
        int potioenl_choice =8;
        for (int j = 0; j < potioenl_choice; ++j) {
            objfuns.minCoeff(&index);
            objfuns(index)= INT8_MAX;
            interest_sets_index.push_back((int)index);
        }
        // refine 3 sets
        VectorXd refined_objfuns = VectorXd::Ones(potioenl_choice); // mse store
        std::vector<Eigen::Matrix4f> refined_movement;
        for (int k=0 ; k<interest_sets_index.size();++k) {
            pcl::PointCloud<pcl::PointXYZ> result_cloud ;
            icp.align(result_cloud,result_motions[interest_sets_index[k]]);
            refined_movement.emplace_back(icp.getFinalTransformation());
            refined_objfuns(k) = icp.getFitnessScore();
        }
        ROS_INFO("select motion with refined_objfuns: %f",refined_objfuns.minCoeff(&index)); // min objfun
        cout<<"refined_objfuns: "<<refined_objfuns<<endl;
//        result_motions[int(index)];
        //(optional) show regis pc
        pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud (new pcl::PointCloud<pcl::PointXYZ>()) ;
        pcl::transformPointCloud(*curr_cloud,*result_cloud,refined_movement[(int)index]);
        pcl_conversions::toPCL(point_msg->header,result_cloud->header);
        result_cloud->header.frame_id = map_tf;
        reloca_pc_puber.publish(result_cloud);
        map_pub.publish(full_map);
        ROS_INFO("cloud republished");

    }

//    void location_retrans_callback(const prm_localization::DriveInfoConstPtr& drive_msg){
////        if(drive_msg->gnss_flag)
//        Vector4f curr_pos;
//        curr_pos << drive_msg->gnss_x,drive_msg->gnss_y,0,1;
//        Vector4f map_curr_pos = Motion_mg * curr_pos;
//        nav_msgs::Odometry odom;
//        odom.header.frame_id=map_tf;
//        odom.pose.pose.position.x=map_curr_pos(0);
//        odom.pose.pose.position.y=map_curr_pos(1);
//        odom.pose.pose.position.z=0;
//        odom_pub.publish(odom);
//    }


private:
    //ros
    ros::NodeHandle nh;
    ros::NodeHandle p_nh;
    //suber and puber
//    ros::Publisher streeing_motion_puber;
    ros::Publisher reloca_pc_puber;
    ros::Publisher map_pub; // optional

    //service
    ros::ServiceServer reloca_service;

    //parameter
    float downsample_res;
    float TransformationEpsilon;
    float ndt_resolution;
    float search_radius;
    string map_tf;
    string lidar_tf;
    string base_tf;
    Matrix4f Motion_mg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr full_map;
    //utility
    pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration;
    pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
    pcl::VoxelGrid<pcl::PointXYZ> downSampler;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;



    //flag
};

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "relocalizaer");
    Localizer_global localizer_global;
    localizer_global.onInit();
    ros::spin();

    return 0;
}
/**init**/
//check transform
//map_tf = "map_f";
//full_map.reset(new pcl::PointCloud<pcl::PointXYZ>());
//pcl::io::loadPCDFile("/home/vickylzy/workspaceROS/MAP_BAG/wuhan/Transform_map_GNSS_3dbag_ls700b_1204_1/wuhan_ls_great.pcd", *full_map);
//float downsample_resolution=0.1;
//boost::shared_ptr<pcl::VoxelGrid<pcl::PointXYZ>> voxelgrid(new pcl::VoxelGrid<pcl::PointXYZ>());
//voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
//voxelgrid->setInputCloud(full_map);
//pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
//voxelgrid->filter(*filtered);
//full_map = filtered;
//map_pub = nh.advertise<sensor_msgs::PointCloud2>("map_cloud",3);
//full_map->header.frame_id=map_tf;
//map_pub_click = nh.advertiseService("/localiztion/map_pub_click",&Localizer_global::click_pub_map_callback,this);
//odom_pub = nh.advertise<nav_msgs::Odometry>("trans_odom",3);

/**map pub srv**/
//bool click_pub_map_callback(std_srvs::TriggerRequest & req, std_srvs::TriggerResponse &res){
//    map_pub.publish(full_map);
//    res.message="map_pubbed";
//    res.success=1;
//    return true;
//}

/**transform check param**/
//ros::ServiceServer map_pub_click;
//ros::Publisher map_pub;
//ros::Publisher odom_pub;
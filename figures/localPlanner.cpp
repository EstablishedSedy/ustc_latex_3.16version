#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

const double localMapSize_ = 10.0;
const double localMapResolution_ = 0.05;
const double localMapOriginX_ = -5.0;
const double localMapOriginY_ = -5.0;
const int localMapWidth_ = localMapSize_ / localMapResolution_;
const int MPNum_ = 729;
const int MPNumPerGroup_ = 81;
const int groupNum_ = 9;

// robot Ackerman param 
const double wheelBase = 0.67;
std::vector<double> alphas  = {-0.349066,-0.261799,-0.174533,-0.0872664,0,0.0872664,0.174533,0.261799,0.349066};

bool newTerrainCloudFlag_ = false;
//state 
double vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
double vehicleX = 0, vehicleY = 0, vehicleZ = 0;

//output 
double velocity_ = 0;
double alphas_ = 0;
double angularVelocity_ = 0;

// goal 
double curGoalX=0,curGoalY =0;
bool hasGoalFlag_ = false;
bool newGoalFlag_ = false;
bool nearGoalFlag_ = false;
std::vector<Eigen::Vector2d> goalVec;
pcl::VoxelGrid<pcl::PointXYZI>  terrainDwzFilter;

pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud;
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudCrop;
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz;
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloud;
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloudCrop;
pcl::PointCloud<pcl::PointXYZI>::Ptr freePathCloud;
pcl::PointCloud<pcl::PointXYZI>::Ptr robotContourCloud;

// pcl::PointCloud<pcl::PointXYZI>::Ptr paths[pathNum];
// pcl::PointCloud<pcl::PointXYZI>::Ptr freePaths;
nav_msgs::Path traj;
//for pic 
std::ofstream vOutFile,alphaOutFile;
double odomTime;
double lastOdomTime;
double lastVehicleX = 0, lastVehicleY = 0;

void init(){
    terrainCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    terrainCloudCrop.reset(new pcl::PointCloud<pcl::PointXYZI>());
    terrainCloudDwz.reset(new pcl::PointCloud<pcl::PointXYZI>());
    // freePaths.reset(new pcl::PointCloud<pcl::PointXYZI>());
    plannerCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    plannerCloudCrop.reset(new pcl::PointCloud<pcl::PointXYZI>());
    freePathCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    robotContourCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    
    terrainDwzFilter.setLeafSize(0.2, 0.2, 0.2);
}
void terrainCloudHandler(const sensor_msgs::PointCloud2ConstPtr& terrainCloudMsg){
    terrainCloud -> clear();
    pcl::fromROSMsg(*terrainCloudMsg, *terrainCloud);
    terrainCloudCrop->clear();
    for(int i = 0 ; i < terrainCloud->size() ; i++){
        pcl::PointXYZI point = terrainCloud->points[i];
        float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) + (point.y - vehicleY) * (point.y - vehicleY));
        if(dis > 5.0) continue;
        if(point.intensity > 0.15) {
            terrainCloudCrop->push_back(point);
        }
    }

    terrainCloudDwz->clear();
    terrainDwzFilter.setInputCloud(terrainCloudCrop);
    terrainDwzFilter.filter(*terrainCloudDwz);
    
    newTerrainCloudFlag_ = true;
}
void wayPointPathHandle(const nav_msgs::Path::ConstPtr& pathMsg){
    goalVec.clear();
    std::cout << "get waypoint path" <<std::endl;
    for(int i = 0 ; i < pathMsg->poses.size() ; i++){
        goalVec.emplace_back(pathMsg->poses[i].pose.position.x,pathMsg->poses[i].pose.position.y);
    }
    hasGoalFlag_ = true;
    newGoalFlag_ = true;
}
void goalHandler(const geometry_msgs::PointStamped::ConstPtr& goalMsg){
    goalVec.clear();

    goalVec.emplace_back(goalMsg->point.x,goalMsg->point.y);
    std::cout << "get new goal" <<std::endl;
    hasGoalFlag_ = true;
    newGoalFlag_ = true;
}
void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  //odomTime = odom->header.stamp.toSec();
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);
  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x ;
  vehicleY = odom->pose.pose.position.y ;
  vehicleZ = odom->pose.pose.position.z ;
  double dis = sqrt((vehicleX - lastVehicleX)*(vehicleX - lastVehicleX) + (vehicleY - lastVehicleY)*(vehicleY - lastVehicleY));
  if(dis > 0.05){
      
      odomTime = odom->header.stamp.toSec();
      double v = dis / (odomTime - lastOdomTime);
      // std::cout << dis <<'\t' << odomTime - lastOdomTime <<'\t' << v << std::endl;
      vOutFile << odomTime <<',' << v <<std::endl;
      geometry_msgs::PoseStamped pose;
      pose.pose = odom->pose.pose;
      traj.poses.push_back(pose);
      lastVehicleX = vehicleX;
      lastVehicleY = vehicleY;
      lastOdomTime = odomTime;
  } 

  if(newGoalFlag_){
      curGoalX = goalVec.front()[0];
      curGoalY = goalVec.front()[1];
      newGoalFlag_ = false;
      std::cout << "get true hasGoal" <<std::endl;
      hasGoalFlag_ = true;
  }
  if(hasGoalFlag_){
        double goalDis = sqrt((curGoalX - vehicleX)*(curGoalX - vehicleX) + (curGoalY - vehicleY) * (curGoalY - vehicleY));
        if(goalDis > 3.0){
            std::cout << "get true hasGoal" <<std::endl;
            hasGoalFlag_ = true;
            nearGoalFlag_ = false;
        }
        else if(goalDis > 1.0){
            if(goalVec.size() > 1){
                goalVec.erase(goalVec.begin());
                curGoalX = goalVec.front()[0];
                curGoalY = goalVec.front()[1];
            }
            else{
                nearGoalFlag_ = true;
            }
        }
        else {
            hasGoalFlag_ = false;
        }
    }
}


template <class PCLPointType>
void pubcloud(const ros::Publisher& cloud_publisher,const PCLPointType& cloud,std::string frame_id){
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*cloud,cloud_msg);
	cloud_msg.header.frame_id = frame_id;
	cloud_msg.header.stamp = ros::Time::now();
	cloud_publisher.publish(cloud_msg);
}

void collisionDetection(std::vector<int> obstacles,std::vector<bool> &MPFlag,std::vector<std::vector<int>> &voxel2MP){
    for(const auto&obe:obstacles){
        for(const auto&MP:voxel2MP[obe]){
            MPFlag[MP] = false;
        }
    }
 
}
void readRobotContour(std::vector<Eigen::Vector2d> &robotContour,std::string path){ 
    std::ifstream file;
    file.open(path.data()); 
    assert(file.is_open());

    robotContour.clear();
    std::string s,item;
    while(std::getline(file,s)){
        std::stringstream ss;
        Eigen::Vector2d point2d;
        ss.str(s);
        std::getline(ss,item,',');
        point2d[0] = std::stod(item);
        std::getline(ss,item,',');
        point2d[1] = std::stod(item);
        robotContour.push_back(point2d);
    }
    file.close();
}
void getRobotContourCloud(std::vector<Eigen::Vector2d> &robotContour,pcl::PointCloud<pcl::PointXYZI>::Ptr &robotContourCloud){
    robotContourCloud->clear();
    for(int i = 0 ; i < robotContour.size() ; i++){
        Eigen::Vector2d start = robotContour[i];
        Eigen::Vector2d end = robotContour[(i+1)%robotContour.size()];
        double length = (start - end).norm();
        Eigen::Vector2d dir = (end - start) / length;
        int step = length/0.01;
        for(int j = 0 ; j < step  ; j++){
            pcl::PointXYZI point;
            Eigen::Vector2d curPosition = start + dir * j * 0.01;
            point.x = curPosition[0];
            point.y = curPosition[1];
            point.z = 0;
            point.intensity = 0;
            robotContourCloud->push_back(point);
        }
    } 
}
void readMP(std::vector<std::vector<Eigen::Vector3d>> &MP,std::string path){
    
    assert(MP.size() == MPNum_);
    std::ifstream file; 
    file.open(path.data());
    assert(file.is_open());

    std::string s,item;
    while(std::getline(file,s)){
        std::stringstream ss;
        ss.str(s);
        int n;
        double x,y,theta;
        std::getline(ss,item,',');
        n = std::stoi(item);
        std::getline(ss,item,',');
        x = std::stod(item);
        std::getline(ss,item,',');
        y = std::stod(item);
        std::getline(ss,item,',');
        theta = std::stod(item);
        MP[n].push_back(Eigen::Vector3d(x,y,theta));
    }
    file.close();
}

void readVoxel2MP(std::vector<std::vector<int>> &voxel2MP,std::string path){
    std::ifstream file; 
    file.open(path.data());
    assert(file.is_open());
    assert(voxel2MP.size() == localMapWidth_*localMapWidth_);

    std::string s,item; 
    int count = 0;
    while(std::getline(file,s)){
        std::stringstream ss;
        ss.str(s);
        std::getline(ss,item,',');
        assert(std::stoi(item) == count);
        while(std::getline(ss,item,',')){
            voxel2MP[count].push_back(std::stoi(item));
        } 
        count++;
    }
    file.close();
}

int main(int argc, char** argv){
    ros::init(argc,argv,"localPlanner");
    ros::NodeHandle nh_private_("~");  
    init();

    std::vector<std::vector<Eigen::Vector3d>> MP(MPNum_);
    readMP(MP,"/home/robot410/Documents/0405_weichao1/path/MP.txt");
    std::vector<std::vector<int>> voxel2MP(localMapWidth_*localMapWidth_);
    readVoxel2MP(voxel2MP,"/home/robot410/Documents/0405_weichao1/path/voxel2MP.txt");
    std::vector<Eigen::Vector2d> robotContour;
    readRobotContour(robotContour,"/home/robot410/Documents/0405_weichao1/path/robotContour.txt");
    getRobotContourCloud(robotContour,robotContourCloud);

    vOutFile.open("/home/shuangyu/Downloads/MCMP2_terriain_ana_ori/v.txt");
    assert(vOutFile.is_open());
    alphaOutFile.open("/home/shuangyu/Downloads/MCMP2_terriain_ana_ori/al.txt");
    assert(alphaOutFile.is_open());


    ros::Subscriber subTerrainCloud = nh_private_.subscribe<sensor_msgs::PointCloud2> ("/terrain_map", 5, terrainCloudHandler);
    ros::Subscriber subOdometry = nh_private_.subscribe<nav_msgs::Odometry> ("/state_estimation", 5, odometryHandler);
    ros::Subscriber subGoal = nh_private_.subscribe<geometry_msgs::PointStamped>("/clicked_point",5,goalHandler);
    ros::Subscriber subPath = nh_private_.subscribe<nav_msgs::Path>("/wayPointPath",5,wayPointPathHandle);
    ros::Publisher  trajPub_ = nh_private_.advertise<nav_msgs::Path>("/traj",5);
    ros::Publisher  cloudPub_ = nh_private_.advertise<sensor_msgs::PointCloud2>("/testCloud",5);
    ros::Publisher  freePathPub_ = nh_private_.advertise<sensor_msgs::PointCloud2>("/freePath",5);
    ros::Publisher  contourCloudPub_ = nh_private_.advertise<sensor_msgs::PointCloud2>("/robotContour",5);
    ros::Publisher  cmdPub_ =   nh_private_.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 5);
    ros::Rate rate(100);



    while(ros::ok()){
        ros::spinOnce();

        if(newTerrainCloudFlag_ ){
            newTerrainCloudFlag_ = false;
            *plannerCloud = *terrainCloudDwz;

            plannerCloudCrop->clear();
            for(int i = 0 ; i < plannerCloud->size() ; i++){
                float pointX1 = plannerCloud->points[i].x - vehicleX;
                float pointY1 = plannerCloud->points[i].y - vehicleY;
                float pointZ1 = plannerCloud->points[i].z - vehicleZ;

                pcl::PointXYZI point;
                point.x = pointX1 * cos(vehicleYaw) + pointY1 * sin(vehicleYaw);
                point.y = -pointX1 * sin(vehicleYaw) + pointY1 * cos(vehicleYaw);
                point.z = pointZ1;
                point.intensity = plannerCloud->points[i].intensity;
                if(sqrt(point.x * point.x + point.y * point.y) > 0.5)
                    plannerCloudCrop -> push_back(point);
            }
            std::vector<int> voxels(localMapWidth_ * localMapWidth_,0);
            std::vector<int> obstacles;
            std::vector<bool> MPFlags(MPNum_,true);
            for(const auto&point:*plannerCloudCrop){
                int indx = (point.x - localMapOriginX_) / localMapResolution_;
                int indy = (point.y - localMapOriginY_) / localMapResolution_;
                int ind = indx + indy * localMapWidth_;
                voxels[ind] ++ ;
            }
            for(int ind = 0 ; ind < localMapWidth_ * localMapWidth_ ; ind++){
                if(voxels[ind] > 1) obstacles.push_back(ind);
            }
            collisionDetection(obstacles,MPFlags,voxel2MP);
            pubcloud(cloudPub_,plannerCloudCrop,"base_link");
 
            int selectGroupID = -1;
            if(hasGoalFlag_)
            {
                double relativeGoalX = ((curGoalX - vehicleX) * cos(vehicleYaw) + (curGoalY - vehicleY) * sin(vehicleYaw));
                double relativeGoalY = (-(curGoalX - vehicleX) * sin(vehicleYaw) + (curGoalY - vehicleY) * cos(vehicleYaw));
                std::cout   << "relativeGoal::" << relativeGoalX <<";;" << relativeGoalY <<std::endl;
                if(nearGoalFlag_){
                    int indx = (relativeGoalX - localMapOriginX_) / localMapResolution_;
                    int indy = (relativeGoalY - localMapOriginY_) / localMapResolution_;
                    int goalInd = indx + indy * localMapWidth_;
                    std::vector<int> MPIDs =  voxel2MP[goalInd];
                    if(MPIDs.size() > 0 )  selectGroupID = MPIDs[MPIDs.size()/2]/MPNumPerGroup_;
                }
                else {
                    double goalDir = atan2(relativeGoalY, relativeGoalX) * 180 / M_PI;
                    std::vector<double> groupScore(groupNum_,0);
                    for(int i = 0 ; i < MPNum_ ; i++){
                        if(!MPFlags[i]) continue;
                        double MPDir = atan2(MP[i].back()[1],MP[i].back()[0]) * 180 / M_PI;
                        double angDiff = std::fabs(goalDir - MPDir);
                        double score = (1 - sqrt(sqrt(0.005*angDiff)));
                        groupScore[i/MPNumPerGroup_] += score;
                    }
                    
                    double selectGroupScore = 0;
                    for(int i = 0 ; i < groupNum_ ;i++){
                        if(selectGroupScore < groupScore[i]){
                            selectGroupID = i;
                            selectGroupScore = groupScore[i];
                        }
                    }
                }
            }
            if(selectGroupID != -1 ){
                alphas_ = alphas[selectGroupID];
                velocity_ = 1.0;
                angularVelocity_ = tan(alphas_)*velocity_/wheelBase;
            }
            else {
                 std::cout << "no goal or can get goal" <<std::endl;
                alphas_ = 0;
                velocity_ = 0;
                angularVelocity_ = 0;
            }
 
            freePathCloud->clear();
            for(int i = 0 ; i < MPNum_; i++){
                if(MPFlags[i]){
                    for(const auto&p:MP[i]) {
                        pcl::PointXYZI point;
                        point.x = p[0];
                        point.y = p[1];
                        point.z = 0;
                        if(selectGroupID == i/MPNumPerGroup_) point.intensity = 100;
                        else  point.intensity = 0 ;
                        freePathCloud->push_back(point);
                    }
                }
            }

            pubcloud(freePathPub_,freePathCloud,"base_link");

            pubcloud(contourCloudPub_,robotContourCloud,"base_link");
            traj.header.stamp = ros::Time::now();
            traj.header.frame_id = "map";
            trajPub_.publish(traj);
            alphaOutFile<<ros::Time::now() <<',' << alphas_ <<std::endl;

            std::cout << "out::" <<alphas_/M_PI * 180 <<";;" <<  velocity_ << std::endl;
        }
        
        geometry_msgs::TwistStamped cmd_vel;
        cmd_vel.twist.angular.z = alphas_;
        cmd_vel.twist.linear.x  = velocity_;
        cmd_vel.header.frame_id = "/vehicle";
        cmd_vel.header.stamp = ros::Time::now();
        cmdPub_.publish(cmd_vel);

        rate.sleep();
    }
}

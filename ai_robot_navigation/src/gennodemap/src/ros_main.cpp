#include "keyframehandle.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
// #include "ai_robot_interfaces/MpTraj.h"
// #include "ai_robot_interfaces/TopoMetric.h"

using namespace std;

class gentopo
{
public:
    gentopo(ros::NodeHandle &_n, const string &paramfile):
        nh(_n), kfh(paramfile)
    {
//        topo_pub = nh.advertise<ai_robot_interfaces::TopoMetric>("/topomap", 1);
//        mptraj_sub = nh.subscribe("/mptraj", 1, &gentopo::mptrajCB, this);
        savedone_sub = nh.subscribe("/slam/savedone", 1, &gentopo::savedoneCB, this);
        gentopodone_pub = nh.advertise<std_msgs::String>("/topo/savedone", 1);
        ROS_INFO("[gentopo]init done.");
    }

    void spin(){ros::spin();}
private:
    ros::NodeHandle nh;
    ros::Publisher topo_pub, gentopodone_pub;
    ros::Subscriber savedone_sub;//mptraj_sub, 

    KeyFrameHandler kfh;

    void savedoneCB(const std_msgs::StringConstPtr& msg){
        string savepath = msg->data;
        string command = "mkdir -p " + savepath + "Topomap/";
        system(command.c_str());
        //gentopo
        kfh.reset(savepath);
        ROS_INFO("[gentopo]start generating topo...");
        kfh.dealKeyFrame();
        kfh.saveResult(savepath+"Topomap/");
        kfh.removeRedundantNode();
        ROS_INFO("[gentopo]generate done. start saving topo...");
        kfh.saveResultOnlyFinal(savepath+"Topomap/");
        // kfh.saveResultPN(savepath+"Topomap/");
        std_msgs::String outmsg;
        outmsg.data = savepath+"Topomap/";
        gentopodone_pub.publish(outmsg);
        ROS_INFO("[gentopo]generate and save topo done.");
    }
/*
    void mptrajCB(const ai_robot_interfaces::MpTrajConstPtr& msg)
    {
        //turn msg to pcl pointcloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr _mapPC(new pcl::PointCloud<pcl::PointXYZRGB>)
                , _keyfPC(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointXYZRGB pt;
        for(int i=0; i<msg->trajectory.poses.size(); ++i){
            pt.x = msg->trajectory.poses[i].pose.position.x;
            pt.y = msg->trajectory.poses[i].pose.position.y;
            pt.z = msg->trajectory.poses[i].pose.position.z;
            _keyfPC->push_back(pt);
        }
        for(int i=0; i<msg->map_point.size(); ++i){
            pt.x = msg->map_point[i].point.x;
            pt.y = msg->map_point[i].point.y;
            pt.z = msg->map_point[i].point.z;
            _mapPC->push_back(pt);
        }
        //get topomap
        kfh.reset(_mapPC, _keyfPC);
        kfh.dealKeyFrame();
        //publish topomap
        ai_robot_interfaces::TopoMetric tmmsg;
        geometry_msgs::PointStamped ptst;
        for(int i=0; i<kfh.denkeyfPC->size(); ++i){
            ptst.point.x = kfh.denkeyfPC->points[i].x;
            ptst.point.y = kfh.denkeyfPC->points[i].y;
            ptst.point.z = kfh.denkeyfPC->points[i].z;
            tmmsg.vertices.push_back(ptst);
        }
        for(int i=0; i<kfh.denkeyfLine.size(); ++i){
            tmmsg.edges.push_back(kfh.denkeyfLine[i][0]);
            tmmsg.edges.push_back(kfh.denkeyfLine[i][1]);
        }
        topo_pub.publish(tmmsg);
    }*/
};


int main(int argc, char** argv)
{
    ros::init(argc, argv,"obsavoid");
    ros::NodeHandle nh;
    if (argc < 2)
    {
        ROS_INFO("[gentopo]filename of param missing.");
        return -1;
    }

    string paramfile = argv[1];
    gentopo gtp(nh, paramfile);
    gtp.spin();
    return 0;
}

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "string"
#include "array"
#include "std_msgs/UInt16.h"
#include "node_director/SpawnNode.h"
#include "node_director/ListNodes.h"
#include "node_director/StopNode.h"
#include "node_director/RestartNode.h"
#include "cob_perception_msgs/DetectionArray.h"

int clountangle = 0, lastAngle = -10, idCamera = 0, idDtector = 0, counTryDetect=0;
bool notDtectPerson = true;
ros::Publisher pub;

void callbackTimer1 (const ros::TimerEvent&)
{
    ROS_INFO("Srabotal timer !!!!");
    //perezapustit node_director !!
}

void Callback_speech(const std_msgs::String command)
{
    ROS_INFO("Cpabotal Callback y Speccha");
    ros::NodeHandle n;
    ros::Timer timer1 =  n.createTimer(ros::Duration(10.0), callbackTimer1);
}

void Callback_laser(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    //ROS_INFO("Center");
    //system("clear");
    int detectObjectAngle = 0, detectObjectAngleCount = 0, detectObjectAngleLast=0, detectObjectAngleFirst =0, lostObjectAngleCount=0;
    bool detectObject = false,detectObjectCut = false;
    for (int i=0; i<scan->ranges.size();i++)
    {
        float range = scan->ranges[i];
        if (range < 0.5f & range > 0.1f)
        {
            if (detectObjectAngleCount == 0 )             
                {
                    if (i == 0) {detectObjectCut = true;}
                    detectObjectAngleFirst = i; 
                    detectObject = true;
                }
            if (detectObject == true) {
                detectObjectAngleLast = i;
                detectObjectAngleCount +=1;
            }
            if (detectObjectCut ==true && i==359)
            {
                lostObjectAngleCount = 0;
                for (int j = 359; j>0; j--)
                {
                    range = scan->ranges[j];
                    if (range < 0.5f & range > 0.1f && detectObjectCut == true)
                    {
                        detectObjectAngleFirst = j;
                    }
                    else
                    {
                        lostObjectAngleCount += 1;
                        if ( lostObjectAngleCount > 5 ) { detectObjectCut=false; lostObjectAngleCount=0; }
                    }  
                }
            }
        }
        else
        {
            lostObjectAngleCount += 1;
            if ( lostObjectAngleCount > 5 ) 
            { 
                detectObject=false; lostObjectAngleCount=0;
                 if ( detectObjectAngleCount < 2) {
                     detectObjectAngleCount = 0;
                      detectObjectAngleLast=0;
                       detectObjectAngleFirst =0; 
                 }
            }
        }
    }
    //ROS_INFO("Range: [%d] ", detectObjectAngleCount);

    if (detectObjectAngleCount !=0)
    {
        if ( detectObjectAngleFirst - detectObjectAngleLast > 180)
        {
            detectObjectAngle = ((detectObjectAngleLast + 360)+detectObjectAngleFirst)/2;
            if (detectObjectAngle > 360) {detectObjectAngle -= 360;}
        } else
        {
            detectObjectAngle = (detectObjectAngleFirst + detectObjectAngleLast) / 2;
        }
        //ROS_INFO("F - [ %d ]  L - [%d]", detectObjectAngleFirst,detectObjectAngleLast);
        ROS_INFO("Center angle-> [ %d ]", detectObjectAngle);

        if (lastAngle < -5)   //попытка зафиксировать объект
        {
            lastAngle = detectObjectAngle;
            clountangle += 1;
        } 
        else
        {
            if ((detectObjectAngle-lastAngle > 350 || lastAngle - detectObjectAngle > 350))
            {
                lastAngle = detectObjectAngle;
                clountangle += 1;   
            }else{
            if ((detectObjectAngle>lastAngle-10)&&(detectObjectAngle<lastAngle+10)) 
            {
                lastAngle = detectObjectAngle;
                clountangle += 1;
            }
            else
            {
                lastAngle = -10;
                clountangle = 0;
            }
            }
        }
        if (clountangle > 10)// время чем больше тем дольше задержка перед реакцией на объект
        {
            ROS_INFO("tyt kto-to est'!!! Seya4as posmotrim !!!");
            ROS_INFO("Ostanavlivau laser i Camera ON");
            ros::NodeHandle n;
            std_msgs::UInt16 uintangle;
            uintangle.data = detectObjectAngle;
            pub.publish(uintangle);
            
            ros::ServiceClient clearClient = n.serviceClient<node_director::ListNodes>("node_director/list_nodes");
            node_director::ListNodes srv;
            clearClient.call(srv);
            
            bool rospackruning = false;
            for(node_director::SpawnedNodeInfo i : srv.response.nodes)
            {
            //ROS_INFO("=  %d", i.id);
                if (i.ros_package == "usb_cam")
                {
                    rospackruning =true;
                }
                if (i.ros_package == "xv_11_laser_driver")
                {
                    ros::ServiceClient clearClient = n.serviceClient<node_director::StopNode>("node_director/stop_node");
                    node_director::StopNode srv;
                    srv.request.id = i.id;
                    clearClient.call(srv);
                    ROS_INFO("XV_11 stopeeeeddd !!!!");

                }
            }
            if (rospackruning == false)
            {
                ros::ServiceClient clearClient = n.serviceClient<node_director::SpawnNode>("node_director/spawn_node");
                node_director::SpawnNode srv;
                srv.request.ros_package = "usb_cam";
                srv.request.executable_name = "usb_cam-test.launch";
                //srv.request.args = {""};
                clearClient.call(srv);
                ROS_INFO("%d",srv.response.id); 
                idCamera =srv.response.id;    

                srv.request.ros_package = "cob_people_object_detection_tensorflow";
                srv.request.executable_name = "cob_people_object_detection_tensorflow.launch";
                //srv.request.args = {""};
                clearClient.call(srv);
                ROS_INFO("%d",srv.response.id);  
                idDtector = srv.response.id;    

                rospackruning = true;         /**/
                ROS_INFO("Zapuskau kamery i raspoznovanie !!!!");
                clountangle = 0;
            }
        }
    }
}


void Callback_detections(const cob_perception_msgs::DetectionArray recive_detections)
{
  cob_perception_msgs::DetectionArray temp_detections = recive_detections;

  if (!temp_detections.detections.empty())
  {
    for (unsigned int it = 0; it < temp_detections.detections.size(); it++)
    {
        //f22276567
        //ROS_INFO("Our new detection %i has the following lable: %s",it+1,temp_detections.detections[it].label.c_str());
        if ((temp_detections.detections[it].label == "person") && notDtectPerson)
        {
            ros::NodeHandle n;
            ROS_INFO("Start Dialog nashel Pesony!!!");
            notDtectPerson = false;   
            //ZAPUSK DIALOGFLOW AND YANDEXSPECHHH !!!!!!!!!!! CODE HERE <----
            
            ros::ServiceClient clearClient = n.serviceClient<node_director::SpawnNode>("node_director/spawn_node");
            node_director::SpawnNode srv1;
            srv1.request.ros_package = "dialogflow_ros";
            srv1.request.executable_name = "dialogflow_ros.launch";
            clearClient.call(srv1);

            srv1.request.ros_package = "yaspeechkit";
            srv1.request.executable_name = "yaspeech.launch";
            clearClient.call(srv1);
        }
        counTryDetect = 0;
    }
  }
  if (counTryDetect>5)
        {
            ROS_INFO("PESRON PROPAL IZ VIDY");
            ros::NodeHandle n;
            counTryDetect = 0;
            ros::ServiceClient clearClient = n.serviceClient<node_director::StopNode>("node_director/stop_node");
            node_director::StopNode srv;
            srv.request.id = idDtector;
            clearClient.call(srv);
            srv.request.id = idCamera;
            clearClient.call(srv);

            clearClient = n.serviceClient<node_director::SpawnNode>("node_director/spawn_node");
            node_director::SpawnNode srv1;
            srv1.request.ros_package = "xv_11_laser_driver";
            srv1.request.executable_name = "neato_laser_publisher.launch";
            clearClient.call(srv1);
        }
counTryDetect += 1;
ROS_INFO("Try +1 ");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lasser_check");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("scan", 10, Callback_laser);
    ros::Subscriber subDetect = n.subscribe("/object_detection/detections", 10, Callback_detections);
    ros::Subscriber speechDtect = n.subscribe("/command", 10, Callback_speech);
    pub = n.advertise<std_msgs::UInt16>("angle", 100);
    ros::ServiceClient clearClient = n.serviceClient<node_director::ListNodes>("node_director/list_nodes");
    node_director::ListNodes srv;
    clearClient.call(srv);
    
    ros::Rate loop_rate(10);
    //while (ros::ok())
    //{
    
    //std::vector<node_director::SpawnedNodeInfo> nodes;
    //nodes = srv.response.nodes;
    for(node_director::SpawnedNodeInfo i : srv.response.nodes)
    {
        //ROS_INFO("=  %d", i.id);
        if (i.ros_package == "xv_11_laser_driver")
        {
            ROS_INFO("xv_11 run");
        }
        else
        {
            ROS_INFO("xv_11 Off");
            /**/
        }
    }
    if (srv.response.nodes.empty())
    {
        ROS_INFO("null");
        ros::ServiceClient clearClient = n.serviceClient<node_director::SpawnNode>("node_director/spawn_node");
        node_director::SpawnNode srv;
        srv.request.ros_package = "xv_11_laser_driver";
        srv.request.executable_name = "neato_laser_publisher.launch";
        //srv.request.args = {""};
        clearClient.call(srv);
        ROS_INFO("%d",srv.response.id);
    }
    loop_rate.sleep();
    //}
    ros::spin();
    return 0;
}
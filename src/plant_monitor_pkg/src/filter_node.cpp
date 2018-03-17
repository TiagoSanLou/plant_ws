#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "plant_monitor_pkg/sensors.h"
#include <time.h>
#include <iostream>

float ambTemp;
float soilTemp; 
int soilMoist; 
int ambLight; 
float ambHumidity;


float prevAmbTemp;
float prevSoilTemp; 
float prevSoilMoist; 
float prevAmbLight; 
float prevAmbHumidity; 

int isFirstRun = 1;
float alpha = 0.1;
float beta = 0.05;

plant_monitor_pkg::sensors filtered_msg;

void ambTempCB(const std_msgs::Float32::ConstPtr& msg)
{   
    ambTemp = msg->data;
    if (isFirstRun == 1){prevAmbTemp = ambTemp;}
    filtered_msg.ambtemp = beta*ambTemp + (1.0-beta)*prevAmbTemp;
    
    //std::cout << "ambTemp = " << ambTemp << std::endl;
    //std::cout << "beta*ambTemp = " << beta*ambTemp << std::endl;
    //std::cout << "1 - beta = " << 1 - beta  << std::endl;
    //std::cout << "(1.0-beta)*prevAmbTemp = " << (1.0-beta)*prevAmbTemp << std::endl;
    
    prevAmbTemp = filtered_msg.ambtemp;

}

void soilTempCB(const std_msgs::Float32::ConstPtr& msg)
{
	soilTemp = msg->data;
    if (isFirstRun == 1){prevSoilTemp = soilTemp;}
    filtered_msg.soiltemp = alpha*soilTemp  + (1.0-alpha)*prevSoilTemp;
    prevSoilTemp = filtered_msg.soiltemp;

}

void soilMoistCB(const std_msgs::Int32::ConstPtr& msg)
{
	soilMoist = msg->data;
    if (isFirstRun == 1){prevSoilMoist = soilMoist;}
    float filteredSoilmoist = round(alpha*soilMoist + (1.0-alpha)*prevSoilMoist);
    prevSoilMoist = filteredSoilmoist;
    filtered_msg.soilmoist = filteredSoilmoist;

}

void ambLightCB(const std_msgs::Int32::ConstPtr& msg)
{
	ambLight = msg->data;
    if (isFirstRun == 1){
        prevAmbLight = ambLight;
    }
    float filteredLight = round(alpha*ambLight  + (1.0-alpha)*prevAmbLight);
    prevAmbLight = filteredLight;
    filtered_msg.light = filteredLight;

}

void ambHumidityCB(const std_msgs::Float32::ConstPtr& msg)
{   
    ambHumidity = msg->data;
    if (isFirstRun == 1){prevAmbHumidity = ambHumidity;}
    filtered_msg.ambHumidity = beta*ambHumidity + (1.0-beta)*prevAmbHumidity;
    
    std::cout << "ambHumidity = " << ambTemp << std::endl;
    std::cout << "beta*ambHumidity = " << beta*ambTemp << std::endl;
    std::cout << "1 - beta = " << 1 - beta  << std::endl;
    std::cout << "(1.0-beta)*prevAmbHumidity = " << (1.0-beta)*prevAmbHumidity << std::endl;
    
    prevAmbHumidity = filtered_msg.ambHumidity;

}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "filter_node");
	ros::NodeHandle n;

	ros::Subscriber ambTempSub = n.subscribe("ambTemperature",1, ambTempCB);
	ros::Subscriber soilTempSub = n.subscribe("soilTemperature",1, soilTempCB);
	ros::Subscriber soilMoistSub = n.subscribe("soilMoisture",1,soilMoistCB);
	ros::Subscriber ambLightSub = n.subscribe("ambLight",1,ambLightCB);
    ros::Subscriber ambHumiditySub = n.subscribe("ambHumidity",1,ambHumidityCB);


    ros::Publisher filter_pub = n.advertise<plant_monitor_pkg::sensors>("filtered_signals",5);

	ros::Rate rate(0.5);
	
	sleep(10); //segundos

	ROS_INFO("Iniciou filter_node...");	
    

	while(ros::ok()){

        if (isFirstRun == 1 ){
		    ros::spinOnce();
	        ROS_INFO("FirstRun do Filtro...");            
            isFirstRun = 0;
        }

        filter_pub.publish(filtered_msg); 

		ros::spinOnce();
		rate.sleep();	
	}
	return 0;	
}

//============================================================================
// Name        : eposCommandSubscriber.cpp
// Author      : Çağatay YÜRÜK
// Version     : 0.02
// Description : Ros Node for sending commands to Epos Motor Drivers in Agv's
//============================================================================

/***********************
 * epos classı tanımla -- class a gerek yok
 * 2 tane ayrı int bu işi çözer
 * class içine open device, close device, set velocity ve halt velocity fonksiyonlarını tanımla
 * twist mesajına subscriber ol
 * teker hızlarını anlık al ve publish et
 * 
 */
#include <iostream>
#include "Definitions.h"
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <math.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/times.h>
#include <sys/time.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

typedef void* HANDLE;
typedef int BOOL;

using namespace std;

void* g_pKeyHandle = 0; //key handle a bir pointer tutucu atıyor
void* g_pKeyHandle_sub = 0;
unsigned short g_usNodeId = 2; //epos node on the right side of agv
unsigned short g_usNodeId1 = 3; //epos node on the left side of agv
string g_deviceName;
string g_protocolStackName;
string g_interfaceName;
string g_portName;
int g_baudrate = 0;

double b = 0.50; //length between two wheels 30cm
int *g_pRPMLeftWheel=new int; //values gathered from encoders
int *g_pRPMRightWheel=new int;
int foo;

long g_lCmdRPMLeftWheel=0;
long g_lCmdRPMRightWheel=0;

double g_dCmdRadius = 0.0;

const string g_programName = "Epos Command";

#ifndef MMC_SUCCESS
	#define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
	#define MMC_FAILED 1
#endif

#ifndef MMC_MAX_LOG_MSG_SIZE
	#define MMC_MAX_LOG_MSG_SIZE 512
#endif

void  LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode);
void  LogInfo(string message);
void  PrintHeader();
void  PrintSettings();
int   OpenDevice(unsigned int* p_pErrorCode);
int   CloseDevice(unsigned int* p_pErrorCode);
void  SetDefaultParameters();

void LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode)
{
	cerr << g_programName << ": " << functionName << " failed (result=" << p_lResult << ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< endl;
}

void LogInfo(string message)
{
	cout << message << endl;
}

//
void SeparatorLine()
{
	const int lineLength = 60;
	for(int i=0; i<lineLength; i++)
	{
		cout << "-";
	}
	cout << endl;
}

void PrintSettings()
{
	stringstream msg;

	msg << "default settings:" << endl;
	msg << "node id             = " << g_usNodeId << endl;
	msg << "device name         = '" << g_deviceName << "'" << endl;
	msg << "protocal stack name = '" << g_protocolStackName << "'" << endl;
	msg << "interface name      = '" << g_interfaceName << "'" << endl;
	msg << "port name           = '" << g_portName << "'"<< endl;
	msg << "baudrate            = " << g_baudrate;

	LogInfo(msg.str());

	SeparatorLine();
}

// Haberleşme Parametrelerini alacak
void SetDefaultParameters()
{
	g_deviceName = "EPOS"; //EPOS version
	g_protocolStackName = "MAXON_RS232"; //MAXON_RS232
	g_interfaceName = "RS232"; //RS232
	g_portName = "/dev/ttyS2"; // /dev/ttyS1
	g_baudrate = 115200; //38400 for agv 1 115200 for agv2
}

//Open device is opens to port for send and receive commands
//
int OpenDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_FAILED;

	char* pDeviceName = new char[255];
	char* pProtocolStackName = new char[255];
	char* pInterfaceName = new char[255];
	char* pPortName = new char[255];

	strcpy(pDeviceName, g_deviceName.c_str());
	strcpy(pProtocolStackName, g_protocolStackName.c_str());
	strcpy(pInterfaceName, g_interfaceName.c_str());
	strcpy(pPortName, g_portName.c_str());

	LogInfo("Open device...");

	g_pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);

	//işlem başarılı ise
	if(g_pKeyHandle!=0 && *p_pErrorCode == 0)
	{
		unsigned int lBaudrate = 0;
		unsigned int lTimeout = 0;

		if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
		{
			if(VCS_SetProtocolStackSettings(g_pKeyHandle, g_baudrate, lTimeout, p_pErrorCode)!=0)
			{
				if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
				{
					if(g_baudrate==(int)lBaudrate)
					{
						lResult = MMC_SUCCESS;
					}
				}
			}
		}
	}
	else
	{
		g_pKeyHandle = 0;
	}
	// 2. epos u açma
	g_pKeyHandle_sub = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);

	if(g_pKeyHandle_sub!=0 && *p_pErrorCode == 0)
	{
		unsigned int lBaudrate = 0;
		unsigned int lTimeout = 0;

		if(VCS_GetProtocolStackSettings(g_pKeyHandle_sub, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
		{
			if(VCS_SetProtocolStackSettings(g_pKeyHandle_sub, g_baudrate, lTimeout, p_pErrorCode)!=0)
			{
				if(VCS_GetProtocolStackSettings(g_pKeyHandle_sub, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
				{
					if(g_baudrate==(int)lBaudrate)
					{
						lResult = MMC_SUCCESS;
					}
				}
			}
		}
	}
	else
	{
		g_pKeyHandle_sub = 0;
	}


	delete []pDeviceName;
	delete []pProtocolStackName;
	delete []pInterfaceName;
	delete []pPortName;

	return lResult;
}

int CloseDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_FAILED;

	*p_pErrorCode = 0;

	LogInfo("Close device");

	if(VCS_CloseDevice(g_pKeyHandle, p_pErrorCode)!=0 && *p_pErrorCode == 0)
	{
		lResult = MMC_SUCCESS;
	}

	return lResult;
}


// bu fonksiyonu ikiye bölüp profile modu aktif edicez diğerinde move with velcot komutunu basıcaz
//target velociy i dışardan almayı ayarla
bool MoveWithVelocity(HANDLE p_DeviceHandle, unsigned short p_usNodeId, long targetVelocity, unsigned int & p_rlErrorCode)
{
	int lResult = MMC_SUCCESS;
	if(VCS_MoveWithVelocity(p_DeviceHandle, p_usNodeId, targetVelocity, &p_rlErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, p_rlErrorCode);
		}
	//ROS_INFO("I write velocity to Node: [%d]", p_usNodeId);
	
	return lResult;
}

bool GetVelocityIs(unsigned int &p_rlErrorCode)
{
	int lResult = MMC_SUCCESS;

	if(VCS_GetVelocityIs(g_pKeyHandle, g_usNodeId, g_pRPMRightWheel, &p_rlErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_GetVelocityIs", lResult, p_rlErrorCode);
	}

	if(VCS_GetVelocityIs(g_pKeyHandle_sub, g_usNodeId1, g_pRPMLeftWheel, &p_rlErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_GetVelocityIs", lResult, p_rlErrorCode);
	}

	return lResult;
}

//Cihazı Profile velocity moduna alma
bool ActivateProfileVelocityMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
{
	int lResult = MMC_SUCCESS;

	if(VCS_ActivateProfileVelocityMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
	{
		LogError("VCS_ActivateProfileVelocityMode", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	ROS_INFO("Device Velocity Mode Activated ");

	return lResult;
}

int PrepareDevices(unsigned int* p_pErrorCode)//error code pointer *error code değer
{
	int lResult = MMC_SUCCESS; //0
	BOOL oIsFault = 0;

	if(VCS_GetFaultState(g_pKeyHandle, g_usNodeId, &oIsFault, p_pErrorCode ) == 0)
	{
		LogError("VCS_GetFaultState", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	if(lResult==0)
	{
		if(oIsFault)
		{
			stringstream msg;
			msg << "clear fault, node = '" << g_usNodeId << "'";
			LogInfo(msg.str());

			if(VCS_ClearFault(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0)
			{
				LogError("VCS_ClearFault", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}
		}

		if(lResult==0)
		{
			BOOL oIsEnabled = 0;

			if(VCS_GetEnableState(g_pKeyHandle, g_usNodeId, &oIsEnabled, p_pErrorCode) == 0)
			{
				LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}

			if(lResult==0)
			{
				if(!oIsEnabled)
				{
					if(VCS_SetEnableState(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0)
					{
						LogError("VCS_SetEnableState", lResult, *p_pErrorCode);
						lResult = MMC_FAILED;
					}
				}
			}
		}
	}

	// node 3 clear fault
	if(VCS_GetFaultState(g_pKeyHandle_sub, g_usNodeId1, &oIsFault, p_pErrorCode ) == 0)
	{
		LogError("VCS_GetFaultState", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	if(lResult==0)
	{
		if(oIsFault)
		{
			stringstream msg;
			msg << "clear fault, node = '" << g_usNodeId << "'";
			LogInfo(msg.str());

			if(VCS_ClearFault(g_pKeyHandle_sub, g_usNodeId1, p_pErrorCode) == 0)
			{
				LogError("VCS_ClearFault", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}
		}

		if(lResult==0)
		{
			BOOL oIsEnabled = 0;

			if(VCS_GetEnableState(g_pKeyHandle_sub, g_usNodeId1, &oIsEnabled, p_pErrorCode) == 0)
			{
				LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}

			if(lResult==0)
			{
				if(!oIsEnabled)
				{
					if(VCS_SetEnableState(g_pKeyHandle_sub, g_usNodeId1, p_pErrorCode) == 0)
					{
						LogError("VCS_SetEnableState", lResult, *p_pErrorCode);
						lResult = MMC_FAILED;
					}
				}
			}
		}
	}

	return lResult;
}


void PrintHeader()
{
	SeparatorLine();

	LogInfo("ITU Robotic Lab Epos Command Program, Çağatay YÜRÜK 2017-2018");

	SeparatorLine();
}

void VelocityCommandCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
  //ROS_INFO("I heard z: [%f]", twist->angular.z);
  //ROS_INFO("I heard x: [%f]", twist->linear.x);
  
  unsigned int ulErrorCode = 0;
  int lResult = MMC_FAILED;

  double x = twist->linear.x;
  double z = twist->angular.z;

  if (z != 0)
  {
	  g_dCmdRadius = x/z;
	  g_lCmdRPMLeftWheel  = -(z * (g_dCmdRadius - b/2))*9523;
	  g_lCmdRPMRightWheel =  (z * (g_dCmdRadius + b/2))*9523;
  }
  else
  {
	  g_lCmdRPMLeftWheel = -(x)*9523;
	  g_lCmdRPMRightWheel = (x)*9523;
  }
  
  ROS_INFO("now im gonna write to  right : [%ld]", g_lCmdRPMRightWheel);
  ROS_INFO("now im gonna write to  left : [%ld]", g_lCmdRPMLeftWheel);

  if((lResult = MoveWithVelocity(g_pKeyHandle,g_usNodeId,g_lCmdRPMRightWheel, ulErrorCode))!=MMC_SUCCESS)
  {
	LogError("MoveWithVelocity Node 2", lResult, ulErrorCode);
  }

  if((lResult = MoveWithVelocity(g_pKeyHandle_sub,g_usNodeId1,g_lCmdRPMLeftWheel, ulErrorCode))!=MMC_SUCCESS)
  {
	LogError("MoveWithVelocity Node 3", lResult, ulErrorCode);
  }
  
}


int main(int argc, char** argv)
{
	int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;

	ros::init(argc, argv, "eposCmd");
	ros::NodeHandle n;

	PrintHeader();

	SetDefaultParameters();
	//TODO: bu parametreleri Ros param serverinden alacak şekilde ayarla

	PrintSettings();

	if((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("OpenDevice", lResult, ulErrorCode);
		return lResult;
	}

	if((lResult = PrepareDevices(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("PrepareDemo", lResult, ulErrorCode);
		return lResult;
	}

	if(VCS_ActivateProfileVelocityMode(g_pKeyHandle, g_usNodeId, &ulErrorCode) == 0)
	{
		LogError("VCS_ActivateProfileVelocityMode Node 2", lResult, ulErrorCode);
		lResult = MMC_FAILED;
	}
	LogInfo("activate 2");

	if(VCS_ActivateProfileVelocityMode(g_pKeyHandle_sub, g_usNodeId1, &ulErrorCode) == 0)
	{
		LogError("VCS_ActivateProfileVelocityMode Node 3", lResult, ulErrorCode);
		lResult = MMC_FAILED;
	}
	LogInfo("activate 3");


	//buradan sonra mesajlara göre hız basacağız
	ros::Subscriber sub = n.subscribe("/agv/cmd_vel", 1, VelocityCommandCallback);

	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	tf::TransformBroadcaster odom_broadcaster;
	
	// başlangıç değerleri
	double x = 0.0;
	double y = 0.0;
	double th = 0.0;
	//başalngıç değerleri
	double vx = 0.0;
	double vy = 0.0;
	double vth = 0.0;

	double vl=0.0;
	double vr=0.0;

	long el = 0.0;
	long er = 0.0;
	
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	
	ros::Rate r(10);

	while(n.ok())
	{	
		ros::spinOnce();               // check for cmd_vel message callbacks
		current_time = ros::Time::now();
		//robotton encoder değerleri alma
		if((lResult = GetVelocityIs (ulErrorCode))!=MMC_SUCCESS)
		{
			LogError("PrepareDemo", lResult, ulErrorCode);
			return lResult;
		}

		el = *g_pRPMLeftWheel;
		er = *g_pRPMRightWheel;
		//ROS_INFO("I said L: [%f]",el);
	    //ROS_INFO("I said L: [%f]",el);

		vl = -el /95.23; 
		vr = (er) /95.23;

		//vl= getvelocityis(rpm)*katsayı
		//vr = - get velocity (rpm)*katsayı	
		vth = (vr-vl)/(100*b);
		vx = (vl+vr)/200;
		//vy = 0

		//compute odometry in a typical way given the velocities of the robot
		double dt = (current_time - last_time).toSec();
		double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
		double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
		double delta_th = vth * dt;
	
		x += delta_x;
		y += delta_y;
		th += delta_th;
		
		//ROS_INFO("My x: [%f]",x);
		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
	
		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";
	
		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;
	
		//send the transform
		odom_broadcaster.sendTransform(odom_trans);
	
		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
	
		//set the position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
	
		//set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = vth;
	
		//publish the message
		odom_pub.publish(odom);
	
		last_time = current_time;
		r.sleep();
	  }
	return lResult;
}
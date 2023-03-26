#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <string>
#include <string.h>
#include "std_msgs/String.h"              //ros定义的String数据类型
#include  "Uwb_Location/trilateration.h"
#include "Uwb_Location/uwb.h"
#include <sensor_msgs/Imu.h>
using namespace std;
char mc_come_flag = 0;
int  receive_data = 0;
unsigned char receive_buf[200] = {0};
vec3d report;
Quaternion q;
int result = 0; 
vec3d anchorArray[4];
float velocityac[3],angleac[3];
Quaternion Q;
//对获取来的数据进行处理
void receive_deal_func()
{
	int aid, tid,lnum, seq, mask,range[4];
    
	int rangetime;
	char c, type,id[2];
    double time;
    float Range_deca[4];
     // A0
    anchorArray[0].x = 2.5; //anchor0.x uint:m
    anchorArray[0].y = 1.5; //anchor0.y uint:m
    anchorArray[0].z = 0.0; //anchor0.z uint:m

    //A1
    anchorArray[1].x = 2.5; //anchor2.x uint:m
    anchorArray[1].y = -1.5; //anchor2.y uint:m
    anchorArray[1].z = 0.0;
    //A20
    anchorArray[2].x = -2.5; //anchor2.x uint:m
    anchorArray[2].y = 1.5; //anchor2.y uint:m
    anchorArray[2].z = 0.0; //anchor2.z uint:m

    //A3
    anchorArray[3].x = 1.6; //anchor2.x uint:m
    anchorArray[3].y = 0.0; //anchor2.y uint:m
    anchorArray[3].z = 0.0; //anchor2.z uint:m

    //这款产品的串口数据:时间，A0距离，A1距离，A2距离，A3距离，X加速度，Y加速度，Z加速度，X角速度，Y角速度
	//int n = sscanf((char*)receive_buf,"m%c %x %x %x %x %x %x %x %x %c%d:%d", &type, &mask, &range[0], &range[1], &range[2], &range[3], &lnum, &seq, &rangetime, &c, &tid, &aid);
    
   printf("result1=%d\n",result);
    int n = sscanf((char*)receive_buf,"m%c %x %x %x %x %x %x %x %x %c%d:%d", &type, &mask, &range[0], &range[1], &range[2], &range[3], &lnum, &seq, &rangetime, &c, &tid, &aid);
	printf("mask=0x%02x\nrange[0]=%d(mm)\nrange[1]=%d(mm)\nrange[2]=%d(mm)\nrange[3]=%d(mm)\r\n",mask,range[0], range[1], range[2], range[3]);
    //cout<<receive_buf<<endl;
    // cout<<"0:"<<range[0]<<endl;
    // cout<<"1:"<<range[1]<<endl;
    // cout<<"2:"<<range[2]<<endl;
    // cout<<"3:"<<range[3]<<endl;
	Range_deca[0] = range[0]; //tag to A0 distance
	Range_deca[1] = range[1]; //tag to A1 distance
	Range_deca[2] = range[2]; //tag to A2 distance
	Range_deca[3] = range[3];

    //获取uwb定位信息
    result = GetLocation(&report, 1, &anchorArray[0],&Range_deca[0]);
   
    //获取imu的四元数信息
    ImuData_t pM;
    pM.accX=velocityac[0];
    pM.accY=velocityac[1];
    pM.accZ=velocityac[2];
    pM.gyroX=angleac[0];
    pM.gyroY=angleac[1];
    pM.gyroZ=angleac[2];
    
    
    Q=GetAngle(&pM,0.05f);
    // Quaternion1 Q;
    cout<<"w:"<<Q.q0<<endl;
    cout<<"x:"<<Q.q1<<endl;
    cout<<"y:"<<Q.q2<<endl;
    cout<<"z:"<<Q.q3<<endl;
    
     printf("result=%d\n",result);
    printf("x=%f\n",report.x);
   printf("y=%f\n",report.y);
   printf("z=%f\n",report.z);

    //printf("tag.x=%.3f\r\ntag.y=%.3f\r\ntag.z=%.3f\r\n",report.x,report.y,report.z);
	//printf("time=%.2f\nrange[0]=%.2f(mm)\nrange[1]=%.2f(mm)\nrange[2]=%.2f(mm)\nrange[3]=%.2f(mm)\nvelocityac[0]=%.2f(m^2/s)\nvelocityac[1]=%.2f(m^2/s)\nvelocityac[2]=%.2f(m^2/s)\nangleac[0]=%.2f(rad/s)\nangleac[1]=%.2ff(rad/s)\nangleac[2]=%.2f(rad/s)\r\n",time,range[0], range[1], range[2], range[3],velocityac[0],velocityac[1],velocityac[2],angleac[0],angleac[1],angleac[2]);
}


int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
	std_msgs::String msg;
	std_msgs::String  msg_mc;
	int  data_size;
	int n;
	int cnt = 0;
    ros::init(argc, argv, "uwb_imu_node");//发布imu,uwb节点
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle nh;
    ros::NodeHandle nh1;
    ros::Publisher uwb_publisher = nh.advertise<Uwb_Location::uwb>("/uwb/data", 1000);//发布uwb数据  话题名 队列大小
    ros::Publisher IMU_read_pub = nh.advertise<sensor_msgs::Imu>( "imu/data", 1000 );//发布imu话题

    //创建一个serial类
    serial::Serial sp;
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(11);
    //设置要打开的串口名称
    sp.setPort("/dev/ttyUSB1");
    //设置串口通信的波特率
    sp.setBaudrate(115200);
    //串口设置timeout
    sp.setTimeout(to);
 
    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB1 is opened.");
    }
    else
    {
        return -1;
    }
    
    ros::Rate loop_rate(11);
    //发布uwb话题
    Uwb_Location::uwb uwb_data;
    //打包IMU数据
    sensor_msgs::Imu imu_data;
    while(ros::ok())
    {
        //获取缓冲区内的字节数
        size_t data_size = sp.available();
        if(data_size>0)
        {
            //uint8_t buffer[1024];
            //读出数据
            //n = sp.read(buffer, n);
			
            
           // for(int i=0; i<n; i++)
           // {
                //16进制的方式打印到屏幕
           //     std::cout << std::hex << (buffer[i] & 0xff) << " ";
           // }
	        //std::string str(&buffer[0],&buffer[n-1]);            //将数组转化为字符串
		//	std_msgs::String msg;
       	//	std::stringstream ss;
       	//	ss <<str;
     
      	//	msg.data = ss.str();
     
   	    //    ROS_INFO("%s", msg.data.c_str());//打印接受到的字符串
	    //    std::cout << std::endl;
            //把数据发送回去
           // sp.write(buffer, n);
		   unsigned char usart_buf[1024]={0};
		   sp.read(usart_buf ,data_size);
		  // n = msg.data.find("mc")
          string strtmp = (char*) usart_buf;
            //cout<<"原始"<<strtmp<<endl;
            //n = strtmp.find("mc");//返回的是T2出现的位置
           //printf("not come\r\n");
		   
		   if(mc_come_flag == 1)
		   {
			   if((data_size - (111 - receive_data)) >= 0)
			   {
				   strncpy((char*)&receive_buf[receive_data],(char*)usart_buf,111-receive_data);			   
			       receive_data = 0;                                                 //接收完成
				   mc_come_flag = 0;
                
				   receive_deal_func();
			    }
                else if(data_size > 0)
			    {
				    strncpy((char*)&receive_buf[receive_data],(char*)usart_buf,receive_data);
				    receive_data += data_size;
			    }
			  
		   }
           else if (n  >= 0 )  //&& ((data_size-n) >= 111)			   
		   {
			   mc_come_flag = 1;
			   if(data_size >= 111)
			   {
					strncpy((char*)receive_buf,(char*)usart_buf,111);//把 usart_buf 所指向的字符串复制到 receive_buf，最多复制111 个字符。当 src 的长度小于 n 时，dest 的剩余部分将用空字节填充
					receive_data = 0;
					mc_come_flag = 0;       //接收完成
					receive_deal_func();
			   }else
			   {
				   strncpy((char*)receive_buf,(char*)usart_buf,data_size);
				   receive_data += data_size;
			   }
			   //msg_mc.data = msg.data.substr(data_size-n,111);    
		   }
			//printf("cnt:%d\r\n",cnt++);	
//---------------------------------UWB----------------------------------------------------
        uwb_data.time=ros::Time::now();
        uwb_data.x = report.x;
        uwb_data.y = report.y;
        uwb_data.z = report.z;
        //printf("tag.x=%.3f\r\ntag.y=%.3f\r\ntag.z=%.3f\r\n",uwb_data.x,uwb_data.y,uwb_data.z);

//--------------------------------------IMU------------------------------------------------
        //发布imu话题
        imu_data.header.stamp =uwb_data.time;
        imu_data.header.frame_id = "base_link";
        imu_data.linear_acceleration.x=velocityac[0];
        imu_data.linear_acceleration.y=velocityac[1];
        imu_data.linear_acceleration.z=velocityac[2];

        //角速度
        imu_data.angular_velocity.x = angleac[0]; 
        imu_data.angular_velocity.y = angleac[1]; 
        imu_data.angular_velocity.z = angleac[2];

        //四元数
        imu_data.orientation.x =Q.q1 ;
        imu_data.orientation.y = Q.q2;
        imu_data.orientation.z = Q.q3;
        imu_data.orientation.w = Q.q0;

//--------------------------------------话题发布------------------------------------
        uwb_publisher.publish(uwb_data);
        IMU_read_pub.publish(imu_data);
        }

        ros::spinOnce(); 
        loop_rate.sleep();
    }
    //关闭串口
    sp.close();
    return 0;
}

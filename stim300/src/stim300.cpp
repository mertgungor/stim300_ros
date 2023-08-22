#include "../include/stim300/includes.h"
#include "imu.cpp"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

extern "C" {void configureCOMObject(PHANDLE pHComm,int portNumber);}
// extern "C" {void reBuffer(void);}
extern "C" {void parseBuffer(void);}

// extern vars
float imu_read[3];
float orientation_read[3];
float gyro_read[3];        

int hComm;  //Serial Handle
unsigned char SerialBuffer[1024];
int wrtPtr;
int readPtr;
int meanStart;

struct sockaddr_in addr;
int txSocketHandle;

class Stim300
{
public:

  static void* reBuffer(void* arg){
    extern int hComm;
    extern unsigned char SerialBuffer[];

    int NoBytesRead;
    extern int wrtPtr;
    int wrtIdx;
    unsigned char buffer[200];
    int a = 0;
    wrtPtr=0;

    while(1) {
        //printf("runningTh\n");
        int NoBytesRead = read(hComm, buffer,200);
        //BUFFERING SECTION
        wrtIdx = wrtPtr;
        for(int i=0;i<NoBytesRead;i++) {
            SerialBuffer[wrtIdx++]=buffer[i];
            wrtIdx &= IDXMASK;
        }
        wrtPtr = wrtIdx;
    }

    return nullptr;
  }

  void publish_imu(){
  
      while (true){
  
          printf("IMU acc_x: %f\n",  imu_read[0]);
          printf("IMU acc_y: %f\n",  imu_read[1]);
          printf("IMU acc_z: %f\n\n",imu_read[2]);
          printf("IMU gyro_x: %f\n",  gyro_read[0]);
          printf("IMU gyro_y: %f\n",  gyro_read[1]);
          printf("IMU gyro_z: %f\n\n",gyro_read[2]);
          printf("IMU orientation_x: %f\n",  orientation_read[0]);
          printf("IMU orientation_y: %f\n",  orientation_read[1]);
          printf("IMU orientation_z: %f\n\n",orientation_read[2]);
          printf("i: %d, num of samples: %d\n", i, numberofsamples);
          printf("--------------------------------------------------\n");
  
          if(imu_read[0]!=0){
              double gx = gyro_read[0]*N_PI/180.0; double gy = -gyro_read[1]*N_PI/180.0; double gz = -gyro_read[2]*N_PI/180.0;
              double ax = -imu_read[0]*M_A;	  double ay = imu_read[1]*M_A;		 double az = imu_read[2]*M_A;
              if (flag){
                  obj.calc_and_set_Quat(ax,ay,az,0.0);
                  flag = false;
              }
              obj.MadgwickIMU(ax,ay,az,gx,gy,gz);
            //   obj.printQuat();
              
              q = obj.get_quat();
              //obj.go_back_eul();
              //obj.printEul();
              //obj.LinVelCalc_from_LinAccInt(ax,ay,az);imu_msg.angular_velocity.x = gyro_read[0];
              imu_msg.header.stamp = ros::Time::now();
              imu_msg.angular_velocity.x = gyro_read[0];
              imu_msg.angular_velocity.y = gyro_read[1];
              imu_msg.angular_velocity.z = gyro_read[2];
  
              imu_msg.linear_acceleration.x = imu_read[0];
              imu_msg.linear_acceleration.y = imu_read[1];
              imu_msg.linear_acceleration.z = imu_read[2];

              imu_msg.orientation.w = q[0];
              imu_msg.orientation.x = q[1];
              imu_msg.orientation.y = q[2];
              imu_msg.orientation.z = q[3];

            //   imu_pub.publish(imu_msg);
              
              i++;
          }
        //   else  printf("imu read: %f\n", imu_read[0]);
  				
  	}
    //   printf("Num of samples %.2f" , numberofsamples);
  
  }

  static void* threadEntryPoint(void* instance) {
    Stim300* myInstance = static_cast<Stim300*>(instance);
    myInstance->publish_imu();
    return nullptr;
  }

  Stim300(int argc, char** argv){
    ros::init(argc, argv, "stim300_driver_node");
    ros::NodeHandle nh;
    ros::Rate rate = ros::Rate(250.0); 

    imu_msg.header.frame_id = "imu";
    imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data", 500);
    obj = LinAccInt(0.0017,250.0);

    configureCOMObject(&hComm,1);

    pthread_create(&thRebuffer,    NULL, reBuffer,   NULL);
    pthread_create(&thparseBuffer, NULL, (void *)parseBuffer,NULL);
    pthread_create(&publish_imu_th,  NULL, threadEntryPoint,   this);

    int keyInfo;
    meanStart=FALSE;
    flag = true;

    timer = nh.createTimer(ros::Duration(1.0 / 250), &Stim300::timerCallback, this);

  }

  ~Stim300(){
    close(hComm);
    pthread_join(thRebuffer,NULL);
    pthread_join(thparseBuffer,NULL);
    pthread_join(publish_imu_th,NULL);

  }

private:
  pthread_t thRebuffer, thparseBuffer, publish_imu_th;
  int numberofsamples = 7500;
  int i = 0;
  LinAccInt obj;
  bool flag;
  double* q;
  sensor_msgs::Imu imu_msg;
  ros::Publisher imu_pub;
  ros::Timer timer;

  void timerCallback(const ros::TimerEvent& event) {
        imu_msg.header.stamp = ros::Time::now();
        imu_pub.publish(imu_msg);
    }

};

int main(int argc, char** argv)
{
  Stim300 stim300(argc, argv);
  ros::spin();
  return 0;
}
  

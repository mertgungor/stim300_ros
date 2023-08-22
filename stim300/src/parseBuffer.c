#include "../include/stim300/includes.h"

void parseBuffer(void) {

    extern float imu_read[];
    extern float orientation_read[];
    extern float gyro_read[];        

    extern int wrtPtr;
    extern int readPtr;

    extern unsigned char SerialBuffer[];
    extern int meanStart;

    int readIdx;
    int dataIdx;
    unsigned char buffer[3];
    unsigned long dWord;
    int dummy;

    int bytesAvailable;


    float dataOut[3];

    int counter,prevCounter,farkCounter;
    float timeCounter;
    int cSum;
    float *pfloat;

    int runOnce;
    int i,j;
    int count = 0;


    float accX,accY,accZ;
    float velX,velY,velZ;
    float xTotal,yTotal,zTotal;
    float meanX,meanY,meanZ;
    int numElem;
    int firstData;


    accX=0;
    accY=0;
    accZ=0;

    firstData = TRUE;
    numElem=0;
    runOnce = FALSE;

    readPtr = 0;
    float G = 9.80665;

    while(TRUE) {


        bytesAvailable = (wrtPtr-readPtr)&IDXMASK;
        readIdx = readPtr;

        // print bytesAvailable
        // printf("bytesAvailable: %d\n",bytesAvailable);

        while(bytesAvailable>50) {
            if(SerialBuffer[readIdx]==HEADER_BO){
                // printf("HEADER BULDU\n");

                // read acc -------------------------------

            dataIdx = (readIdx + 11)&IDXMASK;

                for(int idx=0;idx<3;idx++) {
                    dWord = 0;
                    dWord |= SerialBuffer[dataIdx++]; 
                    dWord = dWord << 8;

                    dWord |= SerialBuffer[dataIdx++]; 
                    dWord = dWord << 8;

                    dWord |= SerialBuffer[dataIdx++]; 

                    if(dWord&0x800000) {
                        dummy = (-0x800000 + (dWord & 0x7FFFFF));
                        dataOut[idx] = (float)dummy*0.000001907*G;
                    }
                    else{
                        dummy = (dWord & 0x7FFFFF);
                        dataOut[idx] = (float)dummy*0.000001907*G;
                    } 

                }

            imu_read[0] = dataOut[0];
            imu_read[1] = dataOut[1];
            imu_read[2] = dataOut[2];

            // read orientation -------------------------------


            dataIdx = readPtr;
            dataIdx = (readIdx + 21)&IDXMASK;

                for(int idx=0;idx<3;idx++) {
                    dWord = 0;
                    dWord |= SerialBuffer[dataIdx++]; 
                    dWord = dWord << 8;

                    dWord |= SerialBuffer[dataIdx++]; 
                    dWord = dWord << 8;

                    dWord |= SerialBuffer[dataIdx++]; 

                    if(dWord&0x800000) {
                        dummy = (-0x800000 + (dWord & 0x7FFFFF));
                        dataOut[idx] = (float)dummy*0.2384e-6*G;
                    }
                    else{
                        dummy = (dWord & 0x7FFFFF);
                        dataOut[idx] = (float)dummy*0.2384e-6*G;
                    } 

                }

                orientation_read[0] = dataOut[0];
                orientation_read[1] = dataOut[1];
                orientation_read[2] = dataOut[2];

                // read ang vel -------------------------------

                dataIdx = readPtr;
                dataIdx = (readIdx + 1)&IDXMASK;

                for(int idx=0;idx<3;idx++) {
                    dWord = 0;
                    dWord |= SerialBuffer[dataIdx++]; 
                    dWord = dWord << 8;

                    dWord |= SerialBuffer[dataIdx++]; 
                    dWord = dWord << 8;

                    dWord |= SerialBuffer[dataIdx++]; 

                    if(dWord&0x800000) {
                        dummy = (-0x800000 + (dWord & 0x7FFFFF));
                        dataOut[idx] = (float)dummy*61.035e-6;
                    }
                    else{
                        dummy = (dWord & 0x7FFFFF);
                        dataOut[idx] = (float)dummy*61.035e-6;
                    } 

                }

                gyro_read[0] = dataOut[0]*3.14159/180;
                gyro_read[1] = dataOut[1]*3.14159/180;
                gyro_read[2] = dataOut[2]*3.14159/180;
                
                // printf("HEADER BULDU\n");

                // printf("accX:%7.3f accY:%7.3f accZ:%7.3f velX:%7.3f velY:%7.3f velZ:%7.3f\n",accX, accY, accZ, velX, velY, velZ);
                readIdx=(readIdx+PACKETSIZE)&IDXMASK;
                bytesAvailable=bytesAvailable-PACKETSIZE;
            }
            else {
                printf("HEADER TUTMADI");
                readIdx=(readIdx+1)&IDXMASK;
                bytesAvailable=bytesAvailable-1;
            }

            

        }
        readPtr=readIdx;

    }
    return;
}
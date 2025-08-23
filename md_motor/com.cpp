#include "md_robot_node/global.hpp"
#include "md_robot_node/main.hpp"
#include "md_robot_node/com.hpp"
#include <boost/thread.hpp>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>

#define MD_PROTOCOL_POS_PID       3
#define MD_PROTOCOL_POS_DATA_LEN  4
#define MD_PROTOCOL_POS_DATA_START 5

serial::Serial ser;

PID_PNT_MAIN_DATA_t curr_pid_pnt_main_data;
PID_ROBOT_MONITOR2_t curr_pid_robot_monitor2;
PID_ROBOT_MONITOR_t curr_pid_robot_monitor;
PID_PNT_IO_MONITOR_t curr_pid_pnt_io_monitor;
PID_GAIN_t curr_pid_gain;

uint8_t serial_comm_rcv_buff[MAX_PACKET_SIZE];
uint8_t serial_comm_snd_buff[MAX_PACKET_SIZE];
int ts_count = 0;

extern INIT_SETTING_STATE_t fgInitsetting;
extern int CalRobotPoseFromRPM(PID_PNT_MAIN_DATA_t *pData);
extern void MakeMDRobotMessage1(PID_PNT_MAIN_DATA_t *pData);
extern void MakeMDRobotMessage2(PID_ROBOT_MONITOR_t *pData);
extern void PubRobotPose(void);
extern void PubMDRobotMessage2(void);
extern void PubMDRobotMessage1(void);

// === [추가] ===
static ros::Publisher g_pub_rpm_left;
static ros::Publisher g_pub_rpm_right;
static ros::Publisher g_pub_current_left;
static ros::Publisher g_pub_current_right;
static ros::Publisher g_pub_state_left;
static ros::Publisher g_pub_state_right;

static void ensure_publishers(double &cur_scale)
{
    static bool inited = false;
    static bool scale_inited = false;

    if (!inited) {
        ros::NodeHandle nh;
        g_pub_rpm_left      = nh.advertise<std_msgs::Float32>("/md/rpm_left", 10);
        g_pub_rpm_right     = nh.advertise<std_msgs::Float32>("/md/rpm_right", 10);
        g_pub_current_left  = nh.advertise<std_msgs::Float32>("/md/current_left", 10);
        g_pub_current_right = nh.advertise<std_msgs::Float32>("/md/current_right", 10);
        g_pub_state_left    = nh.advertise<std_msgs::UInt8>("/md/state_left", 10);
        g_pub_state_right   = nh.advertise<std_msgs::UInt8>("/md/state_right", 10);
        inited = true;
    }
    if (!scale_inited) {
        ros::NodeHandle pnh("~");
        pnh.param("current_scale", cur_scale, 0.1); // 0.1A/LSB 가정 (문서 기준)
        scale_inited = true;
    }
}

int InitSerialComm(void)
{
    try
    {
        ser.setPort("/dev/ttyMotor");//ttyMotor
        ser.setBaudrate(robotParamData.nBaudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1667); //1667 when baud is 57600, 0.6ms
        ser.setTimeout(to);                                       //2857 when baud is 115200, 0.35ms
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()) {
        ROS_INFO_STREAM("Serial Port initialized");
        return 1;
    }
    else {
        return -1;
    }
}

uint16_t CalCheckSum(uint8_t *pData, uint16_t length)
{
    uint16_t sum;

    sum = 0;
    for(int i = 0; i < length; i++) {
        sum += *pData++;
    }

    sum = ~sum + 1; //check sum

    return sum;
}

int PutMdData(PID_CMD_t pid, uint16_t rmid, const uint8_t *pData, uint16_t length)
{
    uint8_t *p;
    uint16_t len;

    len = 0;
    serial_comm_snd_buff[len++] = rmid;
    serial_comm_snd_buff[len++] = robotParamData.nIDPC;
    serial_comm_snd_buff[len++] = 1;
    serial_comm_snd_buff[len++] = (uint8_t)pid;
    serial_comm_snd_buff[len++] = length;

    p = (uint8_t *)&serial_comm_snd_buff[len];
    memcpy((char *)p, (char *)pData, length);
    len += length;

    serial_comm_snd_buff[len++] = CalCheckSum(serial_comm_snd_buff, len);

    if(ser.isOpen() == true) {
        ser.write(serial_comm_snd_buff, len);
        boost::this_thread::sleep_for(boost::chrono::milliseconds(3));
        for(int i = 0; i < len; i++) serial_comm_snd_buff[i] = 0; // LM nano to driver send data reset
    }

    return 1;
}

int MdReceiveProc(void) //save the identified serial data to defined variable according to PID NUMBER data
{
    uint8_t *pRcvBuf;
    uint8_t *pRcvData;
    uint8_t byRcvPID;
    uint8_t byRcvDataSize;

    pRcvBuf = serial_comm_rcv_buff;

    byRcvPID      = pRcvBuf[MD_PROTOCOL_POS_PID];
    byRcvDataSize = pRcvBuf[MD_PROTOCOL_POS_DATA_LEN];
    pRcvData      = &pRcvBuf[MD_PROTOCOL_POS_DATA_START];

    static double CURRENT_SCALE = 0.1;
    ensure_publishers(CURRENT_SCALE);

    switch(byRcvPID)
    {
        case PID_GAIN:
        {
            if(byRcvDataSize == sizeof(PID_GAIN_t)) {
                memcpy((char *)&curr_pid_gain, (char *)pRcvData, sizeof(PID_GAIN_t));
            }
            break;
        }

        case PID_PNT_MAIN_DATA: // 210 (18 bytes, dual-motor: rpm/current/state/pos * 2)
        {
            if(fgInitsetting != INIT_SETTING_STATE_OK) {
                break;
            }

            if(byRcvDataSize == sizeof(PID_PNT_MAIN_DATA_t)) {
                memcpy((char *)&curr_pid_pnt_main_data, (char *)pRcvData, sizeof(PID_PNT_MAIN_DATA_t));

                MakeMDRobotMessage1(&curr_pid_pnt_main_data);
                PubMDRobotMessage1();

                // RPM/Current 퍼블리시
                std_msgs::Float32 m;

                // 왼쪽 모터
                m.data = (float)curr_pid_pnt_main_data.rpm_id1;
                g_pub_rpm_left.publish(m);
                m.data = (float)curr_pid_pnt_main_data.current_id1*CURRENT_SCALE;
                g_pub_current_left.publish(m);

                // 오른쪽 모터
                m.data = (float)curr_pid_pnt_main_data.rpm_id2;
                g_pub_rpm_right.publish(m);
                m.data = (float)curr_pid_pnt_main_data.current_id2*CURRENT_SCALE;
                g_pub_current_right.publish(m);

                std_msgs::UInt8 s;
                s.data = curr_pid_pnt_main_data.mtr_state_id1.val;
                g_pub_state_left.publish(s);
                s.data = curr_pid_pnt_main_data.mtr_state_id2.val;
                g_pub_state_right.publish(s);                

            }
            break;
        }

        case PID_ROBOT_PARAM:  // 247
        {
            if(byRcvDataSize == sizeof(PID_ROBOT_PARAM_t)) {
                PID_ROBOT_PARAM_t *p;
                p = (PID_ROBOT_PARAM_t *)pRcvData;
                robotParamData.sSetDia      = p->nDiameter;            // mm unit
                robotParamData.sSetWheelLen = p->nWheelLength;           // mm unit
                robotParamData.sSetGear     = p->nGearRatio;
            }
            break;
        }

        case PID_ROBOT_MONITOR2:        // 224 (sVoltIn 포함)
        {
            if(byRcvDataSize == sizeof(PID_ROBOT_MONITOR2_t)) {
                memcpy((char *)&curr_pid_robot_monitor2, (char *)pRcvData, sizeof(PID_ROBOT_MONITOR2_t));
            }
            break;
        }

        case PID_PNT_IO_MONITOR:        // 241 (sVoltIn 포함)
        {
            if(byRcvDataSize == sizeof(PID_PNT_IO_MONITOR_t)) {
                memcpy((char *)&curr_pid_pnt_io_monitor, (char *)pRcvData, sizeof(PID_PNT_IO_MONITOR_t));

            }
            break;
        }

        case PID_ROBOT_MONITOR:        // 253
        {
            if(byRcvDataSize == sizeof(PID_ROBOT_MONITOR_t)) {
                memcpy((char *)&curr_pid_robot_monitor, (char *)pRcvData, sizeof(PID_ROBOT_MONITOR_t));
                std_msgs::Float32 b;
                b.data = (float)curr_pid_robot_monitor.battery_percent;
                g_pub_battery.publish(b);

                if(robotParamData.use_MDUI == 1) {    // If using MDUI
                    MakeMDRobotMessage2(&curr_pid_robot_monitor);
                    PubMDRobotMessage2();
                }
            }
            break;
        }
    }
    for(int i = 0; i < MAX_PACKET_SIZE; i++) serial_comm_rcv_buff[i] = 0;
    return 1;
}

int AnalyzeReceivedData(uint8_t byArray[], uint8_t byBufNum) //Analyze the communication data
{
    ros::NodeHandle n;
    ros::Time stamp;
    uint8_t i, j;
    uint8_t data;
    static uint8_t byChkSec;
    static long lExStampSec, lExStampNsec;
    static uint32_t byPacketNum;
    static uint32_t rcv_step;
    static uint8_t byChkSum;
    static uint16_t byMaxDataNum;
    static uint16_t byDataNum;

    if(byPacketNum >= MAX_PACKET_SIZE)
    {
        rcv_step = 0;
        byPacketNum = 0;
        return 0;
    }
    
    for(j = 0; j < byBufNum; j++)
    {
        data = byArray[j];
        switch(rcv_step) {
            case 0:    //Put the reading machin id after checking the data
                if(data == robotParamData.nIDPC)
                {
                    byPacketNum = 0;
                    byChkSum = data;
                    serial_comm_rcv_buff[byPacketNum++] = data;

                    rcv_step++;
                }
                else
                {
                    byPacketNum = 0;
                    ROS_INFO("error.ser: %s, %d", __FILE__, __LINE__);
                }
                break;
            case 1:    //Put the transmitting machin id after checking the data
                if((data == robotParamData.nIDMDUI) || (data == robotParamData.nIDMDT))
                {
                    byChkSum += data;
                    serial_comm_rcv_buff[byPacketNum++] = data;

                    rcv_step++;
                }
                else
                {
                    rcv_step = 0;
                    byPacketNum = 0;
                    ROS_INFO("error.ser: %s, %d", __FILE__, __LINE__);
                }
                break;

            case 2:    //Check ID
                if(data == 1 || data == ID_ALL)
                {
                    byChkSum += data;
                    serial_comm_rcv_buff[byPacketNum++] = data;

                    rcv_step++;
                }
                else
                {
                    rcv_step = 0;
                    byPacketNum = 0;
                    ROS_INFO("error.ser: %s, %d", __FILE__, __LINE__);
                }
                break;
            case 3:    //Put the PID number into the array
                byChkSum += data;
                serial_comm_rcv_buff[byPacketNum++] = data;

                rcv_step++;
                break;

            case 4:    //Put the DATANUM into the array
                byChkSum += data;
                serial_comm_rcv_buff[byPacketNum++] = data;

                byMaxDataNum = data;
                byDataNum = 0;

                rcv_step++;
                break;

            case 5:    //Put the DATA into the array
                byChkSum += data;
                serial_comm_rcv_buff[byPacketNum++] = data;

                if(++byDataNum >= MAX_DATA_SIZE)
                {
                    rcv_step = 0;
                    ROS_INFO("error.ser: %s, %d", __FILE__, __LINE__);
                    break;
                }

                if(byDataNum >= byMaxDataNum) {
                    rcv_step++;
                }
                break;

            case 6:    //Put the check sum after Checking checksum
                byChkSum += data;
                serial_comm_rcv_buff[byPacketNum++] = data;
                if(byChkSum == 0)
                {
                    MdReceiveProc();                     //save the identified serial data to defined variable
                }
                else {
                    ROS_INFO("error.ser: %s, %d", __FILE__, __LINE__);
                }

                byPacketNum = 0;

                rcv_step = 0;
                break;

            default:
                rcv_step = 0;
                break;
        }
    }
    return 1;
}

int ReceiveDataFromController(void) //Analyze the communication data
{
    uint8_t byRcvBuf[250];
    uint8_t byBufNumber;

    static uint8_t tempBuffer[250];
    static uint8_t tempLength;

    byBufNumber = ser.available();
    if(byBufNumber != 0)
    {
        if(byBufNumber > sizeof(byRcvBuf)) {
            byBufNumber = sizeof(byRcvBuf);
        }

        ser.read(byRcvBuf, byBufNumber);
        boost::this_thread::sleep_for(boost::chrono::milliseconds(3));
        memcpy(tempBuffer, byRcvBuf, byBufNumber);
        tempLength = byBufNumber;
        
        AnalyzeReceivedData(tempBuffer, tempLength);
    }

    return 1;
}

// === [추가] 컨트롤러에게 모니터링 데이터 전송을 요청하는 함수 ===
void RequestMonitoringData(void)
{
    // PID_ROBOT_MONITOR2 (전압 포함) 데이터를 요청합니다.
    // PID_PNT_IO_MONITOR도 전압 데이터를 포함하므로 함께 요청합니다.
    // PID_ROBOT_MONITOR (배터리 퍼센트 포함) 데이터를 요청합니다.
    // 데이터 길이는 0으로 설정하면 됩니다.
    PutMdData(PID_ROBOT_MONITOR2, ID_ALL, NULL, 0);
    PutMdData(PID_PNT_IO_MONITOR, ID_ALL, NULL, 0);
    PutMdData(PID_ROBOT_MONITOR, ID_ALL, NULL, 0);
}
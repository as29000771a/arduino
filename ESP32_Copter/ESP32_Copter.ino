#include <esp_now.h> //引用函式庫
#include <WiFi.h>
#include <Wire.h>
#include <EEPROM.h>

#include "RC.h"

#define WIFI_CHANNEL 4  //定義WIFI頻道
#define PWMOUT  // normal esc, uncomment for serial esc
#define LED 2  //定義變數
#define CALSTEPS 256 // gyro and acc calibration steps
//#define externRC // use of external RC receiver in ppmsum mode
//#define webServer // use of webserver to change PID

extern int16_t accZero[3];
extern float yawRate;
extern float rollPitchRate;
extern float P_PID;
extern float I_PID;
extern float D_PID;
extern float P_Level_PID;
extern float I_Level_PID;
extern float D_Level_PID;
//extern 表示此變數已經在別處定義，告知程式到別的地方找尋此變數的定義。

volatile boolean recv;
//被 volatile 宣告的變數 將不會使用最佳化編譯。
//告訴compiler不要自己暫存變數來提升速度，如此這個變數有任何的改變 便會馬上反應出來 
//volatile int peernum = 0;
//esp_now_peer_info_t slave;

void recv_cb(const uint8_t *macaddr, const uint8_t *data, int len) //定義recv_cb()函式
{
  recv = true;
  //Serial.print("recv_cb ");
  //Serial.println(len); 
  if (len == RCdataSize) //如果RCdataSize=len的值 
  {
    for (int i=0;i<RCdataSize;i++) //當i小於RCdataSize時執行以下迴圈
        RCdata.data[i] = data[i]; 
  }
  /*
  if (!esp_now_is_peer_exist(macaddr))
  {
    Serial.println("adding peer ");
    esp_now_add_peer(macaddr, ESP_NOW_ROLE_COMBO, WIFI_CHANNEL, NULL, 0);
    peernum++;
  }
  */
};

#define ACCRESO 4096
#define CYCLETIME 3
#define MINTHROTTLE 1090
#define MIDRUD 1495
#define THRCORR 19
//drfine表示定義變數

enum ang { ROLL,PITCH,YAW }; 
//列舉角度有滾動、俯仰、偏離
//enum用在宣告僅有少數值的型別，像是一星期內的日期 (day of week) 或是一年內的月份等

static int16_t gyroADC[3];
static int16_t accADC[3];
static int16_t gyroData[3];
static float angle[2]    = {0,0};  
//static宣告在函式呼叫之間將其值保留在記憶體中的變數;這種型別的變數有一個靜態的儲存期限。
extern int calibratingA;
//extern 表示此變數已經在別處定義，告知程式到別的地方找尋此變數的定義。

#ifdef flysky  //ifdef若識別字(flysky)被定義,編譯器會編譯此部分程式。
  #define ROL 0
  #define PIT 1
  #define THR 2
  #define RUD 3
#else //orangerx  //否則編譯此部分程式。
  #define ROL 1
  #define PIT 2
  #define THR 0
  #define RUD 3
#endif  

#define AU1 4
#define AU2 5
static int16_t rcCommand[] = {0,0,0};  //static宣告在函式呼叫之間將其值保留在記憶體中的變數

#define GYRO     0
#define STABI    1
static int8_t flightmode;
static int8_t oldflightmode;
//static宣告在函式呼叫之間將其值保留在記憶體中的變數

boolean armed = false;  //armed的boolean(布林值)為零。
uint8_t armct = 0;
int debugvalue = 0;

void setup() //定義setup()函式
{
  Serial.begin(115200); Serial.println(); //每秒資料傳輸115200 bit/s。 //輸出文字並自動換行。

  delay(3000); // give it some time to stop shaking after battery plugin  //延遲 3 秒再往下執行
  MPU6050_init();  //輸入MPU6050 ID number
  MPU6050_readId(); // must be 0x68, 104dec //讀取MPU6050 ID number
  
  EEPROM.begin(64); //申請操作64位元組資料
  if (EEPROM.read(63) != 0x55) Serial.println("Need to do ACC calib"); //如果讀到的63位元組不是0x55的格式，印出Need to do ACC calib。
  else ACC_Read(); // eeprom is initialized //否則ACC成功讀取。
  if (EEPROM.read(62) != 0xAA) Serial.println("Need to check and write PID");//如果讀到的62位元組不是0x55的格式，印出Need to check and write PID。
  else PID_Read(); // eeprom is initialized //否則PID成功讀取。

  
  WiFi.mode(WIFI_STA); // Station mode for esp-now //連接WIFI
  #if defined webServer //如果webServer被定義
    setupwebserver(); //啟動 setupwebserver
    delay(500); 
  #endif


  #if defined externRC //如果externRC被定義
    init_RC(); //寫入RC程式
  #else //否則
    Serial.printf("This mac: %s, ", WiFi.macAddress().c_str()); 
    Serial.printf(", channel: %i\n", WIFI_CHANNEL); 
    if (esp_now_init() != 0) Serial.println("*** ESP_Now init failed");
    //如果esp_now_init()的傳回值 不為0(true)，印出ESP_Now初始化失敗。
    esp_now_register_recv_cb(recv_cb); 
  #endif

  delay(500); 
  pinMode(LED,OUTPUT); //設定輸出的PIN接腳
  digitalWrite(LED,LOW); //設定PIN LED腳位為低電位 = 0V
  initServo();
}

uint32_t rxt; // receive time, used for falisave

void loop() //定義loop()函式
{
  uint32_t now,mnow,diff; 
  now = millis(); // actual time
  if (debugvalue == 5) mnow = micros();

  #if defined webServer //如果webServer被定義
    loopwebserver(); //啟動loopwebserver
  #endif

  if (recv) //如果有接收到
  {
    recv = false;  
    #if !defined externRC  //如果externRC沒有定義
      buf_to_rc(); //執行buf_to_rc()
    #endif

    if (debugvalue == 4) Serial.printf("%4d %4d %4d %4d \n", rcValue[0], rcValue[1], rcValue[2], rcValue[3]); 
  
    if      (rcValue[AU1] < 1300) flightmode = GYRO; //如果rcValue[AU1]小於1300，飛行模式設定成重力模式
    else                          flightmode = STABI;//否則設定成穩定模式
    if (oldflightmode != flightmode) //如果原本的飛機模式不等於新的飛機模式
    {
      zeroGyroAccI(); //執行zeroGyroAccI()，歸零重力加速度
      oldflightmode = flightmode; //新飛機模式指定給舊飛機模式
    }

    if (armed) //如果有配備
    {
      rcValue[THR]    -= THRCORR;
      rcCommand[ROLL]  = rcValue[ROL] - MIDRUD; //rcValue[ROL] - MIDRUD指定給接收滾動命令
      rcCommand[PITCH] = rcValue[PIT] - MIDRUD; //rcValue[PIT] - MIDRUD指定給接收俯仰命令
      rcCommand[YAW]   = rcValue[RUD] - MIDRUD; //rcValue[RUD] - MIDRUD指定給接收偏差命令
    }  
    else
    {  
      if (rcValue[THR] < MINTHROTTLE) armct++; 
      if (armct >= 25) 
      { 
        digitalWrite(LED,HIGH); //設定PIN LED腳位為高電位=5V
        armed = true;
      }
    }

    if (debugvalue == 5) Serial.printf("RC input ms: %d\n",now - rxt);
    rxt = millis();
  }

  Gyro_getADC(); //執行紀錄角速度傳感器
  
  ACC_getADC(); //執行加速度傳感器

  getEstimatedAttitude(); //執行估計姿態函數

  pid(); //執行pid控制器

  mix(); //執行mix函式

  writeServo(); //執行紀錄函式
  
  // Failsave part //故障保護
  if (now > rxt+90)
  {
    rcValue[THR] = MINTHROTTLE;
    if (debugvalue == 5) Serial.printf("RC Failsafe after %d \n",now-rxt);
    rxt = now;
  }

  // parser part //數據分析
  if (Serial.available())
  {
    char ch = Serial.read();
    // Perform ACC calibration //加速度校準
    if (ch == 10) Serial.println();
    else if (ch == 'A')
    { 
      Serial.println("Doing ACC calib");
      calibratingA = CALSTEPS;
      while (calibratingA != 0)
      {
        delay(CYCLETIME);
        ACC_getADC(); 
      }
      ACC_Store();
      Serial.println("ACC calib Done");
    }
    else if (ch == 'R') //如果ch的值等於'R'
    {
      Serial.print("Act Rate :  ");
      Serial.print(yawRate); Serial.print("  ");
      Serial.print(rollPitchRate); Serial.println();
      Serial.println("Act PID :");
      Serial.print(P_PID); Serial.print("  ");
      Serial.print(I_PID); Serial.print("  ");
      Serial.print(D_PID); Serial.println();
      Serial.print(P_Level_PID); Serial.print("  ");
      Serial.print(I_Level_PID); Serial.print("  ");
      Serial.print(D_Level_PID); Serial.println();
    }
    else if (ch == 'D') //如果ch的值等於'D'
    {
      Serial.println("Loading default PID");
      yawRate = 6.0;
      rollPitchRate = 5.0;
      P_PID = 0.15;    // P8
      I_PID = 0.00;    // I8
      D_PID = 0.08; 
      P_Level_PID = 0.35;   // P8
      I_Level_PID = 0.00;   // I8
      D_Level_PID = 0.10;
      PID_Store();
    }
    else if (ch == 'W') //如果ch的值等於'W'
    {
      char ch = Serial.read();
      int n = Serial.available();
      if (n == 3)
      {
        n = readsernum();        
        if      (ch == 'p') { P_PID       = float(n) * 0.01 + 0.004; Serial.print("pid P ");       Serial.print(P_PID); }
        else if (ch == 'i') { I_PID       = float(n) * 0.01 + 0.004; Serial.print("pid I ");       Serial.print(I_PID); }
        else if (ch == 'd') { D_PID       = float(n) * 0.01 + 0.004; Serial.print("pid D ");       Serial.print(D_PID); }
        else if (ch == 'P') { P_Level_PID = float(n) * 0.01 + 0.004; Serial.print("pid Level P "); Serial.print(P_Level_PID); }
        else if (ch == 'I') { I_Level_PID = float(n) * 0.01 + 0.004; Serial.print("pid Level I "); Serial.print(I_Level_PID); }
        else if (ch == 'D') { D_Level_PID = float(n) * 0.01 + 0.004; Serial.print("pid Level D "); Serial.print(D_Level_PID); }
        else Serial.println("unknown command");
      }
      else if (ch == 'S') { PID_Store(); Serial.print("stored in EEPROM"); } //如果ch的值等於'S'，PID的值被儲存，且印出stored in EEPROM
      else 
      {
        Serial.println("Input format wrong");
        Serial.println("Wpxx, Wixx, Wdxx - write gyro PID, example: Wd13");
        Serial.println("WPxx, WIxx, WDxx - write level PID, example: WD21");
      }
    }
    else if (ch >= '0' && ch <='9') debugvalue = ch -'0'; //如果ch的值在'0'到'9'之間，則debugvalue被指定為ch -'0'
    else
    {
      Serial.println("A - acc calib");
      Serial.println("D - write default PID");
      Serial.println("R - read actual PID");
      Serial.println("Wpxx, Wixx, Wdxx - write gyro PID");
      Serial.println("WPxx, WIxx, WDxx - write level PID");
      Serial.println("WS - Store PID in EEPROM");
      Serial.println("Display data:");
      Serial.println("0 - off");
      Serial.println("1 - Gyro values");
      Serial.println("2 - Acc values");
      Serial.println("3 - Angle values");
      Serial.println("4 - RC values");
      Serial.println("5 - Cycletime");
    }
  }

  if      (debugvalue == 1) Serial.printf("%4d %4d %4d \n", gyroADC[0], gyroADC[1], gyroADC[2]); //如果debugvalue等於1，輸出角速度的值
  else if (debugvalue == 2) Serial.printf("%5d %5d %5d \n", accADC[0], accADC[1], accADC[2]); //如果debugvalue等於2，輸出加速度的值
  else if (debugvalue == 3) Serial.printf("%3f %3f \n", angle[0], angle[1]);  //如果debugvalue等於3，輸出角度的值
  
  delay(CYCLETIME-1);  

  if (debugvalue == 5) 
  {
    diff = micros() - mnow;
    Serial.println(diff); 
  }
}

int readsernum()
{
  int num;
  char numStr[3];  //定義numStr字串
  numStr[0] = Serial.read(); //將Serial.read()的讀取值指定給字串的第一個
  numStr[1] = Serial.read(); //將Serial.read()的另一個讀取值指定給字串的第二個
  return atol(numStr); //傳回numStr傳遞給函數調用的C-type字符串轉換為長整數
}

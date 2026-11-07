// NestTemp_Base_A -- 方案 A 的基站 (配合伪蛋节点, 须持续供电, 不能深睡)
// 供电红线: BLE 扫描不能深睡, 持续扫描约 80-120mA, 必须市电 USB 供电,
//           或 20000mAh 充电宝 + 太阳能板. 用电池+深睡的基站会漏收, 不要这样搭.
// 硬件: 经典 ESP32 开发板 (USB 供电) + DS3231 RTC(带 CR2032, 同样注意 ZS-042 充电电阻)
// 数据: 收到的每条广播打上基站 RTC 时间戳, 存 LittleFS /egg_log.csv, 串口 DUMP 取回
//       seq 不连续即丢包, 在日志里记 GAP, 不要事后补数.
// 距离: 开阔地 BLE 约 10-20 米, 树叶+雨水会缩到 5-10 米. 基站放巢下 5 米内, 别隔一堵墙.
// 对时: 串口 T<unix> 写 DS3231, 写法与 Host B 相同.

#include <Arduino.h>
#include <Wire.h>
#include <LittleFS.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#define RTC_VCC_PIN 33
#define RTC_SDA_PIN 21
#define RTC_SCL_PIN 22
#define DS3231_ADDR 0x68
#define LOG_PATH "/egg_log.csv"
#define COMPANY_ID 0xFFFF

static uint8_t bcd2dec(uint8_t v){ return (v>>4)*10+(v&0x0F); }
static uint8_t dec2bcd(uint8_t v){ return ((v/10)<<4)|(v%10); }
int g_rtc_year=0, g_rtc_month=0, g_rtc_day=0, g_rtc_hour=0, g_rtc_min=0, g_rtc_sec=0;
static bool ds3231Read(){

  pinMode(RTC_VCC_PIN,OUTPUT); digitalWrite(RTC_VCC_PIN,HIGH); delay(15);
  Wire.begin(RTC_SDA_PIN,RTC_SCL_PIN);
  Wire.beginTransmission(DS3231_ADDR); Wire.write(0x00);
  if(Wire.endTransmission()!=0){ Wire.end(); digitalWrite(RTC_VCC_PIN,LOW); return false; }
  Wire.requestFrom(DS3231_ADDR,7);
  if(Wire.available()<7){ Wire.end(); digitalWrite(RTC_VCC_PIN,LOW); return false; }
  uint8_t s=Wire.read(),m=Wire.read(),h=Wire.read(),dow=Wire.read(),d=Wire.read(),mo=Wire.read(),y=Wire.read();
  Wire.end(); digitalWrite(RTC_VCC_PIN,LOW); (void)dow;
  if(s==0xFF&&m==0xFF&&h==0xFF) return false;
  g_rtc_sec=bcd2dec(s&0x7F); g_rtc_min=bcd2dec(m&0x7F); g_rtc_hour=bcd2dec(h&0x3F);
  g_rtc_day=bcd2dec(d&0x3F); g_rtc_month=bcd2dec(mo&0x1F); g_rtc_year=2000+bcd2dec(y);
  if(g_rtc_year<2024||g_rtc_year>2035||g_rtc_month<1||g_rtc_month>12||g_rtc_day<1||g_rtc_day>31||g_rtc_hour>23||g_rtc_min>59||g_rtc_sec>59) return false;
  return true;
}
static bool ds3231WriteUnix(uint32_t unixSec){
  uint32_t local=unixSec+8*3600, days=local/86400, rem=local%86400;
  int hh=rem/3600, mm=(rem%3600)/60, ss=rem%60, y=1970;
  while(true){ int diy=((y%4==0&&y%100!=0)||y%400==0)?366:365; if(days>=(uint32_t)diy){days-=diy;y++;}else break; }
  int mdays[]={31,28,31,30,31,30,31,31,30,31,30,31};
  if((y%4==0&&y%100!=0)||y%400==0) mdays[1]=29;
  int mo=0; while(days>=(uint32_t)mdays[mo]){days-=mdays[mo];mo++;}
  pinMode(RTC_VCC_PIN,OUTPUT); digitalWrite(RTC_VCC_PIN,HIGH); delay(15);
  Wire.begin(RTC_SDA_PIN,RTC_SCL_PIN);
  Wire.beginTransmission(DS3231_ADDR); Wire.write(0x00);
  Wire.write(dec2bcd(ss)); Wire.write(dec2bcd(mm)); Wire.write(dec2bcd(hh));
  Wire.write(dec2bcd(1)); Wire.write(dec2bcd(days+1)); Wire.write(dec2bcd(mo+1)); Wire.write(dec2bcd(y-2000));
  bool ok=(Wire.endTransmission()==0); Wire.end(); digitalWrite(RTC_VCC_PIN,LOW); return ok;
}
static String ds3231ToString(){ char b[24]; snprintf(b,sizeof(b),"%04d-%02d-%02d %02d:%02d:%02d",g_rtc_year,g_rtc_month,g_rtc_day,g_rtc_hour,g_rtc_min,g_rtc_sec); return String(b); }

static uint32_t lastSeq=0; static bool haveLast=false;
class AdvertisedCallback: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) override {
    if(!dev.haveManufacturerData()) return;
    std::string md=dev.getManufacturerData();
    if(md.size()<8) return;
    uint16_t cid=(uint8_t)md[0]|((uint8_t)md[1]<<8);
    if(cid!=COMPANY_ID) return;
    uint16_t seq=(uint8_t)md[2]|((uint8_t)md[3]<<8);
    int16_t t100=(int16_t)((uint8_t)md[4]|((uint8_t)md[5]<<8));
    uint16_t bmv=(uint8_t)md[6]|((uint8_t)md[7]<<8);
    if(haveLast && seq==lastSeq) return; // 同一条广播被扫到多次, 去重
    String gap="";
    if(haveLast && seq!=lastSeq+1) gap=String(" GAP_")+String(seq-lastSeq-1)+String("_missed");
    lastSeq=seq; haveLast=true;
    bool okR=ds3231Read();
    String ts=okR?ds3231ToString():"NO_RTC_TIME";
    bool existed=LittleFS.exists(LOG_PATH);
    File f=LittleFS.open(LOG_PATH,"a");
    if(f){ if(!existed) f.println("timestamp,seq,temp_C,batt_V,rssi,gap");
      char line[128]; snprintf(line,sizeof(line),"%s,%u,%.2f,%.2f,%d%s",ts.c_str(),seq,t100/100.0, bmv/1000.0, dev.getRSSI(), gap.c_str());
      f.println(line); f.close(); Serial.print("收: "); Serial.println(line);
    }
  }
};
void setup(){
  Serial.begin(115200); delay(300);
  Serial.println("\n[NestTemp Base A] 持续扫描中, 须 USB 常供电, 不要深睡");
  if(!LittleFS.begin(true)) Serial.println("LittleFS 挂载失败!");
  Serial.println(ds3231Read()?("RTC: "+ds3231ToString()):"RTC: NO_RTC_TIME, 先串口发 T<unix> 对时");
  BLEDevice::init("NestTemp-Base");
  BLEScan *scan=BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new AdvertisedCallback());
  scan->setActiveScan(true); scan->setInterval(100); scan->setWindow(99);
  scan->start(0, nullptr, false); // 0=一直扫
  Serial.println("扫描已启动. 串口命令: DUMP 打印日志 / T<unix> 对时 / CLEAR 清空");
}
void loop(){
  if(Serial.available()){
    String s=Serial.readStringUntil('\n'); s.trim();
    if(s=="DUMP"){ File f=LittleFS.open(LOG_PATH,"r"); if(!f){Serial.println("无日志");} else { Serial.println("--- BEGIN egg_log.csv ---"); while(f.available()) Serial.write(f.read()); Serial.println("--- END ---"); f.close(); } }
    else if(s.startsWith("T")){ Serial.println(ds3231WriteUnix(s.substring(1).toInt())?"RTC 对时成功":"RTC 对时失败"); }
    else if(s=="CLEAR"){ LittleFS.remove(LOG_PATH); Serial.println("日志已清空"); }
  }
  delay(200);
}

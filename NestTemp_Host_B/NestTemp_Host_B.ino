// NestTemp_Host_B -- 推荐方案 B: 有线探头 + 树干主机
// 目标: 完整记录一个孵化期 (12-21 天) 的巢温, 每 60 秒一笔, 电池供电, 超低功耗
// 硬件: ESP32-C3 SuperMini (默认) 或经典 ESP32, DS18B20 防水探头, DS3231 RTC(带 CR2032)
// 数据: 存 LittleFS /nest_log.csv, 取回时 USB 接电脑, 串口发 DUMP 打印全部数据
// 时间戳: 只认 DS3231 寄存器读出的合法时间; 读不到/不合法就明写 NO_RTC_TIME,
//        绝不用深睡内部 RTC 凑出来的时间冒充 (内部 RC 深睡 1 小时能快 30 秒以上,
//        10 小时能快半小时, 孵化期全程会漂几个小时, 不能用)
// 功耗: 深睡约 13uA, 每 60 秒醒 0.35 秒约 62mA, 平均约 0.37mA
//        18650 3400mAh 理论 380 天, 打 5 折 (低温+自放电+余量) 仍 >150 天, 远超一个孵化期
//
// 接线 (ESP32-C3 SuperMini, 默认):
//   DS18B20 红(VCC) -> 3V3, 黑(GND) -> GND, 黄(DATA) -> GPIO4, DATA 与 3V3 间接 4.7k 上拉
//   DS3231 模块 VCC -> GPIO10 (程序开关电源, 睡眠时断电, 时间靠模块上 CR2032 维持)
//            GND -> GND, SDA -> GPIO8, SCL -> GPIO9
//            注意: ZS-042 类模块自带充电电路会给 CR2032 充电, 必须拆掉充电电阻
//            或改用无充电版 / LIR2032, 否则有鼓包风险. 上电前万用表先确认.
//   电池 18650+ -> 主机 VIN/5V (经保护板), 电池电压采样: 电池+ 经 100k+100k 分压到 GPIO3
//            分压比 2.0, 上电前先用万用表量 GPIO3 脚电压, 与电池电压/2 对照, 对不上先查线,
//            不要在代码里调系数去凑数 (浮空脚会读出几百 mV 的假数)
//   按钮 (可选): GPIO5 接按钮到 GND, 上电/唤醒时按住 = 进入 120 秒 BLE 广播 (手机 nRF Connect 看实时温度)
//            不按 = 直接测温存盘睡回去
// 经典 ESP32 改引脚: ONE_WIRE 4, BAT_ADC 34, RTC_VCC 33, SDA 21, SCL 22, BUTTON 0(慎用, BOOT 脚)
//
// 首次使用:
//   1. 先在桌上通电, 串口 115200 看启动日志, 确认 RTC 时间合法、温度合理、电池电压与万用表一致
//   2. 对时: 串口发 T<unix秒> 回车, 如 T1792345600; 或等 BLE 对时 (见 README)
//      对时后必须看到下一行日志时间正确才算对上, 光显示 "已同步" 不算
//   3. 探头放冰水 (0.0 度) 和温水里各测一次, 误差应在 +-0.5 度内
//   4. 再装到树干上. 孵化中途不要拆主机, 取数等雏鸟离巢后 USB 接电脑发 DUMP

#include <Arduino.h>
#include <Wire.h>
#include <LittleFS.h>
#include <esp_sleep.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// ---------------- 配置: 按需只改这里 ----------------
#define SAMPLE_INTERVAL_SEC 60        // 采样间隔(秒): 60 秒能抓住亲鸟离巢/回巢的每次波动
#define ONE_WIRE_PIN       4         // DS18B20 DATA
#define BAT_ADC_PIN        3         // 电池分压采样 (C3: GPIO3=ADC1_CH3)
#define BAT_DIVIDER_RATIO  2.0f      // 100k + 100k 分压, 电池电压 = 脚电压 x 2.0
#define RTC_VCC_PIN        10        // 给 DS3231 供电的 GPIO, 睡眠时拉低断电
#define RTC_SDA_PIN        8
#define RTC_SCL_PIN        9
#define BUTTON_PIN         5         // 按住上电 = BLE 广播 120 秒 (低电平有效)
#define BLE_MODE_SECONDS   120
#define LOG_PATH           "/nest_log.csv"
#define DS3231_ADDR        0x68
// ------------------------------------------------------

RTC_DATA_ATTR uint32_t bootCount = 0;
RTC_DATA_ATTR uint32_t seqCount = 0;

static uint8_t bcd2dec(uint8_t v) { return (v >> 4) * 10 + (v & 0x0F); }
static uint8_t dec2bcd(uint8_t v) { return ((v / 10) << 4) | (v % 10); }

// ---------- DS18B20: 自写 OneWire, 不依赖外部库 ----------
static void owWriteBit(int v) {
  pinMode(ONE_WIRE_PIN, OUTPUT);
  digitalWrite(ONE_WIRE_PIN, LOW);
  if (v) { delayMicroseconds(6); pinMode(ONE_WIRE_PIN, INPUT_PULLUP); delayMicroseconds(64); }
  else   { delayMicroseconds(65); pinMode(ONE_WIRE_PIN, INPUT_PULLUP); delayMicroseconds(5); }
}
static int owReadBit() {
  pinMode(ONE_WIRE_PIN, OUTPUT);
  digitalWrite(ONE_WIRE_PIN, LOW);
  delayMicroseconds(6);
  pinMode(ONE_WIRE_PIN, INPUT_PULLUP);
  delayMicroseconds(9);
  int v = digitalRead(ONE_WIRE_PIN);
  delayMicroseconds(55);
  return v;
}
static bool owReset() {
  pinMode(ONE_WIRE_PIN, OUTPUT);
  digitalWrite(ONE_WIRE_PIN, LOW);
  delayMicroseconds(480);
  pinMode(ONE_WIRE_PIN, INPUT_PULLUP);
  delayMicroseconds(70);
  int presence = digitalRead(ONE_WIRE_PIN);
  delayMicroseconds(410);
  return presence == 0;
}
static void owWriteByte(uint8_t b) { for (int i = 0; i < 8; i++) owWriteBit((b >> i) & 1); }
static uint8_t owReadByte() { uint8_t b = 0; for (int i = 0; i < 8; i++) if (owReadBit()) b |= (1 << i); return b; }

// 读温度. resolution 10-bit (0.25 度, 转换 187ms, 省电又够用). 成功返回 true.
static bool ds18b20Read(float *tempC) {
  pinMode(ONE_WIRE_PIN, INPUT_PULLUP);
  if (!owReset()) return false;
  owWriteByte(0xCC); owWriteByte(0x4E); owWriteByte(0x00); owWriteByte(0x00); owWriteByte(0x3F); // 10-bit
  if (!owReset()) return false;
  owWriteByte(0xCC); owWriteByte(0x44);
  pinMode(ONE_WIRE_PIN, INPUT_PULLUP);
  delay(200); // 10-bit 转换 187ms, 留余量
  if (!owReset()) return false;
  owWriteByte(0xCC); owWriteByte(0xBE);
  uint8_t d[9]; for (int i = 0; i < 9; i++) d[i] = owReadByte();
  if (d[8] != 0 && d[0] == 0xFF && d[1] == 0xFF) return false; // 未接 / 断线
  // CRC8 校验 (多项式 x^8+x^5+x^4+1)
  uint8_t crc = 0; for (int i = 0; i < 8; i++) { crc ^= d[i]; for (int j = 0; j < 8; j++) crc = (crc & 1) ? (crc >> 1) ^ 0x8C : (crc >> 1); }
  if (crc != d[8]) return false;
  int16_t raw = (int16_t)((d[1] << 8) | d[0]);
  *tempC = raw / 16.0f;
  if (*tempC < -20 || *tempC > 60) return false;
  return true;
}

// ---------- DS3231: 只认寄存器读出的合法时间 ----------
// Arduino 会在文件头自动生成函数原型, 自定义 struct 作参数会与原型冲突,
// 所以 RTC 时间用全局变量传, 不用 struct 作参数.
int g_rtc_year=0, g_rtc_month=0, g_rtc_day=0, g_rtc_hour=0, g_rtc_min=0, g_rtc_sec=0;
static bool ds3231Read() {

  pinMode(RTC_VCC_PIN, OUTPUT); digitalWrite(RTC_VCC_PIN, HIGH); delay(15);
  Wire.begin(RTC_SDA_PIN, RTC_SCL_PIN);
  Wire.beginTransmission(DS3231_ADDR); Wire.write(0x00);
  if (Wire.endTransmission() != 0) { Wire.end(); digitalWrite(RTC_VCC_PIN, LOW); return false; }
  Wire.requestFrom(DS3231_ADDR, 7);
  if (Wire.available() < 7) { Wire.end(); digitalWrite(RTC_VCC_PIN, LOW); return false; }
  uint8_t s = Wire.read(), m = Wire.read(), h = Wire.read(), dow = Wire.read();
  uint8_t d = Wire.read(), mo = Wire.read(), y = Wire.read();
  Wire.end(); digitalWrite(RTC_VCC_PIN, LOW);
  (void)dow;
  // 全 1 / 全 0 = 没接好或没电池, 即使 I2C 应答也不能算有 RTC
  if (s == 0xFF && m == 0xFF && h == 0xFF) return false;
  g_rtc_sec = bcd2dec(s & 0x7F); g_rtc_min = bcd2dec(m & 0x7F); g_rtc_hour = bcd2dec(h & 0x3F);
  g_rtc_day = bcd2dec(d & 0x3F); g_rtc_month = bcd2dec(mo & 0x1F); g_rtc_year = 2000 + bcd2dec(y);
  if (g_rtc_year < 2024 || g_rtc_year > 2035 || g_rtc_month < 1 || g_rtc_month > 12 ||
      g_rtc_day < 1 || g_rtc_day > 31 || g_rtc_hour > 23 || g_rtc_min > 59 || g_rtc_sec > 59) return false;
  return true;
}
static bool ds3231WriteUnix(uint32_t unixSec) {
  // unix -> 年月日时分秒 (不依赖 time.h 时区, 直接算 UTC+8 的本地时间写进 RTC)
  uint32_t local = unixSec + 8 * 3600;
  uint32_t days = local / 86400; uint32_t rem = local % 86400;
  int hh = rem / 3600, mm = (rem % 3600) / 60, ss = rem % 60;
  int y = 1970; while (true) { int diy = ((y % 4 == 0 && y % 100 != 0) || y % 400 == 0) ? 366 : 365; if (days >= (uint32_t)diy) { days -= diy; y++; } else break; }
  int mdays[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if ((y % 4 == 0 && y % 100 != 0) || y % 400 == 0) mdays[1] = 29;
  int mo = 0; while (days >= (uint32_t)mdays[mo]) { days -= mdays[mo]; mo++; }
  pinMode(RTC_VCC_PIN, OUTPUT); digitalWrite(RTC_VCC_PIN, HIGH); delay(15);
  Wire.begin(RTC_SDA_PIN, RTC_SCL_PIN);
  Wire.beginTransmission(DS3231_ADDR); Wire.write(0x00);
  Wire.write(dec2bcd(ss)); Wire.write(dec2bcd(mm)); Wire.write(dec2bcd(hh));
  Wire.write(dec2bcd(1)); Wire.write(dec2bcd(days + 1)); Wire.write(dec2bcd(mo + 1)); Wire.write(dec2bcd(y - 2000));
  bool ok = (Wire.endTransmission() == 0);
  Wire.end(); digitalWrite(RTC_VCC_PIN, LOW); return ok;
}
static String ds3231ToString() {
  char buf[24]; snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d", g_rtc_year, g_rtc_month, g_rtc_day, g_rtc_hour, g_rtc_min, g_rtc_sec);
  return String(buf);
}

static float readBatteryV() {
  analogSetAttenuation(ADC_11db);
  uint32_t mv = 0; for (int i = 0; i < 8; i++) { mv += analogReadMilliVolts(BAT_ADC_PIN); delay(5); }
  mv /= 8;
  return (mv / 1000.0f) * BAT_DIVIDER_RATIO;
}

static void bleLiveBroadcast(float tempC, uint32_t seq) {
  // 只广播实时温度 120 秒, 供手机 nRF Connect 查看, 不常开. 广播完就睡.
  BLEDevice::init("NestTemp-Host");
  BLEServer *srv = BLEDevice::createServer();
  BLEService *svc = srv->createService("181A"); // Environmental Sensing
  BLECharacteristic *ch = svc->createCharacteristic("2A6E", BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  int16_t t100 = (int16_t)(tempC * 100);
  ch->setValue((uint8_t*)&t100, 2);
  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  BLEAdvertisementData ad; String name = "NestTemp " + String(tempC, 1) + "C #" + String(seq);
  ad.setName(name.c_str()); adv->setAdvertisementData(ad); adv->start();
  Serial.printf("BLE 广播中 %d 秒: %s, 串口发 T<unix> 可对时\n", BLE_MODE_SECONDS, name.c_str());
  unsigned long t0 = millis();
  while (millis() - t0 < BLE_MODE_SECONDS * 1000UL) {
    if (Serial.available()) { String s = Serial.readStringUntil('\n'); s.trim(); if (s.startsWith("T")) { if (ds3231WriteUnix(s.substring(1).toInt())) Serial.println("RTC 已对时, 下次唤醒验证"); } }
    delay(200);
  }
  adv->stop(); BLEDevice::deinit();
}

static void handleSerialCommands() {
  // 上电后 2 秒内可发命令: DUMP 打印全部日志, T<unix> 对时, CLEAR 清空日志
  unsigned long t0 = millis();
  while (millis() - t0 < 2000) {
    if (!Serial.available()) { delay(50); continue; }
    String s = Serial.readStringUntil('\n'); s.trim();
    if (s == "DUMP") {
      File f = LittleFS.open(LOG_PATH, "r");
      if (!f) { Serial.println("无日志文件"); continue; }
      Serial.println("--- BEGIN nest_log.csv ---");
      while (f.available()) Serial.write(f.read());
      Serial.println("--- END ---"); f.close();
    } else if (s.startsWith("T")) {
      Serial.println(ds3231WriteUnix(s.substring(1).toInt()) ? "RTC 对时写入成功" : "RTC 对时写入失败");
      if (ds3231Read()) Serial.println("验证读回: " + ds3231ToString());
    } else if (s == "CLEAR") {
      LittleFS.remove(LOG_PATH); Serial.println("日志已清空");
    }
  }
}

void setup() {
  Serial.begin(115200); delay(300);
  bootCount++; seqCount++;
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.printf("\n[NestTemp Host B] boot=%lu seq=%lu\n", (unsigned long)bootCount, (unsigned long)seqCount);
  handleSerialCommands();
  if (!LittleFS.begin(true)) Serial.println("LittleFS 挂载失败!");
  float tempC = 0; bool okTemp = ds18b20Read(&tempC);
  float battV = readBatteryV();
  bool okRtc = ds3231Read();
  Serial.printf("温度: %s %.2f C | 电池: %.2f V | RTC: %s\n",
    okTemp ? "OK" : "FAIL", tempC, battV, okRtc ? ds3231ToString().c_str() : "NO_RTC_TIME(未对时/无RTC)");
  // 写盘: 时间戳只写 RTC 读到的合法值, 没 RTC 就写 NO_RTC_TIME + seq, 绝不编一个时间
  // 表头只在文件新建时写一次: 先查 exists, 再 open(a) 追加
  bool existed = LittleFS.exists(LOG_PATH);
  File f = LittleFS.open(LOG_PATH, "a");
  if (f) {
    if (!existed) f.println("timestamp,seq,temp_C,batt_V,boot");
    char line[96];
    if (okRtc) snprintf(line, sizeof(line), "%s,%lu,%.2f,%.2f,%lu", ds3231ToString().c_str(), (unsigned long)seqCount, okTemp ? tempC : -999, battV, (unsigned long)bootCount);
    else snprintf(line, sizeof(line), "NO_RTC_TIME,%lu,%.2f,%.2f,%lu", (unsigned long)seqCount, okTemp ? tempC : -999, battV, (unsigned long)bootCount);
    f.println(line); f.close();
    Serial.print("已存盘: "); Serial.println(line);
  } else Serial.println("写盘失败!");
  if (digitalRead(BUTTON_PIN) == LOW) { if (okTemp) bleLiveBroadcast(tempC, seqCount); }
  Serial.printf("进入深睡 %d 秒...\n", SAMPLE_INTERVAL_SEC);
  Serial.flush(); delay(100);
  esp_sleep_enable_timer_wakeup((uint64_t)SAMPLE_INTERVAL_SEC * 1000000ULL);
  esp_deep_sleep_start();
}
void loop() {}

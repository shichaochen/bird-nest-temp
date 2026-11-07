// NestTemp_EggNode_A -- 备选方案 A: 伪蛋节点 (巢内)
// 伪蛋里只放: ESP32-C3 SuperMini + 裸片 DS18B20 + 小锂电, 不放 RTC、不放 SD
// 逻辑: 每 300 秒醒一次 -> 测温 -> 存 RTC 内存环形缓冲 (掉电丢, 但深睡不丢) ->
//       BLE 广播 3 秒 (seq, 温度, 电量) -> 深睡. 时间戳由基站收到时打 (基站有 DS3231).
// 尺寸红线: 整机必须塞进目标鸟种的蛋尺寸. 麻雀蛋约 20x15mm 塞不下本方案,
//           喜鹊/乌鸫/斑鸠蛋 30mm 以上才考虑. 蛋尺寸没定之前不要先打蛋壳.
// 电池: 502030 500mAh 锂电 (30x20x5mm) + 保护板, 5 分钟间隔平均约 0.17mA,
//       理论 120 天, 打 5 折仍约 60 天. 2 分钟间隔约 0.43mA, 约 25 天 (打折后).
//       锂电在巢内有被啄破风险, 蛋壳必须全密封、电池用环氧包住, 破损立即取出.
// 注意: 本节点不存长期数据, 基站必须在附近持续监听. 基站断电/超出距离期间的数据
//       只留在节点 RTC 缓冲 (最近 48 条), 超出即丢. 所以 A 方案不能保证"完整孵化期"
//       不丢数, 只能保证"基站在场时"不丢数. 这是 A 相对 B 最大的弱点, 见 README 对比表.

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <esp_sleep.h>

// ---------------- 配置 ----------------
#define SAMPLE_INTERVAL_SEC 300       // 300 秒=5 分钟. 想更密改 120, 续航减半; 想更省改 600
#define ONE_WIRE_PIN       4         // 裸片 DS18B20 DATA (蛋尖内侧, 涂导热硅脂后环氧封住)
#define BAT_ADC_PIN        3
#define BAT_DIVIDER_RATIO  2.0f
#define ADV_SECONDS        3         // 每次广播时长(秒)
#define COMPANY_ID         0xFFFF    // 测试用厂商 ID, 自家基站只认这个 ID
// ----------------------------------------

RTC_DATA_ATTR uint32_t seqCount = 0;
RTC_DATA_ATTR uint16_t ringTemp[48];  // 最近 48 条温度 x100, 循环覆盖
RTC_DATA_ATTR uint8_t ringHead = 0;

static void owWriteBit(int v) {
  pinMode(ONE_WIRE_PIN, OUTPUT); digitalWrite(ONE_WIRE_PIN, LOW);
  if (v) { delayMicroseconds(6); pinMode(ONE_WIRE_PIN, INPUT_PULLUP); delayMicroseconds(64); }
  else   { delayMicroseconds(65); pinMode(ONE_WIRE_PIN, INPUT_PULLUP); delayMicroseconds(5); }
}
static int owReadBit() {
  pinMode(ONE_WIRE_PIN, OUTPUT); digitalWrite(ONE_WIRE_PIN, LOW); delayMicroseconds(6);
  pinMode(ONE_WIRE_PIN, INPUT_PULLUP); delayMicroseconds(9);
  int v = digitalRead(ONE_WIRE_PIN); delayMicroseconds(55); return v;
}
static bool owReset() {
  pinMode(ONE_WIRE_PIN, OUTPUT); digitalWrite(ONE_WIRE_PIN, LOW); delayMicroseconds(480);
  pinMode(ONE_WIRE_PIN, INPUT_PULLUP); delayMicroseconds(70);
  int p = digitalRead(ONE_WIRE_PIN); delayMicroseconds(410); return p == 0;
}
static void owWriteByte(uint8_t b) { for (int i=0;i<8;i++) owWriteBit((b>>i)&1); }
static uint8_t owReadByte() { uint8_t b=0; for (int i=0;i<8;i++) if (owReadBit()) b|=(1<<i); return b; }
static bool ds18b20Read(float *t) {
  pinMode(ONE_WIRE_PIN, INPUT_PULLUP);
  if (!owReset()) return false;
  owWriteByte(0xCC); owWriteByte(0x4E); owWriteByte(0x00); owWriteByte(0x00); owWriteByte(0x3F);
  if (!owReset()) return false;
  owWriteByte(0xCC); owWriteByte(0x44); delay(200);
  if (!owReset()) return false;
  owWriteByte(0xCC); owWriteByte(0xBE);
  uint8_t d[9]; for (int i=0;i<9;i++) d[i]=owReadByte();
  uint8_t crc=0; for (int i=0;i<8;i++){ crc^=d[i]; for(int j=0;j<8;j++) crc=(crc&1)?(crc>>1)^0x8C:(crc>>1); }
  if (crc!=d[8]) return false;
  int16_t raw=(int16_t)((d[1]<<8)|d[0]); *t=raw/16.0f;
  return (*t>-20 && *t<60);
}

void setup() {
  Serial.begin(115200); delay(200);
  seqCount++;
  float tempC=0; bool ok=ds18b20Read(&tempC);
  analogSetAttenuation(ADC_11db);
  uint32_t mv=0; for(int i=0;i<8;i++){ mv+=analogReadMilliVolts(BAT_ADC_PIN); delay(5);} mv/=8;
  float battV=(mv/1000.0f)*BAT_DIVIDER_RATIO;
  if (ok) { ringTemp[ringHead]=(uint16_t)(tempC*100); ringHead=(ringHead+1)%48; }
  Serial.printf("[Egg A] seq=%lu temp=%s %.2f C batt=%.2f V\n",(unsigned long)seqCount, ok?"OK":"FAIL", tempC, battV);
  // BLE 广播: 厂商数据 = seq(2B) + temp_x100(2B, 有符号) + batt_mV(2B)
  BLEDevice::init("NestEgg");
  BLEAdvertising *adv=BLEDevice::getAdvertising();
  BLEAdvertisementData ad;
  uint8_t payload[8];
  payload[0]=COMPANY_ID & 0xFF; payload[1]=(COMPANY_ID>>8)&0xFF;
  payload[2]=seqCount & 0xFF; payload[3]=(seqCount>>8)&0xFF;
  int16_t t100=(int16_t)(tempC*100); payload[4]=t100 & 0xFF; payload[5]=(t100>>8)&0xFF;
  uint16_t bmv=(uint16_t)(battV*1000); payload[6]=bmv & 0xFF; payload[7]=(bmv>>8)&0xFF;
  ad.setManufacturerData(std::string((char*)payload, 8));
  ad.setName("NestEgg");
  adv->setAdvertisementData(ad);
  adv->start();
  delay(ADV_SECONDS*1000UL);
  adv->stop(); BLEDevice::deinit();
  Serial.printf("广播 %d 秒结束, 深睡 %d 秒\n", ADV_SECONDS, SAMPLE_INTERVAL_SEC);
  Serial.flush(); delay(100);
  esp_sleep_enable_timer_wakeup((uint64_t)SAMPLE_INTERVAL_SEC*1000000ULL);
  esp_deep_sleep_start();
}
void loop() {}

/************************************************
Connection:    UNO         TOFxxxH
             5V/3.3V   ---  VIN  
               GND     ---  GND 
                2      ---  RXD 
                3      ---  TXD  
************************************************/
#include <MsTimer2.h>
#include <SoftwareSerial.h>
#include <Wire.h>

SoftwareSerial DT(3, 2); //TX--3  RX--2
/**********************************type类型定义**************************/
typedef signed char     int8;
typedef unsigned char   uint8;
typedef signed short    int16;
typedef unsigned short    uint16;
typedef signed long     int32;
typedef unsigned long   uint32;
typedef unsigned long long  uint64;
typedef signed long long  int64;
/**********************************time定时配置**************************/
uint32 timCount = 0;
void tim_isr(void)
{
  timCount++;
  timCount++;
}
uint32 tim_get_count(void)
{
  uint32 count;;
  do{
    count = timCount;
  }while(count != timCount);
  return count;
}
uint8 tim_check_timeout(uint32 start, uint32 now, uint32 invt)
{
  if((uint32)(start + invt) >= start)//未溢出
  {
    if((now >= (uint32)(start + invt)) || (now < start))
    {
      return 1;
    }
  }
  else//溢出
  {
    if((now < start) && (now >= (uint32)(start + invt)))
    {
      return 1;
    }
  }
  return 0;
}
/**********************************modbus set 配置**************************/
//modbus接收到正确数据帧回调函数
void modbus_recv_data(uint8 *p_data, uint16 len)
{
/*读命令响应  
  deviceAddr = p_data[0];//响应的设备地址
  cmd = p_data[1];      //响应的命令,读03
  byteNum = p_data[2];  //返回读数据长
  readData = p_data[3]; //读回数据H
  readData <<= 8;
  readData |= p_data[4];//读回数据H
*/
/*写命令响应  
  deviceAddr = p_data[0];//响应的设备地址
  cmd = p_data[1];      //响应的命令,写06
  regAddr = p_data[2];  //寄存器地址H
  regAddr <<= 8;
  regAddr |= p_data[3]; //寄存器地址L
  writeData = p_data[4];//写数据H
  writeData <<= 8;
  writeData |= p_data[5];//写数据L
*/
  //Serial.write(p_data, len);
}
uint8 rxRdIndex = 0;
uint8 rxWrIndex = 0;
uint32 lastRxTimestamp = 0;
uint8 framTimeInvt = 4;
#define RX_BUF_LEN  80
#define TX_BUF_LEN  80
uint8 a_rxBuff[RX_BUF_LEN];
uint8 a_tempBuff[RX_BUF_LEN];
uint8 a_txBuff[TX_BUF_LEN];
static const uint8 a_crc16mTableHi[] = {
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
  0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
  0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
  0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
  0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
  0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
  0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
  0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
  0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
  0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
  0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
  0x40
};
static const uint8 a_crc16mTableLo[] = {
  0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
  0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
  0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
  0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
  0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
  0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
  0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
  0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
  0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
  0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
  0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
  0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
  0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
  0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
  0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
  0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
  0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
  0x40
};
uint16 crc16_calculate_modbus(uint8 *p_data, uint16 len)
{
  uint8 crchi = 0xff;
  uint8 crclo = 0xff;
  uint16 index;
  while (len--)
  {
    index = crclo ^ *p_data++;
    crclo = crchi ^ a_crc16mTableHi[index];
    crchi = a_crc16mTableLo[index];
  }
  return (crchi << 8 | crclo);
}
uint16 uart_read_isr(uint8 channel, uint8 *p_buff)
{
  uint16 i = 0, wrIndex = 0;
  for(wrIndex=rxWrIndex; wrIndex!=rxRdIndex; rxRdIndex=(rxRdIndex+1)%RX_BUF_LEN,i++)
  {
    p_buff[i] = a_rxBuff[rxRdIndex];
  }
  return i;
}
uint8 uart_tpm_rx_isr(uint8 channel, uint8 data)
{
  if(rxRdIndex == ((rxWrIndex + 1) % RX_BUF_LEN))
  {
    return 1;
  }
  else
  {
    a_rxBuff[rxWrIndex] = data;
    rxWrIndex = (rxWrIndex+1)%RX_BUF_LEN;
  }
  lastRxTimestamp = tim_get_count();
  return 0;
}
void uart_tpm_main(void)
{
  uint16 rxLen = 0;
  uint16 crc16 = 0;
  if(tim_check_timeout(lastRxTimestamp, tim_get_count(), framTimeInvt))
  {
    rxLen = uart_read_isr(0, a_tempBuff);
    if(rxLen > 2)
    {
      crc16 = a_tempBuff[rxLen-1];
      crc16 <<= 8;
      crc16 |= a_tempBuff[rxLen-2];
      if(crc16 == crc16_calculate_modbus(a_tempBuff, rxLen-2))
      {
          modbus_recv_data(a_tempBuff, rxLen-2);
      }
    }
    lastRxTimestamp = tim_get_count();
  }
}
uint8 uart_tpm_tx_data(uint8 channel, uint8 *p_data, uint16 len)
{
  uint16 crc16 = 0;
  if(NULL == p_data || 0 == len ||
    len + 2 > TX_BUF_LEN)
  {
    return 1;
  }
  memcpy(a_txBuff, p_data, len);
  crc16 = crc16_calculate_modbus(a_txBuff, len);
  a_txBuff[len] = crc16;    //crc16l
  a_txBuff[len+1] = crc16>>8;   //crc16h
  //Serial.write(a_txBuff, len+2);
  DT.write(a_txBuff, len+2);
  return 1;
}

/**********************************test set 测试配置**************************/
//led测试
void led_flash_main(void)
{
  static uint32 ledCount = 0;
  if(tim_check_timeout(ledCount, tim_get_count(), 500))
  {
    digitalWrite(13, !digitalRead(13));
    ledCount = tim_get_count();
  }  
}
uint8 a_testBuff[20];

//读测量距离命令***********************read distance reg****************************************************************
#define REG_RD_ADDR  0x0010//测量结果寄存器
void read_cmd_test_main(void)
{
  static uint32 timStamp = 0;
  if(tim_check_timeout(timStamp, tim_get_count(), 500))//500ms发送一次读命令，改变尺寸可以定义多久读一次从机的测距值
  {
    a_testBuff[0] = 0;//广播地址
    a_testBuff[1] = 3;//读命令码
    a_testBuff[2] = REG_RD_ADDR>>8;//regH
    a_testBuff[3] = REG_RD_ADDR;//regL
    a_testBuff[4] = 0;//numH 仅支持读单个寄存器
    a_testBuff[5] = 1;//numL
    uart_tpm_tx_data(0, a_testBuff, 6);
    timStamp = tim_get_count();
  }  
}

//写设置模块的寄存器********************************write reg*********************************************************************
#define REG_WR_ADDR  0x0005//连续输出控制寄存器
#define REG_WR_DATA  500//写入数据，0表示不连续输出，xx表示间隔xxmS从机自动输出测距值
void write_cmd_test_main(void)
{
  static uint32 timStamp = 0;
  static uint8 flag = 1;
  if(tim_check_timeout(timStamp, tim_get_count(), 500))//500ms后只发送一次写命令
  {
    if(flag)//只发送一次命令
    {
      flag = 0;
      a_testBuff[0] = 0;//广播地址
      a_testBuff[1] = 6;//写命令码
      a_testBuff[2] = REG_WR_ADDR>>8;//regH
      a_testBuff[3] = REG_WR_ADDR;//regL
      a_testBuff[4] = REG_WR_DATA>>8;//dataH
      a_testBuff[5] = REG_WR_DATA;//dataL
      uart_tpm_tx_data(0, a_testBuff, 6);
    }
    timStamp = tim_get_count();
  }  
}

//测试loop，测试写功能或者读功能**************************test loop******************************************************************
void cmd_test_main(void)
{
  //write_cmd_test_main();//发送写命令，设置测距模块是否连续输出以及连续输出的时间间隔

  read_cmd_test_main();//发送读命令，读模块的测距值
}

// write baud rate *non-functional*
#define reg_wr_addr 0x0003
#define reg_write_data 2

void writeBaudRate(void) {
  a_testBuff[0] = 0;
  a_testBuff[1] = 6;
  a_testBuff[2] = reg_wr_addr>>8;
  a_testBuff[3] = reg_wr_addr;
  a_testBuff[4] = 0x0002>>8;
  a_testBuff[5] = 0x0002;

  uart_tpm_tx_data(0, a_testBuff, 6);
}

// restart module *non-functional*

#define reg_wr_addr 0x0001
#define reg_wr_data 0x1000

void restartModule(void) {
  a_testBuff[0] = 0;
  a_testBuff[1] = 6;
  a_testBuff[2] = reg_wr_addr>>8;
  a_testBuff[3] = reg_wr_addr;
  a_testBuff[4] = reg_wr_data>>8;
  a_testBuff[5] = reg_wr_data;

  uart_tpm_tx_data(0, a_testBuff, 6);
}



void setup() {
  // put your setup code here, to run once:

  pinMode(13, OUTPUT);
  Serial.begin(115200);
  DT.begin(115200);
  MsTimer2::set(2 ,tim_isr);
  MsTimer2::start();
}

void loop() {
  // put your main code here, to run repeatedly:
  uart_tpm_main();
  led_flash_main();
  cmd_test_main();

  if (DT.available() > 6)
  {
    char a = DT.read();
    if(a != 0x01)          //测距模块的首地址，若改变首地址这里要改对.默认01
      return;
    byte Buf[6];
    DT.readBytes(Buf, 6);
 /*   for (int i = 0; i < 6; i++)
    {
      if (Buf[i] < 0x10)
        Serial.print("0x0");
       else
        Serial.print("0x");
      Serial.print(Buf[i], HEX);
      Serial.print(" ");
    }*/
    // Serial.println(Buf[1]);
    // Serial.println(Buf[2]);
    // Serial.println(Buf[3]);
    // Serial.println(Buf[4]);
    // Serial.println(Buf[5]);
    // Serial.println(Buf[6]);
    Serial.println();

    if (Buf[2] == 0xFF)
    {
      Serial.print("Distance:");
      Serial.print("invalid");
    }

    else
    {
    long distance = Buf[2] * 256 + Buf[3];
    Serial.print("Distance:");
    Serial.print(distance);
    Serial.print("mm");
    }
  }

  
}

void serialEvent()
{
  //uart_tpm_rx_isr(0, (uint8)Serial.read());
  uart_tpm_rx_isr(0, (uint8)DT.read());
}
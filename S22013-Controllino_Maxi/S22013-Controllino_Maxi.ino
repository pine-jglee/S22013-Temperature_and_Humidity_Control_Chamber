#include <EEPROM.h>

#include <TimerThree.h>
#include <TimerOne.h>

#define BLDM1_break 22
#define BLDM1_dir 23
#define BLDM2_break 24
#define BLDM2_dir 25
#define CYLINDER_left 28
#define CYLINDER_right 29
#define PRESSURE_switch A0
#define ITV_feedback A2
#define PELTIER_left 2
#define PELTIER_right 3
#define ITV_pwm 5
#define MIXING_fan 30
#define DCPC_power 31

long passtime = 0, nowtime = 0, GENERATOR_passtime = 0, dcpc_resettime = 0;
long s_pass = 0, s_now = 0;

// Modbus RTU Library
#include <ModbusSlave.h>
#define SLAVEID 1
Modbus slave(SLAVEID, 13);
uint16_t RTUROM[60];

//PID Controll
float err_I[] = {0, 0, 0};
float err_oP[] = {0, 0, 0};

// D-CPC Communication
uint8_t OPC_Buffer[150];
uint8_t OPC_RQ[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x46, 0xC4, 0x38};

uint8_t CRC[2] ;
int opc_count = 0;
int cal_exit = 0;

void setup(void)
{

  EEPROM.get(8 * 2, RTUROM[8]);       // System Operating Time
  EEPROM_Initial(8, 60);


  EEPROM.get(16 * 2, RTUROM[16]);       // Set Chamber Humidity
  EEPROM_Initial(16, 5000);


  EEPROM.get(21 * 2, RTUROM[21]);       // Chamber Pv
  EEPROM_Initial(21, 1200);
  EEPROM.get(24 * 2, RTUROM[24]);       // Chamber Pv
  EEPROM_Initial(24, 250);

  EEPROM.get(26 * 2, RTUROM[26]);       // Chamber Pv
  EEPROM_Initial(26, 250);
  EEPROM.get(27 * 2, RTUROM[27]);       // Chamber Pv
  EEPROM_Initial(27, 63);

  EEPROM.get(31 * 2, RTUROM[31]);       // Chamber Pv
  EEPROM_Initial(31, 50);
  EEPROM.get(32 * 2, RTUROM[32]);       // Chamber Iv
  EEPROM_Initial(32, 5);
  EEPROM.get(33 * 2, RTUROM[33]);       // Chamber Dv
  EEPROM_Initial(33, 50);
  EEPROM.get(34 * 2, RTUROM[34]);       // Chamber Set Temp
  EEPROM_Initial(34, 3000);

  /*
    #define BLDM1_break = 22;
    #define BLDM1_dir = 23;
    #define BLDM2_break = 24;
    #define BLDM2_dir = 25;
    #define CYLINDER_left = 28;
    #define CYLINDER_right = 29;
    #define PRESSURE_switch = A0;
    #define ITV_feedback = A2;
    #define PELTIER_left = 2;
    #define PELTIER_right = 3;
    #define ITV_pwm = 5;
  */

  pinMode(BLDM1_break, OUTPUT);
  digitalWrite(BLDM1_break, LOW);//LOW - STOP

  pinMode(BLDM1_dir, OUTPUT);
  digitalWrite(BLDM1_dir, LOW);//LOW - FORWARD

  pinMode(BLDM2_break, OUTPUT);
  digitalWrite(BLDM2_break, LOW);//LOW - STOP

  pinMode(BLDM2_dir, OUTPUT);
  digitalWrite(BLDM2_dir, LOW);//LOW - STOP

  pinMode(CYLINDER_left, OUTPUT);
  digitalWrite(CYLINDER_left, LOW);//LOW - 릴리즈

  pinMode(CYLINDER_right, OUTPUT);
  digitalWrite(CYLINDER_right, LOW);//LOW - 릴리즈

  pinMode(PELTIER_left, OUTPUT);
  pinMode(PELTIER_right, OUTPUT);
  pinMode(ITV_pwm, OUTPUT);

  pinMode(MIXING_fan, OUTPUT);
  digitalWrite(MIXING_fan, LOW);

  pinMode(DCPC_power, OUTPUT);
  digitalWrite(DCPC_power, LOW);


  Timer3.initialize(12.5);  // 40 us = 25 kHz
  Timer1.initialize(40);  // 40 us = 25 kHz


  Timer3.pwm(PELTIER_left, 0);
  Timer3.pwm(PELTIER_right, 0);
  Timer3.pwm(ITV_pwm, 0);


  // HMI Baudrate (Modbus RTU)
  Serial.begin(115200);
  slave.cbVector[CB_READ_HOLDING_REGISTERS] = readMemory;
  slave.cbVector[CB_WRITE_HOLDING_REGISTERS] = writeMemory;
  slave.begin(115200);

  //D-CPC Baudrate
  Serial2.begin(115200);
  Serial2.setTimeout(1000);


}

void EEPROM_Initial(int address, int initial_value) {
  if (RTUROM[address] == 65535) {
    EEPROM.put(address * 2, initial_value);
    RTUROM[address] = initial_value;
  }
}
void EEPROM_write(int address, int value) {
  EEPROM.put(address * 2, value);
  RTUROM[address] = value;
}

void loop(void)
{
  if (nowtime - passtime > 500) {

    // Monitoring
    DCPC_PULL();
    SYSTEM_PRESSURE_CHECK();
    DCPC_RESET();

    // Operating
    MIXING_FAN();
    CHAMBER_TEMP();
    GENERATOR();
    SOLVALVE();
    passtime = millis();

  }

  DCPC_DATA();


  nowtime = millis();
  s_now = millis();
  slave.poll();

}


void DCPC_RESET() {

  if (nowtime - dcpc_resettime > 5000) {
    if (RTUROM[15] > 10000) {
      digitalWrite(DCPC_power, LOW);
      dcpc_resettime = millis();
    }
  }
}

void SOLVALVE() {
  if (RTUROM[7] == 1) {
    digitalWrite(CYLINDER_left, HIGH);
    digitalWrite(CYLINDER_right, HIGH);
  } else {
    digitalWrite(CYLINDER_left, LOW);
    digitalWrite(CYLINDER_right, LOW);
  }
}

void SYSTEM_PRESSURE_CHECK() {
  RTUROM[5] = analogRead(PRESSURE_switch);
}

void MIXING_FAN() {
  if (RTUROM[19] == 1) {
    digitalWrite(MIXING_fan, HIGH);
  } else {
    digitalWrite(MIXING_fan, LOW);
  }
}

void DCPC_PULL() {

  // Power On the DCPC Board
  digitalWrite(DCPC_power, HIGH);
  if (opc_count == 0)
  {
    for (uint8_t i = 0 ; i < sizeof(OPC_RQ) ; i ++)
    {
      Serial2.write(OPC_RQ[i]);
      s_pass = millis();
    }
  }
}

void DCPC_DATA() {

  if (s_now - s_pass > 100) {
    opc_count = 0;
    memset(OPC_Buffer, 0, sizeof(OPC_Buffer));
    Serial2.flush();
  }

  if (Serial2.available())
  {
    while (Serial2.available() > 0)
    {
      OPC_Buffer[opc_count] = Serial2.read();
      opc_count++;
      if (opc_count > 144)
      {
        CRC_Cal(OPC_Buffer, opc_count - 3, CRC);
        if (CRC[0] == OPC_Buffer[opc_count - 2] && CRC[1] == OPC_Buffer[opc_count - 1])
        {

          RTUROM[14] = OPC_Buffer[11 * 2 + 3] * 256 + OPC_Buffer[11 * 2 + 4];
          RTUROM[15] = OPC_Buffer[21 * 2 + 3] * 256 + OPC_Buffer[21 * 2 + 4];
          RTUROM[18] = OPC_Buffer[42 * 2 + 3] * 256 + OPC_Buffer[42 * 2 + 4];

          for (int i = 0 ; i < 2 ; i ++)
          {
            RTUROM[i + 36] = OPC_Buffer[(i + 22) * 2 + 3] * 256 + OPC_Buffer[(i + 22) * 2 + 4];
          }
        }
        opc_count = 0;
      }
    }

  }
}

void DCPC_datapull(uint16_t value, uint16_t index) {

  uint8_t MODBUS_pull[] = {0x01, 0x06, 0x00, 0x0D, 0x09, 0x60, 0x1E, 0x71};

  MODBUS_pull[2] = (index >> 8) & 0xFF;
  MODBUS_pull[3] = index & 0xFF;
  MODBUS_pull[4] = (value >> 8) & 0xFF;
  MODBUS_pull[5] = value & 0xFF;

  CRC_Cal(MODBUS_pull, 5, CRC);
  MODBUS_pull[6] = CRC[0];
  MODBUS_pull[7] = CRC[1];

  for (int i = 0 ; i < sizeof(MODBUS_pull) ; i ++)
  {
    Serial2.write(MODBUS_pull[i]);
  }


}

void CRC_Cal(uint8_t BF[], int count, uint8_t pCRC[]) {
  uint16_t C = 0xFFFF;
  uint8_t D;
  uint16_t CY, DD;

  for (int i = 0 ; i <= count ; i ++)
  {
    D = BF[i];
    C = C ^ D;
    for (int j = 1 ; j <= 8 ; j++)
    {
      CY = C & 1;
      C = C / 2 ;
      if (CY == 1) C = C ^ 40961;
    }
  }
  DD = C / 256;
  pCRC[1] = (byte) DD ;
  pCRC[0] = (byte) (C - DD * 256);
}

void GENERATOR() {
  float temp = analogRead(ITV_feedback);
  float temp1 = RTUROM[27];

  RTUROM[25] = temp1 / 1000 * (temp - 185);

  if (RTUROM[28] == 1) {

    if (GENERATOR_passtime == 0) {
      GENERATOR_passtime = millis();
      RTUROM[19] = 0;
    } else {
      RTUROM[22] = (nowtime - GENERATOR_passtime) / 1000;
    }


    if ((nowtime - GENERATOR_passtime) / 100 <= 100 ) {

      RTUROM[19] = 0;
      Timer3.pwm(ITV_pwm, 0);

    } else if ((nowtime - GENERATOR_passtime) / 100 <= RTUROM[21] + 100) {

      RTUROM[19] = 0;
      temp = RTUROM[24];
      temp1 = RTUROM[26];
      temp /= temp1 / 1000;
      Timer3.pwm(ITV_pwm, temp);

    } else if ((nowtime - GENERATOR_passtime) / 100 <= RTUROM[21] + 100 + 600) {

      RTUROM[19] = 0;
      Timer3.pwm(ITV_pwm, 0);

    } else {

      RTUROM[19] = 1;
      RTUROM[28] = 0;
      RTUROM[22] = 0;
      GENERATOR_passtime = 0;

    }



  } else {

    Timer3.pwm(ITV_pwm, 0);
    RTUROM[22] = 0;
    GENERATOR_passtime = 0;

  }

}

void CHAMBER_TEMP() {

  RTUROM[35] = RTUROM[2];

  if (RTUROM[38] == 1) {

    float temp  = pid_cal(RTUROM[34], RTUROM[35], RTUROM[31], RTUROM[32], RTUROM[33], 0, 1023, -1023);
    int TS_pwm = round(temp);
    RTUROM[39] = TS_pwm;

    if (TS_pwm > 0) {

      digitalWrite(BLDM1_break, HIGH);//LOW - STOP
      digitalWrite(BLDM1_dir, HIGH);//HIGH - BACKWARD
      digitalWrite(BLDM2_break, HIGH);//LOW - STOP
      digitalWrite(BLDM2_dir, HIGH);//HIGH - BACKWARD

      Timer3.pwm(PELTIER_left, TS_pwm);
      Timer3.pwm(PELTIER_right, TS_pwm);

    } else if (temp < 0) {

      digitalWrite(BLDM1_break, HIGH);//LOW - STOP
      digitalWrite(BLDM1_dir, LOW);//LOW - FORWARD
      digitalWrite(BLDM2_break, HIGH);//LOW - STOP
      digitalWrite(BLDM2_dir, LOW);//LOW - FORWARD

      Timer3.pwm(PELTIER_left, abs(TS_pwm));
      Timer3.pwm(PELTIER_right, abs(TS_pwm));
    } if (TS_pwm == 0) {
      digitalWrite(BLDM1_break, LOW);//LOW - STOP
      digitalWrite(BLDM2_break, LOW);//LOW - STOP
    }

  } else {

    digitalWrite(BLDM1_break, LOW);//LOW - STOP
    digitalWrite(BLDM2_break, LOW);//LOW - STOP
    RTUROM[39] = 0;
    err_I[0] = 0;
    err_oP[0] = 0;

  }
}

float pid_cal(float pid_target, float pid_actual, float Pv, float Iv, int Dv, int index , int sat_max, int sat_min) {

  float err_P = 0;
  float pid_cal = 0;
  float pid_set = 0;

  err_P = (pid_target - pid_actual);
  RTUROM[41] = (err_P * Pv / 10000);
  err_I[index] += err_P;
  RTUROM[42] = (err_I[index] * Iv / 10000);
  RTUROM[43] = ((err_P - err_oP[index]) * Dv / 100);

  pid_cal = (err_P * Pv / 10000) + (err_I[index] * Iv / 100000) + ((err_P - err_oP[index]) * Dv / 100);
  err_oP[index] = err_P;

  if (pid_cal > sat_max) {
    pid_set = sat_max;
  } else if (pid_cal < sat_min) {
    pid_set = sat_min;
  } else {
    pid_set = pid_cal;
  }

  /* Special Contorl for Temperature Chamber
    if ((RTUROM[36] + RTUROM [37]) / 2 < RTUROM[34] * 1.01 && (RTUROM[36] + RTUROM [37]) / 2 > RTUROM[34] * 0.99) {
    err_I[index] = 0;
    }
  */

  if ( pid_cal != pid_set && ((err_P > 0 || pid_cal > 0) && ((err_P < 0 || pid_cal < 0))) ) {
    err_I[index] = 0;
  }



  return pid_set;
}


uint8_t readMemory(uint8_t fc, uint16_t address, uint16_t length) {
  uint16_t value;


  // read program memory.
  for (uint16_t i = 0; i < length; i++)
  {
    value = RTUROM[address + i];
    slave.writeRegisterToBuffer(i, value);
  }
  return STATUS_OK;
}

uint8_t writeMemory(uint8_t fc, uint16_t address, uint16_t length) {
  uint16_t value;
  uint16_t registerIndex;

  for (uint16_t i = 0; i < length; i++) {
    registerIndex = address + i;
    value = slave.readRegisterFromBuffer(i);

    if (registerIndex == 6) {
      RTUROM[registerIndex] = value;
      // Main System On / Off
      if (value == 1) {
        RTUROM[18] = 1;
        RTUROM[19] = 1;
        RTUROM[38] = 1;
      } else if (value == 0) {
        RTUROM[18] = 0;
        RTUROM[19] = 0;
        RTUROM[38] = 0;
        RTUROM[28] = 0; // Particle Generator Off
      }
    } else if (registerIndex == 8) {
      EEPROM_write(registerIndex, value);
    } else if (registerIndex == 14) {
      DCPC_datapull(value, 11);
    } else if (registerIndex == 16) {
      EEPROM_write(registerIndex, value);
    } else if (registerIndex == 18) {
      DCPC_datapull(value, 41);
    } else if (registerIndex == 21) {
      EEPROM_write(registerIndex, value);
    } else if (registerIndex == 24) {
      EEPROM_write(registerIndex, value);
    } else if (registerIndex == 26) {
      EEPROM_write(registerIndex, value);
    } else if (registerIndex == 27) {
      EEPROM_write(registerIndex, value);
    } else if (registerIndex == 28) {
      if (value == 0) {
        RTUROM[19] = 1;
        RTUROM[registerIndex] = value;
      } else if (value == 1) {
        RTUROM[registerIndex] = value;
      }
    } else if (registerIndex == 31) {
      EEPROM_write(registerIndex, value);
    } else if (registerIndex == 32) {
      EEPROM_write(registerIndex, value);
    } else if (registerIndex == 33) {
      EEPROM_write(registerIndex, value);
    } else if (registerIndex == 34) {
      EEPROM_write(registerIndex, value);
    } else {
      RTUROM[registerIndex] = value;
    }

  }
  return STATUS_OK;
}

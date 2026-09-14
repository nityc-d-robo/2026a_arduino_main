#include <SPI.h>
#include <mcp_can.h>
#include <avr/wdt.h>

//*****ボタンのビット番号*****
const byte btnBit_CROSS     = 0;
const byte btnBit_CIRCLE    = 1;
const byte btnBit_SQUARE    = 2;
const byte btnBit_TRIANGLE  = 3;
const byte btnBit_UP        = 4;
const byte btnBit_DOWN      = 5;
const byte btnBit_LEFT      = 6;
const byte btnBit_RIGHT     = 7;
const byte btnBit_L1        = 8;
const byte btnBit_R1        = 9;
const byte btnBit_L3        = 10;
const byte btnBit_R3        = 11;
const byte btnBit_L2        = 12;
const byte btnBit_R2        = 13;
const byte btnBit_SHARE     = 14;
const byte btnBit_START     = 15;
const byte btnBit_PS        = 16;
const byte btnBit_MUTE      = 17;

//*****エアシリンダーのバルブ番号(仮)*****
const byte Bucket = 3;
const byte LEFT_4X = 4;
const byte RIGHT_4X = 5;
const byte LEFT_2X = 6;
const byte RIGHT_2X = 7;

const unsigned int pulseStopInterval = 1000;

/**************************************************************************************************/
//構造体定義・初期化
/**************************************************************************************************/

//*****機構の構造体*****
struct MotorNode {
  byte motorType;
  byte controlMode;
  byte maxValueKind;
  unsigned int maxValue;
  byte typeId;
  byte nodeNum;
  byte deviceId;
};

//*****機構の構造体の配列の初期化・代入*****
MotorNode motorNodes[] = {
  {1, 3, 0, 0, 0b011, 0, 0},//FL
  {1, 3, 0, 0, 0b011, 0, 1},//FR
  {1, 3, 0, 0, 0b011, 0, 2},//RR
  {1, 3, 0, 0, 0b011, 0, 3},//RL
};

//*****オムニの構造体*****
struct OmniWheel {
  float vxCoef;
  float vyCoef;
  float omegaCoef;
  byte targetNode;
  const char *name;
};

//*****オムニ初期化*****
OmniWheel omni[] {
  {0.7071f, 0.7071f, 1.0f, 0, "FL"},
  {0.7071f, -0.7071f, 1.0f, 1, "FR"},
  { -0.7071f, -0.7071f, 1.0f, 2, "RR"},
  { -0.7071f, 0.7071f, 1.0f, 3, "RL"}
};

//*****エアシリンダー用の構造体*****
struct PulseButtonCommand {
  byte button;
  byte valveNum;
  unsigned int sendPulseTimeLength;
};

//*****エアシリンダー用構造体の初期化*****
PulseButtonCommand pulseCommands[] {
  {btnBit_SQUARE,   LEFT_4X,  pulseStopInterval}, 
  {btnBit_CIRCLE,   RIGHT_4X, pulseStopInterval}, 
  {btnBit_TRIANGLE, LEFT_2X,  pulseStopInterval}, 
  {btnBit_CROSS,    RIGHT_2X, pulseStopInterval}  
};

//*****パケットを受け取る構造体*****
struct __attribute__((packed)) ControllerPacket {
  int8_t leftX;
  int8_t leftY;
  int8_t rightX;
  int8_t rightY;

  uint8_t leftTrigger;
  uint8_t rightTrigger;

  uint32_t buttons;
};

//*****初期化*****
ControllerPacket controller = {0, 0, 0, 0, 0, 0, 0};

/**************************************************************************************************/
//変数定義・初期化
/**************************************************************************************************/

//*****モーターの数を臨機応変に*****
const byte motorNodesCount = sizeof(motorNodes) / sizeof(motorNodes[0]);

//*****オムニ臨機応変（仮）*****
const byte omniCount = sizeof(omni) / sizeof(omni[0]);

//*****パルス機構の数を臨機応変に*****
const byte pulseCommandsCount = sizeof(pulseCommands) / sizeof(pulseCommands[0]);

//*****MCP2515とSPI通信のための宣言*****
const byte CAN_CS_PIN = 53;
MCP_CAN CAN0(CAN_CS_PIN);

//*****CAN0.beginに渡す値*****
const byte CAN_BUS_SPEED = CAN_1000KBPS;
const byte MCP2515_CLOCK = MCP_16MHZ;

//*****INITのデータ*****
byte initData[8] = {0};

//*****canの状態*****
bool canReady = false;

//*****canの失敗回数*****
byte canFailCount = 0;

//*****最後にリトライした時刻*****
unsigned long lastRetryTime = 0;

//*****can失敗回数の上限*****
const byte maxCanFailCount = 8;

//*****リトライの間隔(ms)*****
const unsigned long retryInterval = 1000;

//*****非常停止ボタン状態監視*****
bool emergencyStopLatched = false;

//*****INIT送信を最後に行った時刻*****
unsigned long lastINITSendTime = 0;

//*****INIT送信のインターバル*****
const unsigned long INITSendInterval = 1000;

//*****RPMの送信を最後に行った時刻*****
unsigned long lastSpeedSendTime = 0;

//*****RPMの送信のインターバル*****
const unsigned long speedSendInterval = 40;

//*****最後にデバッグ出力をした時刻*****
unsigned long lastDebugPrintTime = 0;

//*****RPM表示のインターバル*****
const unsigned long debugPrintInterval = 1000;

//***************************************************************************
//*****並進最大RPM*****
const int translateMaxRPM = 170;

//*****回転最大RPM*****
const int rotateMaxRPM = 140;

//*****最大RPM*****
const int maxRPM = 200;

//*****毎秒変化できる最大RPM*****
const int RPMLimitPerSec = 400;
//****************************************************************************

//*****正規化用の係数の絶対値*****
const float coefValue = 0.7071f;

//*****スティックの値の中心値*****
const byte stickMaxValue = 127;

//*****スティックのデッドゾーン*****
const byte stickDeadZone = 18;

//SUCCESSを最後に送った時間
unsigned long lastCanSuccessSendTime = 0;

//SUCCESSの送信インターバル
const unsigned long canSuccessSendInterval = 1000;

//FAILを最後に送った時間
unsigned long lastCanFailSendTime = 0;

//FAILの送信インターバル
const unsigned long canFailSendInterval = 250;

//ZEROを最後に送った時間
unsigned long lastZeroSendTime = 0;

//ZEROの送信インターバル
const unsigned long zeroSendInterval = 50;

//*****オムニをprintするか*****
const bool printOmni = true;


//*****速度倍率_低速*****
const float slowSpeedScale = 0.4f;

//*****速度倍率_高速*****
const float fastSpeedScale = 1.5f;

//*****接続診断（切断）
bool Pad1_wasConnected = false;

//*****切断した時点の時間*****
unsigned long lastDisconnectTime = 0;

//*****切断後SHAREを押さないと動作しなくなる時間*****
const unsigned long needSHAREButtonTime = 1000;
//*****wio*****
#define wioSerial Serial1

//*****wioタイムアウト*****
const unsigned long wioLinkTimeout = 300;

//*****開始バイト待ちか受信中か*****
bool recieveState = false;

//*****カウンタ*****
byte byteCounta = 0;

//最後に受け取った時間
unsigned long lastWioRecieveTime = 0;

//*****バッファ*****
byte wioBuffer[11];

uint32_t lastButtonsState = 0;


unsigned long lastLoopStartTime = 0;
unsigned long maxLoopDuration = 0;

const byte packetId = 1;
const byte totalValveNum = 8;

const byte funcCode_Air = 0x01;
const byte typeId_Air = 0b011;
const byte nodeNum_Air = 1;


/**************************************************************************************************/
//配列
/**************************************************************************************************/

//*****生のRPMの値*****
float rawRPM[omniCount];

//*****スケールダウン処理をしたRPM*****
int downScaleRPM[omniCount];

//******調整して実際に送る値*****
int actualSendValues[omniCount];

//*****エアシリンダーの状態管理
bool pulseOn[totalValveNum] = {false};
bool pulseSent[totalValveNum] = {false};
unsigned long pulseOffTime[totalValveNum] = {0};

/**************************************************************************************************/
//関数
/**************************************************************************************************/

//*****モーターとエアシリンダーのCanIdを作る*****
bool makeCanId(byte funcCode, byte typeId, byte nodeNum, byte deviceId, unsigned long &canId) {
  if (typeId > 7) {
    return false;
  } else if (nodeNum > 3) {
    return false;
  } else if (deviceId > 7) {
    return false;
  } else {
    canId = (funcCode << 8) | (typeId << 5) | (nodeNum << 3) | (deviceId);
    return true;
  }
}
//*****接続判定*****
bool wioConnected(void) {
  if ((unsigned long)(millis() - lastWioRecieveTime) < wioLinkTimeout) {
    return true;
  } else {
    return false;
  }
}

//*****クリック判定*****
bool wioButtonClicked(byte bit) {
  byte currentState = ((controller.buttons >> bit) & 1);
  byte lastState = ((lastButtonsState >> bit) & 1);
  if (currentState && !lastState) {
    return true;
  } else {
    return false;
  }
}

//*****受信関数*****
void recieveWioData(void) {
  while (wioSerial.available()) {
    byte b = wioSerial.read();
    if (!recieveState) {
      if (b == 0xAA) {
        byteCounta = 0;
        recieveState = true;
      } else {
        continue;
      }
    } else {
      wioBuffer[byteCounta] = b;
      byteCounta++;
      if (byteCounta == 11) {
        //wioBuffer の先頭10バイトをXORし、計算結果を求める
        byte result = 0;
        for (int i = 0; i < 10; i++) {
          result ^= wioBuffer[i];
        }
        if (result == wioBuffer[10]) {
          //wioBuffer の先頭10バイトを controller 構造体にコピーする
          memcpy(&controller, wioBuffer, sizeof(controller));
          lastWioRecieveTime = millis();
        }
        recieveState = false;
        byteCounta = 0;
      }
    }
  }
}

//*****canId共通関数*****
bool resolveCanId(byte targetNode, byte funcCode, unsigned long &canId) {
  if (targetNode >= motorNodesCount) {
    return false;
  } else {
    byte typeId = motorNodes[targetNode].typeId;
    byte nodeNum = motorNodes[targetNode].nodeNum;
    byte deviceId = motorNodes[targetNode].deviceId;

    return makeCanId(funcCode, typeId, nodeNum, deviceId, canId);
  }
}

//*****canFailCountをリセット/+1する関数
bool canSendChecker(unsigned long canId, byte data[8]) {
  byte result = CAN0.sendMsgBuf(canId, 0, 8, data);
  if (result == CAN_OK) {
    if ((unsigned long)(millis() - lastCanSuccessSendTime) >= canSuccessSendInterval) {
      lastCanSuccessSendTime = millis();
      Serial.println(F("CAN SEND SUCCESS"));
    }
    canFailCount = 0;
    return true;
  } else {
    if ((unsigned long)(millis() - lastCanFailSendTime) >= canFailSendInterval) {
      lastCanFailSendTime = millis();
      Serial.println(F("CAN SEND FAILED"));
      Serial.print(F("errorCountTX = "));
      Serial.println(CAN0.errorCountTX());
      Serial.print(F("errorCountRX = "));
      Serial.println(CAN0.errorCountRX());

      byte eflg = CAN0.getError();
      printMcpError(eflg);
      printCanResultName(result);
    }
    canFailCount++;
    if (canFailCount >= maxCanFailCount) {
      canReady = false;
      lastRetryTime = millis();
    }
    return false;
  }
}

//*****canのリトライ*****
void canRetry(void) {
  if (canReady) {
    return;
  } else {
    if ((unsigned long)(millis() - lastRetryTime) >= retryInterval) {
      lastRetryTime = millis();
      pinMode(53, OUTPUT);          // MegaのハードウェアSSピンをOUTPUTに固定（SPI安定化）
      digitalWrite(53, HIGH);
      pinMode(CAN_CS_PIN, OUTPUT);
      digitalWrite(CAN_CS_PIN, HIGH);
      if (CAN0.begin(MCP_ANY, CAN_BUS_SPEED, MCP2515_CLOCK) == CAN_OK) {
        canReady = true;
        CAN0.setMode(MCP_NORMAL);
        canFailCount = 0;
        lastINITSendTime = millis() - INITSendInterval;
        for (int i = 0; i < omniCount; i++) {
          actualSendValues[i] = 0;
        }
      }
    }
  }
}

//*****CANエラー種別のプリント*****
void printMcpError(byte eflg) {
  bool printed = false;

  if (eflg & MCP_EFLG_EWARN) {
    Serial.println(F("エラー警告:TEC or RECが警告レベルに到達"));
    printed = true;
  }
  if (eflg & MCP_EFLG_RXWAR) {
    Serial.println(F("受信警告:エラー増加"));
    printed = true;
  }
  if (eflg & MCP_EFLG_TXWAR) {
    Serial.println(F("送信警告:エラー増加"));
    printed = true;
  }
  if (eflg & MCP_EFLG_TXEP) {
    Serial.println(F("TX_エラー多発"));
    printed = true;
  }
  if (eflg & MCP_EFLG_RXEP) {
    Serial.println(F("RX_エラー多発"));
    printed = true;
  }
  if (eflg & MCP_EFLG_TXBO) {
    Serial.println(F("バスOFF，送信不可"));
    printed = true;
  }
  if (eflg & MCP_EFLG_RX0OVR) {
    Serial.println(F("RX0:OVERFLOW"));
    printed = true;
  }
  if (eflg & MCP_EFLG_RX1OVR) {
    Serial.println(F("RX1:OVERFLOW"));
    printed = true;
  }
  if (!printed) {
    Serial.println(F("None"));
  }
}

//*****sendMsgBufの戻り値*****
void printCanResultName(byte result) {
  if (result == CAN_FAILINIT) {
    Serial.println(F("初期化失敗"));
  }
  if (result == CAN_FAILTX) {
    Serial.println(F("送信失敗"));
  }
  if (result == CAN_GETTXBFTIMEOUT) {
    Serial.println(F("送信バッファを取得できない"));
  }
  if (result == CAN_SENDMSGTIMEOUT) {
    Serial.println(F("送信バッファは確保，送信がタイムアウト"));
  }
}

//*****INIT送信*****
void sendINIT(void) {
  if (canReady) {
    if (emergencyStopLatched) {
      return;
    }
    for (int i = 0; i < motorNodesCount; i++) {
      if (!canReady) {
        break;
      }
      initData[0] = 0;
      initData[1] = motorNodes[i].motorType;
      initData[2] = motorNodes[i].controlMode;
      initData[3] = motorNodes[i].maxValueKind;
      initData[4] = (motorNodes[i].maxValue >> 8) & 0xFF;
      initData[5] = motorNodes[i].maxValue & 0xFF;
      byte funcCode = 0x01;
      unsigned long canId;
      //念のため判定
      if (!resolveCanId(i, funcCode, canId)) {
        continue;
      }
      canSendChecker(canId, initData);
    }
  }
}

//*****INIT定期送信*****
void ReSendINIT(void) {
  if (!canReady) {
    return;
  } else {
    if ((unsigned long)(millis() - lastINITSendTime) >= INITSendInterval) {
      lastINITSendTime = millis();
      sendINIT();
    }
  }
}

//*****スルーレート（）*****
void applySlewLimit(void) {
  int RPMLimitPerFrame = (RPMLimitPerSec * speedSendInterval) / 1000;
  if (RPMLimitPerFrame <= 0) {
    RPMLimitPerFrame = 1;
  }
  for (int i = 0; i < omniCount; i++) {
    if (downScaleRPM[i] >= actualSendValues[i]) {
      actualSendValues[i] += RPMLimitPerFrame;
      if (downScaleRPM[i] <= actualSendValues[i]) {
        actualSendValues[i] = downScaleRPM[i];
      }
    } else if (downScaleRPM[i] <= actualSendValues[i]) {
      actualSendValues[i] -= RPMLimitPerFrame;
      if (downScaleRPM[i] >= actualSendValues[i]) {
        actualSendValues[i] = downScaleRPM[i];
      }
    }
  }
}

//*****スティックの値を正規化する関数*****
float readStickRawValue(int8_t rawValue, bool needReverse) {
  int difference = (int)(rawValue);
  if (needReverse) {
    difference = -difference;
  }
  if (abs(difference) <= stickDeadZone) {
    return 0.0f;
  }
  float remainingRange = stickMaxValue - stickDeadZone;
  float normalizationValue = (abs(difference) - stickDeadZone) / remainingRange;
  if (normalizationValue >= 1.0) {
    normalizationValue = 1.0;
  }
  if (difference < 0.0) {
    return -normalizationValue;
  } else {
    return normalizationValue;
  }
}

//*****生のRPMを計算*****
void calcRawRPM(float vx, float vy, float omega) {
  for (int i = 0; i < omniCount; i++) {
    float calcTranslation = (omni[i].vxCoef * vx + omni[i].vyCoef * vy) / coefValue;
    float calcRotate = omni[i].omegaCoef * omega;
    float calcRawValue = (calcTranslation * translateMaxRPM) + (calcRotate * rotateMaxRPM);
    rawRPM[i] = calcRawValue;
  }
}

//*****値がでかい時にスケールダウンさせる関数*****
void scaleDown(void) {
  float peak = 0.0;
  float ratio = 0.0;
  float afterDownScale = 0.0;
  for (int i = 0; i < omniCount; i++) {
    if (abs(rawRPM[i]) > peak) {
      peak = abs(rawRPM[i]);
    }
  }
  if (peak >= maxRPM) {
    ratio = maxRPM / peak;
  } else {
    ratio = 1.0;
  }
  for (int i = 0; i < omniCount; i++) {
    afterDownScale = rawRPM[i] * ratio;
    if (afterDownScale >= maxRPM) {
      afterDownScale = maxRPM;
    } else if (afterDownScale <= -maxRPM) {
      afterDownScale = -maxRPM;
    }
    downScaleRPM[i] = (int)afterDownScale;
  }
}

//*****RPMのCAN送信*****
void sendSpeedCan(void) {
  for (int i = 0; i < omniCount; i++) {
    if (!canReady) {
      if ((unsigned long)(millis() - lastCanFailSendTime) >= canFailSendInterval) {
        lastCanFailSendTime = millis();
        Serial.println(F("CAN FAILS. RPM SEND SKIPPED"));
      }
      break;
    } else {
      int sendValue = actualSendValues[i];
      byte targetNode = omni[i].targetNode;
      byte funcCode = 0x01;
      unsigned long canId;
      if (!resolveCanId(targetNode, funcCode, canId)) {
        continue;
      }
      byte data[8] = {1, (byte)(((unsigned int)sendValue >> 8) & 0xFF), (byte)(sendValue & 0xFF), 0, 0, 0, 0, 0};
      canSendChecker(canId, data);
    }
  }
}

//*****RPM定期送信*****
void ReSendSpeed(void) {
  if (!canReady) {
    return;
  } else {
    if ((unsigned long)(millis() - lastSpeedSendTime) >= speedSendInterval) {
      lastSpeedSendTime = millis();
      applySlewLimit();
      sendSpeedCan();
    }
  }
}

//*****RPM表示*****
void printOmniValue(void) {
  if (!printOmni) {
    return;
  }
  if ((unsigned long)(millis() - lastDebugPrintTime) < debugPrintInterval) {
    return;
  }
  lastDebugPrintTime = millis();
  Serial.println(F("**** OMNI RPM VALUE *****"));
  for (int i = 0; i < omniCount; i++) {
    Serial.print(omni[i].name);
    Serial.print(F("/"));
    Serial.print(rawRPM[i]);
    Serial.print(F("/"));
    Serial.print(downScaleRPM[i]);
    Serial.print(F("/"));
    Serial.println(actualSendValues[i]);
  }
}

//*****高速/低速/通常を返す関数*****
float getSpeedScale(void) {
  bool slow = (controller.buttons >> btnBit_R1) & 1;
  bool fast = (controller.buttons >> btnBit_L1) & 1;
  if (slow == fast) {
    return 1.0f;
  } else if (slow) {
    return slowSpeedScale;
  } else {
    return fastSpeedScale;
  }
}

//*****オムニを0にする*****
void omniZero(void) {
  for (int i = 0; i < omniCount; i++) {
    downScaleRPM[i] = 0;
    actualSendValues[i] = 0;
  }
}

//*****オムニに0を送り続ける*****
void stopOmniNow(void) {
  omniZero();
  sendSpeedCan();
}

//*****オムニ0送信再送*****
void ReSendOmniStop(void) {
  if (!canReady) {
    return;
  }
  if ((unsigned long)(millis() - lastSpeedSendTime) >= speedSendInterval) {
    lastSpeedSendTime = millis();
    stopOmniNow();
  }
}

//*****エアシリンダーONを送る*****
bool sendAirFrame(byte valveNum, bool AirState) {
  if (!canReady) {
    return false;
  }
  if (valveNum >= totalValveNum) {
    return false;
  }
  unsigned long canId;
  if (!makeCanId(funcCode_Air, typeId_Air, nodeNum_Air, valveNum, canId)) {
    return false;
  } else {
    byte data[8] = {packetId, (byte)AirState, 0, 0, 0, 0, 0, 0};
    return canSendChecker(canId, data);
  }
}

//*****開始を書き込む*****
void writeValveOn(byte valveNum, unsigned int pulseLength) {
  if (valveNum >= totalValveNum) {
    return;
  } else {
    pulseOn[valveNum] = true;
    pulseOffTime[valveNum] = millis() + pulseLength;
  }
}

//*****時間が来たらpulseOnをfalseに*****
void pulseTimeObserve(void) {
  for (int i = 0; i < totalValveNum; i++) {
    if (i == Bucket) {
      continue;
    }
    if (pulseOn[i] && (long)(millis() - pulseOffTime[i]) >= 0) {
      pulseOn[i] = false;
    }
  }
}

//*****pulse送信*****
void sendPulseCan(bool needZero) {
  for (int i = 0; i < totalValveNum; i++) {
    if (needZero) {
      if (sendAirFrame(i, false)) {
        pulseSent[i] = false;
      }
    } else if (pulseOn[i] != pulseSent[i]) {
      if (sendAirFrame(i, pulseOn[i])) {
        pulseSent[i] = pulseOn[i];
      }
    }
  }
}

//*****全エアシリンダーOFF*****
void allPulseStop(void) {
  pulseOn_OFF();
  sendPulseCan(true);
}

//*****pulseOnをfalseにするだけ*****
void pulseOn_OFF(void) {
  for (int i = 0; i < totalValveNum; i++) {
    pulseOn[i] = false;
  }
}

//*****エアシリンダーのボタンが押されたかチェックする関数*****
void checkPulseButtons(void) {
  for (int i = 0; i < pulseCommandsCount; i++) {
    if (wioButtonClicked(pulseCommands[i].button)) {
      writeValveOn(pulseCommands[i].valveNum, pulseCommands[i].sendPulseTimeLength);
    }
  }
}

void checkBucket(void) {
  if (PS4.getButtonClick(R2)) {
    pulseOn[Bucket] = !pulseOn[Bucket];
  }
}

//*****非常停止共通関数*****
void sendCANStop(void) {
  if (!canReady) {
    Serial.println(F("CAN FAILS. EMERGENCY SEND SKIPPED"));
  } else {
    unsigned long canId = (0x00 << 8) | 0x00;
    byte emergencyData[8] = {0};
    emergencyData[0] = 1;
    canSendChecker(canId, emergencyData);
  }
}

//*****非常停止*****
void emergencyStop(void) {
  bool psClicked = wioButtonClicked(btnBit_PS);
  if (psClicked && !emergencyStopLatched) {
    emergencyStopLatched = true;
    
    allPulseStop();
    stopOmniNow();
    sendAllZero();

    sendCANStop();
  }
}

//*****非常停止解除*****
//0x03は要検討
void unlockEmergency(void) {
  bool shareClicked = wioButtonClicked(btnBit_SHARE);
  if (!shareClicked || !emergencyStopLatched) {
    return;
  }
  if (!canReady) {
    Serial.println(F("CAN FAILS. UNLOCK EMERGENCY SEND SKIPPED"));
    emergencyStopLatched = true;
  } else {
    unsigned long canId = (0x03 << 8) | 0x00;
    byte unlockEmergencyData[8] = {0};
    unlockEmergencyData[0] = 1;
    if (!canSendChecker(canId, unlockEmergencyData)) {
      emergencyStopLatched = true;
    } else {
      emergencyStopLatched = false;
      lastButtonsState = controller.buttons;
    }
  }
}
*/

//*****何らかの原因でMegaが再起動したとき0を送る*****
void sendAllZero(void) {
  for (int i = 0; i < motorNodesCount; i++) {
    if (!canReady) {
      break;
    }
    byte funcCode = 0x01;
    unsigned long canId;
    if (!resolveCanId(i, funcCode, canId)) {
      continue;
    }
    byte data[8] = {1, 0, 0, 0, 0, 0, 0, 0};
    canSendChecker(canId, data);
  }
}

//*****全機構に0を送る*****
void ReSendAllZero(void) {
  if (!canReady) {
    return;
  }
  if ((unsigned long)(millis() - lastZeroSendTime) >= zeroSendInterval) {
    lastZeroSendTime = millis();
    sendAllZero();
  }
}


/**************************************************************************************************/
//Setup
/**************************************************************************************************/
void setup() {
  byte resetCause = MCUSR;
  MCUSR = 0;
  wdt_disable();
  wdt_enable(WDTO_2S);
  Serial.begin(115200);
  wioSerial.begin(115200);
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  pinMode(CAN_CS_PIN, OUTPUT);
  digitalWrite(CAN_CS_PIN, HIGH);

  lastDisconnectTime = millis() - needSHAREButtonTime;

  //*****リセットが行われたときに原因をprint
  bool printed = false;
  if (resetCause & (1 << WDRF)) {
    Serial.println(F("Reset cause: WATCHDOG"));
    printed = true;
  }
  if (resetCause & (1 << BORF)) {
    Serial.println(F("Reset cause: BROWNOUT(電圧低下)"));
    printed = true;
  }
  if (resetCause & (1 << PORF)) {
    Serial.println(F("Reset cause: POWER ON"));
    printed = true;
  }
  if (resetCause & (1 << EXTRF)) {
    Serial.println(F("Reset cause: EXTERNAL(外部リセット)"));
    printed = true;
  }
  if (!printed) {
    Serial.println(F("Reset cause: None(正しく診断できていない)"));
  }

  //*****CAN成功/失敗判定*****
  if (CAN0.begin(MCP_ANY, CAN_BUS_SPEED, MCP2515_CLOCK) == CAN_OK) {
    Serial.println(F("CAN_Successful"));
    CAN0.setMode(MCP_NORMAL);
    canReady = true;
  } else {
    Serial.println(F("CAN_Failed"));
    canReady = false;
  }
  sendINIT();
  sendAllZero();
  for (int i = 0; i < totalValveNum; i++) {
    pulseSent[i] = true;
  }
  allPulseStop();

  //デバッグ用
  Serial.print("sizeof(ControllerPacket) = ");
  Serial.println(sizeof(ControllerPacket));

  lastLoopStartTime = millis();
}


/**************************************************************************************************/
//loop
/**************************************************************************************************/
void loop() {

  unsigned long now = millis();
  unsigned long loopDuration = (unsigned long)(now - lastLoopStartTime);
  lastLoopStartTime = now;
  if (loopDuration > maxLoopDuration) {
    maxLoopDuration = loopDuration;
    Serial.print(F("NEW MAX LOOP TIME:"));
    Serial.println(maxLoopDuration);
  }

  wdt_reset();
  recieveWioData();
  canRetry();
  ReSendINIT();
  pulseTimeObserve();
  sendPulseCan(false);

  bool isConnected = wioConnected();

  //*****接続状態のエッジ検出*****
  if (Pad1_wasConnected != isConnected) {
    if (!isConnected) {
      lastDisconnectTime = millis();
      sendAllZero();
      sendCANStop();
      allPulseStop();
    } else {
      if ((unsigned long)(millis() - lastDisconnectTime) >= needSHAREButtonTime) {
        emergencyStopLatched = true;
      }
    }
    Pad1_wasConnected = isConnected;
  }


  //*****接続されていないときはloop先頭に戻る*****
  if (!isConnected) {
    omniZero();
    ReSendOmniStop();
    ReSendAllZero();
    pulseOn_OFF();
    return;
  }
  unlockEmergency();
  emergencyStop();

  if (emergencyStopLatched) {
    ReSendOmniStop();
    return;
  }

  checkPulseButtons();
  checkBucket();

  float vx = readStickRawValue(controller.rightX, false);
  float vy = readStickRawValue(controller.rightY, true);
  float omega = readStickRawValue(controller.leftX, false);

  float speedScale = getSpeedScale();
  vx *= speedScale;
  vy *= speedScale;
  omega *= speedScale;

  calcRawRPM(vx, vy, omega);
  scaleDown();
  printOmniValue();

  ReSendSpeed();

  lastButtonsState = controller.buttons;
}

#include <SPI.h>
#include <mcp_can.h>

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

//*****エアシリンダーON*****
const bool pulseOnValue = 1;

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
  {0, 5, 0, 0, 0b011, 1, 0},//2X_R
  {0, 5, 0, 0, 0b011, 1, 1},//2X_L
  {0, 5, 0, 0, 0b011, 1, 2},//4X_R
  {0, 5, 0, 0, 0b011, 1, 3},//4X_L
  {0, 5, 0, 0, 0b011, 2, 0},//バケツ
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
  byte funcCode;
  byte targetNode;
  int pulseValue;
  unsigned int sendPulseTimeLength;
};

//*****エアシリンダー用構造体の初期化*****
PulseButtonCommand pulseCommands[] {
  {btnBit_CIRCLE,    0x01, 6, pulseOnValue, 300}, //4X_R
  {btnBit_SQUARE,    0x01, 4, pulseOnValue, 300}, //2X_R
  {btnBit_RIGHT,     0x01, 5, pulseOnValue, 300}, //2X_L
  {btnBit_LEFT,      0x01, 7, pulseOnValue, 300}, //4X_L
  {btnBit_TRIANGLE,  0x01, 8, pulseOnValue, 300}  //バケツ
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
const byte MCP2515_CLOCK = MCP_8MHZ;

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
const unsigned long debugPrintInterval = 300;

//*****並進最大RPM*****
const int translateMaxRPM = 170;

//*****回転最大RPM*****
const int rotateMaxRPM = 100;

//*****最大RPM*****
const int maxRPM = 200;

//*****毎秒変化できる最大RPM*****
const int RPMLimitPerSec = 400;

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
bool wasConnected = false;

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

/**************************************************************************************************/
//配列
/**************************************************************************************************/

//*****生のRPMの値*****
float rawRPM[omniCount];

//*****スケールダウン処理をしたRPM*****
int downScaleRPM[omniCount];

//******調整して実際に送る値*****
int actualSendValues[omniCount];

//*****パルス中かどうかの状態管理*****
bool pulseCommandsStates[pulseCommandsCount] = {false};

//*****パルスがいつ終わるかの予定時刻*****
unsigned long pulseUntilMs[pulseCommandsCount] = {0};

/**************************************************************************************************/
//関数
/**************************************************************************************************/

//*****ボタンの名前を返す関数*****
const char* buttonName(byte b) {
  switch (b) {
    case btnBit_TRIANGLE:
      return "TRIANGLE";
    case btnBit_CIRCLE:
      return "CIRCLE";
    case btnBit_CROSS:
      return "CROSS";
    case btnBit_SQUARE:
      return "SQUARE";
    case btnBit_L1:
      return "L1";
    case btnBit_R1:
      return "R1";
    case btnBit_L2:
      return "L2";
    case btnBit_R2:
      return "R2";
    case btnBit_LEFT:
      return "LEFT";
    case btnBit_RIGHT:
      return "RIGHT";
    case btnBit_SHARE:
      return "SHARE";
    case btnBit_PS:
      return "PS";
    default:
      return "Unknown_Button";
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
    //範囲外チェック
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
}

//*****canFailCountをリセット/+1する関数
bool canSendChecker(unsigned long canId, byte data[8]) {
  byte result = CAN0.sendMsgBuf(canId, 0, 8, data);
  if (result == CAN_OK) {
    if ((unsigned long)(millis() - lastCanSuccessSendTime) >= canSuccessSendInterval) {
      lastCanSuccessSendTime = millis();
      Serial.println("CAN SEND SUCCESS");
    }
    canFailCount = 0;
    return true;
  } else {
    if ((unsigned long)(millis() - lastCanFailSendTime) >= canFailSendInterval) {
      lastCanFailSendTime = millis();
      Serial.println("CAN SEND FAILED");
      Serial.print("errorCountTX = ");
      Serial.println(CAN0.errorCountTX());
      Serial.print("errorCountRX = ");
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
      }
    }
  }
}

//*****CANエラー種別のプリント*****
void printMcpError(byte eflg) {
  bool printed = false;

  if (eflg & MCP_EFLG_EWARN) {
    Serial.println("エラー警告:TEC or RECが警告レベルに到達");
    printed = true;
  }
  if (eflg & MCP_EFLG_RXWAR) {
    Serial.println("受信警告:エラー増加");
    printed = true;
  }
  if (eflg & MCP_EFLG_TXWAR) {
    Serial.println("送信警告:エラー増加");
    printed = true;
  }
  if (eflg & MCP_EFLG_TXEP) {
    Serial.println("TX_エラー多発");
    printed = true;
  }
  if (eflg & MCP_EFLG_RXEP) {
    Serial.println("RX_エラー多発");
    printed = true;
  }
  if (eflg & MCP_EFLG_TXBO) {
    Serial.println("バスOFF，送信不可");
    printed = true;
  }
  if (eflg & MCP_EFLG_RX0OVR) {
    Serial.println("RX0:OVERFLOW");
    printed = true;
  }
  if (eflg & MCP_EFLG_RX1OVR) {
    Serial.println("RX1:OVERFLOW");
    printed = true;
  }
  if (!printed) {
    Serial.println("None");
  }
}

//*****sendMsgBufの戻り値*****
void printCanResultName(byte result) {
  if (result == CAN_FAILINIT) {
    Serial.println("初期化失敗");
  }
  if (result == CAN_FAILTX) {
    Serial.println("送信失敗");
  }
  if (result == CAN_GETTXBFTIMEOUT) {
    Serial.println("送信バッファを取得できない");
  }
  if (result == CAN_SENDMSGTIMEOUT) {
    Serial.println("送信バッファは確保，送信がタイムアウト");
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
        Serial.println("CAN FAILS. RPM SEND SKIPPED");
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
  Serial.println("**** OMNI RPM VALUE *****");
  for (int i = 0; i < omniCount; i++) {
    Serial.print(omni[i].name);
    Serial.print("/");
    Serial.print(rawRPM[i]);
    Serial.print("/");
    Serial.print(downScaleRPM[i]);
    Serial.print("/");
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

//*****オムニに0を送り続ける*****
void stopOmniNow(void) {
  for (int i = 0; i < omniCount; i++) {
    downScaleRPM[i] = 0;
    actualSendValues[i] = 0;
  }
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

//*****エアシリンダーのボタンが押されたかチェックする関数*****
void checkPulseButtons(void) {
  bool TRIANGLE_Clicked = wioButtonClicked(btnBit_TRIANGLE);
  bool SQUARE_Clicked = wioButtonClicked(btnBit_SQUARE);
  bool CIRCLE_Clicked = wioButtonClicked(btnBit_CIRCLE);
  bool RIGHT_Clicked = wioButtonClicked(btnBit_RIGHT);
  bool LEFT_Clicked = wioButtonClicked(btnBit_LEFT);

  for (int i = 0; i < pulseCommandsCount; i++) {
    bool pulseButtonClicked = false;

    if (pulseCommands[i].button == btnBit_TRIANGLE) {
      pulseButtonClicked = TRIANGLE_Clicked;
    } else if (pulseCommands[i].button == btnBit_SQUARE) {
      pulseButtonClicked = SQUARE_Clicked;
    } else if (pulseCommands[i].button == btnBit_CIRCLE) {
      pulseButtonClicked = CIRCLE_Clicked;
    } else if (pulseCommands[i].button == btnBit_RIGHT) {
      pulseButtonClicked = RIGHT_Clicked;
    } else if (pulseCommands[i].button == btnBit_LEFT) {
      pulseButtonClicked = LEFT_Clicked;
    }

    if (pulseButtonClicked) {
      startPulse(i);
    }
  }
}

//*****エアシリンダーのボタンが押されるとON値を送信し、成功したら終了予定時刻をセット*****
void startPulse(byte index) {
  if (!canReady) {
    Serial.println("CAN FAILS. PULSE START SKIPPED");
    return;
  }
  byte targetNode = pulseCommands[index].targetNode;
  byte funcCode = pulseCommands[index].funcCode;
  unsigned long canId;
  if (!resolveCanId(targetNode, funcCode, canId)) {
    return;
  }
  int sendValue = pulseCommands[index].pulseValue;
  byte data[8] = {1, (byte)(((unsigned int)sendValue >> 8) & 0xFF), (byte)(sendValue & 0xFF), 0, 0, 0, 0, 0};
  if (canSendChecker(canId, data)) {
    pulseCommandsStates[index] = true;
    pulseUntilMs[index] = millis() + pulseCommands[index].sendPulseTimeLength;
  }
}

//*****毎ループ呼ばれ、終了予定時刻を過ぎたパルスを自動的に止める*****
void autoStopPulses(void) {
  for (int i = 0; i < pulseCommandsCount; i++) {
    if (!pulseCommandsStates[i]) {
      continue;
    }
    if ((long)(millis() - pulseUntilMs[i]) >= 0) {
      stopPulse(i);
    }
  }
}

//*****パルス終了：OFF(0)を送信。失敗時はactiveのままにして次ループで再送*****
void stopPulse(byte index) {
  if (!canReady) {
    return;
  }
  byte targetNode = pulseCommands[index].targetNode;
  byte funcCode = pulseCommands[index].funcCode;
  unsigned long canId;
  if (!resolveCanId(targetNode, funcCode, canId)) {
    return;
  }
  byte data[8] = {1, 0, 0, 0, 0, 0, 0, 0};
  if (canSendChecker(canId, data)) {
    pulseCommandsStates[index] = false;
  }
}

//*****全パルスを強制終了させる（非常停止）*****
void setPulsesEndTime(void) {
  for (int i = 0; i < pulseCommandsCount; i++) {
    if (pulseCommandsStates[i]) {
      pulseUntilMs[i] = millis();  // 終了予定時刻を「今」にして、autoStopPulses()に即座に処理させる
    }
  }
}

//*****非常停止共通関数*****
void sendCANStop(void) {
  if (!canReady) {
    Serial.println("CAN FAILS. EMERGENCY SEND SKIPPED");
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
    setPulsesEndTime();
    autoStopPulses();
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
    Serial.println("CAN FAILS. UNLOCK EMERGENCY SEND SKIPPED");
    emergencyStopLatched = true;
  } else {
    unsigned long canId = (0x03 << 8) | 0x00;
    byte unlockEmergencyData[8] = {0};
    unlockEmergencyData[0] = 1;
    if (!canSendChecker(canId, unlockEmergencyData)) {
      emergencyStopLatched = true;
    } else {
      emergencyStopLatched = false;      
    }
  }
}

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
  Serial.begin(115200);
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  pinMode(CAN_CS_PIN, OUTPUT);
  digitalWrite(CAN_CS_PIN, HIGH);

  lastDisconnectTime = millis();

  //*****CAN成功/失敗判定*****
  if (CAN0.begin(MCP_ANY, CAN_BUS_SPEED, MCP2515_CLOCK) == CAN_OK) {
    Serial.println("CAN_Successful");
    CAN0.setMode(MCP_NORMAL);
    canReady = true;
  } else {
    Serial.println("CAN_Failed");
    canReady = false;
  }
  sendINIT();
  sendAllZero();

  //デバッグ用
  Serial.print("sizeof(ControllerPacket) = ");
  Serial.println(sizeof(ControllerPacket));
}


/**************************************************************************************************/
//loop
//切断時のエッジ検出＋ESTOPフレーム送信（B-2）はwio実装まで不明なため未実装。
/**************************************************************************************************/
void loop() {
  recieveWioData();
  canRetry();
  ReSendINIT();
  autoStopPulses();

  bool isConnected = wioConnected();

  //*****接続状態のエッジ検出*****
  if (wasConnected != isConnected) {
    if (!isConnected) {
      lastDisconnectTime = millis();
      ReSendAllZero();
      sendCANStop();
    } else {
      if ((unsigned long)(millis() - lastDisconnectTime) >= needSHAREButtonTime) {
        emergencyStopLatched = true;
      }
    }
    wasConnected = isConnected;
  }


  //*****接続されていないときはloop先頭に戻る*****
  if (!isConnected) {
    ReSendOmniStop();
    ReSendAllZero();
    setPulsesEndTime();
    autoStopPulses();
    return;
  }

  emergencyStop();
  unlockEmergency();

  if (emergencyStopLatched) {
    ReSendSpeed();
    return;
  }

  checkPulseButtons();

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

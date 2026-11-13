#include <SPI.h>
#include <Wire.h>
#include <avr/wdt.h>
#include <mcp_can.h>
#include <string.h>
#include <util/atomic.h>

/**************************************************************************************************/
// 構造体定義
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

//*****オムニの構造体*****
struct OmniWheel {
  float vxCoef;
  float vyCoef;
  float omegaCoef;
  byte targetNode;
  const char *name;
};

//*****エアシリンダー用の構造体*****
struct PulseButtonCommand {
  byte button;
  byte valveNum;
  unsigned int sendPulseTimeLength;
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

/**************************************************************************************************/
// 定数定義
/**************************************************************************************************/

//*****wio関係*****
const uint8_t wioAddress = 0x14;

//*****I2C関係*****
const byte I2C_OperationFrameSize = 11;
const byte I2C_ControllerDataSize = 10;
const byte tenKeyFrameCapacity = 8;

//*****ボタンのビット番号*****
const byte btnBit_CROSS = 0;
const byte btnBit_CIRCLE = 1;
const byte btnBit_SQUARE = 2;
const byte btnBit_TRIANGLE = 3;
const byte btnBit_LEFT = 4;
const byte btnBit_RIGHT = 5;
const byte btnBit_UP = 6;
const byte btnBit_DOWN = 7;
const byte btnBit_L1 = 8;
const byte btnBit_R1 = 9;
const byte btnBit_L2 = 10;
const byte btnBit_R2 = 11;
const byte btnBit_SHARE = 12;
const byte btnBit_START = 13;
const byte btnBit_L3 = 14;
const byte btnBit_R3 = 15;
const byte btnBit_PS = 16;
const byte btnBit_TOUCHPAD = 17;
const byte btnBit_MIC = 18;

//*****エアシリンダーが自動で戻る間隔*****
const unsigned int pulseStopInterval = 500;

//*****エアシリンダーのバルブ番号(仮)*****
const byte Bucket = 6;
const byte LEFT_4X = 3;
const byte RIGHT_4X = 2;
const byte LEFT_2X = 0;
const byte RIGHT_2X = 1;
const byte L2Valve = 7;
const byte sequenceValve0 = LEFT_2X;
const byte sequenceValve1 = RIGHT_2X;
const byte sequenceValve3 = LEFT_4X;
const byte sequenceValve5 = 5;
const unsigned long sequencePulseLength = pulseStopInterval;
const unsigned long sequenceValve3Delay = 500;

//*****エアシリンダー関係*****
const byte packetId = 1;
const byte totalValveNum = 8;
const byte funcCode_Air = 0x01;
const byte typeId_Air = 0b011;
const byte nodeNum_Air = 1;

//*****レギュレータ関係*****
const unsigned long regulatorCanId = 0x003;
const byte regulatorCommand = 0x04;
const uint16_t maxRegulatorPressure = 72; // 上限0.72Mpa

//*****エアーの調整幅*****
const uint16_t regulatorPressureStep = 1;

//*****各モードの番号*****
const byte pressureModeCross = 0;
const byte pressureModeCircle = 1;
const byte pressureModeTriangle = 2;
const byte pressureModeSquare = 3;

//*****リミットスイッチの入力とGND,装填用モータのtargetNode*****
const int sequenceMotor_LimitSensePin = A14;
const int sequenceMotor_LimitGroundPin = A15;
const int sequenceMotor_TargetNode = 4;

//*****MCP2515とSPI通信のための宣言*****
const byte CAN_CS_PIN = 53;
MCP_CAN CAN0(CAN_CS_PIN);

//*****CAN0.beginに渡す値*****
const byte CAN_BUS_SPEED = CAN_1000KBPS;
const byte MCP2515_CLOCK = MCP_16MHZ;

//*****can失敗回数の上限*****
const byte maxCanFailCount = 8;

//*****切断後SHAREを押さないと動作しなくなる時間*****
const unsigned long needSHAREButtonTime = 1000;

//*****wioタイムアウト*****
const unsigned long wioLinkTimeout = 400;

//*****弁7を一瞬切る間隔*****
const unsigned long L2ValveOffDuration = 200;

//*****リトライの間隔(ms)*****
const unsigned long retryInterval = 1000;

//*****INIT送信のインターバル*****
const unsigned long INITSendInterval = 1000;

//*****SUCCESSの送信インターバル*****
const unsigned long canSuccessSendInterval = 1000;

//*****FAILの送信インターバル*****
const unsigned long canFailSendInterval = 250;

//*****ZEROの送信インターバル*****
const unsigned long zeroSendInterval = 50;

//*****デバッグ用*****
const unsigned long disconnectReportInterval = 10000;

//*****RPM表示のインターバル*****
const unsigned long debugPrintInterval = 1000;

//*****RPMの送信のインターバル*****
const unsigned long speedSendInterval = 40;

//***************************************************************************
//*****並進最大RPM*****
const int translateMaxRPM = 170;

//*****回転最大RPM*****
const int rotateMaxRPM = 140;

//*****最大RPM*****
const int maxRPM = 200;

//*****毎秒変化できる最大RPM*****
const int RPMLimitPerSec = 400;

//*****速度倍率_低速*****
const float slowSpeedScale = 0.4f;

//*****速度倍率_高速*****
const float fastSpeedScale = 1.5f;
//****************************************************************************

//*****オムニをprintするか*****
const bool printOmni = true;

//*****正規化用の係数の絶対値*****
const float coefValue = 0.7071f;

//*****スティックの値の最大振れ幅*****
const byte stickMaxValue = 127;

//*****スティックのデッドゾーン*****
const byte stickDeadZone = 18;

//*****モーター指令用(負の値が通常の方向，正にすると逆転する（Homing中）)*****
const int sequenceMotor_RPM = -80;

//*****回転数を送信するインターバル*****
const unsigned long sequenceMotor_SendInterval = 40;

//*****モーターが動き出した直後読まない時間*****
const unsigned long sequenceMotor_LimitSWCheckDelay = 500;

//*****弁３が終わってからモーターを動かし始めるまでの時間*****
const unsigned long sequenceMotor_AfterValve3Delay = 500;

//*****モーターを止めて，一回装填するまでの時間*****
const unsigned long sequenceValve5ReturnDelay = 500;

/**************************************************************************************************/
// 変数定義・初期化
/**************************************************************************************************/

//*****INITのデータ*****
byte initData[8] = {0};

//*****canの状態*****
bool canReady = false;

//*****canの失敗回数*****
byte canFailCount = 0;

//*****最後にリトライした時刻*****
unsigned long lastRetryTime = 0;

//*****非常停止ボタン状態監視*****
bool emergencyStopLatched = false;

//*****INIT送信を最後に行った時刻*****
unsigned long lastINITSendTime = 0;

//*****RPMの送信を最後に行った時刻*****
unsigned long lastSpeedSendTime = 0;

//*****最後にデバッグ出力をした時刻*****
unsigned long lastDebugPrintTime = 0;

// SUCCESSを最後に送った時間
unsigned long lastCanSuccessSendTime = 0;

// FAILを最後に送った時間
unsigned long lastCanFailSendTime = 0;

// ZEROを最後に送った時間
unsigned long lastZeroSendTime = 0;

//*****接続診断（切断）
bool Pad1_wasConnected = false;

//*****切断した時点の時間*****
unsigned long lastDisconnectTime = 0;

//*****デバッグのprint用（切断時）*****
unsigned long disconnectCountUntil_10s = 0;
unsigned long disconnectTimeUntil_10s = 0;
unsigned long lastDisconnectReportTime = 0;

// 最後に受け取った時間
unsigned long lastWioReceiveTime = 0;

uint32_t lastButtonsState = 0;

unsigned long lastLoopStartTime = 0;
unsigned long maxLoopDuration = 0;

uint32_t lastRawButtonsState = 0;

bool firstConnect = false;

uint16_t regulatorPressure = 60; // 0.01MPa単位の整数
bool regulatorPressurePending = true;

//*****最初のモード*****
byte selectedPressureMode = pressureModeCross;

//*****各モードの値の初期値*****
uint16_t crossValve0Pressure = 43;
uint16_t crossValve1Pressure = 37;
uint16_t circleValve0Pressure = 60;
uint16_t circleValve1Pressure = 60;
uint16_t triangleValve0Pressure = 44;
uint16_t triangleValve1Pressure = 37;
uint16_t squareValve0Pressure = 26;
uint16_t squareValve1Pressure = 24;

//*****弁7の復帰用*****
bool L2ValveReturnReservation = false;
unsigned long L2ValveReturnTime = 0;

//*****番号によりシーケンス制御の動作を決める*****
byte sequenceStep = 0;

//*****シーケンス制御をやり始めたボタン（255で未選択）*****
byte sequenceControlButton = 255;

//*****弁1の圧力設定が予約中かどうか*****
bool sequenceValve1PressurePending = false;

//*****予約中の弁1の圧力値*****
uint16_t sequenceValve1PendingPressure = 0; // 0.01MPa単位

//*****弁3が予約中かどうか*****
bool sequenceValve3Pending = false;

//*****弁3を発射する時刻*****
unsigned long sequenceValve3StartTime = 0;

//*****発射し，装填要求を受け付けてよいか．弁1が発射されるとtrue，ボタン選択・装填開始・非常停止でfalse*****
bool sequenceBackendReady = false;

//*****L2で装填要求されたかどうか．予約されたらfalse*****
bool sequenceBackendRequested = false;

//*****弁3の発射が終わるのを待っているか*****
bool sequenceAfterValve3Pending = false;

//*****モーターを回しているか*****
bool sequenceMotor_Running = false;

//*****モーター始動が予約中か*****
bool sequenceMotor_StartPending = false;

//*****始動予約する時刻*****
unsigned long sequenceMotor_StartTime = 0;

//*****実際に始動した時刻*****
unsigned long sequenceMotor_StartedTime = 0;

//*****この時刻までリミットスイッチを読み飛ばす*****
unsigned long sequenceMotor_LimitIgnoreUntil = 0;

//*****押されていない状態を確認済みか*****
bool sequenceMotor_CheckedLimitSWClosed = false;

//*****リミットスイッチをあと何回読み飛ばすか*****
byte sequenceMotor_LimitSWsToSkip = 1;

//*****回転数を最後に送信した時間*****
unsigned long sequenceMotor_LastSendTime = 0;

//*****現在未使用*****
bool sequenceValve5_On = false;

//*****モーター停止後弁5をONにする予約が残っているか*****
bool sequenceValve5ReturnPending = false;

//*****「弁5をONにした瞬間->モーター動いて次のカセットへ」の間だけtrue*****
bool sequenceMotor_ReturnPhase = false;

//*****装填機構を原点に戻しているか*****
bool sequenceMotor_Homing = false;

//*****モーター始動を待つ時の弁5の状態*****
bool sequenceMotor_PendingValve5State = true;

//*****装填が何回終わったか*****
byte sequenceReloadCount = 0;

//*****弁5をONにする時刻*****
unsigned long sequenceValve5ReturnTime = 0;

//*****I2Cで送られてきたデータを格納*****
volatile byte pendingControllerData[10] = {0};
volatile bool pendingControllerFrame = false;

volatile byte tenKeyFrameQueue[8][10] = {0};
volatile byte tenKeyQueueHead = 0;
volatile byte tenKeyQueueTail = 0;
volatile byte tenKeyQueueCount = 0;

volatile uint32_t I2C_ControllerFrames = 0;
volatile uint32_t I2C_IgnoredFrames = 0;
volatile uint32_t I2C_InvalidFrames = 0;
volatile uint32_t I2C_tenKeyDroppedFrames = 0;

byte robotStateForBridge[10] = {0};

/**************************************************************************************************/
// 構造体初期化
/**************************************************************************************************/

//*****機構の構造体の配列の初期化・代入*****
MotorNode motorNodes[] = {
    {1, 3, 0, 0, 0b011, 0, 0}, // FL
    {1, 3, 0, 0, 0b011, 0, 1}, // FR
    {1, 3, 0, 0, 0b011, 0, 2}, // RR
    {1, 3, 0, 0, 0b011, 0, 3}, // RL
    {1, 3, 0, 0, 0b011, 0, 4}  // 追加モーター（確定後名称変更）
};
//*****モーターの数を臨機応変に*****
const byte motorNodesCount = sizeof(motorNodes) / sizeof(motorNodes[0]);

//*****オムニ初期化*****
OmniWheel omni[]{
    {0.7071f, 0.7071f, 1.0f, 0, "FL"},
    {0.7071f, -0.7071f, 1.0f, 1, "FR"},
    {-0.7071f, -0.7071f, 1.0f, 2, "RR"},
    {-0.7071f, 0.7071f, 1.0f, 3, "RL"}};
//*****オムニ臨機応変（仮）*****
const byte omniCount = sizeof(omni) / sizeof(omni[0]);

//*****エアシリンダー用構造体の初期化*****
PulseButtonCommand pulseCommands[]{
    {btnBit_RIGHT, RIGHT_4X, pulseStopInterval}};
//*****パルス機構の数を臨機応変に*****
const byte pulseCommandsCount = sizeof(pulseCommands) / sizeof(pulseCommands[0]);

//*****初期化*****
ControllerPacket controller = {0, 0, 0, 0, 0, 0, 0};

/**************************************************************************************************/
// 配列
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
// 関数
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
  if ((unsigned long)(millis() - lastWioReceiveTime) < wioLinkTimeout) {
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

//*****I2C受信*****
void receiveI2CEvent(int howMany) {
  bool tooLong = false;
  byte size = 0;
  byte frame[I2C_OperationFrameSize] = {0};

  while (Wire.available()) {
    if (size < I2C_OperationFrameSize) {
      frame[size] = Wire.read();
      size++;
    } else {
      tooLong = true;
    }
  }

  if ((howMany != I2C_OperationFrameSize) || tooLong || size != 11) {
    I2C_InvalidFrames++;
    return;
  }

  if (frame[0] == 0x01) {
    if (tenKeyQueueCount >= tenKeyFrameCapacity) {
      I2C_tenKeyDroppedFrames++;
      return;
    }
    for (int i = 0; i < I2C_ControllerDataSize; i++) {
      tenKeyFrameQueue[tenKeyQueueTail][i] = frame[i + 1];
    }
    tenKeyQueueTail++;
    if (tenKeyQueueTail == tenKeyFrameCapacity) {
      tenKeyQueueTail = 0;
    }
    tenKeyQueueCount++;
    I2C_IgnoredFrames++;
    return;
  }
  if (frame[0] != 0x02) {
    I2C_InvalidFrames++;
    return;
  } else {
    for (int i = 0; i < I2C_ControllerDataSize; i++) {
      pendingControllerData[i] = frame[i + 1];
    }
    pendingControllerFrame = true;
    I2C_ControllerFrames++;
  }
}

void requestI2CEvent(void) {
  Wire.write(robotStateForBridge, sizeof(robotStateForBridge));
}

bool consumeI2CControllerFrame(void) {
  byte snapshot[10] = {0};
  bool ready = false;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (pendingControllerFrame) {
      for (int i = 0; i < I2C_ControllerDataSize; i++) {
        snapshot[i] = pendingControllerData[i];
      }
      pendingControllerFrame = false;
      ready = true;
    }
  }
  if (ready) {
    memcpy(&controller, snapshot, sizeof(controller));
  }
  return ready;
}

bool popTenKeyFrame(byte *frame) {
  bool ready = false;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (tenKeyQueueCount >= 1) {
      for (int i = 0; i < I2C_ControllerDataSize; i++) {
        frame[i] = tenKeyFrameQueue[tenKeyQueueHead][i];
      }
      tenKeyQueueHead++;
      tenKeyQueueCount--;
      if (tenKeyQueueHead == tenKeyFrameCapacity) {
        tenKeyQueueHead = 0;
      }
      ready = true;
    }
  }
  return ready;
}

//*****チェックサム関数*****
uint8_t calcCrc(const uint8_t data[10]) {
  uint8_t crc = 0xFF;
  for (int i = 0; i < 10; i++) {
    crc ^= data[i];
    for (uint8_t bit = 8; bit > 0; --bit) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x31u;
      } else {
        crc = (crc << 1);
      }
    }
  }
  return crc;
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
      pinMode(53, OUTPUT); // MegaのハードウェアSSピンをOUTPUTに固定（SPI安定化）
      digitalWrite(53, HIGH);
      pinMode(CAN_CS_PIN, OUTPUT);
      digitalWrite(CAN_CS_PIN, HIGH);
      if (CAN0.begin(MCP_ANY, CAN_BUS_SPEED, MCP2515_CLOCK) == CAN_OK) {
        canReady = true;
        CAN0.setMode(MCP_NORMAL);
        regulatorPressurePending = true;
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
    Serial.println(F("エラー警告:TEC or RECが警告レベルに到達")); // 送信/受信のエラーカウンタ
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

//*****10秒ごとに切断回数・時間をprint****
void printDisconnect(void) {
  if ((unsigned long)(millis() - lastDisconnectReportTime) >= disconnectReportInterval) {
    lastDisconnectReportTime = millis();
    Serial.print(F("disconnectCount= "));
    Serial.println(disconnectCountUntil_10s);
    Serial.print(F("disconnectTime= "));
    Serial.println(disconnectTimeUntil_10s);
    disconnectCountUntil_10s = 0;
    disconnectTimeUntil_10s = 0;
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
      // 念のため判定
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
  bool R1 = ((controller.buttons >> btnBit_R1) & 1);
  bool L1 = ((controller.buttons >> btnBit_L1) & 1);
  bool UP = ((controller.buttons >> btnBit_UP) & 1);
  bool DOWN = ((controller.buttons >> btnBit_DOWN) & 1);
  bool adjustPressure = false;
  if ((R1 || L1) && (UP || DOWN)) {
    adjustPressure = true;
  }
  bool slow = R1 && !adjustPressure;
  bool fast = L1 && !adjustPressure;

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
    reserveL2Valve(valveNum);
    pulseOn[valveNum] = true;
    pulseOffTime[valveNum] = millis() + pulseLength;
  }
}

//*****時間が来たらpulseOnをfalseに*****
void pulseTimeObserve(void) {
  for (int i = 0; i < totalValveNum; i++) {
    if (i == Bucket || i == L2Valve || i == sequenceValve5) {
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
  L2ValveReturnReservation = false;
  pulseOn_OFF();
  sendPulseCan(true);
  sequenceValve5_On = false;
}

//*****pulseOnをfalseにするだけ*****
void pulseOn_OFF(void) {
  for (int i = 0; i < totalValveNum; i++) {
    pulseOn[i] = false;
  }
}

//*****バケツ専用*****
void checkBucket(void) {
  if (wioButtonClicked(btnBit_R2)) {
    pulseOn[Bucket] = !pulseOn[Bucket];
    reserveL2Valve(Bucket);
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

//*****圧力調整*****
void checkRegulatorPressureButtons(void) {
  bool L1Pressed = (controller.buttons >> btnBit_L1) & 1;
  bool R1Pressed = (controller.buttons >> btnBit_R1) & 1;
  uint16_t *p = NULL;

  if (L1Pressed == R1Pressed) {
    return;
  }

  if (L1Pressed) {
    if (selectedPressureMode == pressureModeCross) {
      p = &crossValve0Pressure;
    } else if (selectedPressureMode == pressureModeCircle) {
      p = &circleValve0Pressure;
    } else if (selectedPressureMode == pressureModeTriangle) {
      p = &triangleValve0Pressure;
    } else if (selectedPressureMode == pressureModeSquare) {
      p = &squareValve0Pressure;
    }
  }
  if (R1Pressed) {
    if (selectedPressureMode == pressureModeCross) {
      p = &crossValve1Pressure;
    } else if (selectedPressureMode == pressureModeCircle) {
      p = &circleValve1Pressure;
    } else if (selectedPressureMode == pressureModeTriangle) {
      p = &triangleValve1Pressure;
    } else if (selectedPressureMode == pressureModeSquare) {
      p = &squareValve1Pressure;
    }
  }

  if (p == NULL) {
    return;
  }

  if (wioButtonClicked(btnBit_UP)) {
    if (maxRegulatorPressure > *p) {
      *p += regulatorPressureStep;
      setRegulatorPressure(*p);
    }
  }
  if (wioButtonClicked(btnBit_DOWN)) {
    if (regulatorPressureStep <= *p) {
      *p -= regulatorPressureStep;
      setRegulatorPressure(*p);
    }
  }
}

//*****STARTでモードを変更*****
void checkSTARTButton(void) {
  if (!wioButtonClicked(btnBit_START)) {
    return;
  }
  selectedPressureMode = (selectedPressureMode + 1) % 4; // 余りで番号を判別

  if (selectedPressureMode == pressureModeCross) {
    Serial.println(F("PressureMode= CROSS"));
  } else if (selectedPressureMode == pressureModeCircle) {
    Serial.println(F("PressureMode= CIRCLE"));
  } else if (selectedPressureMode == pressureModeTriangle) {
    Serial.println(F("PressureMode= TRIANGLE"));
  } else if (selectedPressureMode == pressureModeSquare) {
    Serial.println(F("PressureMode= SQUARE"));
  }
}

//*****電空レギュレータの値を決定（丸めなど）*****
void setRegulatorPressure(uint16_t pressure) {
  if (pressure > maxRegulatorPressure) {
    pressure = maxRegulatorPressure;
  }
  regulatorPressure = pressure;
  regulatorPressurePending = true;
  Serial.print(F("CHANGED VALUE = "));
  Serial.print(regulatorPressure);
  Serial.println(F("(*0.01MPa)"));
}

//*****電空レギュレータへの命令を送信*****
bool sendRegulatorPressure(void) {
  if (!canReady) {
    return false;
  }
  if (regulatorPressure > maxRegulatorPressure) {
    regulatorPressure = maxRegulatorPressure;
  }
  float converted = regulatorPressure / 100.0f;
  uint8_t divided[4];
  memcpy(divided, &converted, sizeof(float));
  byte data[8] = {regulatorCommand, divided[3], divided[2], divided[1], divided[0], 0, 0, 0};

  if (!canSendChecker(regulatorCanId, data)) {
    return false;
  } else {
    regulatorPressurePending = false;
    return true;
  }
}

//*****RPMをモーターに送る*****
bool sendSequenceMotorRpm(int rpm) {
  if (!canReady) {
    return false;
  }
  unsigned long canId;
  if (!resolveCanId(sequenceMotor_TargetNode, 0x01, canId)) {
    return false;
  }

  byte data[8] = {1, (byte)(rpm >> 8), (byte)rpm, 0, 0, 0, 0, 0};

  return canSendChecker(canId, data);
}

//*****リミットスイッチの状態を読む*****
bool sequenceMotorLimitPressed(void) {
  return digitalRead(sequenceMotor_LimitSensePin);
}

//*****他の弁が操作されると弁7（コンプレッサー）をOFF,復帰の予約を入れる*****
void reserveL2Valve(byte otherValve) {
  if (otherValve == L2Valve) {
    return;
  }
  if (pulseOn[L2Valve]) {
    pulseOn[L2Valve] = false;
  }
  L2ValveReturnReservation = true;
  L2ValveReturnTime = millis() + L2ValveOffDuration;
}

//*****一定時間たったら弁７をONに*****
void observeL2Valve(void) {
  if (L2ValveReturnReservation && ((long)(millis() - L2ValveReturnTime) >= 0)) {
    L2ValveReturnReservation = false;
    if (!pulseOn[L2Valve]) {
      pulseOn[L2Valve] = true;
    }
  }
}

//*****発射シーケンス*****
void checkSequenceControls(void) {
  if (sequenceValve1PressurePending && !pulseOn[sequenceValve0] && !pulseSent[sequenceValve0]) {
    setRegulatorPressure(sequenceValve1PendingPressure);
    sequenceValve1PressurePending = false;
    sequenceValve1PendingPressure = 0;
  }

  if (sequenceValve3Pending && ((long)(millis() - sequenceValve3StartTime) >= 0)) {
    writeValveOn(sequenceValve3, sequencePulseLength);
    sequenceValve3Pending = false;
  }

  bool CROSS_Clicked = wioButtonClicked(btnBit_CROSS);
  bool CIRCLE_Clicked = wioButtonClicked(btnBit_CIRCLE);
  bool TRIANGLE_Clicked = wioButtonClicked(btnBit_TRIANGLE);
  bool SQUARE_Clicked = wioButtonClicked(btnBit_SQUARE);
  bool sameButtonClicked =
      (sequenceControlButton == btnBit_CROSS && CROSS_Clicked) ||
      (sequenceControlButton == btnBit_CIRCLE && CIRCLE_Clicked) ||
      (sequenceControlButton == btnBit_TRIANGLE && TRIANGLE_Clicked) ||
      (sequenceControlButton == btnBit_SQUARE && SQUARE_Clicked);

  if (!sequenceValve3Pending &&
      !pulseOn[sequenceValve0] &&
      !pulseOn[sequenceValve1] &&
      !pulseOn[sequenceValve3] &&
      !sequenceBackendRequested &&
      !sequenceAfterValve3Pending &&
      !sequenceMotor_Running &&
      !sequenceMotor_StartPending &&
      !sequenceValve5ReturnPending) {

    if (sequenceStep == 0) {
      if (CROSS_Clicked) {
        sequenceBackendReady = false;
        setRegulatorPressure(crossValve0Pressure);
        sequenceControlButton = btnBit_CROSS;
        sequenceStep = 1;
        sequenceBackendRequested = false;
      } else if (CIRCLE_Clicked) {
        sequenceBackendReady = false;
        setRegulatorPressure(circleValve0Pressure);
        sequenceControlButton = btnBit_CIRCLE;
        sequenceStep = 1;
        sequenceBackendRequested = false;
      } else if (TRIANGLE_Clicked) {
        sequenceBackendReady = false;
        setRegulatorPressure(triangleValve0Pressure);
        sequenceControlButton = btnBit_TRIANGLE;
        sequenceStep = 1;
        sequenceBackendRequested = false;
      } else if (SQUARE_Clicked) {
        sequenceBackendReady = false;
        setRegulatorPressure(squareValve0Pressure);
        sequenceControlButton = btnBit_SQUARE;
        sequenceStep = 1;
        sequenceBackendRequested = false;
      }
    } else if (sequenceStep == 1 && sameButtonClicked) {
      writeValveOn(sequenceValve0, sequencePulseLength);
      sequenceStep = 2;
      sequenceValve1PressurePending = true;
      if (sequenceControlButton == btnBit_CROSS) {
        sequenceValve1PendingPressure = crossValve1Pressure;
      } else if (sequenceControlButton == btnBit_CIRCLE) {
        sequenceValve1PendingPressure = circleValve1Pressure;
      } else if (sequenceControlButton == btnBit_TRIANGLE) {
        sequenceValve1PendingPressure = triangleValve1Pressure;
      } else if (sequenceControlButton == btnBit_SQUARE) {
        sequenceValve1PendingPressure = squareValve1Pressure;
      }
    } else if (sequenceStep == 2 && sameButtonClicked && !sequenceValve1PressurePending) {
      writeValveOn(sequenceValve1, sequencePulseLength);
      sequenceStep = 0;
      sequenceControlButton = 255;
      sequenceBackendReady = true;
      sequenceValve3Pending = true;
      sequenceValve3StartTime = millis() + sequenceValve3Delay;
    }
  }
}

//*****「装填したい」になっていれば予約する*****
void reserveReloadMotor(void) {
  if (sequenceBackendReady &&
      sequenceBackendRequested &&
      !sequenceMotor_Running &&
      !sequenceMotor_StartPending &&
      !sequenceValve3Pending &&
      !pulseOn[sequenceValve3] &&
      !sequenceValve5ReturnPending) {
    sequenceAfterValve3Pending = true;
    sequenceStep = 5;
    sequenceBackendReady = false;
    sequenceBackendRequested = false;
  }

  if (sequenceAfterValve3Pending && sequenceStep == 5 && !sequenceMotor_Running && !sequenceMotor_StartPending && !sequenceValve3Pending && !pulseOn[sequenceValve3]) {
    sequenceAfterValve3Pending = false;
    sequenceValve1PressurePending = false;
    sequenceMotor_StartPending = true;
    sequenceMotor_PendingValve5State = false;
    sequenceMotor_StartTime = millis() + sequenceMotor_AfterValve3Delay;
  }
}

//*****条件がそろったらstartSequenceMotor()を呼ぶ*****
void startReloading(void) {
  if (sequenceMotor_StartPending && (pulseSent[sequenceValve5] == sequenceMotor_PendingValve5State) && ((long)(millis() - sequenceMotor_StartTime) >= 0)) {
    sequenceMotor_StartPending = false;
    startSequenceMotor();

    if (!sequenceMotor_Running) {
      if (sequenceMotor_Homing) {
        sequenceMotor_Homing = false;
        sequenceReloadCount = 1;
        sequenceStep = 0;
        sequenceControlButton = 255;
      } else if (sequenceMotor_ReturnPhase) {
        sequenceMotor_ReturnPhase = false;
        sequenceStep = 0;
        sequenceControlButton = 255;
      } else {
        sequenceStep = 0;
        sequenceControlButton = 255;
        sequenceBackendReady = true;
        sequenceBackendRequested = false;
      }
    }
  }
}

//*****片方装填してもう一度予約*****
void reReserveReloadMotor(void) {
  if (sequenceValve5ReturnPending && ((long)(millis() - sequenceValve5ReturnTime) >= 0)) {
    sequenceValve5ReturnPending = false;
    sequenceValve5_On = true;
    reserveL2Valve(sequenceValve5);
    pulseOn[sequenceValve5] = true;
    sequenceMotor_ReturnPhase = true;
    sequenceMotor_StartPending = true;
    sequenceMotor_PendingValve5State = true;
    sequenceMotor_StartTime = millis() + sequenceMotor_AfterValve3Delay;
  }
}

//*****装填要求，予約，始動，2回目の予約を毎周期呼ぶ*****
void requestLoading(void) {
  bool L2_Clicked = wioButtonClicked(btnBit_L2);
  if (sequenceBackendReady && L2_Clicked) {
    sequenceBackendRequested = true;
  }

  reserveReloadMotor();
  startReloading();
  reReserveReloadMotor();
}

//*****復帰中だけ正転を返す*****
int currentSequenceMotorRPM(void) {
  if (sequenceMotor_Homing) {
    return -sequenceMotor_RPM;
  } else {
    return sequenceMotor_RPM;
  }
}

//*****装填用モータの始動*****
void startSequenceMotor(void) {
  if (currentSequenceMotorRPM() == 0) {
    return;
  }
  unsigned long canId;
  if (!resolveCanId(sequenceMotor_TargetNode, 0x01, canId)) {
    return;
  }

  sequenceMotor_CheckedLimitSWClosed = false;
  sequenceMotor_Running = sendSequenceMotorRpm(currentSequenceMotorRPM());

  if (sequenceMotor_Running) {
    sequenceMotor_StartedTime = millis();
    sequenceMotor_LimitIgnoreUntil = sequenceMotor_StartedTime + sequenceMotor_LimitSWCheckDelay;
    sequenceMotor_LastSendTime = sequenceMotor_StartedTime;
  }
}

//*****時間が来たらリミットスイッチを見て止める*****
void observeSequenceMotor(void) {
  if (!sequenceMotor_Running) {
    return;
  }
  unsigned long now = millis();

  if (((unsigned long)(now - sequenceMotor_StartedTime) >= sequenceMotor_LimitSWCheckDelay) && ((long)(now - sequenceMotor_LimitIgnoreUntil) >= 0)) {
    bool limitSW_Pressed = sequenceMotorLimitPressed();
    if (!limitSW_Pressed) {
      sequenceMotor_CheckedLimitSWClosed = true;
    }
    if (limitSW_Pressed && sequenceMotor_CheckedLimitSWClosed) {
      if (sequenceMotor_LimitSWsToSkip > 0) {
        sequenceMotor_LimitSWsToSkip--;
        sequenceMotor_CheckedLimitSWClosed = false;
        sequenceMotor_LimitIgnoreUntil = now + sequenceMotor_LimitSWCheckDelay;
      } else {
        if (!sendSequenceMotorRpm(0)) {
          return;
        } else {
          sequenceMotor_Running = false;
          sequenceMotor_CheckedLimitSWClosed = false;

          if (sequenceMotor_Homing) {
            sequenceMotor_Homing = false;
            sequenceReloadCount = 0;
            sequenceMotor_LimitSWsToSkip = 1;
            sequenceStep = 0;
            sequenceControlButton = 255;
          } else if (sequenceMotor_ReturnPhase) {
            sequenceMotor_ReturnPhase = false;
            sequenceValve5_On = false;
            reserveL2Valve(sequenceValve5);
            pulseOn[sequenceValve5] = false;
            sequenceReloadCount++;
            if (sequenceReloadCount >= 2) {
              sequenceMotor_Homing = true;
              sequenceMotor_LimitSWsToSkip = 4;
              sequenceMotor_StartPending = true;
              sequenceMotor_PendingValve5State = false;
              sequenceMotor_StartTime = now + sequenceMotor_AfterValve3Delay;
            } else {
              // １回目の装填
              sequenceStep = 0;
              sequenceControlButton = 255;
            }
          } else {
            sequenceValve5ReturnPending = true;
            sequenceValve5ReturnTime = now + sequenceValve5ReturnDelay;
          }
          return;
        }
      }
    }
  }
  if ((unsigned long)(now - sequenceMotor_LastSendTime) >= sequenceMotor_SendInterval) {
    if (sendSequenceMotorRpm(currentSequenceMotorRPM())) {
      sequenceMotor_LastSendTime = now;
    }
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

    sequenceValve3Pending = false;
    sequenceValve1PressurePending = false;
    sequenceValve1PendingPressure = 0;
    sequenceStep = 0;
    sequenceControlButton = 255;

    sequenceBackendReady = false;
    sequenceBackendRequested = false;
    sequenceAfterValve3Pending = false;

    sequenceMotor_Running = false;
    sequenceMotor_StartPending = false;
    sequenceMotor_CheckedLimitSWClosed = false;
    sendSequenceMotorRpm(0);

    sequenceMotor_ReturnPhase = false;
    sequenceValve5ReturnPending = false;

    allPulseStop();
    stopOmniNow();
    sendAllZero();

    sendCANStop();
  }
}

//*****非常停止解除*****
// 0x03は要検討
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
// Setup
/**************************************************************************************************/
void setup() {
  byte resetCause = MCUSR;
  MCUSR = 0;
  wdt_disable();
  wdt_enable(WDTO_2S);

  Serial.begin(115200);
  Wire.begin(wioAddress);
  Wire.onReceive(receiveI2CEvent);
  Wire.onRequest(requestI2CEvent);
  // この二行は削除検討
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);

  //*****明示的にピンを設定しておく*****
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  pinMode(CAN_CS_PIN, OUTPUT);
  digitalWrite(CAN_CS_PIN, HIGH);

  pinMode(sequenceMotor_LimitSensePin, INPUT_PULLUP);
  digitalWrite(sequenceMotor_LimitGroundPin, LOW);
  pinMode(sequenceMotor_LimitGroundPin, OUTPUT);

  lastDisconnectTime = millis() - needSHAREButtonTime;
  lastWioReceiveTime = millis() - wioLinkTimeout;

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

  // デバッグ用
  Serial.print(F("sizeof(ControllerPacket) = "));
  Serial.println(sizeof(ControllerPacket));

  lastLoopStartTime = millis();
}

/**************************************************************************************************/
// loop
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

  if (consumeI2CControllerFrame()) {
    lastWioReceiveTime = millis();
    uint32_t rawValue = controller.buttons;

    if (!bitRead(lastRawButtonsState, btnBit_SQUARE)) {
      bitClear(controller.buttons, btnBit_SQUARE);
    }
    if (!bitRead(lastRawButtonsState, btnBit_CIRCLE)) {
      bitClear(controller.buttons, btnBit_CIRCLE);
    }
    if (!bitRead(lastRawButtonsState, btnBit_TRIANGLE)) {
      bitClear(controller.buttons, btnBit_TRIANGLE);
    }
    if (!bitRead(lastRawButtonsState, btnBit_CROSS)) {
      bitClear(controller.buttons, btnBit_CROSS);
    }
    if (!bitRead(lastRawButtonsState, btnBit_R2)) {
      bitClear(controller.buttons, btnBit_R2);
    }
    lastRawButtonsState = rawValue;
  }

  canRetry();
  if (regulatorPressurePending) {
    sendRegulatorPressure();
  }
  ReSendINIT();
  printDisconnect();
  pulseTimeObserve();
  observeL2Valve();
  sendPulseCan(false);

  bool isConnected = wioConnected();

  //*****接続状態のエッジ検出*****
  if (Pad1_wasConnected != isConnected) {
    if (!isConnected) {
      lastDisconnectTime = millis();
      sendAllZero();
      sendCANStop();
      allPulseStop();

      sequenceValve3Pending = false;
      sequenceValve1PressurePending = false;
      sequenceValve1PendingPressure = 0;
      sequenceStep = 0;
      sequenceControlButton = 255;

      sequenceBackendReady = false;
      sequenceBackendRequested = false;
      sequenceAfterValve3Pending = false;

      sequenceMotor_Running = false;
      sequenceMotor_StartPending = false;
      sequenceMotor_CheckedLimitSWClosed = false;
      sendSequenceMotorRpm(0);

      sequenceMotor_ReturnPhase = false;
      sequenceValve5ReturnPending = false;
      sequenceValve5_On = false;

      disconnectCountUntil_10s++;
    } else {
      unsigned long duration = (unsigned long)(millis() - lastDisconnectTime);
      if (!firstConnect) {
        firstConnect = true;
      } else {
        disconnectTimeUntil_10s += duration;
        if (duration >= needSHAREButtonTime) {
          emergencyStopLatched = true;
        }
      }
      lastButtonsState = controller.buttons;
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
    lastButtonsState = controller.buttons;
    return;
  }

  checkRegulatorPressureButtons();
  checkSTARTButton();
  checkPulseButtons();
  checkSequenceControls();
  requestLoading();
  observeSequenceMotor();
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

#include <PS4BT.h>
#include <usbhub.h>
#include <SPI.h>
#include <mcp_can.h>

//*****PWMのフルパワー*****
const int pulseMaxValue = 1000;

//*****オブジェクト宣言*****
USB Usb;
BTD Btd(&Usb);
PS4BT PS4(&Btd);
PS4BT PS4_2(&Btd);

/**************************************************************************************************/
//構造体定義・初期化
/**************************************************************************************************/

//*****機構の構造体*****
struct MotorNode {
  byte motorType;
  byte controlMode;
  byte maxValueKind;
  unsigned int maxValue;
  byte nodeNum;
  byte deviceId;
};

//*****機構の構造体の配列の初期化・代入*****
MotorNode motorNodes[] = {
  {1, 3, 0, 0, 0, 0},//FL
  {1, 3, 0, 0, 0, 1},//FR
  {1, 3, 0, 0, 0, 2},//RR
  {1, 3, 0, 0, 0, 3},//RL
  {0, 1, 0, 0, 1, 0},//2X_FL
  {0, 1, 0, 0, 1, 1},//2X_FR
  {0, 1, 0, 0, 1, 2},//2X_RR
  {0, 1, 0, 0, 1, 3},//2X_RL
  {0, 1, 0, 0, 1, 4},//4X_R
  {0, 1, 0, 0, 1, 5},//4X_L
  {0, 1, 0, 0, 2, 0},//バケツ（モーター）
  {0, 1, 0, 0, 2, 1},//バケツ（エアー？）
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
  {-0.7071f, -0.7071f, 1.0f, 2, "RR"},
  {-0.7071f, 0.7071f, 1.0f, 3, "RL"}
};

//*****エアシリンダー用の構造体*****
struct PulseButtonCommand {
  ButtonEnum button;         
  byte funcCode;
  byte targetNode;
  int pulseValue;
  unsigned int sendPulseTimeLength;
};

//*****エアシリンダー用構造体の初期化*****
PulseButtonCommand pulseCommands[] {
  {TRIANGLE,  0x01, 4, pulseMaxValue, 300},  //2X_FL
  {CROSS,     0x01, 5, pulseMaxValue, 300},  //2X_FR
  {SQUARE,    0x01, 6, pulseMaxValue, 300},  //2X_RR
  {CIRCLE,    0x01, 7, pulseMaxValue, 300},  //2X_RL
  {R1,        0x01, 8, pulseMaxValue, 300},  //4X_R
  {L1,        0x01, 9, pulseMaxValue, 300},  //4X_L
};

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

//*****共通なので構造体にしなかったやつ*****
const byte typeId = 0b011;

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

//*****バケツ用の状態監視*****
bool bucketChargeActive = false;

//*****コントローラ切断時の送信を最後に行った時刻*****
unsigned long lastSystemOffSendTime = 0;

//*****コントローラ切断時の送信のインターバル*****
const unsigned long systemOffSendInterval = 40;

//*****最後にデバッグ出力をした時刻*****
unsigned long lastDebugPrintTime = 0;

//*****RPM表示のインターバル*****
const unsigned long debugPrintInterval = 300;

//*****並進最大RPM*****
const int translateMaxRPM = 170;

//*****回転最大RPM*****
const int rotateMaxRPM = 80;

//*****最大RPM*****
const int maxRPM = 200;

//*****毎秒変化できる最大RPM*****
const int RPMLimitPerSec = 400;

//*****正規化用の係数の絶対値*****
const float coefValue = 0.7071f;

//*****スティックの値の中心値*****
const byte stickCenter = 127;

//*****スティックのデッドゾーン*****
const byte stickDeadZone = 18;

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
const char* buttonName(ButtonEnum b) {
  switch (b) {
    case TRIANGLE:
      return "TRIANGLE";
    case CIRCLE:
      return "CIRCLE";
    case CROSS:
      return "CROSS";
    case SQUARE:
      return "SQUARE";
    case L1:
      return "L1";
    case R1:
      return "R1";
    case L2:
      return "L2";
    case R2:
      return "R2";
    case SHARE:
      return "SHARE";
    case PS:
      return "PS";
    default:
      return "Unknown_Button";
  }
}

//*****canId共通関数*****
bool resolveCanId(byte targetNode, byte funcCode, unsigned long &canId) {
  if (targetNode >= motorNodesCount) {
    return false;
  } else {
    byte nodeNum = motorNodes[targetNode].nodeNum;
    byte deviceId = motorNodes[targetNode].deviceId;
    //範囲外チェック
    if (nodeNum > 3) {
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
  if (CAN0.sendMsgBuf(canId, 0, 8, data) == CAN_OK) {
    canFailCount = 0;
    return true;
  } else {
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


//*****PS4切断時に全機構OFF*****
void allSystemOff(void) {
  if (!canReady) {
    Serial.println("CAN FAILS.SYSTEMOFF SKIPPED");
    return;
  }
  //バケツ用
  if (bucketChargeActive) {
    byte targetNode = 10;
    byte funcCode = 0x01;
    unsigned long canId;
    if (!resolveCanId(targetNode, funcCode, canId)) {
      return;
    }
    byte data[8] = {1, 0, 0, 0, 0, 0, 0, 0};
    if (canSendChecker(canId, data)) {
      Serial.println("BUCKET SEND SUCCESS");
      bucketChargeActive = false;
    } else {
      Serial.println("BUCKET SEND FAILED");
    }
  }
}

//*****コントローラ切断時継続送信*****
void ReSendSystemOff(void) {
  if (!canReady) {
    return;
  } else {
    if ((unsigned long)(millis() - lastSystemOffSendTime) >= systemOffSendInterval) {
      lastSystemOffSendTime = millis();
      allSystemOff();
    }
  }
}

//*****INIT送信*****
void sendINIT(void) {
  if (canReady) {
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
      if (canSendChecker(canId, initData) == true) {
        Serial.println("INIT SEND SUCCESS");
      } else {
        Serial.println("INIT SEND FAILED");
      } 
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
  int RPMLimitPerFlame = (RPMLimitPerSec * speedSendInterval) / 1000;
  for (int i = 0; i < omniCount; i++) {
    if (downScaleRPM[i] >= actualSendValues[i]) {
      actualSendValues[i] += RPMLimitPerFlame;
      if (downScaleRPM[i] <= actualSendValues[i]) {
        actualSendValues[i] = downScaleRPM[i];
      }
    } else if (downScaleRPM[i] <= actualSendValues[i]) {
      actualSendValues[i] -= RPMLimitPerFlame;
      if (downScaleRPM[i] >= actualSendValues[i]) {
        actualSendValues[i] = downScaleRPM[i];
      }
    }
  }
}

//*****スティックの値を正規化する関数*****
float readStickRawValue(byte rawValue, bool needReverse) {
  int difference = (int)(rawValue - stickCenter);
  if (needReverse) {
    difference = -difference;
  }
  if (abs(difference) <= stickDeadZone) {
    return 0.0f;
  }
  float remainingRange = stickCenter - stickDeadZone;
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
  for (int i= 0; i < omniCount; i++) {
    if (!canReady) {
      Serial.println("CAN FAILS. RPM SEND SKIPPED");
      break;
    } else {
      int sendValue = actualSendValues[i];
      byte targetNode = omni[i].targetNode;
      byte funcCode = 0x01;
      unsigned long canId;
      if (!resolveCanId(targetNode, funcCode, canId)) {
        continue;
      }
      byte data[8] = {1, (byte)((sendValue >> 8) & 0xFF), (byte)(sendValue & 0xFF), 0, 0, 0, 0, 0};
      if (canSendChecker(canId, data)) {
        Serial.println("CAN SEND SUCCESS");
      } else {
        Serial.println("CAN SEND FAILED");
      }
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
  if((unsigned long)(millis() - lastDebugPrintTime) < debugPrintInterval) {
    return;
  }
  lastDebugPrintTime = millis();
  Serial.println("**** OMNI RPM VALUE *****");
  for (int i = 0; i < omniCount; i++) {
    Serial.print(omni[i].name);
    Serial.print(":rawValue = ");
    Serial.print(rawRPM[i]);
    Serial.print(" -> downScaleValue = ");
    Serial.println(downScaleRPM[i]);
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
  bool TRIANGLE_Clicked = PS4.getButtonClick(TRIANGLE);
  bool CROSS_Clicked = PS4.getButtonClick(CROSS);
  bool SQUARE_Clicked = PS4.getButtonClick(SQUARE);
  bool CIRCLE_Clicked = PS4.getButtonClick(CIRCLE);
  bool R1_Clicked = PS4.getButtonClick(R1);
  bool L1_Clicked = PS4.getButtonClick(L1);
  
  for (int i = 0; i < pulseCommandsCount; i++) {
    bool pulseButtonClicked = false;

    if (pulseCommands[i].button == TRIANGLE) {
      pulseButtonClicked = TRIANGLE_Clicked;
    } else if (pulseCommands[i].button == CROSS) {
      pulseButtonClicked = CROSS_Clicked;
    } else if (pulseCommands[i].button == SQUARE) {
      pulseButtonClicked = SQUARE_Clicked;
    } else if (pulseCommands[i].button == CIRCLE) {
      pulseButtonClicked = CIRCLE_Clicked;
    } else if (pulseCommands[i].button == R1) {
      pulseButtonClicked = R1_Clicked;
    } else if (pulseCommands[i].button == L1) {
      pulseButtonClicked = L1_Clicked;
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
  byte data[8] = {1, (byte)((sendValue >> 8) & 0xFF), (byte)(sendValue & 0xFF), 0, 0, 0, 0, 0};
  if (canSendChecker(canId, data)) {
    Serial.print(buttonName(pulseCommands[index].button));
    Serial.println(" PRESSED: PULSE START SEND SUCCESS");
    pulseCommandsStates[index] = true;
    pulseUntilMs[index] = millis() + pulseCommands[index].sendPulseTimeLength;
  } else {
    Serial.print(buttonName(pulseCommands[index].button));
    Serial.println(" PRESSED: PULSE START SEND FAILED");  }
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
    Serial.print(buttonName(pulseCommands[index].button));
    Serial.println(" PRESSED: PULSE STOP SEND SUCCESS");
    pulseCommandsStates[index] = false;
  } else {
    Serial.print(buttonName(pulseCommands[index].button));
    Serial.println(" PRESSED: PULSE STOP SEND FAILED");
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

//*****非常停止*****
void emergencyStop(void) {
  bool psClicked = PS4.getButtonClick(PS);
  if (psClicked && !emergencyStopLatched) {
    emergencyStopLatched = true;
    setPulsesEndTime();
    autoStopPulses();
    stopOmniNow();
    allSystemOff();
    if (!canReady) {
      Serial.println("CAN FAILS. EMERGENCY SEND SKIPPED");
    } else {
      unsigned long canId = (0x00 << 8) | 0x00;
      byte emergencyData[8] = {0};
      emergencyData[0] = 1;
      canSendChecker(canId, emergencyData);
    }
  }
}

//*****非常停止解除*****
//0x03は要検討
void unlockEmergency(void) {
  bool shareClicked = PS4.getButtonClick(SHARE);
  if (!shareClicked || !emergencyStopLatched) {
    return;
  }
  emergencyStopLatched = false;
  if (!canReady) {
    Serial.println("CAN FAILS. UNLOCK EMERGENCY SEND SKIPPED");
  } else {
    unsigned long canId = (0x03 << 8) | 0x00;
    byte unlockEmergencyData[8] = {0};
    unlockEmergencyData[0] = 1;
    canSendChecker(canId, unlockEmergencyData);
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
    if (canSendChecker(canId, data)) {
      Serial.println("ZERO SEND SUCCESS");
    } else { 
      Serial.println("ZERO SEND FAILED");
    }
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

//*****USB成功/失敗判定*****
  if (Usb.Init() == -1) {
    Serial.println("USB host did not start.");
    while (1);
  } else {
    Serial.println("USB Host Ready.");
    
  }

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
}


/**************************************************************************************************/
//loop
//切断時のエッジ検出＋ESTOPフレーム送信（B-2）はwio実装まで不明なため未実装。
/**************************************************************************************************/
void loop() {
  Usb.Task();
  canRetry();
  ReSendINIT();
  autoStopPulses();
  
//*****接続されていないときはloop先頭に戻る*****
  if (!PS4.connected()) {
    ReSendSystemOff();
    ReSendOmniStop();
    setPulsesEndTime();
    autoStopPulses();
    return;
  }
    
  emergencyStop();
  unlockEmergency();

  if (emergencyStopLatched) {
    ReSendSystemOff();
    ReSendSpeed();    
    return;
  }

  checkPulseButtons();

  float vx = readStickRawValue(PS4.getAnalogHat(RightHatX), false);
  float vy = readStickRawValue(PS4.getAnalogHat(RightHatY), true);
  float omega = readStickRawValue(PS4.getAnalogHat(LeftHatX), false);

  calcRawRPM(vx, vy, omega);
  scaleDown();
  printOmniValue();

  ReSendSpeed();
}

#include <PS4BT.h>
#include <usbhub.h>
#include <SPI.h>
#include <mcp_can.h>

//*****エアシリンダー用の最大値*****
const int airMaxValue = 1000;

//*****PWMのフルパワー*****
const int pulseMaxValue = 1000;

//*****オブジェクト宣言*****
USB Usb;
BTD Btd(&Usb);
PS4BT PS4(&Btd);

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
  {0, 1, 0, 0, 0, 0},//マブチ想定（エアシリンダー），PWM（UFOキャッチ）
  {0, 1, 0, 0, 0, 0},//マブチ想定（エアシリンダー），PWM（スタック上下）
  {0, 1, 0, 0, 0, 0},//マブチ想定（エアシリンダー），PWM（エア発射）
  {1, 3, 0, 0, 0, 0},//FL:
  {1, 3, 0, 0, 0, 1},//FR:
  {1, 3, 0, 0, 1, 2},//RR:
  {1, 3, 0, 0, 1, 3},//RL:
  {0, 1, 0, 0, 0, 4},//マブチ，PWM（投射機LAUNCH R）
  {0, 1, 0, 0, 0, 6},//マブチ，PWM（投射機LAUNCH L）
  {0, 1, 0, 0, 0, 7},//マブチ，PWM（エア発射R=AIR_PROJECTION_LAUNCH R）
  {0, 1, 0, 0, 0, 8},//マブチ，PWM（エア発射L=AIR_PROJECTION_LAUNCH L）
  {0, 1, 0, 0, 0, 9},//マブチ，PWM（バケツリフト）
  {0, 1, 0, 0, 0, 10},//UFO左右
  {0, 1, 0, 0, 0, 11},//雑巾のせ台＿右:昇降
  {0, 1, 0, 0, 0, 12},//雑巾のせ台＿左:昇降
  {0, 1, 0, 0, 0, 13},//エアシリンダー左右調整＿右
  {0, 1, 0, 0, 0, 14}//エアシリンダー左右調整＿左
};

//*****トグルボタン*****
struct ToggleButtonCommand {
  ButtonEnum button;
  byte funcCode;
  byte targetNode;
};

//*****トグルコマンドの初期化*****
ToggleButtonCommand toggleCommands[] {
  {CROSS, 0x01, 0},
  {TRIANGLE, 0x01, 1}
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
  {0.7071f, 0.7071f, 1.0f, 3, "FL"},
  {0.7071f, -0.7071f, 1.0f, 4, "FR"},
  {-0.7071f, -0.7071f, 1.0f, 5, "RR"},
  {-0.7071f, 0.7071f, 1.0f, 6, "RL"}
};

//*****エアシリンダー用の構造体*****
struct PulseButtonCommand {
  ButtonEnum modifier1;          //修飾ボタン1
  ButtonEnum modifier2;          //修飾ボタン2（修飾が1つでいい場合はmodifier1と同じ値を入れる）
  ButtonEnum button;             //クリックするボタン
  byte funcCode;
  byte targetNode;
  int pulseValue;
  unsigned int sendPulseTimeLength;
};

//*****エアシリンダー用構造体の初期化*****
PulseButtonCommand pulseCommands[] {
  {R2, R2, CROSS,    0x01, 7,  pulseMaxValue, 300},  //投射機LAUNCH R
  {L2, L2, CROSS,    0x01, 8,  pulseMaxValue, 300},  //投射機LAUNCH L
  {R2, R2, CIRCLE,   0x01, 9,  pulseMaxValue, 200},  //エア発射R
  {L2, L2, CIRCLE,   0x01, 10, pulseMaxValue, 200},  //エア発射L
  {R2, L2, TRIANGLE, 0x01, 11, pulseMaxValue, 500}   //バケツリフト（R2とL2両方）
};

//*****押している間のみ動くボタンの構造体*****
struct HoldButtonCommand {
  ButtonEnum modifier;
  bool useModifier;
  ButtonEnum button;
  byte funcCode;
  byte targetNode;
  int sendValue;  
};

//*****初期化*****
HoldButtonCommand holdCommands[] {
  {R2, false, R1, 0x01, 12, 320},   //UFO右
  {L2, false, L1, 0x01, 12, -320},  //UFO左
  {R2, true, UP, 0x01, 13, 320},    //カタパルト雑巾のせ部分_右:上昇
  {R2, true, DOWN, 0x01, 13, -320}, //カタパルト雑巾のせ部分_右:下降
  {L2, true, UP, 0x01, 14, 320},    //カタパルト雑巾のせ部分_左:上昇
  {L2, true, DOWN, 0x01, 14, -320}, //カタパルト雑巾のせ部分_左:下降
  {R2, true, RIGHT, 0x01, 15, 150}, //エアシリンダー左右調整_右:右方へ
  {R2, true, LEFT, 0x01, 15, -150}, //エアシリンダー左右調整_右:左方へ
  {L2, true, RIGHT, 0x01, 16, 150}, //エアシリンダー左右調整_左:右方へ
  {L2, true, LEFT, 0x01, 16, -150}  //エアシリンダー左右調整_左:左方へ
};

//*****角度制御用(引き)の構造体*****
struct AngleButtonCommand {
  ButtonEnum modifier;
  ButtonEnum button;
  float targetAngle;
  int maxSpeed;
  byte targetNode;
  byte funcCode;
};

//*****初期化*****
AngleButtonCommand angleCommands[] {
  {R2, CIRCLE, 0.0, 0, 0, 0}, //targetNodeとfuncCodeは不明
  {L2, CIRCLE, 0.0, 0, 0, 0}  //同上
};

//*****角度制御用（発射）の構造体*****
struct AngleReleaseButtonCommand {
  ButtonEnum modifier;
  ButtonEnum button;
  byte targetNode;
  byte funcCode;
};

//*****初期化*****
AngleReleaseButtonCommand angleReleaseCommands[] {
  {R2, SQUARE, 0, 0},
  {L2, SQUARE, 0, 0}
};

/**************************************************************************************************/
//変数定義・初期化
/**************************************************************************************************/

//*****モーターの数を臨機応変に*****
const byte motorNodesCount = sizeof(motorNodes) / sizeof(motorNodes[0]);

//*****トグルボタンの数を臨機応変に*****
const byte toggleCommandsCount = sizeof(toggleCommands) / sizeof(toggleCommands[0]);

//*****オムニ臨機応変（仮）*****
const byte omniCount = sizeof(omni) / sizeof(omni[0]);

//*****パルス機構の数を臨機応変に*****
const byte pulseCommandsCount = sizeof(pulseCommands) / sizeof(pulseCommands[0]);

//*****hold機構の数を臨機応変に*****
const byte holdCommandsCount = sizeof(holdCommands) / sizeof(holdCommands[0]);

//*****angleの機構の数を臨機応変に*****
const byte angleCommandsCount = sizeof(angleCommands) / sizeof(angleCommands[0]);

//*****angleの機構の数を臨機応変に*****
const byte angleReleaseCommandsCount = sizeof(angleReleaseCommands) / sizeof(angleReleaseCommands[0]);

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

//*****holdの送信を最後に行った時刻*****
unsigned long lastHoldSendTime[holdCommandsCount] = {0};

//*****hold送信のインターバル*****
const unsigned long holdSendInterval = 40;

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

//*****トグルボタンが押される前の状態*****
bool toggleCommandsStates[toggleCommandsCount] = {false};

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

//*****holdがアクティブだったか記憶する配列*****
bool holdCommandsStates[holdCommandsCount] = {false};

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
    case L2:
      return "L2";
    case R2:
      return "R2";
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

//*****EmergencyStop時に即時ZERO送信*****
void sendAllZeroNow(void) {
  for (int i = 0; i < omniCount; i++) {
    downScaleRPM[i] = 0;
  }
  sendSpeedCan();
}

//*****PS4切断時に全機構OFF*****
void allSystemOff(void) {
  if (!canReady) {
    Serial.println("CAN FAILS.SYSTEMOFF SKIPPED");
    return;
  }

  //TOGGLE用
  for (int i = 0; i < toggleCommandsCount; i++) {
    if (!canReady) {
      break;
    }
    if (!toggleCommandsStates[i]) {
      continue;
    }
    byte targetNode = toggleCommands[i].targetNode;
    byte funcCode = toggleCommands[i].funcCode;
    unsigned long canId;
    if (!resolveCanId(targetNode, funcCode, canId)) {
      continue;
    }
    byte data[8] = {1, 0, 0, 0, 0, 0, 0, 0};
    if (canSendChecker(canId, data)) {
      Serial.println("TOGGLE SYSTEMOFF SEND SUCCESS.");
      toggleCommandsStates[i] = false;
    } else {
      Serial.println("TOGGLE SYSTEMOFF SEND FAILED");
    }
  }

  //HOLD用
  for (int i = 0; i < holdCommandsCount; i++) {
    if (!canReady) {
      break;
    }
    if (!holdCommandsStates[i]) {
      continue;
    }
    byte targetNode = holdCommands[i].targetNode;
    byte funcCode = holdCommands[i].funcCode;
    unsigned long canId;
    if (!resolveCanId(targetNode, funcCode, canId)) {
      continue;
    }
    byte data[8] = {1, 0, 0, 0, 0, 0, 0, 0};
    if (canSendChecker(canId, data)) {
      Serial.println("HOLD SYSTEMOFF SEND SUCCESS");
      holdCommandsStates[i] = false;
    } else {
      Serial.println("HOLD SYSTEMOFF SEND FAILED");
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

//*****エアシリンダーON/OFFボタン（仮）*****
void checkToggleButtons(void) {
  for (int i = 0; i < toggleCommandsCount; i++) {    
    if (readStateChecker(toggleCommands[i].button)) {
      continue;
    }
    bool toggleClicked = PS4.getButtonClick(toggleCommands[i].button);
    if (!canReady) {
      Serial.println("CAN FAILS. TOGGLE SEND SKIPPED");
    } else {
      if (toggleClicked) {
        bool nextState = !toggleCommandsStates[i];
        int sendValue;
        if (nextState == false) {
          sendValue = 0;
        } else {
          sendValue = airMaxValue;
        }
        byte targetNode = toggleCommands[i].targetNode;
        byte funcCode = toggleCommands[i].funcCode;
        unsigned long canId;
        if (!resolveCanId(targetNode, funcCode, canId)) {
          continue;
        }
        byte data[8] = {1, (byte)((sendValue >> 8) & 0xFF), (byte)(sendValue & 0xFF), 0, 0, 0, 0, 0};
        if (canSendChecker(canId, data)) {
          Serial.println("CAN SEND SUCCESS");
          toggleCommandsStates[i] = nextState;
        } else {
          Serial.println("CAN SEND FAILED");
        }
      }
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
      int sendValue = downScaleRPM[i];
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
      sendSpeedCan();
    }
  }
}

//*****DownScaleRPMを切断時0に*****
void clearDownScaleRPM(void) {
  for (int i = 0; i < omniCount; i++) {
    downScaleRPM[i] = 0;
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

//*****エアシリンダーのボタンが押されたかチェックする関数*****
void checkPulseButtons(void) {
  bool CROSSClicked = PS4.getButtonClick(CROSS);
  bool CIRCLEClicked = PS4.getButtonClick(CIRCLE);
  bool TRIANGLEClicked = PS4.getButtonClick(TRIANGLE);
  
  for (int i = 0; i < pulseCommandsCount; i++) {
    bool modifier_1_Clicked = PS4.getButtonPress(pulseCommands[i].modifier1);
    bool modifier_2_Clicked = PS4.getButtonPress(pulseCommands[i].modifier2);
    bool pulseButtonClicked = false;

    if (pulseCommands[i].button == CROSS) {
      pulseButtonClicked = CROSSClicked;
    } else if (pulseCommands[i].button == CIRCLE) {
      pulseButtonClicked = CIRCLEClicked;
    } else if (pulseCommands[i].button == TRIANGLE) {
      pulseButtonClicked = TRIANGLEClicked;
    }
    
    if (pulseButtonClicked && modifier_1_Clicked && modifier_2_Clicked) {
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
    Serial.println("CAN SEND SUCCESS (PULSE START)");
    pulseCommandsStates[index] = true;
    pulseUntilMs[index] = millis() + pulseCommands[index].sendPulseTimeLength;
  } else {
    Serial.println("CAN SEND FAILED (PULSE START)");
  }
}

//*****毎ループ呼ばれ、終了予定時刻を過ぎたパルスを自動的に止める*****
void servicePulses(void) {
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
    Serial.println("CAN SEND SUCCESS (PULSE STOP)");
    pulseCommandsStates[index] = false;
  } else {
    Serial.println("CAN SEND FAILED (PULSE STOP)");
  }
}

//*****全パルスを強制終了させる（非常停止）*****
void cancelAllPulses(void) {
  for (int i = 0; i < pulseCommandsCount; i++) {
    if (pulseCommandsStates[i]) {
      pulseUntilMs[i] = millis();  // 終了予定時刻を「今」にして、servicePulses()に即座に処理させる
    }
  }
}

//*****押している間だけ動かす（hold）関数*****
void checkHoldButtons(void) {
  for (int i = 0; i < holdCommandsCount; i++) {
    if (!canReady) {
      Serial.println("CAN FAILS. HOLD SEND SKIPPED");
      continue;
    } else {
      bool useModifier = holdCommands[i].useModifier;
      bool modifierPressed = false;
      bool buttonPressed = PS4.getButtonPress(holdCommands[i].button);
      if (useModifier) {
        modifierPressed = PS4.getButtonPress(holdCommands[i].modifier);
      } else {
        modifierPressed = true;
      }
      if (buttonPressed && modifierPressed) {
        if ((unsigned long)(millis() - lastHoldSendTime[i]) >= holdSendInterval) {
          lastHoldSendTime[i] = millis();
          int sendValue = holdCommands[i].sendValue;
          byte targetNode = holdCommands[i].targetNode;
          byte funcCode = holdCommands[i].funcCode;
          unsigned long canId;
          if (!resolveCanId(targetNode, funcCode, canId)) {
            continue;
          }
          byte data[8] = {1, (byte)((sendValue >> 8) & 0xFF), (byte)(sendValue & 0xFF), 0, 0, 0, 0, 0};
          if (canSendChecker(canId, data)) {
            Serial.println("CAN SEND SUCCESS");
            holdCommandsStates[i] = true;
          } else {
            Serial.println("CAN SEND FAILED");
          }
        }
      } else if (holdCommandsStates[i]) {
        int sendValue = 0;
        byte targetNode = holdCommands[i].targetNode;
        byte funcCode = holdCommands[i].funcCode;
        unsigned long canId;
        if (!resolveCanId(targetNode, funcCode, canId)) {
          continue;
        }
        byte data[8] = {1, (byte)((sendValue >> 8) & 0xFF), (byte)(sendValue & 0xFF), 0, 0, 0, 0, 0};
        if (canSendChecker(canId, data)) {
          Serial.println("CAN SEND SUCCESS");
          holdCommandsStates[i] = false;
        } else {
         Serial.println("CAN SEND FAILED");
        }
      }
    }
  }
}

//*****角度制御（引き）*****
void checkAngleButtons(void) {
  bool CIRCLEPressed = PS4.getButtonClick(CIRCLE);
  for (int i = 0; i < angleCommandsCount; i++) {
    if (!canReady) {
      Serial.println("CAN FAILS. ANGLE SEND SKIPPED");
      continue;
    } else {
      bool useModifier = PS4.getButtonPress(angleCommands[i].modifier);
      if (useModifier && CIRCLEPressed) {
        //CAN送信は不明！
        sendAngleCAN(i);
      }
    }
  }
}

void sendAngleCAN(byte index) { 
  
}

//*****角度制御（放す）*****
void checkAngleReleaseButtons(void) {
  bool SQUAREPressed = PS4.getButtonClick(SQUARE);
  for (int i = 0; i < angleReleaseCommandsCount; i++) {
    if (!canReady) {
      Serial.println("CAN FAILS. ANGLE RELEASE SEND SKIPPED");
      continue;
    } else {
      bool useModifier = PS4.getButtonPress(angleReleaseCommands[i].modifier);
      if (useModifier && SQUAREPressed) {
        sendReleaseAngleCAN(i);
      }
    }
  }
}

//*****リリースCAN送信*****
void sendReleaseAngleCAN(byte index) {
  
}

//*****非常停止*****
void emergencyStop(void) {
  bool psClicked = PS4.getButtonClick(PS);
  if (psClicked && !emergencyStopLatched) {
    emergencyStopLatched = true;
    cancelAllPulses();
    sendAllZeroNow();
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
  if (!canReady) {
    Serial.println("CAN FAILS. UNLOCK EMERGENCY SEND SKIPPED");
  } else {
    if (shareClicked && emergencyStopLatched) {
      unsigned long canId = (0x03 << 8) | 0x00;
      byte unlockEmergencyData[8] = {0};
      unlockEmergencyData[0] = 1;
      canSendChecker(canId, unlockEmergencyData);
      emergencyStopLatched = false;
    }
  }
}

//*****状態読み取りの重複回避*****
bool readStateChecker(ButtonEnum button) {
  bool R2Pressed = PS4.getButtonPress(R2);
  bool L2Pressed = PS4.getButtonPress(L2);
  if (button == CROSS) {
    if (R2Pressed || L2Pressed) {
      return true;
    } else {
      return false;
    }
  } else if (button == TRIANGLE) {
    if (R2Pressed && L2Pressed) {
      return true;
    } else {
      return false;
    }
  } else { 
    return false;
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
/**************************************************************************************************/
void loop() {
  Usb.Task();
  canRetry();
  ReSendINIT();
  servicePulses();
  
//*****接続されていないときはloop先頭に戻る*****
  if (!PS4.connected()) {
    ReSendSystemOff();
    clearDownScaleRPM();
    ReSendSpeed();
    return;
  }
    
  emergencyStop();
  unlockEmergency();
  ReSendSpeed();

  if (emergencyStopLatched) {
    ReSendSystemOff();
    return;
  }

  checkToggleButtons();
  checkPulseButtons();
  checkHoldButtons();
  checkAngleButtons();
  checkAngleReleaseButtons();

  float vx = readStickRawValue(PS4.getAnalogHat(RightHatX), false);
  float vy = readStickRawValue(PS4.getAnalogHat(RightHatY), true);
  float omega = readStickRawValue(PS4.getAnalogHat(LeftHatX), false);

  calcRawRPM(vx, vy, omega);
  scaleDown();
  printOmniValue();

  ReSendSpeed();
}

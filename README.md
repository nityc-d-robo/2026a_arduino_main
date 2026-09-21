# 9/8のpushから変更内容をメモしていきたいと思う
9/8からwio用のやつはブランチを切った

## 9/9 変更内容
* wioからの受信ができるように```ControllerPacket```構造体を追加
* ボタンのbit番号を冒頭に記載．なお，今後を見据えて全ボタン分を記述
* PS4.Connectedの代わりとなる```wioConnected()```を追加
* ボタンクリック判定用の```wioButtonClicked()```を追加
* ```recieveWioData()```を追加，データが正しければ```wioBuffer```を```controlle```rにコピー
* スティック値の値域を0～255からint8_tの-127～127に変更
* getButtonClickを```wioButtonClicked(btnBit_LEFT)```などに置き換え
* ```<PS4BT.h>```と```<usbhub.h>```のインクルードを削除，それに伴ってPS4系とUSB系を完全削除<br>
<font color="red">~~**開始バイトとチェックサムは大和さんとのすり合わせなし**~~</font> ←話し合い済

## 9/10 masterでの変更をrebase, 以下にmasterでの変更を記す
* applySlewLimitのRPMLimitPerFlameが0未満にならないように，下回ると1にする処理を追加
* printMcpError()やprintCanResultName()などを```canSendChecker()```に追加し，どのようなエラーが出て"CAN FAIL"になっているかをprint
* constで```bool printOmni```を追加し，不要になった時にfalseにするだけで消せるようにしたserialprintの中身を
* ```printOmni```の表示方法をちょっと修正
* sendSpeedCanとstartPulseのsendValueを```(unsigned int)```にキャスト（こうしないとどうやら不安定になるらしい）

## 9/10 rebase追記
* speedScaleを追加し，R1で低速，L1で高速になるようにした

## 9/11 masterからrebaseを実行，以下にmasterの変更点
* 関数位置の調整（高速/低速用関数など）
* ```ReSendAllZero```を(!PS4.connected)の中に追加し，コントローラの接続が切れたときに0を送り続けるようにした
* sendINITをemergencyStopLatchedがtrueの時はreturnするようにした（非常停止時にINITしないように ※要検討）
* コントローラー切断時needSHAREButtonTime以上経過すると，SHAREボタンを押さないと動かなくなる（非常停止継続）ようにした
* ```sendCANStop()```を追加し，emergencyStop()とloop内の「接続状態のエッジ検出」のif文の中での0x00送信を共通の関数にまとめた
* unlockEmergency()の```emergencyStopLatched```のtrue/falseの位置変更・修正

## 9/11 追記
* setup()にwioSerial.beginを追加（これないと動かん）
<font color="red">ボーレートは要検討！</font>

## 9/14 masterでの変更をrebase，以下にmasterでの変更点
* clearButtons()を追加し，非常停止時に溜まっていたクリックの情報を，解除時に（SHAREクリック時に）消費して意図しないエアシリンダーの動作を防止
* バケツ用のボタンをPS4_2に変更し，2台目に対応
* LEDは多分wioではできないので削除

## 9/14 rebase後追記
* clearButtons()を以下のように変更<br>
```C++
lastButtonsState = controller.buttons;
```
* 2つ目のコントローラーの値を読む関数bucketButtonClicked()を追加<br>
<font color="red">関数の中身は送る方針が決まったら変更する</font>

## 9/15 変更点
* コメントの微修正

## 9/15 rebaseしたmasterでの変更点
* ウォッチドッグなるものを追加，フリーズ時などに自動的に再起動するように変更
* どこにも使用していないため```buttonName()```を削除
* F()マクロをSerial.printに追加
* ```translateMaxRPM```を170から150に，```rotateMaxRPM```を100から90に

## 9/15にもう一回rebaseしたやつのmasterでの変更点
* loop()が1周するまでの最大時間を記録する処理をloopに追加
* 何らかの原因ででリセットされてsetup()に戻った時，リセットされた理由をprintする処理を追加
* 先輩との調整後，```translateMaxRPM```を150から170に，```rotateMaxRPM```を90から100に変更，とりあえず確定

## 9/18 変更点
* UART完成，対応するようにコードを更新
* ただし，基盤側の問題によりI2Cに！

## 9/19 変更点 急遽I2Cに変更！
* アドレスは0x12
* Wireを導入
* UART関連は削除（一部を除く）
* 構造体は変わってないのでbitIndexなどはUARTのものを流用
* 割り込みなので```receiveWioData()```を最小限に（copyWioDataに変更）
* <font color="red">急ぎで書いたのでloopがごっちゃごちゃ　関数化は後日</font>
***
    AI曰く
* 電圧が異なる？(3.3と5)
* アドレスがHALライブラリではシフトされてる可能性があるので注意

## 9/21 変更点
```
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);
```
の二行を追加

## 9/21 rebase後追記（ソレノイド基板ができたので，各バルブに番号を振ってCANを送れるように変更）
* バルブ番号は
    * Bucket    : 0  ※ただしオムニと重複しているため，バケツ動かず
    * 4X_LEFT   : 4
    * 4X_RIGHT  : 5
    * 2X_LEFT   : 6
    * 2X_RIGHT  : 7
* エアの状態管理をボタン単位からバルブ単位に変更し，以下の配列で管理
    * ```pulseOn[]```     :
    * ```pulseSent[]```   :
    * ```pulseOffTime[]```:
* ボタン配置の変更
    * Bucket    : ```R2```, 2台目の```CIRCLE```
    * 4X_LEFT   : ```SQUARE```
    * 4X_RIGHT  : ```CIRCLE```
    * 2X_LEFT   : ```TRIANGLE```
    * 2X_RIGHT  : ```CROSS```
* ```motorNodes[]```からエアシリンダー系を削除
* ```makeCanId()```でCanId組み立てを共通化
* ```sendAirFrame()```でCANフレームを送る
* ```writeValveOn()```でpulaseOnをtrueに，```pulseTimeObserve()```でfalseにする
* ```sendPulseCan()```でエアシリンダーのCANを送る．緊急時(needZeroがtrue)は一律OFFになる
* ```allPulseStop()```で全シリンダーをOFFに
* ```pulseOn_OFF()```はpulseOnをすべてfalseにするだけ
* emergencyStop()と切断時は```allPulseStop()```（全バルブにOFFを強制送信）
* 切断中のループは```pulseOn_OFF()```のみ（CANには送らない．OFFの送信は毎ループ先頭のsendPulseCan(false)が担当）
* loop()先頭に```pulseTimeObserve()```と```sendPulseCan(false)```を配置
* setup()はpulseSentを全てtrueにしてから```allPulseStop()```を呼ぶ(!canReadyのときも再開時にOFFを送る)

## 9/22 rebase後追記
* オムニの値を0にする処理を関数化，非常停止時や切断時のstop処理の前に追加
* setupのlastDisconnectTimeを```millis()```から```millis() - needSHAREButtonTime```に変更

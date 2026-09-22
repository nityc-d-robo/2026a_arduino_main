# 9/8のpushから変更内容をメモしていきたいと思う

## 9/8 変更点
* ソレノイドの規格に対応
* ```motorNodes```の中身を現在の機構に合わせて修正
    * 2スト:2つ
    * 4スト:2つ
    * バケツ:1つ
* ボタンの再マッピング（仮）
* モーターが消えたため，```bucketChargeActive()```,```allSystemOff()```,```ReSendSystemOff()```を削除
* ```rotateMaxRPM```を80->100に変更
* ```actualSendValues```をprintOmniValue()に追加
* Serialが大変なことになってたので，とりあえずcanSendChecker()の中で定期的にログ（SUCCESS,FAIL）を送るようにした

## 9/8追加
* ```typeId```をconstではなくmotorNodes構造体に追加<br>
<font color="red">注：typeIdが変わるとその下のビット演算が変わるのでそこを留意する</font>
* emergencyStop()に```sendAllZero()```を追加

## 9/10 変更点
* applySlewLimitのRPMLimitPerFlameが0未満にならないように，下回ると1にする処理を追加
* printMcpError()やprintCanResultName()などを```canSendChecker()```に追加し，どのようなエラーが出て"CAN FAIL"になっているかをprint
* constで```bool printOmni```を追加し，不要になった時にfalseにするだけで消せるようにしたserialprintの中身を
* ```printOmni```の表示方法をちょっと修正
* sendSpeedCanとstartPulseのsendValueを```(unsigned int)```にキャスト（こうしないとどうやら不安定になるらしい）

## 9/10 追加
* ```speedScale```を追加し，R1で低速，L1で高速になるようにした

## 9/11 変更点
* 関数位置の調整（高速/低速用関数など）
* ```ReSendAllZero```を(!PS4.connected)の中に追加し，コントローラの接続が切れたときに0を送り続けるようにした
* sendINITをemergencyStopLatchedがtrueの時はreturnするようにした（非常停止時にINITしないように ※要検討）
* コントローラー切断時needSHAREButtonTime以上経過すると，SHAREボタンを押さないと動かなくなる（非常停止継続）ようにした
* ```sendCANStop()```を追加し，emergencyStop()とloop内の「接続状態のエッジ検出」のif文の中での0x00送信を共通の関数にまとめた
* unlockEmergency()の```emergencyStopLatched```のtrue/falseの位置変更・修正

## 9/14 変更点
* ```clearButtons()```を追加し，非常停止時に溜まっていたクリックの情報を，解除時に（SHAREクリック時に）消費して意図しないエアシリンダーの動作を防止
* emergencyStopLatchedの状態に合わせてコントローラーのLEDを赤/青になるように変更（おそらくwio側では実装しない/できない）
* バケツ用のボタンをPS4_2に変更し，2台目に対応

## 9/14 追記
* ウォッチドッグなるものを追加，フリーズ時などに自動的に再起動するように変更
* どこにも使用していないため```buttonName()```を削除

## 9/15 変更点
* F()マクロをSerial.printに追加
* ```translateMaxRPM```を170から150に，```rotateMaxRPM```を100から90に

## 9/15 追記
* PS4_2の```clearButtons()```への対応
* PS4_2接続時に緑色LEDが点灯するようにした
* loop内の```setLEDColor()```などの位置変更
* loop()が1周するまでの最大時間を記録する処理をloopに追加
* 何らかの原因ででリセットされてsetup()に戻った時，リセットされた理由をprintする処理を追加
* 先輩との調整後，```translateMaxRPM```を150から170に，```rotateMaxRPM```を90から100に変更，とりあえず確定

## 9/18 変更点
* 切断時に確実に０を送れるように一部のReSendAllZero()を```sendAllZero()```に変更
* コントローラーが切断されるとsetLEDが何もしないようにした

## 9/21 変更点（ソレノイド基板ができたので，各バルブに番号を振ってCANを送れるように変更）
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

## 9/22 変更点
* オムニの値を0にする処理を関数化，非常停止時や切断時のstop処理の前に追加
* setupのlastDisconnectTimeを```millis()```から```millis() - needSHAREButtonTime```に変更

## 9/22 軽微な修正
* ```pulseStopInterval```にconstをつけた
* ```PulseCommandsCommand```からfuncCode,pulsevalueを削除
* ```PulseCommandsCommand```のtargetNodeをvalveNumに変更

## 9/22 さらに追記
* loop()のif(emmergency~)の中にも```lastButtonsState = controller.buttons```を追加
* loop()内の```emergencyStop()```と```unlockEmergency()```の順番を入れ替えた
* canRetry()内に```actualSendValues```を0にするループを追加
* BucketのvalveNumを3番に決定
* ```pulseCommands[]```からBucketを削除
* バケツはR2で上下
* ```checkBucket()```を追加，R2を押すたびに1/0を切り替え
* ```pulseTimeObserve()```の中でバケツを自動OFFのタイマーから除外
* rotateMaxRPMを140に変更
* clearButtons()のPS4_2.getButtonClick(CIRCLE)を削除，PS4.getButtonClick(R2)に変更
* ```checkBucket()```を追加，R2を押すたびに1/0を切り替え

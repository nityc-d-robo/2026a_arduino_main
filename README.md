# 9/8のpushから変更内容をメモしていきたいと思う

## 9/8 変更点
* ソレノイドの規格に対応
* motorNodesの中身を現在の機構に合わせて修正
    * 2スト:2つ
    * 4スト:2つ
    * バケツ:1つ
* ボタンの再マッピング（仮）
* モーターが消えたため，bucketChargeActive(),allSystemOff(),ReSendSystemOff()を削除
* rotateMaxRPMを80->100に変更
* actualSendValuesをprintOmniValue()に追加
* Serialが大変なことになってたので，とりあえずcanSendChecker()の中で定期的にログ（SUCCESS,FAIL）を送るようにした

## 9/8追加
* typeIdをconstではなくmotorNodes構造体に追加<br>
<font color="red">注：typeIdが変わるとその下のビット演算が変わるのでそこを留意する</font>
* emergencyStop()にsendAllZero()を追加

## 9/10 変更点
* applySlewLimitのRPMLimitPerFlameが0未満にならないように，下回ると1にする処理を追加
* printMcpError()やprintCanResultName()などをcanSendCheckerに追加し，どのようなエラーが出て"CAN FAIL"になっているかをprint
* constでbool printOmniを追加し，不要になった時にfalseにするだけで消せるようにしたserialprintの中身を
* printOmniの表示方法をちょっと修正
* sendSpeedCanとstartPulseのsendValueを(unsigned int)にキャスト（こうしないとどうやら不安定になるらしい）

## 9/10 追加
* speedScaleを追加し，R1で低速，L1で高速になるようにした

## 9/11 変更点
* 関数位置の調整（高速/低速用関数など）
* ReSendAllZeroを(!PS4.connected)の中に追加し，コントローラの接続が切れたときに0を送り続けるようにした
* sendINITをemergencyStopLatchedがtrueの時はreturnするようにした（非常停止時にINITしないように ※要検討）
* コントローラー切断時needSHAREButtonTime以上経過すると，SHAREボタンを押さないと動かなくなる（非常停止継続）ようにした
* sendCANStop()を追加し，emergencyStop()とloop内の「接続状態のエッジ検出」のif文の中での0x00送信を共通の関数にまとめた
* unlockEmergency()のemergencyStopLatchedのtrue/falseの位置変更・修正

## 9/14 変更点
* clearButtons()を追加し，非常停止時に溜まっていたクリックの情報を，解除時に（SHAREクリック時に）消費して意図しないエアシリンダーの動作を防止
* emergencyStopLatchedの状態に合わせてコントローラーのLEDを赤/青になるように変更（おそらくwio側では実装しない/できない）
* バケツ用のボタンをPS4_2に変更し，2台目に対応

## 9/14 追記
* ウォッチドッグなるものを追加，フリーズ時などに自動的に再起動するように変更
* どこにも使用していないためbuttonName()を削除

## 9/15 変更点
* F()マクロをSerial.printに追加
* translateMaxRPMを170から150に，rotateMaxRPMを100から90に

## 9/15 追記
* PS4_2のclearButtons()への対応
* PS4_2接続時に緑色LEDが点灯するようにした
* loop内のsetLEDColor()などの位置変更
* loop()が1周するまでの最大時間を記録する処理をloopに追加
* 何らかの原因ででリセットされてsetup()に戻った時，リセットされた理由をprintする処理を追加
* 先輩との調整後，translateMaxRPMを150から170に，rotateMaxRPMを90から100に変更，とりあえず確定
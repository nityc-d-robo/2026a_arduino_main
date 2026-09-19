# 9/8のpushから変更内容をメモしていきたいと思う
9/8からwio用のやつはブランチを切った

## 9/9 変更内容
* wioからの受信ができるようにControllerPacket構造体を追加
* ボタンのbit番号を冒頭に記載．なお，今後を見据えて全ボタン分を記述
* PS4.Connectedの代わりとなるwioConnected()を追加
* ボタンクリック判定用のwioButtonClicked()を追加
* recieveWioData()を追加，データが正しければwioBufferをcontrollerにコピー
* スティック値の値域を0～255からint8_tの-127～127に変更
* getButtonClickをwioButtonClicked(btnBit_LEFT)などに置き換え
* PS4BT.hとusbhub.hのインクルードを削除，それに伴ってPS4系とUSB系を完全削除<br>
<font color="red">~~**開始バイトとチェックサムは大和さんとのすり合わせなし**~~</font> ←話し合い済

## 9/10 masterでの変更をrebase, 以下にmasterでの変更を記す
* applySlewLimitのRPMLimitPerFlameが0未満にならないように，下回ると1にする処理を追加
* printMcpError()やprintCanResultName()などをcanSendCheckerに追加し，どのようなエラーが出て"CAN FAIL"になっているかをprint
* constでbool printOmniを追加し，不要になった時にfalseにするだけで消せるようにした
* printOmniの表示方法をちょっと修正
* sendSpeedCanとstartPulseのsendValueを(unsigned int)にキャスト（こうしないとどうやら不安定になるらしい）

## 9/10 rebase追記
* speedScaleを追加し，R1で低速，L1で高速になるようにした

## 9/11 masterからrebaseを実行，以下にmasterの変更点
* 関数位置の調整（高速/低速用関数など）
* ReSendAllZeroを(!PS4.connected)の中に追加し，コントローラの接続が切れたときに0を送り続けるようにした
* sendINITをemergencyStopLatchedがtrueの時はreturnするようにした（非常停止時にINITしないように ※要検討）
* コントローラー切断時needSHAREButtonTime以上経過すると，SHAREボタンを押さないと動かなくなる（非常停止継続）ようにした
* sendCANStop()を追加し，emergencyStop()とloop内の「接続状態のエッジ検出」のif文の中での0x00送信を共通の関数にまとめた
* unlockEmergency()のemergencyStopLatchedのtrue/falseの位置変更・修正

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
* どこにも使用していないためbuttonName()を削除
* F()マクロをSerial.printに追加
* translateMaxRPMを170から150に，rotateMaxRPMを100から90に

## 9/15にもう一回rebaseしたやつのmasterでの変更点
* loop()が1周するまでの最大時間を記録する処理をloopに追加
* 何らかの原因ででリセットされてsetup()に戻った時，リセットされた理由をprintする処理を追加
* 先輩との調整後，translateMaxRPMを150から170に，rotateMaxRPMを90から100に変更，とりあえず確定

## 9/18 変更点
* UART完成，対応するようにコードを更新
* ただし，基盤側の問題によりI2Cに！

## 9/19 変更点 急遽I2Cに変更！
* アドレスは0x12
* Wireを導入
* UART関連は削除（一部を除く）
* 構造体は変わってないのでbitIndexなどはUARTのものを流用
* 割り込みなのでreceiveWioData()を最小限に（copyWioDataに変更）
* <font color="red">急ぎで書いたのでloopがごっちゃごちゃ　関数化は後日</font>
***
    AI曰く
* 電圧が異なる？(3.3と5)
* アドレスがHALライブラリではシフトされてる可能性があるので注意
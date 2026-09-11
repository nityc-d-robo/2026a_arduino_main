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
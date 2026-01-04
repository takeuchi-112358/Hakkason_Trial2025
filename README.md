# Hakkason_Trial2025
ハッカソントライアル2025参加作品

## Functions
ロボットがあなたの顔を覚えてあなたを追いかけます
あなたが笑顔になるとロボットも喜びます(表情の変化、腕振り)
親ペンギンに子ペンギンが追従します

### Equipment
- Turtlebot3 burger
- RaspberryPi4
- OpenCR
- Realsense
- Arduino
- Toio

### Requirement
ROS環境(本プロジェクトではROS1 noeticで作成)
Realsense-ROS
Arduino IDE

### Preparation
- [こちらのサイト](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) にしたがってturtlebot及びOpenCRの設定を行ってください。
- 本リポジトリのモデルに従い組み立てを行います。必要に応じて3Dプリンタ等を利用して骨格を作成してください。
- Realsense-ROSパッケージをインストールします。バージョン等の依存関係によりapt installが行えない場合はソースからのビルドを行ってください。
- Arduino IDEをインストールし、.inoファイルをArduinoに書き込んでください。

## Usage
ROSを使用するためのIPの設定を行います。
```bash
   ip a
```
で表示された無線LANのinetの欄のipアドレスを控えてください。(多くの場合wlan0が該当します)これをROS_IPに使用します。
もしinetが表示されておらずinet6のみが表示されている場合
```bash
   sudo dhclient -v wlan0
```
でinetのipを取得してください。

ipを取得したら以下のコマンドでROS_MASTER_URIおよびROS_IPを設定します。
```bash
   export ROS_MASTER_URI=(your ip)
   export ROS_IP=(your ip)
   source ~/.bashrc
```
roscoreを起動したら別ターミナルでturtlebot_bringupを起動します。
```bash
   rosrun turtlebot3_bringup turtlebot3_robot.launch
```
更に別のターミナルを開き顔認識及び移動パッケージを起動してください。
```bash
   rosrun face_tracking face_tracking_node.py
```
```bash
   rosrun pen_lavot PenLavot.py
```
顔認識についてはRealsenseの起動が必要になるので、face_tracking_node.pyを起動した際のターミナルの指示にしたがって起動してください。
ディスプレイ、サーボモータに関してはROSトピックを受け取って自動的に動作します。

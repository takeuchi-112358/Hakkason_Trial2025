# Hakkason_Trial2025
ハッカソントライアル2025参加作品

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
で表示された無線LANのinetの欄のipアドレスを控えてください。(多くの場合wlan0が
該当します多これをROS_IPに使用します。
もしinetが表示されていない場合
catkin_ws以下にROS関係のパッケージがあるので以下はすべてこのディレクトリで行います。
```bash
   cd catkin_ws
```

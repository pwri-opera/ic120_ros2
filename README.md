# ic120_ros
OPERA対応クローラダンプIC120の土木研究所公開ROS2パッケージ群

## 概説
- 国立研究開発法人土木研究所が公開するOPERA（Open Platform for Eathwork with Robotics Autonomy）対応のクローラダンプであるIC120用のROS2パッケージ群である
- 本パッケージに含まれる各launchファイルを起動することで、実機やシミュレータを動作させるのに必要なROSノード群が立ち上がる
- 動作確認済のROS2 Version : ROS Humble Hawksbill + Ubuntu 22.04 LTS

## ビルド方法
- ワークスペースの作成（既にwsを作成済の場合は不要．以下、新規作成するワークスペースの名称を"ic120_ros2_ws"と仮定して表記）
  ```bash
  $ cd ~/
  $ mkdir --parents ic120_ros2_ws/src
  $ cd ic120_ros2_ws
  $ colcon build
  ```

- 依存パッケージ群をインストールした上でパッケージのビルドと自分のワークスペースをインストール環境上にOverlayする  
  [vcstoolに関する参考サイト](https://qiita.com/strv/items/dbde72e20a8efe62ef95)
  ```bash
  $ cd ~/ic120_ros2_ws/src
  $ git clone https://github.com/pwri-opera/ic120_ros2.git
  $ cd ~/ic120_ros2_ws
  $ colcon build && source install/setup.bash
  ```

## kakaaaaaa

## 含有するサブパッケージ
### ic120_bringup:
- ic120の実機を動作させる際に必要なノード群を一括起動するためのlaunch用のサブパッケージ

### ic120_description:
- ic120用のロボットモデルファイル(urdf, xacro含む)群

### ic120_gazebo:
- ic120をgazeboシミュレータ上で動作させるのに必要なノード群を一括起動するためのlaunch用のサブパッケージ

### ic120_navigation:
- ic120の自動走行のためのライブラリ
- [Nav2](https://navigation.ros.org/) に準拠

### ic120_unity:
- OPERAのUnityシミュレータと連携するために必要なノード群を一括起動するためのlaunch用のサブパッケージ

## 各ROSノード群の起動方法
- 実機動作に必要なROSノード群の起動方法
  ```bash
  $ ros2 launch ic120_bringup ic120_vehicle.launch.py
  ```
- 実機遠隔操作に必要なROSノード群の起動方法
  ```bash
  $ ros2 launch ic120_bringup ic120_remote.launch.py
  ```
- Unityシミュレータとの連携に必要なROSノード群の起動方法
  ```bash
  $ ros2 launch ic120_unity ic120_standby_ekf.launch.py
  ```

### Unity版IC120のナビゲーション設定

`ic120_standby_ekf.launch.py` は、Unity側の旋回上限 **0.08726646 rad/s（5度/秒）** に合わせた設定を読み込みます。DWB・速度平滑化・Spin回復動作の上限を揃え、低速移動とその場旋回を許可しています。実機用の設定は変更していません。

- ROS 2 Humbleでは `progress_checker_plugin`（単数形）で `PoseProgressChecker` を指定し、並進だけでなく旋回も進捗として判定します。
- 回復動作の `global_frame` は、衝突判定に使うローカルコストマップと同じ `<common_prefix>_tf/odom` です。`map` 座標の姿勢をそのままローカルグリッドに照合しないようにします。
- `NavigateToPose` 用のBTは `ic120_navigate_to_pose_w_replanning_and_recovery.xml` を使用します。90度のSpinは上限速度でも最低18秒必要なため、許容時間を45秒にしています。
- Unity側の `/clock` とセンサーメッセージは同じ物理ステップの時刻で配信してください。TFの許容時間を広げて時刻のずれを隠す設定にはしていません。
 
### ハードウェアシステム
![ic120_hardware_system](https://user-images.githubusercontent.com/24404939/159679362-c82d3720-089a-47f1-9251-a02f9e8a7fd4.jpg)

## ソフトウェアシステム
### ros2 launch ic120_unity ic120_standy.launch.py実行時のノード/トピックパイプライン（rqt_graph）
![unity_launch](https://user-images.githubusercontent.com/24404939/175253675-c9fe28b6-398b-46c4-963f-aad9289c3c9b.png)

### ros2 launch ic120_bringup ic120_vehicle.launch.py実行時のノード/トピックパイプライン（rqt_graph）
![Screenshot from 2022-07-22 17-20-43](https://user-images.githubusercontent.com/24404939/180416808-acab38d4-04b0-48aa-a50f-15a0c7be0808.png)

### ros2 launch ic120_launch ic120_remote.launch.py実行時のノード/トピックパイプライン（rqt_graph）
![Screenshot from 2022-07-22 17-18-45](https://user-images.githubusercontent.com/24404939/180416944-6568d3dc-23ad-4508-9e3a-55f378f093f1.png)

# rsf_simulator

Gazebo (Ignition Fortress) 上に RSF ロボットとワールドを起動する。

## 起動

```bash
ros2 launch rsf_simulator rsf_simulator.launch.py
```

## ワールドの選択

| `world` |
|---|---|
| `tsudanuma2-3`（デフォルト）|
| `tsudanuma` |

```bash
ros2 launch rsf_simulator rsf_simulator.launch.py world:=tsudanuma
```

`worlds/` に `<名前>.sdf` を置き、launch の `choices` に名前を追加すれば選択肢を増やせる。

`interlace:=N`（1〜20、デフォルト 1）で YVT-35LX のインターレースを再現する。各 20Hz フレームの照射方位が 6°/N ずつ巡回シフトし、N フレーム蓄積で水平密度が N 倍になる。

## topic

`ros_gz_bridge` が以下を ROS 側に橋渡しする。

| ROS トピック | 向き | 内容 |
|---|---|---|
| `/clock` | output | シミュレーション時刻 |
| `/rsf/hokuyo_cloud2` | output | 3D LiDAR の点群|
| `/rsf/imu` | output | IMU |
| `/rsf/nav_sat_fix` | output | GNSS |
| `/rsf/rsf_odom` | output | オドメトリ |
| `/cmd_vel` | input | 速度指令 |

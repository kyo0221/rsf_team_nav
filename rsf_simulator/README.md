# rsf_simulator

Gazebo (Ignition Fortress) 上に RSF ロボットとワールドを起動する。

## 起動

```bash
ros2 launch rsf_simulator rsf_simulator.launch.py
```

ロボットは world 側に include されているので、これだけでロボットごと立ち上がる。

## ワールドの選択

| `world` | 内容 |
|---|---|
| `tsudanuma2-3`（既定） | Gazebo Building Editor で作った屋内環境 |
| `tsudanuma` | 占有格子地図から [map2sdf](https://github.com/kyo0221/map2sdf) で生成した屋外環境（154m × 124m） |

```bash
ros2 launch rsf_simulator rsf_simulator.launch.py world:=tsudanuma
```

`worlds/` に `<名前>.sdf` を置き、launch の `choices` に名前を追加すれば選択肢を増やせる。

## 出入りするトピック

`ros_gz_bridge` が以下を ROS 側に橋渡しする。

| ROS トピック | 向き | 内容 |
|---|---|---|
| `/clock` | 出 | シミュレーション時刻 |
| `/rsf/hokuyo_cloud2` | 出 | 3D LiDAR の点群 |
| `/rsf/imu` | 出 | IMU |
| `/rsf/nav_sat_fix` | 出 | GNSS |
| `/rsf/rsf_odom` | 出 | オドメトリ |
| `/cmd_vel` | 入 | 速度指令 |

## ワールドを追加するときの注意

`gpu_lidar` の描画には Sensors システムが必要で、これは **world 直下**に置く。

```xml
<plugin filename="libignition-gazebo-sensors-system.so" name="ignition::gazebo::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
```

ロボットモデル側にも同じ宣言があるとレンダースレッドが 2 本起動し、Gazebo が segfault する。
`models/orne_boxF/orne_boxF.sdf` からは宣言を外してあるので、world 側に必ず入れること。

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

world ファイルには、環境そのものとは別に **本機を出すための追加**が必要である。
`worlds/tsudanuma.sdf` では末尾にコメントで囲んだブロックとしてまとめてある。

```xml
    <!-- ここから下は rsf_simulator 固有の追加 -->
    <plugin filename="libignition-gazebo-sensors-system.so" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <include>
      <uri>package://rsf_simulator/models/orne_boxF</uri>
      <pose>x y z 0 0 yaw</pose>
    </include>
  </world>
```

- **Sensors システム**は `gpu_lidar` の描画に必要で、world 直下に置く。
  `models/orne_boxF/orne_boxF.sdf` 側では宣言していないので、world 側に必ず入れること。
  両方に宣言があるとレンダースレッドが 2 本起動して Gazebo が segfault する
- **ロボットの `<include>`** は自由空間の座標に置く。地面に埋まらないよう z は少し上げる

外部ツールで生成した環境を持ち込む場合、生成物にはロボットが含まれない。
上記ブロックを足すのは本パッケージの責務であり、環境を作り直したときは足し直す。

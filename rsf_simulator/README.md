# rsf_simulator

Gazebo (Ignition Fortress / Gazebo Harmonic) 上に RSF ロボットとワールドを起動する。

## 起動

```bash
ros2 launch rsf_simulator rsf_simulator.launch.py
```

## ワールドの選択

| `world` |
|---|
| `tsudanuma2-3`（デフォルト）|
| `tsudanuma` |

```bash
ros2 launch rsf_simulator rsf_simulator.launch.py world:=tsudanuma
```

`worlds/` に `<名前>.sdf` を置き、launch の `choices` に名前を追加すれば選択肢を増やせる。

## 3D LiDAR の表現

YVT-35LX を非蓄積モード相当（水平 36 方位 = 6° 間隔、垂直 74 本、20Hz）の
`gpu_lidar` で模擬し、Gazebo の点群をそのまま `/rsf/hokuyo_cloud2` に橋渡しする。

実機のインターレース（フレームごとに照射方位を巡回シフトして蓄積密度を上げる機能）は
再現しない。下流（pointcloud_to_laserscan → emcl2 / nav2 costmap）はフレーム単位で
処理するためインターレースの密度向上が個々の出力に現れず、一方で細グリッド走査は
GPU レンダリング負荷が約 4 倍になり、RViz や Gazebo GUI と同時使用すると RTF が
大きく低下するため（実測 0.61 → 0.97）、nav2 の動作検証というシミュレータの目的に
対して割に合わない。インターレースの実装（細グリッド + 間引きノード方式）が必要に
なった場合は git 履歴の `feat/interlace` ブランチ時代の実装を参照のこと。

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

トピック名とフレーム名は実機ドライバ `hokuyo_rsf` の設定に揃えてあり、
実機とシミュレータで下流ノードの設定を変えずに済むようにしてある。

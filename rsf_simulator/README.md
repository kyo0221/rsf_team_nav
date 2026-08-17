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

## インターレース

`interlace:=N` で YVT-35LX のインターレースを再現する。各 20Hz フレームの照射方位が
6°/N ずつ巡回シフトし、N フレーム蓄積で水平密度が N 倍になる。

```bash
ros2 launch rsf_simulator rsf_simulator.launch.py interlace:=4
```

実装は「細グリッドで撮って位相をずらして間引く」方式である。SDF の LiDAR は
水平 141 本（1.5° 間隔 = 6°/4）で走査し、`interlace_decimate_node` が
フレームごとに 4 本おきの列を位相をずらして抜き出して 36 本（6° 間隔）にする。
センサリンクは静止したままなので、URDF の `rsf_hokuyo3d_joint`（fixed）と一致する。

| `interlace` | 1 フレームの方位数 | 蓄積後の方位間隔 | 蓄積フレーム数 |
|---|---|---|---|
| `1`（デフォルト） | 36 | 6.0° | 1 |
| `2` | 36 / 35 | 3.0° | 2 |
| `4` | 36 / 35 / 35 / 35 | 1.5° | 4 |

位相をずらした列は画角の端で 1 本外れるため、フレームによって方位数が 36 と 35 で
入れ替わる。実機と同じく機械的な画角は固定である。

`interlace` は SDF の細グリッド倍率（141 本 = 36 + 35×4 より 4）の約数でなければならない。
倍率を変えるときは `models/orne_boxF/orne_boxF.sdf` の `<horizontal><samples>` を
`36 + 35×M` にし、launch の `choices` を M の約数に合わせる。`M=8`（281 本）は
ROS ブリッジで取りこぼしが出て 17Hz まで落ちるため、4 を上限としている。

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

点群は間引き後の 1 本だけを公開する。Gazebo の細グリッド点群はブリッジで
`/rsf/hokuyo3d/points_raw` にリマップした `interlace_decimate_node` 専用の入力である。

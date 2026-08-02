# rsf_navigation_executor

emcl2 による自己位置推定と Navigation2 による経路追従、およびウェイポイント走行。

3D LiDAR の点群は `pointcloud_to_laserscan` で 2D の `/scan` に変換してから使う。

## 1. 地図を作る（SLAM）

```bash
ros2 launch rsf_navigation_executor slam.launch.py
```

`slam_toolbox` が起動する。手動操縦で走り回ったあと保存する。

```bash
ros2 run nav2_map_server map_saver_cli -f maps/<名前>
```

| 引数 | 既定値 |
|---|---|
| `use_sim_time` | `true` |

このlaunchは `slam_toolbox` だけを起動する。`/scan` は含まれないので、別端末で
`pointcloud_to_laserscan` を起動しておく。

```bash
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
  --ros-args -r __node:=pointcloud_to_laserscan \
  --params-file $(ros2 pkg prefix --share rsf_navigation_executor)/config/pointcloud_to_laserscan_params.yaml \
  -r cloud_in:=/rsf/hokuyo_cloud2 -r scan:=/scan
```

パラメータファイルのキーが `pointcloud_to_laserscan` なので、ノード名を合わせないと設定が読まれない。

## 2. 自己位置推定とナビゲーション

```bash
ros2 launch rsf_navigation_executor navigation.launch.py
```

`pointcloud_to_laserscan` / `map_server` / `emcl2` / `nav2` / `lifecycle_manager` が起動する。

| 引数 | 既定値 |
|---|---|
| `map` | `maps/tsudanuma2-3.yaml` |
| `use_sim_time` | `true` |
| `autostart` | `true` |

別の地図を使う場合:

```bash
ros2 launch rsf_navigation_executor navigation.launch.py \
  map:=$(ros2 pkg prefix --share rsf_navigation_executor)/maps/<名前>.yaml
```

起動後、RViz の 2D Pose Estimate で初期位置を与える。

## 3. ウェイポイントを記録する

`navigation.launch.py` を起動した状態で:

```bash
ros2 launch rsf_navigation_executor waypoint_recording.launch.py
```

手動操縦で走らせ、通過させたい地点で以下を呼ぶ。`mcl_pose` の現在推定値が記録される。

```bash
ros2 service call /waypoint_recorder/record std_srvs/srv/Trigger   # 現在地を1点追加
ros2 service call /waypoint_recorder/save   std_srvs/srv/Trigger   # ファイルに書き出す
```

| 引数 | 既定値 |
|---|---|
| `output_file` | `config/recorded_waypoints.yaml` |

## 4. ウェイポイント走行

```bash
ros2 launch rsf_navigation_executor waypoint_navigation.launch.py
```

```bash
ros2 service call /waypoint_navigator/start  std_srvs/srv/Trigger
ros2 service call /waypoint_navigator/pause  std_srvs/srv/Trigger
ros2 service call /waypoint_navigator/resume std_srvs/srv/Trigger
```

| 引数 | 既定値 |
|---|---|
| `waypoints_file` | `config/waypoints.yaml` |

### waypoints.yaml の書式

```yaml
loop: false
waypoints:
  - {x: 3.0, y: 0.0, yaw: 0.0, speed_limit: 50.0}
  - {x: 3.0, y: 3.0, yaw: 1.5708, checkpoint: true}
  - {x: 0.0, y: 3.0, yaw: 3.14159}
```

| キー | 説明 |
|---|---|
| `x`, `y`, `yaw` | 目標姿勢（map 座標系、yaw はラジアン） |
| `speed_limit` | 次の点までの速度上限（省略可） |
| `checkpoint` | `true` なら到達必須。省略時はスキップ可 |
| `loop` | `true` で最終点から先頭に戻る |

## 設定ファイル

| ファイル | 内容 |
|---|---|
| `config/emcl2_params.yaml` | 自己位置推定 |
| `config/nav2_params.yaml` | コストマップ・プランナ・コントローラ |
| `config/pointcloud_to_laserscan_params.yaml` | 点群から切り出す高さ（LiDAR 基準 ±0.3 m） |
| `config/slam_toolbox_params.yaml` | SLAM |

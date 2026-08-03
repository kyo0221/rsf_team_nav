# rsf_navigation_executor

emcl2 による自己位置推定と Navigation2 による経路追従、およびウェイポイント走行。

3D LiDAR の点群は `pointcloud_to_laserscan` で 2D の `/scan` に変換してから使う。

## 1. 地図を作る

**オンライン SLAM は行わない。** 地図は走行後にオフラインで作る。

```
rosbag 取得（実機で走行、/rsf/hokuyo_cloud2 と /rsf/imu を記録）
  -> GLIM で 3D 地図を生成（開発機で実行。車載 PC では回さない）
  -> 3D 点群を地面基準の高さ帯でスライスして 2D 占有格子へ投影
  -> maps/<名前>.pgm + maps/<名前>.yaml
```

3D から 2D への投影は、`pointcloud_to_laserscan` が切り出す帯
（LiDAR 基準 ±0.3m = 平地でワールド z 0.05〜0.65m）と一致させる必要がある。
全高を単純に投影すると、樹冠や庇のようにスキャンが観測しない占有セルが載り、
emcl2 の尤度場が系統的に狂う。地面高の起伏があるため、絶対高ではなく
**地面基準**で切ること。

**この投影ツールは未実装。** 現在 `maps/` にある `tsudanuma2-3.*` は
以前 slam_toolbox で作ったもので、これのみが利用可能な地図である。

`maps/` に置く 2D 地図は、`rsf_simulator` の world 生成にも使える
（[map2sdf](https://github.com/kyo0221/map2sdf) で SDF に変換できる）。
同じ 2D 地図から world と localization 用地図の両方を作れば、両者が原理的にずれない。

## 2. 自己位置推定とナビゲーション

**前提**: この launch は TF を配信しない。先に `rsf_bringup` を起動しておくこと。
`robot_state_publisher`（`base_footprint` 以下の TF）と `tf_odom_to_footprint`
（`rsf_odom` -> `base_footprint`）が無いと、コストマップが
`Invalid frame ID "rsf_odom"` で止まりロボットは動かない。

```bash
ros2 launch rsf_simulator rsf_simulator.launch.py   # 端末1
ros2 launch rsf_bringup rsf_bringup.launch.py       # 端末2
ros2 launch rsf_navigation_executor navigation.launch.py   # 端末3
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
| `output_file` | `recorded_waypoints.yaml`（launch を叩いたカレントディレクトリ） |

書き出したファイルは `config/` にコピーして git 管理下に置くこと。
`output_file` に絶対パスを渡せば直接そこへ書ける。

```bash
ros2 launch rsf_navigation_executor waypoint_recording.launch.py \
  output_file:=$HOME/rsf_ws/src/rsf_navigation_executor/config/course_a.yaml
```

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
| `checkpoint` | `true` なら到達必須。到達すると停止して `resume` を待ち、失敗すると同じ点を再試行する。省略時は失敗してもスキップして次へ進む |
| `loop` | `true` で最終点から先頭に戻る |

## 設定ファイル

| ファイル | 内容 |
|---|---|
| `config/emcl2_params.yaml` | 自己位置推定 |
| `config/nav2_params.yaml` | コストマップ・プランナ・コントローラ |
| `config/pointcloud_to_laserscan_params.yaml` | 点群から切り出す高さ（LiDAR 基準 ±0.3 m） |
| `behavior_trees/*.xml` | BackUp を外した behavior tree |

### behavior tree を差し替えている理由

本機の LiDAR は水平 FOV が ±105° で後方 150° が未計測なので、`BackUp` リカバリは
見えていない方向へ後退することになる。そのため nav2 既定の behavior tree から
`BackUp` を除いたものを `behavior_trees/` に置き、`navigation.launch.py` が
`RewrittenYaml` で絶対パスに差し替えている。

`bt_navigator` は configure 時に nav_to_pose と nav_through_poses の**両方**を読むため、
片方だけ差し替えると `Action server backup not available` で起動に失敗する。

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

**前提**: これらの launch は TF を配信しない。先に `rsf_bringup` を起動しておくこと。
`robot_state_publisher`（`base_footprint` 以下の TF）と `tf_odom_to_footprint`
（`rsf_odom` -> `base_footprint`）が無いと、コストマップが
`Invalid frame ID "rsf_odom"` で止まりロボットは動かない。

launch は 2 段構成にしてある。

| launch | 起動するもの | 用途 |
|---|---|---|
| `localization.launch.py` | `pointcloud_to_laserscan` / `map_server` / `emcl2` / `lifecycle_manager_localization` | 自己位置推定だけ。ウェイポイント記録はこれで足りる |
| `navigation.launch.py` | 上を include + nav2 一式 | 自律走行 |

```bash
ros2 launch rsf_simulator rsf_simulator.launch.py   # 端末1
ros2 launch rsf_bringup rsf_bringup.launch.py       # 端末2
ros2 launch rsf_navigation_executor navigation.launch.py   # 端末3
```

引数は両方の launch で共通で、`navigation.launch.py` に渡した値はそのまま
`localization.launch.py` に転送される。

| 引数 | 既定値 |
|---|---|
| `map` | `maps/tsudanuma2-3.yaml` |
| `use_sim_time` | `true` |
| `autostart` | `true` |
| `use_rviz` | `true`（`navigation.launch.py` のみ） |

別の地図を使う場合:

```bash
ros2 launch rsf_navigation_executor navigation.launch.py \
  map:=$(ros2 pkg prefix --share rsf_navigation_executor)/maps/<名前>.yaml
```

起動後、RViz の 2D Pose Estimate で初期位置を与える。

### RViz

`navigation.launch.py` は `rviz/navigation.rviz` を読んだ RViz を一緒に起動する。

| 表示 | トピック | 既定 |
|---|---|---|
| Map | `/map` | on |
| LaserScan | `/scan` | on |
| Hokuyo3D PointCloud | `/rsf/hokuyo_cloud2` | off（重い） |
| MCL Particles | `/particlecloud` | on |
| MCL Pose | `/mcl_pose` | on |
| Waypoints | `/waypoint_navigator/waypoints` | on |
| Global Costmap / Global Plan / Global Footprint | `/global_costmap/costmap`, `/plan`, `/global_costmap/published_footprint` | on |
| Local Costmap / Local Footprint | `/local_costmap/costmap`, `/local_costmap/published_footprint` | on |
| MPPI Transformed Plan / MPPI Trajectories | `/transformed_global_plan`, `/trajectories` | off |
| RobotModel / TF / Wheel Odometry | `/robot_description`, `/tf`, `/rsf/rsf_odom` | RobotModel のみ on |

MPPI の 2 つは `nav2_params.yaml` の `FollowPath.visualize` を `true` にしないと配信されない。
コントローラを調整するときだけ有効にすること。

`rsf_bringup` も RViz を起動するため、両方立てるとノード名 `rviz2` が衝突する。
どちらか片方にすること。

```bash
ros2 launch rsf_navigation_executor navigation.launch.py use_rviz:=false
```

## 3. ウェイポイントを記録する

記録に必要なのは `mcl_pose` だけなので、nav2 は起動しなくてよい。

```bash
ros2 launch rsf_navigation_executor localization.launch.py     # 端末3
ros2 launch rsf_navigation_executor waypoint_recording.launch.py   # 端末4
```

記録される座標は `map` 系なので、**先に初期位置を合わせておくこと**。
起動すると `record_interval` 秒ごとに `mcl_pose` の現在推定値が自動で 1 点追加される。
ただし直前の記録点から `min_distance` 以上離れていなければ捨てるので、
信号待ちなどで停車している間に同じ座標が積み上がることはない。

手動操縦でコースを一周し、走り終えたら書き出す。コマンドは起動時にログへ出る。

```
[waypoint_recorder]: save with: ros2 service call /waypoint_recorder/save std_srvs/srv/Trigger
```

| 引数 | 既定値 |
|---|---|
| `output_file` | `recorded_waypoints.yaml`（launch を叩いたカレントディレクトリ） |
| `record_interval` | `5.0`（秒） |
| `min_distance` | `0.5`（m） |

`mcl_pose` が届くまでの間は `no pose received yet` を出して記録をスキップする。

```bash
ros2 launch rsf_navigation_executor waypoint_recording.launch.py \
  record_interval:=2.0 min_distance:=1.0
```

`min_distance` は実行中でも変えられる。

```bash
ros2 param set /waypoint_recorder min_distance 1.5
```

書き出したファイルは `waypoints/` に置いて git 管理下に置くこと。
地図と対応が付くよう `<地図名>_wp.yaml` で揃える。
`output_file` に絶対パスを渡せば直接そこへ書ける。

```bash
ros2 launch rsf_navigation_executor waypoint_recording.launch.py \
  output_file:=$HOME/rsf_ws/src/rsf_navigation_executor/waypoints/tsudanuma_wp.yaml
```

## 4. ウェイポイント走行

実装は公式の `nav2_waypoint_follower`（`FollowWaypoints` アクション）を
`nav2_simple_commander` の `BasicNavigator` で駆動する薄いクライアントである。
走行そのものは waypoint_follower / bt_navigator 側が行い、このノードは
yaml の読み込み・checkpoint によるレグ分割・`start`/`pause`/`resume` サービス・
速度制限の publish を担う。

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
| `waypoints_file` | `waypoints/tsudanuma2-3_wp.yaml`（既定の地図 `maps/tsudanuma2-3.yaml` と対） |

別のコースを走る場合:

```bash
ros2 launch rsf_navigation_executor waypoint_navigation.launch.py \
  waypoints_file:=$(ros2 pkg prefix --share rsf_navigation_executor)/waypoints/<名前>_wp.yaml
```

### waypoints yaml の書式

```yaml
loop: false
waypoints:
  - {x: 3.0, y: 0.0, yaw: 0.0, speed_limit: 50.0}
  - {x: 3.0, y: 3.0, yaw: 1.5708, checkpoint: true}
  - {x: 0.0, y: 3.0, yaw: 3.14159}
```

`loop` はウェイポイントのキーではなく yaml のトップレベルキー。

| キー | 説明 |
|---|---|
| `loop`（トップレベル） | `true` で最終点から先頭に戻る |
| `x`, `y`, `yaw` | 目標姿勢（map 座標系、yaw はラジアン） |
| `speed_limit` | **その点へ向かう区間**の速度上限。0〜100 の % 値、0.0（省略時）は制限解除 |
| `checkpoint` | `true` なら到達必須。到達すると停止して `resume` を待ち、失敗すると同じ点を再試行する。省略時は失敗してもスキップして次へ進む |

checkpoint はウェイポイント列をレグ（区間）に分割する境界で、そこまでを 1 回の
`FollowWaypoints` で走る。レグ内の非 checkpoint の点は公式サーバの
`stop_on_failure: false` によりスキップされる（スキップされた点は `missed waypoints`
としてログに出るだけで、走行は止まらない）。checkpoint 自体への到達が失敗した場合の
リトライ方法はレグの終わり方によって異なる:

- レグ全体は成功したが checkpoint の点だけ missed だった場合は、その checkpoint 1 点
  だけを対象にリトライする。
- レグ自体が失敗した場合は、直前まで走行中だった点(feedback から分かる位置)から
  末尾の checkpoint までを再送する。feedback を一度も受信していなければレグ全体を
  再送する。

既知の制限: Humble の `nav2_waypoint_follower` は cancel(`pause`)しても内部の
スキップ済み点リストをクリアしないため、`pause` → `resume` の直後に完了する
レグの `missed_waypoints` に pause 前のスキップ点が残留することがある。実害は
良性の空リトライが 1 回余分に走る程度で、走行自体は継続する。

BackUp を除いた behavior tree（下記）は `FollowWaypoints` → waypoint_follower →
`NavigateToPose` 経由でも変わらず適用される。

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

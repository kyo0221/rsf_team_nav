# rsf_description

RSF ロボット（orne_boxF）の URDF/xacro とメッシュ。`robot_state_publisher` で TF を配信する。

## 起動

```bash
ros2 launch rsf_description display.launch.py
```

`robot_state_publisher` / `joint_state_publisher` が起動し、TF と `/robot_description` を配信する。

**RViz はこの launch では起動しない。** ナビゲーション用の RViz は
`rsf_navigation_executor` の `navigation.launch.py` が起動する（ノード名 `rviz2` が
衝突するため一箇所に集約した）。URDF だけ見たいときは手動で開くこと。

```bash
rviz2 -d $(ros2 pkg prefix --share rsf_description)/rviz/description.rviz
```

| 引数 | 既定値 | 説明 |
|---|---|---|
| `use_sim_time` | `false` | Gazebo の `/clock` を使う場合は `true` |
| `use_joint_state_publisher` | `true` | 車輪・キャスタの関節状態を配信する |

シミュレータと併用する場合:

```bash
ros2 launch rsf_description display.launch.py use_sim_time:=true
```

## 構成

| パス | 内容 |
|---|---|
| `urdf/orne_boxF.urdf.xacro` | ルート。以下を include する |
| `urdf/links/` | base_link / 車輪 / キャスタ |
| `urdf/sensors/rsf_x001.urdf.xacro` | 3D LiDAR・IMU・GNSS のフレーム |
| `urdf/material_colors.xacro` | 色定義 |
| `meshes/RSF-X001.stl` | 外装メッシュ |

3D LiDAR のフレーム `rsf_hokuyo3d` は base_link から x = 0.3 m, z = 0.35 m。

## URDF の確認

```bash
xacro $(ros2 pkg prefix --share rsf_description)/urdf/orne_boxF.urdf.xacro
```

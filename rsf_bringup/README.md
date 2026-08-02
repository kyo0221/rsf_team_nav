# rsf_bringup

ロボットを手動操縦できる状態まで立ち上げる。シミュレータと実機のどちらでも使う。

## 起動

```bash
ros2 launch rsf_bringup rsf_bringup.launch.py
```

起動するもの:

- `joy_node` / `teleop_twist_joy_node` … ゲームパッドで `/cmd_vel` を出す
- `rsf_description/display.launch.py` … TF と RViz
- `tf_odom_to_footprint` … `/rsf/rsf_odom` を購読し `rsf_odom` -> `base_footprint` の TF を配信

## シミュレータと実機の切り替え

launch 引数ではなく `config/bringup_params.yaml` の `sim` で切り替える。

```yaml
launch:
  ros__parameters:
    sim: true      # false にすると実機用のノードも起動する
```

`sim: false` のとき追加で起動するもの:

- `icart_driver/icart_drive.launch.py` … 台車ドライバ
- `hokuyo_rsf` … 3D LiDAR ドライバ

シミュレータで使う場合は `sim: true` のまま、別端末で以下を起動する。

```bash
ros2 launch rsf_simulator rsf_simulator.launch.py
```

## 操縦

`config/bringup_params.yaml` の `teleop_twist_joy_node` で割り当てを変える。

| 項目 | 既定値 |
|---|---|
| 前後 | 軸 1、最大 0.5 m/s |
| 旋回 | 軸 2、最大 1.0 rad/s |
| enable ボタン | 不要（`require_enable_button: false`） |

軸番号とスティックの対応はパッドによって変わるので、合わない場合は `ros2 topic echo /joy` で確認する。

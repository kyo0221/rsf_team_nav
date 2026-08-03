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

`interlace:=N`（1〜20、デフォルト 1）で YVT-35LX のインターレースを再現する。各 20Hz フレームの照射方位が 6°/N ずつ巡回シフトし、N フレーム蓄積で水平密度が N 倍になる。

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

## GUI 設定

world に `<gui>` を書くと、既定の `gui.config` を**置き換える**（マージではない）。
そのため 3D ビューだけでなく操作パネル類もすべて world 側に並べる必要がある。

シーンプラグインは公式推奨の `MinimalScene` を使う
（[公式の移行手順](https://github.com/gazebosim/gz-sim/blob/gz-sim8/Migration.md)）。
両 world で以下の 6 個に揃えてある。

| プラグイン | 役割 | 省略すると |
|---|---|---|
| `MinimalScene` | 描画本体。`<engine>` `<camera_pose>` はここ | 3D ビューが出ない |
| `GzSceneManager` | サーバ側のエンティティをシーンへ反映 | **何も映らない** |
| `InteractiveViewControl` | マウスでの視点操作 | **視点が動かせない** |
| `WorldControl` | 再生 / 一時停止 / ステップ | 操作パネルが出ない |
| `WorldStats` | sim time / RTF | 時刻表示が出ない |
| `VisualizeLidar` | `/rsf/hokuyo3d` の点群表示 | LiDAR が可視化されない |

`GzSceneManager` と `InteractiveViewControl` は UI を持たないが、プロパティを書かないと
gz-gui が既定サイズのカードを作って画面右に並んでしまう。既定 `gui.config` と同じく
**5x5px の非表示フローティング**にしてある。

```xml
<ignition-gui>
  <property key="resizable" type="bool">false</property>
  <property key="width" type="double">5</property>
  <property key="height" type="double">5</property>
  <property key="state" type="string">floating</property>
  <property key="showTitleBar" type="bool">false</property>
</ignition-gui>
```

`WorldControl` / `WorldStats` の `anchors target` はプラグインの `name` ではなく
`<title>` を参照するので、`MinimalScene` 側に `<title>3D View</title>` を書いておくこと。

公式のコンパニオン一覧には他に `CameraTracking`（Follow / Move to）、
`EntityContextMenuPlugin`（右クリックメニュー）、`SelectEntities`、`MarkerManager`、
`Spawn`、`VisualizationCapabilities`（衝突形状の表示）がある。いずれも必須ではないので
外してあるが、GUI 上でロボットを追従したい場合は `CameraTracking` と
`EntityContextMenuPlugin` と `SelectEntities` の 3 つを戻す必要がある
（右クリックメニュー経由で Follow を呼ぶため）。

## Ubuntu 24.04 (ROS 2 Jazzy / Gazebo Harmonic) へ移行するとき

GUI プラグインのファイル名は Harmonic でも同じなので、上記の `<gui>` ブロックは
そのまま使える。一方で**システムプラグインの記述は書き換えが必要**である。

| Fortress（現在） | Harmonic |
|---|---|
| `filename="libignition-gazebo-physics-system.so"` | `filename="gz-sim-physics-system"` |
| `name="ignition::gazebo::systems::Physics"` | `name="gz::sim::systems::Physics"` |

Physics / UserCommands / SceneBroadcaster / Sensors / Imu / NavSat すべてが対象。
`<ignition-gui>` タグも `<gz-gui>` へ変わる可能性が高いが、移行時に実機で確認すること。
`GzScene3D` は Garden 以降で削除済みなので、`MinimalScene` にしてある本構成なら
この点は影響を受けない。

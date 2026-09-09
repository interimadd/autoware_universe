# 信号認識パイプライン オフライン評価ツール

t4dataset のパスを指定すると、このパッケージの ROS 非依存コアを全フレームに対して実行し、
その結果を rosbag に書き出すツール群です。2 本の実行ファイルがあります。

| 実行ファイル                               | 回すコア                              | 出力                                                                                                                       |
| ------------------------------------------ | ------------------------------------- | -------------------------------------------------------------------------------------------------------------------------- |
| `run_traffic_light_recognition_evaluation` | 前段（`TrafficLightRecognition`）のみ | カメラごとの `merged_signals` / `selected_rois`                                                                            |
| `run_traffic_light_pipeline_evaluation`    | 前段 + 後段（`TrafficLightFusion`）   | 前段の出力に加え、`TrafficLightGroupArray`（本番の `/perception/traffic_light_recognition/internal/traffic_signals` 相当） |

`autoware_traffic_light_component_test` の `run_traffic_light_pipeline` の移植です。以下は移植して
いません。

- `ParameterLoader` によるコンポーネント別 param.yaml のマージ
  （`TrafficLightRecognitionConfig` / `TrafficLightFusionConfig` が既にフラットなので、評価用
  yaml 1 枚に直接書く）

`rclcpp::init` / executor / DDS は一切使わず、コアライブラリと rosbag2 のみで動作します。

## 使い方

```bash
CFG=$(ros2 pkg prefix --share autoware_traffic_light_pipeline)/evaluation/config/x2_v4.4.evaluation.yaml

# 前段のみ
ros2 run autoware_traffic_light_pipeline run_traffic_light_recognition_evaluation \
  --config $CFG \
  --dataset <t4dataset のパス> \
  --output-bag result/recognition_bag

# 前段 + 後段
ros2 run autoware_traffic_light_pipeline run_traffic_light_pipeline_evaluation \
  --config $CFG \
  --dataset <t4dataset のパス> \
  --output-bag result/pipeline_bag
```

データセットのレイアウトは固定です（Component Test ハーネスと同じ規約）。

```text
<dataset>/input_bag
<dataset>/map/lanelet2_map.osm      # MGRS 座標系のみ対応
<dataset>/map/map_projector_info.yaml
```

## 入出力

|                                                          | 内容                                                                                           |
| -------------------------------------------------------- | ---------------------------------------------------------------------------------------------- |
| 入力                                                     | `<dataset>/input_bag` の `camera_info` / `image_raw(/compressed)` / `/tf` / `/tf_static`       |
| 前段出力                                                 | `run()` の `merged_signals`（`TrafficLightArray`）と `selected_rois`（`TrafficLightRoiArray`） |
| 後段出力（`run_traffic_light_pipeline_evaluation` のみ） | `TrafficLightFusion::run()` の出力（`TrafficLightGroupArray`）                                 |

- 画像と camera_info は本番の `message_filters::ExactTime` と同じくヘッダスタンプ完全一致で
  ペアリングし、相手のいないメッセージは捨てます。
- 出力トピック名は評価用 yaml の `cameras[].output_topics`（前段）と `fusion.output_topic`
  （後段）で指定します（本番のトピック名）。
- **注（暫定）**: `TrafficLightFusion::run()` は現在 crosswalk_estimator の処理をスキップしており、
  後段出力は **arbiter の出力**です（`src/traffic_light_fusion/traffic_light_fusion.cpp` の
  TEMPORARY コメント参照）。本番の `internal/traffic_signals`（multi_camera_fusion の出力）とは
  段が 1 つ違いますが、external(V2X) 入力が無く arbiter は素通りなので内容は同一です（実測 0/1194）。
- pass B（後段）への投入順は `(stamp, camera_index)` 昇順です。データセットのみから決まるので
  常に決定的で、リファレンス実行を必要としません。
  ただしこれは**フルシステム実行と一致するとは限りません**。本番では 2 台のカメラが別 Jetson で
  動くため、1 サイクル内でどちらの triple が multi_camera_fusion に先に着くかは前段のレイテンシ
  次第で毎サイクル変わります（x2 実測で camera5 が先: 597 サイクル中 244 = 40.9%）。
  `message_lifespan` がカメラ周期より大きければどちらの順でも両眼融合になるので、順序が効くのは
  2 台の判定が食い違うフレームだけですが、そこでは相手カメラの cycle-N / cycle-N-1 のどちらが
  混ざるかが変わり、融合色が変わることがあります。
  TLR_UC_001531_shiojiri_gen2_sunny_02 での実測（criteria_1 / criteria_2 / criteria_4）:

  | pass B の投入順                                | criteria_1     | criteria_2     | criteria_4       |
  | ---------------------------------------------- | -------------- | -------------- | ---------------- |
  | camera4 先着固定（= 本ツールの投入順）         | 23/26 = 88.46% | 34/34 = 100%   | 762/762 = 100%   |
  | camera5 先着固定                               | 22/26 = 84.62% | 34/34 = 100%   | 762/762 = 100%   |
  | フルシステム実行の実順序（camera5 先が 40.9%） | 22/26 = 84.62% | 33/34 = 97.06% | 761/762 = 99.87% |

  フルシステム実行はどちらの固定順よりも悪い値になります（順序が途中で切り替わることで、
  一貫した順序なら起きない取りこぼしが出るため）。つまり**静的な順序ではフルシステム実行を
  再現できません**。厳密に突き合わせる必要が生じた場合は、本ツールにフラグを足すのではなく
  使い捨ての検証スクリプトで実行順を再生してください（component test 自体が特定の実行結果に
  依存しないようにするため）。

- 各メッセージはヘッダスタンプの時刻で書き込むため、同じデータセットからは常に同じ bag が
  得られます（実行時刻には依存しません）。
- 出力 bag のストレージ形式は入力 bag と同じものを自動で使います。
- `run()` が失敗したフレーム/イベントは stderr に出して読み飛ばします（Node が捨てるのと同じ
  挙動）。

### `run_traffic_light_pipeline_evaluation` の処理順序

前段を全フレーム処理し終えてから、まとめて後段を回します（フレームごとに前段→後段を交互に
呼ぶ component_test 版とは順序が異なります）。`TrafficLightFusion` はステートフルですが時計を
読まないため、同じ入力列を同じ順序で流せば本番と同一の出力になります。前段の結果は
`camera_info` も含めてメモリに保持されるため（画像は保持しません）、bag を読み直す必要は
ありません。

## 評価用 yaml

`config/x2_v4.4.evaluation.yaml` を参照してください。`cameras[]` に 1 エントリ書くごとに
`TrafficLightRecognition` インスタンスが 1 つ生成されます。

**チューニング値はすべてパッケージの `config/traffic_light_recognition.param.yaml` /
`config/traffic_light_fusion.param.yaml` から読みます**（launch ファイルが Node に渡すのと同じ
ファイルを `load_package_param_yaml()` で直接読んでいます）。評価用 yaml 側から上書きすることは
できません。評価用 yaml に書くのは

- そのデータセット固有の情報（`cameras[]` のトピック名、`fusion.output_topic`）
- ユーザーの `$HOME` 配下にある model_path / label_path（本番でも launch 側から注入されるため
  `config/*.param.yaml` には存在しません）

の 2 つだけです。閾値は `config/` に 1 か所だけ存在するので、評価が本番と違う値を測ることが
構造的に起きません。`map_based_detector.min/max_timestamp_offset` も同様で、
`CameraConfig` はカメラごとに値を持ちますが、入るのは全カメラともパッケージ config の値です。

`fusion:` セクションは `run_traffic_light_pipeline_evaluation` でのみ使用し、書けるのは
`output_topic`（パッケージ config に対応する値がない、評価の出力先）だけです。
`multi_camera_fusion.*` / `arbiter.*` は `config/traffic_light_fusion.param.yaml` から、
`crosswalk_estimator` は `traffic_light_fusion_node.cpp` の `declare_fusion_config()` と同じ
固定値をツール側でハードコードしたものが入ります（本番でも parameter 化されていない）。

model_path / label_path 類は本番（webauto CI）と同じ `/opt/autoware/mlmodels/` 配下の
ML package 実体を指しています。**別の場所にある同じ `.onnx` に差し替えるときは注意が必要です**:
`.onnx` が同一でも、その隣にキャッシュされている TensorRT の `.engine` が別バージョンで焼かれて
いると fp16 の演算結果がわずかに変わります（実測: confidence が 0.99998 ではなく厳密に 1.0 に
なる）。`multi_camera_fusion` の `has_higher_or_equal_priority()` は同 confidence 時に
「後から来た record を採用」する厳密比較なので、この差だけで融合結果の色が変わります。
本番と数値を合わせたい場合は engine ごと共有する（= 本番と同じパスを指す）のが確実です。

## rvizでの可視化

`run_traffic_light_pipeline_evaluation` の出力 bag は、`../launch/visualize_traffic_light_pipeline_result.launch.xml`
（`autoware_component_test` の同名ランチファイルの移植）で rviz 可視化できます。

```bash
ros2 launch autoware_traffic_light_pipeline visualize_traffic_light_pipeline_result.launch.xml \
  dataset_path:=<t4dataset のパス（run_traffic_light_pipeline_evaluation --dataset と同じもの）> \
  output_bag_path:=<run_traffic_light_pipeline_evaluation --output-bag で書き出した bag>
```

後段の融合結果（`fusion.output_topic`）を地図上のバルブ色 SPHERE マーカーとして、各カメラの
`output_topics.rois` / `output_topics.traffic_signals` を入力 bag の画像に重ねたラベル付き矩形
として表示します。`tl_state_topic` / `cameraN_rois_topic` / `cameraN_traffic_signals_topic` の
デフォルト値は `config/x2_v4.4.evaluation.yaml` のトピック名に合わせてあるので、別の評価用 yaml
を使った場合は該当する引数を上書きしてください。詳細はランチファイル自身のコメントを参照してください。

## 精度評価

上記 2 本の実行ファイルは出力 bag を書き出すだけで、精度評価そのものは行いません。
`run_traffic_light_pipeline_evaluation` の出力 bag を t4dataset の `annotation/` と突き合わせて
距離ビンごとに精度評価するツールは [scripts/README.md](scripts/README.md) を参照してください。

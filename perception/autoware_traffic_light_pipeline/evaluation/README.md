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

# 前段 + 後段（フルシステム実行の到着順を再現して厳密に突き合わせる場合。下記「入出力」参照）
ros2 run autoware_traffic_light_pipeline run_traffic_light_pipeline_evaluation \
  --config $CFG \
  --dataset <t4dataset のパス> \
  --output-bag result/pipeline_bag \
  --arrival-order-bag <フルシステム実行の result_bag>
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
- pass B（後段）への投入順は既定で `(stamp, camera_index)` 昇順です。本番では 2 台のカメラが
  別 Jetson で動くため、1 サイクル内でどちらの triple が multi_camera_fusion に先に着くかは
  前段のレイテンシ次第で毎サイクル変わります（x2 実測で camera5 が先: 597 サイクル中 244 =
  40.9%）。`message_lifespan` がカメラ周期より大きければどちらの順でも両眼融合になるので、
  順序が効くのは 2 台の判定が食い違うフレームだけですが、そこでは相手カメラの cycle-N /
  cycle-N-1 のどちらが混ざるかが変わり、融合色が変わることがあります。
  フルシステム実行と厳密に突き合わせたい場合は `--arrival-order-bag <その実行の result_bag>`
  を渡すと、その実行の `internal/traffic_signals` の publish 順で pass B を再生します。
  既定にしていないのは、リファレンス実行なしで決定的・自己完結に回せることを優先しているためです。
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
`TrafficLightRecognition` インスタンスが 1 つ生成されます。`recognition:` 以下は全カメラ共通で、
`config/traffic_light_recognition.param.yaml` と `launch/traffic_light_recognition.launch.xml`
の値をそのまま持ってきたものです。カメラごとに変わるのは
`map_based_detector.min/max_timestamp_offset` だけなので、そこだけ `cameras[]` 側にあります。

`fusion:` セクションは `run_traffic_light_pipeline_evaluation` でのみ使用します。
`crosswalk_estimator` は `traffic_light_fusion_node.cpp` の `declare_fusion_config()` と同じ
固定値をツール側でハードコードしており（本番でも parameter 化されていない）、yaml に書くのは
parameter 化されている `multi_camera_fusion.*` / `arbiter.*` のみです
（`config/traffic_light_fusion.param.yaml` と同値）。

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

# PatternDictionary圧縮・可視化デモ

pointcloud_compressorでPCDファイルを圧縮し、compressed_viewerでPatternDictionary可視化を同時に実行するデモです。

## 実行方法

### 1. 基本的なデモ実行

```bash
cd /home/ryo/image_compressor_ws
source install/setup.bash

# デモ起動（RViz2付き）
ros2 launch compressed_viewer demo.launch.py
```

### 2. PCDファイルを指定して実行

```bash
# 特定のPCDファイルを使用
ros2 launch compressed_viewer demo.launch.py input_file:=/path/to/your/file.pcd

# または従来のパラメータ名でも可能
ros2 launch compressed_viewer demo.launch.py pcd_file:=/path/to/your/file.pcd
```

### 3. 圧縮パラメータをカスタマイズ

```bash
# より細かい圧縮設定
ros2 launch compressed_viewer demo.launch.py \
  input_file:=/path/to/file.pcd \
  voxel_size:=0.005 \
  block_size:=8 \
  target_patterns:=512
```

### 4. カスタム設定ファイルを使用

```bash
# 独自のYAML設定ファイルを使用
ros2 launch compressed_viewer demo.launch.py \
  config_file:=/path/to/custom_config.yaml \
  input_file:=/path/to/file.pcd
```

### 5. RViz2なしで実行

```bash
# RViz2を起動せずに実行
ros2 launch compressed_viewer demo.launch.py launch_rviz:=false
```

## RViz2での表示内容

デモでは以下のトピックが表示されます：

### MarkerArrayトピック
- `/pattern_markers` - パターン辞書の3D可視化
- `/visualization_markers` - 復元された点群のマーカー表示
- `/statistics_markers` - 圧縮統計情報の表示

### PointCloud2トピック
- `/decompressed_pointcloud` - 復元された点群データ

## パラメータ設定

`config/demo_params.yaml`で以下の設定が可能：

### 圧縮設定
- `voxel_size`: ボクセルサイズ（デフォルト: 0.01）
- `block_size`: ブロックサイズ（デフォルト: 8）
- `target_patterns`: 目標パターン数（デフォルト: 128）

### 可視化設定
- `pattern_spacing`: パターン間隔（デフォルト: 2.5m）
- `pattern_voxel_size`: パターン表示時のボクセルサイズ（デフォルト: 0.08）
- `show_pattern_visualization`: パターン可視化ON/OFF（デフォルト: true）

### 表示色設定
- `point_color_*`: 点群の色（R,G,B,A）
- `bbox_color_*`: バウンディングボックスの色
- `stats_text_color_*`: 統計テキストの色

## トラブルシューティング

### パッケージが見つからない場合
```bash
cd /home/ryo/image_compressor_ws
colcon build --packages-select compressed_viewer pointcloud_compressor
source install/setup.bash
```

### PCDファイルが見つからない場合
```bash
# テスト用PCDファイルを作成
echo "VERSION .7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH 3
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 3
DATA ascii
0.0 0.0 0.0
1.0 1.0 1.0
2.0 2.0 2.0" > /tmp/sample.pcd
```

### RViz2でマーカーが表示されない場合
1. Fixed Frame を "map" に設定
2. MarkerArray表示の追加: Add → By topic → /pattern_markers → MarkerArray
3. PointCloud2表示の追加: Add → By topic → /decompressed_pointcloud → PointCloud2

## 実行の流れ

1. `pointcloud_compressor_node` がPCDファイルを読み込み、圧縮
2. 圧縮されたデータ（CompressedPointCloud）とパターン辞書（PatternDictionary）をパブリッシュ
3. `compressed_viewer_node` がこれらを受信し、復元・可視化
4. RViz2でリアルタイムに表示

パターン辞書は各ユニークなボクセルパターンを色分けして表示され、圧縮効率を視覚的に確認できます。
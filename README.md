# crossing_traffic_manager_cpp
これは、ros2のパッケージです.  
つくばチャレンジ2024の選択課題B(信号あり認識横断)のために作成されました.(C++ ver)

# 使用方法
* traffic_observer_cpp.cpp  
コード内の以下の部分(33行目付近)を実行するスクリプトの絶対パスに置き換えてください.
```
const char *command = "/home/orne-box/traffic_shell/connect_yolov8.sh";
```
* traffic_judgementer_cpp.cpp  
コード内の以下の部分(43行目付近)を自身の環境に合った絶対パスに置き換えてください.
```
static constexpr const char* CROSSPOINT_PATH = "/home/orne-box/orne_ws/src/cross_traffic_manager_cpp/config/crossing_points/tukuba2024.yaml";
```
認識する対象を変更する場合は, 以下の部分(69行目付近)の"Blue"を変更してください.
```
traffic_msg_.find("Blue") != std::string::npos && !traffic_request_) {
```

* config/crossing_points/tukuba2024.yaml  
yamiファイル内には, 信号前の道路端で使用しているstop_wpの番号を指定します.
```
crossing_point_numbers:
  - 14
  - 16
  - 36
  - 39
```
※  yamlファイルはコピーしてお使いください.
# 起動方法

* traffic_observer_cpp.cpp
```
$ ros2 run crossing_traffic_manager_cpp traffic_observer_cpp
```
* traffic_judgementer_cpp.cpp
```
$ ros2 run crossing_traffic_manager_cpp traffic_judgementer_cpp
```


# ライセンスについて
* このソフトウェアパッケージは、3条項BSDライセンスの下、再分布および使用が許可されています
。
* © 2024 Shuma Suzuki

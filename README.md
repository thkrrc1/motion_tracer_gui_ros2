# motion_tracer_gui_ros2
PythonのCustomTkinterを使用した簡易的なGUI
## 1.関連PKGインストール
1. [motion_tracer_ros2](https://github.com/thkrrc1/motion_tracer_ros2/tree/airoa)のREADMEに従ってプロジェクトをクローンする。

2. リーダーデバイスPCのhome直下にて、motion_tracer_gui_ros2をクローンする。(homeディレクトリ：/home/seed/ の前提)

3. 下記のコマンドを実行する。
```
sudo apt install -y \
  python3-venv \
  python3-pip \
  python3-tk \
  gnome-terminal \
  sshpass \
  mesa-utils \
  libgl1-mesa-dev \
  libglu1-mesa-dev \
  ros-jazzy-pinocchio
```

## 2.venv仮想環境設定
下記のコマンドを実行して、motion_tracer_gui_ros2用のPython仮想環境を作成する。
```
cd motion_tracer_gui_ros2/
python3 -m venv venv
cd
source ~/motion_tracer_gui_ros2/venv/bin/activate
pip install pipreqs
pip install -r requirements.txt
pip install pyyaml
pip install numpy<2
```

## 3.GUIの実行
フォロワーデバイスPC、リーダーデバイスPCを同一ネットワーク上に接続する。

新規ターミナルにおいて下記のコマンドを実行して、テレオペレーション用のGUIを起動する。
```
./motion_tracer_gui_ros2/scripts/motion_tracer_gui_ros2.sh
```

**※ROS2のNode同士の通信が失敗する恐れがあるため、GUI実行中にネットワークの経由方法が変更された場合（ex. 無線LANから有線LANなど）は、GUIを起動し直してください。**
# 指ダンス（finger_dance）用の環境（研究用。専攻科実験のフォルダやイメージには依存しない）
# 中身は専攻科実験 2026 の環境（授業/13専攻科実験/2026/手順書/docker）のうち必要なものに、MediaPipe とモデルを足したもの。
# 元のイメージは個人が公開しているもので中身が更新されるため、版（ダイジェスト）を固定している（2026-10-18 版）。
FROM tiryoh/ros2-desktop-vnc:jazzy@sha256:41d91a7486f19e63955b20618157a277438466ac0051e68238a8e7bbe7186868

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-turtlesim \
        ros-jazzy-rmw-zenoh-cpp \
        ros-jazzy-raspimouse-msgs \
        ros-jazzy-rosbridge-server \
        python3-colcon-common-extensions \
        git nano \
    && rm -rf /var/lib/apt/lists/*

# 元のイメージに pip で入っている NumPy 2 を削除する（apt の OpenCV・cv_bridge は NumPy 1 用のため）
RUN rm -rf /usr/local/lib/python3.12/dist-packages/numpy /usr/local/lib/python3.12/dist-packages/numpy-*.dist-info /usr/local/lib/python3.12/dist-packages/numpy.libs

# MediaPipe（"numpy<2" が必須。ないと NumPy 2 が入り、apt の OpenCV・cv_bridge が壊れる）
RUN pip install --no-cache-dir --break-system-packages "numpy<2" mediapipe

# 手の21点を求めるモデル（Hand Landmarker、約8MB）
RUN mkdir -p /opt/finger_dance && python3 -c "import urllib.request; urllib.request.urlretrieve('https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task', '/opt/finger_dance/hand_landmarker.task')" \
    && chmod 644 /opt/finger_dance/hand_landmarker.task

# 体の 33 点を求めるモデル（Pose Landmarker full、約 9 MB。人のダンスに合わせて踊るときに使う）
RUN python3 -c "import urllib.request; urllib.request.urlretrieve('https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_full/float16/latest/pose_landmarker_full.task', '/opt/finger_dance/pose_landmarker_full.task')" \
    && chmod 644 /opt/finger_dance/pose_landmarker_full.task

# 実機との接続：robot_connect／robot_disconnect／set_domain_id コマンド
# （--chmod=644：ファイルが自分だけ読める権限でも、コンテナの ubuntu ユーザが読めるようにする）
# （フォルダは先に作っておく。COPY に作らせると、新しい Docker ではフォルダにも 644 が付き、中が読めなくなる）
RUN mkdir -p /usr/local/share/finger_dance
COPY --chmod=644 robot.bash /usr/local/share/finger_dance/robot.bash
RUN echo 'source /usr/local/share/finger_dance/robot.bash' >> /etc/bash.bashrc

# PC のカメラの映像をブラウザから送るページ（http://localhost:6081/camera.html?port=9091）
COPY --chmod=644 camera.html /usr/lib/novnc/camera.html

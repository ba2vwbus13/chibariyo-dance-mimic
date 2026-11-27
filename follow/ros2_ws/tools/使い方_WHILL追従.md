# チバリヨカー 2nd ゲート（2026-11-16〜17）で WHILL Model CR を動かしたコード

担当の学生が 2026-11-26 に zip で送ったもの（Issue #1）を、そのままリポジトリに入れた。記録・動画はリポジトリには入れず、Teams の共有フォルダに置いてある。
下の場所は、リポジトリのいちばん上のフォルダから見たもの。

## 用意するもの
- Python 3.11（2026-11 に Windows で動いた。Mac は確かめていない）
- 追従:   python3 -m pip install -r follow/ros2_ws/tools/requirements.txt
- 橋渡し: python3 -m pip install -r whill/requirements.txt（whill は 1.5.0 に固定）
- 骨格のモデル pose_landmarker_full.task を MediaPipe の Pose Landmarker のページから取り、
  リポジトリのいちばん上の local/ に置く（または --model で場所を指定する）
- カメラ（撮影では Web カメラを --camera 0 で使った）、WHILL と PC をつなぐ USB-RS232C

## 撮影で実行したコマンド（撮影の手順 2026-11-16 の改訂のとおり。ターミナルを 2 つ使う）
ターミナル 1（リポジトリのいちばん上のフォルダで）: 追従。骨格推定のウィンドウが出る
```
python follow/ros2_ws/tools/pose_follow_whill.py --camera 0 --max-linear 0.7 --max-angular 0.6 --distance-median 5 --release-s 0.3 --log-follow local/demo_follow_<時刻>.csv
```
ターミナル 2（whill フォルダで）: WHILL への橋渡し。最初に --dry-run で送る値を確かめてから、本番
```
python whill_bridge.py --dry-run --linear-max 0.7 --front-cap 42 --cap-limit-front 45 --angular-max 0.6 --side-cap 54 --cap-limit-side 90 --ramp 80 --stream-ms 20
python whill_bridge.py --port <装置名> --linear-max 0.7 --front-cap 42 --cap-limit-front 45 --angular-max 0.6 --side-cap 54 --cap-limit-side 90 --ramp 80 --stream-ms 20 --log ../local/demo_bridge_<時刻>.csv --log-motor ../local/demo_motor_<時刻>.csv
```
- 試し運転は 0.3 m/s: ターミナル 1 を --max-linear 0.3、ターミナル 2 を --linear-max 0.3 --front-cap 18 --angular-max 0.6 --side-cap 54 --cap-limit-side 90 --ramp 80 --stream-ms 20
- 2 つのターミナルの上限は同じ値にする（--max-linear と --linear-max、--max-angular と --angular-max）
- 起動の順はターミナル 1 → 2。追う人が 2.0 m に立ち、映像のウィンドウで c（体の大きさを測る。押すまで 0 しか送らない）
- 止める順: ターミナル 2 で Ctrl + C → 映像のウィンドウで q。とっさのときは映像のウィンドウでスペース（すぐ 0）、
  ターミナル 2 で Enter（停止）、最後は車いすの電源ボタン
- 画面の録画は別のスクリプト（リポジトリには入れていない）

## 撮影で使ったファイル
- follow/ros2_ws/tools/pose_follow_whill.py   … 本物のカメラで骨格から追従の指令を作り、whill_bridge.py へ渡す（ターミナル 1 で実行）
- follow/ros2_ws/tools/body_cut.py            … 近すぎ（頭か膝が画面の上下で切れる）なら前進 0 にする判定（pose_follow_whill.py が読み込む）
- follow/ros2_ws/tools/pose_distance_camera.py … カメラを開く・モデルを探す・骨格を描く部分を pose_follow_whill.py が読み込む
                                                 （このファイル自体は 11/4〜6 に距離の測定で実行した。撮影では実行していない）
- follow/ros2_ws/src/wheelchair_follower/wheelchair_follower/pose_follow_core.py … 先生の骨格追従の計算（距離・方向・人の選び方・指令）。pose_follow_whill.py が読み込む
- whill/whill_bridge.py                        … 速度の指令を WHILL のジョイスティックの倒し量に直して送る（先生のコードに引数を足した。ターミナル 2 で実行）
- follow/ros2_ws/tools/requirements.txt、whill/requirements.txt … 必要なライブラリの一覧

## 撮影では使っていないファイル（消していない）
- follow/ros2_ws/tools/test_body_cut.py       … body_cut.py の試験（pytest）
- whill/whill_profile.py                       … WHILL 本体の加速・減速を読む・書く・戻す道具（11/5 の試験で使った。撮影の手順には無い）
- whill/motor_speed.py                         … 記録からモータの速さを出す解析（11/5〜6 の解析で使った）
- whill/RESULTS_20261105_cmd_tracking.md       … 11/5 の指令と車輪の速さの合わせ込みの結果
- whill/whill_check.py                         … 先生の接続の確認（撮影の日に使ったかは不明）
- whill/手順.md                                … 先生の WHILL の手順（安全の決まり）
- follow/ros2_ws/tools/reid_eval/              … 先生の Re-ID の評価（使っていない）
- follow/ros2_ws/src/wheelchair_follower/wheelchair_follower/__init__.py … 読み込みのための空のファイル
- follow/ros2_ws/metrics_pose/RESULTS_20261104.md … 段階 1（シミュレーション）の記録で、WHILL では使っていない

## 安全の決まり（whill/手順.md のとおり）
- 人を乗せない。周り 1 m 以上を空ける。車いすの電源ボタンを押せる人をそばに置く。先生の立ち会いのもとで動かす
- 新しいプログラムは、まず --dry-run で送る値を確かめる。最初は車輪を浮かせ、旋回 ±10 %・前後 0 % から
- 倒し量の上限は既定 20 %。--cap-limit-front（最大 45 %）・--cap-limit-side（最大 90 %）で 20 % を超えるのは、
  先生の許可（2026-11-06・11-16）を受けた無人の開発用である
- 指令が 0.3 秒届かないと止まる仕組みは変えていない。NaN・Inf の指令は 0 にする

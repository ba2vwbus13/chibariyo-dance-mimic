# 学生に渡す Raspberry Pi Mouse の SD カード

作成：2026-10-26（Mac Studio）

学生が自分の Wi-Fi ルータで Raspberry Pi Mouse を使うための SD カードである。学生は、Raspberry Pi に画面（HDMI）とキーボード（USB）をつないで、自分のルータの Wi-Fi を設定する。

## 1. 学生用のイメージ

| 項目 | 内容 |
|---|---|
| イメージ | `~/chibamouse_image/student_20261026.img`（Mac Studio、11.3 GB。16 GB 以上のカードに入る） |
| 元 | `chibamouse_base_20261020.img`（専攻科実験の pimouse1 の複製）。ROS 2 Jazzy、rmw_zenoh、Raspberry Pi Mouse のドライバ、raspimouse ノードが入っている |
| ネットワーク | 有線 LAN（eth0）だけが、自動で IP をもらう設定になっている。**Wi-Fi は 1 つも登録していない** |
| 消したもの | 元のイメージに入っていた Wi-Fi の名前とパスワード（先生の自宅のルータ、実験用ルータ <授業用ルータのSSID>）。設定ファイル 2 つ（`/etc/netplan/50-cloud-init.yaml`、`system-boot/network-config`）、cloud-init のキャッシュ 2 つ、ログ 3 つ（`syslog`、`cloud-init.log`、`cloud-init-output.log`）、`~/.bash_history` から消し、どこにも残っていないことを検索で確かめた |
| 足したもの | `chibamouse-setname`（名前・ROS_DOMAIN_ID）、`chibamouse-wifi`（Wi-Fi の追加）を `/usr/local/sbin/` に入れ、ログインしたときに手順が表示されるようにした（`/etc/motd`） |
| そのまま残したもの | ユーザ ubuntu とそのパスワード（専攻科実験と同じ）、パスワードなしの sudo、先生の Mac の公開鍵（先生は鍵でログインできる） |

## 2. SD カードへの書き込み（先生、Mac、約 7 分）

カードのディスク番号を `diskutil list` で確かめる（Mac Studio のスロットなら「Built In SDXC Reader」）。`N` をその番号にして実行する。

```bash
diskutil unmountDisk /dev/diskN
```

```bash
sudo dd if=$HOME/chibamouse_image/student_20261026.img of=/dev/rdiskN bs=4m status=progress
```

```bash
diskutil eject /dev/diskN
```

## 3. 学生の手順（初めて使うとき）

1. SD カードを Raspberry Pi に挿し、**画面（HDMI）とキーボード（USB）をつないで**から電源を入れる。初回はカードいっぱいに区画を広げるので、2〜3 分かかる。
2. 画面に `login:` が出たら、ユーザ名 `ubuntu` と、授業で使っているパスワードを入力する。ログインすると、手順の案内が表示される。
3. 名前と ROS_DOMAIN_ID を決める（名前は英小文字・数字・`-`。ROS_DOMAIN_ID は学籍番号の下 2 桁）。

   ```bash
   sudo chibamouse-setname mymouse1 45
   ```

4. 自分のルータの Wi-Fi を足す。Raspberry Pi 3 は **2.4 GHz の Wi-Fi にしかつながらない**。Wi-Fi の名前に空白があるときは `"` で囲む。

   ```bash
   sudo chibamouse-wifi add 自分のWi-Fiの名前
   ```

   次のように Wi-Fi のパスワードを聞かれるので、**ルータの Wi-Fi のパスワード**（ルータ本体のシールの「暗号化キー」「KEY」など）を入力して Enter を押す。打った文字は画面に出ない。8 文字より短いと受け付けない。最後に「設定に誤りはありません。」と出れば終わりである。

   ```
   「自分のWi-Fiの名前」のパスワード（画面には出ません）:
   「自分のWi-Fiの名前」を足しました（/etc/netplan/60-chibamouse-wifi.yaml）。
   設定に誤りはありません。sudo poweroff → 電源を入れ直すと使われます（reboot は使わない）。
   ```

5. 電源を切る。緑の LED が消えてから電源スイッチを切り、もう一度入れる（**reboot は使わない**。Raspberry Pi 3 が起動しなくなる）。

   ```bash
   sudo poweroff
   ```

6. 起動したらもう一度ログインし、Wi-Fi でもらった IP アドレスを確かめる。`inet …` の部分が IP アドレスである。

   ```bash
   ip -4 addr show wlan0
   ```

7. 以後は、画面とキーボードを外し、PC から `ssh ubuntu@<IPアドレス>` でログインできる。PC のコンテナでは `set_domain_id 45`、`robot_connect <IPアドレス>` を実行する（専攻科実験の手順書 R1 と同じ）。

## 4. 気をつけること

- キーボードの配列が日本語（JIS）と英語（US）で違うと、記号（`-`、`@`、`_` など）が違うキーに入ることがある。パスワードや Wi-Fi の名前に記号があるときは、画面の表示で確かめる。
- Wi-Fi のパスワードを打ち間違えたときは、同じ名前でもう一度 `sudo chibamouse-wifi add 自分のWi-Fiの名前` を実行すると、パスワードが上書きされる。登録した Wi-Fi は `sudo chibamouse-wifi list` で見られる。
- LAN ケーブルでルータにつなげば、画面とキーボードがなくても、有線でログインして同じ設定ができる。

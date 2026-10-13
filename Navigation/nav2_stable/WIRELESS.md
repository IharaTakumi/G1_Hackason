# 無線で Nav2 を動かす（当日手順・最短版）

更新: 2026-10-13 / ⚠️ **この手順は実機で未検証**（部品の heartbeat・ssh・`g1up.sh --enable`・`patrol_ctl.sh` は検証済み）

**有線で立ち上げてから、ケーブルを抜いて無線で操作する。** 立ち上げ（`g1up.sh`、§7 の照合、RViz の目視）は
これまでどおり有線で行う。無線化で増える作業は約 10 分。

## 方針: DDS は無線に通さない

9/15 にテザリングで踏んだ失敗は、すべて DDS を無線に通そうとして起きた
（[Planning.md](Planning.md) ⑩⑪：「トピックは見えるのにデータが来ない」「RViz が真っ暗」）。
無線で要るのは次の 2 つだけで、どちらも DDS を使わない。

| 無線に乗るもの | 方式 | 根拠 |
|---|---|---|
| heartbeat（操作PC → PC2、5Hz） | UDP。PC2 は全インターフェースで受信 | `g1_sdk_bridge_cpp/src/heartbeat.cpp` の `INADDR_ANY` |
| 走行許可・巡回の開始停止・状態確認・停止 | ssh で PC2 上の ROS コマンドを叩く | `g1up.sh --enable` / `tools/patrol_ctl.sh` / `Navigation/real/nav_stop.sh` |

PC2 の CycloneDDS は eth0 固定のまま触らない。

**代償**: 歩行中は RViz が見られない（状態は `status`、機体は目視）。RViz の「2D Goal Pose」も使えないので、
**Goal は巡回路（yaml）で与える。** 通信断で止まるまでの時間は 2.0 秒（`g1up.sh` の既定）。
止まるまでに約 0.6m 進むので、D-31 の受入目標 0.35m は満たせない（[QUESTIONS.md](QUESTIONS.md) Q12 の (b)）。

## 前提（当日の前に済ませておく）

**ルータ**: TP-Link **TL-WR1502X**（AX1500 トラベルルータ。9/29 に用意したもの）の **`Physical_AI_5G`（5GHz）**。
構成・パスワードの在処・5GHz の設定は `origin/main` の `Common/network/WIFI_ROUTER.md`（このブランチには未取り込み）。
9/29 時点で G1 Jetson（PC2）は `192.168.0.82`（DHCP なので当日 `wifi_ops.sh ip` で確かめる）。

- [ ] **2.4GHz の `Physical_AI` には繋がない**（9/29 の実測で遅延が最大 1.4 秒）。Linux の操作PC は `iw reg get` が JP であること
- [ ] ルータの AP 分離（端末同士の通信遮断）が無効であること。会場のフリーWiFi（`Fujitsu_free_Wi-Fi`）は AP 分離があるので使えない
- [ ] PC2 が `Physical_AI_5G` に繋がっている（9/29 に設定済みのはず）。繋がっていなければ有線で入って:
      `ssh -t g1 'sudo nmcli device wifi connect Physical_AI_5G password <Slack で確認>'`
- [ ] 操作PC に `g1_heartbeat_sender` がある（`command -v g1_heartbeat_sender`）
- [ ] **巡回路**: `config/patrol_room_a.yaml`（6点。**9/25 に room_a で1周完走した版**）を使う。
      無線中は RViz で引き直せないので、変えるなら有線のうちに
- [ ] **今回の変更を PC2 へ配置する**（`g1up.sh` に `--tf-timeout` を足した）:
      `rsync -a deploy/pc2_humble/g1up.sh g1:/home/unitree/g1_nav2/`（[deploy/README.md](deploy/README.md) の手順）

## 当日の流れ

道具はすべて [tools/wifi_ops.sh](tools/wifi_ops.sh)（操作PC で叩く）。

### 1. 有線で立ち上げる（いつもどおり）

[safety/runbook_nav2_session.md](safety/runbook_nav2_session.md) のとおり、PC2 で:

```bash
~/g1_nav2/g1up.sh --tf-timeout 1.0 \
  --patrol ~/g1_nav2/g1_ws/install/g1_navigation/share/g1_navigation/config/patrol_room_a.yaml
```

`--tf-timeout 1.0` は 9/25 の巡回1周と同じ値（内蔵SLAM を 1801 で上げ直すと odom が約4Hz に落ち、
既定 0.5 秒では enable 直後に必ず `tf_stale` FAULT になった）。**RViz の目視確認まで**済ませる。有線の heartbeat（`g1_heartbeat_sender --host 192.168.123.164`）も上げておく。
**まだ `enable` しない。**

### 2. 無線の準備（ケーブルはまだ挿したまま）

```bash
cd <repo>/Navigation/nav2_stable
tools/wifi_ops.sh ip              # 有線経由で PC2 の wlan0 の IP を表示
export G1_WIFI_IP=192.168.x.y     # 上で出た IP
tools/wifi_ops.sh check           # ping / 無線 ssh / 省電力 / DDS が eth0 固定か
tools/wifi_ops.sh fix             # 省電力が on と出たときだけ（PC2 の sudo パスワードを聞かれる）
tools/wifi_ops.sh clearloop start # costmap の消し残りを 20 秒ごとに消す（9/25 の1周はこれ込み）
tools/wifi_ops.sh handover        # heartbeat を有線→無線へ。「ケーブルを抜いてよい」が出るまで待つ
```

⚠️ `clearloop` は暫定処置（[findings/collision_ahead_cause_20260925.md](findings/collision_ahead_cause_20260925.md) §4）。
これが無いと、死角の中に消し残りが溜まって `collision ahead` で止まる。
**消す瞬間は本物の障害物も消え、死角(〜1.6m)の中のものは立ち直らない。人が見ている場でのみ使う。**

`handover` は、無線の heartbeat を足してから有線の heartbeat を止め、**無線だけで** alive かを確かめる。
alive にならなければ有線の heartbeat を戻して止まる（READY のうちなので機体には影響しない）。

### 3. ケーブルを抜いて歩かせる

```bash
tools/wifi_ops.sh status          # message: READY / operator_heartbeat: alive を確認
# 発進ゲートを開く（意図的に自動化していない。g1up.sh の最後に出る ④ の手順）
tools/wifi_ops.sh enable          # → NAVIGATING になること
tools/wifi_ops.sh patrol start
tools/wifi_ops.sh status          # 歩いている間はこれと目視で見る
tools/wifi_ops.sh patrol pause    # 止める（次の start は同じ点から）
```

発進ゲートを開くには `sudo` が要るので、無線の ssh で入って打つ:
`ssh -t unitree@$G1_WIFI_IP` → g1up.sh の最後に表示された ④ のコマンド。

### 止める

| 場面 | すること |
|---|---|
| まず止めたい | **純正リモコン**（停止係が常に持つ） |
| ナビを丸ごと止める | `G1_SSH=unitree@$G1_WIFI_IP bash ../real/nav_stop.sh --no-ros` |
| 巡回だけ止める | `tools/wifi_ops.sh patrol pause` |
| FAULT から戻す | `tools/wifi_ops.sh clear` → 原因を確かめてから `enable` |
| 終わったら | `tools/wifi_ops.sh clearloop stop`（`nav_stop.sh` はこのループを止めない） |

通信が途絶えたら、2.0 秒で `operator_lost` → FAULT になって止まる。**自動では再開しない**（人が `clear_fault` を叩く）。

## 時間に余裕があればやること

- `tools/wifi_ops.sh hbtest 60` で heartbeat の遅れを測る。最大が 0.5 秒未満なら、次回から
  `g1up.sh --operator-timeout 1.0` で受入目標 0.35m を満たせる（Q12 の (c)）
- 歩行中に `tools/wifi_ops.sh hb-stop` で heartbeat を止め、FAULT で止まることを確かめる（通信断の試験）

## 分かっていないこと

- 当日のルータで UDP（heartbeat）が端末間を通るか → `handover` が確かめる
- 当日のルータでの heartbeat の遅れ → `hbtest`
- 機体から離れたとき・壁の陰での電波の強さ

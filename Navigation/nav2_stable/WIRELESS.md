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

- [ ] ルータの条件: **操作PC と PC2 が同じルータに繋がり、端末同士が通信できる**こと
      （AP 分離・プライバシーセパレータが無効）。会場のフリーWiFi（`Fujitsu_free_Wi-Fi`）は AP 分離があるので使えない
- [ ] 5GHz を使う（2.4GHz は 9/29 の実測で遅延が最大 1.4 秒）。Linux の端末は `iw reg get` が JP であること
- [ ] PC2 をそのルータの WiFi に繋いでおく（有線で入って）:
      `ssh -t g1 'sudo nmcli device wifi connect <SSID> password <パスワード>'`
- [ ] 操作PC に `g1_heartbeat_sender` がある（`command -v g1_heartbeat_sender`）
- [ ] 巡回路の yaml がある（`g1up.sh --patrol` に渡すもの）。**無線では RViz で引けない**ので、
      有線のうちに `record_waypoints.py` か `patrol_ctl.sh teach` で作っておく

## 当日の流れ

道具はすべて [tools/wifi_ops.sh](tools/wifi_ops.sh)（操作PC で叩く）。

### 1. 有線で立ち上げる（いつもどおり）

[safety/runbook_nav2_session.md](safety/runbook_nav2_session.md) / `g1up.sh --patrol <yaml>` で、
**RViz の目視確認まで**済ませる。有線の heartbeat（`g1_heartbeat_sender --host 192.168.123.164`）も上げておく。
**まだ `enable` しない。**

### 2. 無線の準備（ケーブルはまだ挿したまま）

```bash
cd <repo>/Navigation/nav2_stable
tools/wifi_ops.sh ip              # 有線経由で PC2 の wlan0 の IP を表示
export G1_WIFI_IP=192.168.x.y     # 上で出た IP
tools/wifi_ops.sh check           # ping / 無線 ssh / 省電力 / DDS が eth0 固定か
tools/wifi_ops.sh fix             # 省電力が on と出たときだけ（PC2 の sudo パスワードを聞かれる）
tools/wifi_ops.sh handover        # heartbeat を有線→無線へ。「ケーブルを抜いてよい」が出るまで待つ
```

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

通信が途絶えたら、2.0 秒で `operator_lost` → FAULT になって止まる。**自動では再開しない**（人が `clear_fault` を叩く）。

## 時間に余裕があればやること

- `tools/wifi_ops.sh hbtest 60` で heartbeat の遅れを測る。最大が 0.5 秒未満なら、次回から
  `g1up.sh --operator-timeout 1.0` で受入目標 0.35m を満たせる（Q12 の (c)）
- 歩行中に `tools/wifi_ops.sh hb-stop` で heartbeat を止め、FAULT で止まることを確かめる（通信断の試験）

## 分かっていないこと

- 当日のルータで UDP（heartbeat）が端末間を通るか → `handover` が確かめる
- 当日のルータでの heartbeat の遅れ → `hbtest`
- 機体から離れたとき・壁の陰での電波の強さ

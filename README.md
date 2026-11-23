# BlueBoat 航行監視の研究

Blue Robotics BlueBoat（Raspberry Pi、Navigator、ArduRover 4.7.0-STABLE）で、固定GPS座標への接近精度と異常時の停止を評価する研究です。開発環境は Windows 11 と WSL2 Ubuntu 22.04 です。

## 構成と実装状況

ルート直下に Python ソース、test_ で始まる unittest、設計資料を配置しています。blueboat_navigator_live_v3.py は GNSS・EKF・HEARTBEAT・位置の鮮度を監視する DRY RUN です。実機自律航行は未完成です。判定仕様は blueboat_dry_run_v3.md、EKF の扱いは blueboat_ekf_integration.md にあります。

coex_target_receiver.py は目標データ受信、replay_blueboat_tlog.py と analyze_blueboat_tlog_gaps.py は MAVLink 記録の再生・解析用です。生ログと通信キャプチャは公開対象外です。

## オフラインテスト

Python 3 の仮想環境で依存関係を準備し、リポジトリのルートで実行します。依存関係のインストールには事前に取得したパッケージも利用できます。

```sh
python -m pip install -r requirements.txt
python -m unittest discover -p "test_*.py" -v
```

## 安全上の注意と次の開発

thruster_ で始まる試験スクリプトは実際に MAVLink の手動制御指令を送信します。check_mavlink_uplink.py も送信を行います。オフライン評価でこれらを起動しないでください。


## 固定座標のオフライン評価（v4）

Python 3.11 以降で blueboat_navigator_v4.Trial(trial_id, lat, lon) を作成し、復号済みの記録を observe(msg, at) に受信順で渡します。at は試行開始を基準にした単調秒です。無通信区間も poll(at) で進めます。permit(at) は操作者の許可を記録し、その後の HEARTBEAT で ARM 済みの GUIDED を観測して接近監視を開始します。手動介入は manual_intervention(at) または observe の manual=True で伝えます。入力終了時は finish(at)、評価記録の取得は jsonl() を使います。同じ受信時刻の入力をまとめ、次の時刻が届いた時、または同時刻の全入力が揃って close_timestamp() を呼んだ時に判定します。state と jsonl() は確定済みの時刻までを返します。時刻を閉じた後に同じ時刻や過去の入力が届くと中断します。finish(at) は末尾の時刻も確定します。

健全な開始位置と元目標から固定接近点を一度計算します。元目標までの許容距離は 3 m 超、100 m 以下です。元目標の 3 m 手前という設定は仮値です。接近点の 1 m 以内で停止要求を記録し、その後に届いた GLOBAL_POSITION_INT の水平速度と位置を評価します。試験用の停止条件は速度 0.2 m/s 以下、観測点間の最大距離 0.5 m 以下です。受信時刻と機体時刻の双方で観測間隔 1.5 秒以下、継続時間 2 秒以上を要します。停止成立は記録上の判定で、GNSSの偏りや観測間の運動は独立した測位・速度記録との照合が必要です。

JSON Lines には試行ID、相対受信秒、健康状態、GPS（受信秒・fix・衛星数・eph）、EKFフラグ、機体の位置サンプル時刻（boot_ms）、元目標と接近点それぞれの距離、状態遷移、停止要求と根拠サンプルを記録します。外部ログと比較するときは試行IDと受信時間の原点も保存してください。

上記の unittest discovery は合成系列を含みます。11月19日の実記録2本も検証する場合は環境変数 BLUEBOAT_NOV19_LOG_DIR に元の .tlog 2本を含むローカルディレクトリを指定してください。未指定なら実記録のテスト1件をスキップします。SITL前には速度・変位・時間の閾値と入力時刻の整合性を検証し、実機試験前には独立した停止手段と介入手順の確認が必要です。

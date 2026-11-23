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

次の段階は固定接近点を使う v4 のオフライン監視と停止観測の検証です。停止距離 3 m は仮値です。SITL と実機での検証は今後の課題です。

# BlueBoat v3：オフラインDRY RUN

v2を未変更で残し、v3は入力を注入して目標点の緯度・経度だけを計算する。実機接続、ソケット、MAVLink指令生成・送信、ARM、パラメータ変更、実機停止制御はいずれも実装していない。ラッチは**目標点の計算を禁止するだけ**で、実機を停止させる機能ではない。

## 状態とラッチ

- 起動時にGPS No Fix、EKF未受信、またはEKF異常なら、計算せず正常な入力を待つ。観測の `flags=135` とGPS No Fixでは、3D Fixなし・一定位置モード・絶対水平位置なしを記録する。正常化後も待機中の目標は再利用せず、新たな有効パケットだけを評価する。
- 一度、有効なHEARTBEATのもとでGNSSとEKFの両方が健全になった後は、位置推定の喪失または期限切れを即座にラッチする。COEX目標の到着を待たない。初回受信後のHEARTBEAT断と、最初の正常受理後のCOEX断も、それぞれ独立してラッチする。
- 初期モードは従来どおり `MANUAL` / `HOLD` のいずれか。起動後のモード変更（`HOLD` から `MANUAL` を含む）、ARM検知、明示的な手動介入通知、COEXセッション変更はラッチ対象。モードが変わらない操舵をHEARTBEATだけでは検知しない。
- 最初のラッチ理由を保持し、旧目標のキューやラッチ解除APIは持たない。ラッチは同一インスタンス内で維持し、プロセスをまたぐ永続化はしない。新セッションや再起動は新たなDRY RUNの開始にすぎず、自律制御の再許可ではない。v3に自律制御許可の経路は存在しない。

## 入力インターフェース

`DryRunNavigator(coex_source=(送信元IPv4, UDP送信元ポート), coex_session_id=UUIDv4)` を使用する。

- `observe_telemetry(msg, received_at=...)`：モックや記録済みのMAVLinkメッセージを受信順に1件ずつ渡す。受信時刻は同じ単調時計系とし、受信の前後を評価して回復パケットで異常が隠れないようにする。
- `accept_target(raw, source, wall_now=..., monotonic_now=...)`：受信データグラムをその都度検証し、条件を満たせば `DryRunWaypoint`、満たさなければ `None` を返す。`wall_now` はUNIX秒、単調時刻はテレメトリと同じ時計系。第1タスク形式（`session_id`・`seq` なし）は下位互換にせず拒否する。新形式については `coex_target_protocol.md` を参照する。送信元照合は認証ではない。
- `poll(monotonic_now=...)`：データが途絶えたときも、呼び出し側が定期実行する。返す理由は現在の機体側判定とラッチ状況であり、空でもCOEX目標の受理や操作者の許可を意味しない。`manual_intervention()` は信頼できる操作者イベントを渡すための入口であり、RC入力監視は未実装。
- `INFO` で受理したEKF報告間の受信間隔と目標点、`WARNING` で禁止理由、EKFロガーの `DEBUG` でフラグと分散フィールドを記録する。EKFの3秒は暫定値のまま。GNSS/位置更新、EKF受信、COEXの最後の正常受理および `sent_at` を別々に評価する。分散値の閾値は未設定。

## 実機DRY RUNの確認項目（受動記録とオフライン再生）

1. 操作者が承認済みの読み取り専用監視経路で、HEARTBEAT・GPS_RAW_INT・GLOBAL_POSITION_INT・EKF_STATUS_REPORTと受信時刻を記録する。v3から機体へ接続しない。ARM・モーター作動・航行指令・`FS_EKF_ACTION` を含む設定変更は行わない。
2. `flags=135` とGPS No Fixのログは、受信順・間隔を保ってオフライン再生し、起動待機の理由が出て目標を生成しないことを確認する。正常から異常への切り替え、通信断・復旧、HOLD→MANUALはモックまたは記録のオフライン加工で確認する。
3. 約0.7 Hzは単発観測なので、複数の記録からEKF受信間隔の中央値・上位分位・最大値と3秒超過の回数を確認する。受信処理の滞留と実際の配信間隔は区別する。許容する検知遅れと併せて評価し、観測だけで閾値を緩めない。

```powershell
.\.venv\Scripts\python.exe -m unittest -v test_coex_target test_coex_target_receiver test_blueboat_ekf_health test_blueboat_navigator_live_v3
```

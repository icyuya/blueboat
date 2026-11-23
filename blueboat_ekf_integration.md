# BlueBoat GNSS・EKF判定の統合メモ

## 判定方針

`blueboat_ekf_health.py` は接続・送信しない。`blueboat_navigator_live_v2.py` とCOEX診断受信器は未変更。`ekf_block_reasons()` は、受信がない、受信時刻が不正、期限切れ、フラグ形式が不正、`EKF_POS_HORIZ_ABS` が未設定、または `EKF_CONST_POS_MODE`・`EKF_UNINITIALIZED`・`EKF_GPS_GLITCHING` のいずれかが設定されていれば、禁止理由を返す。相対位置や予測位置は絶対位置の代用にしない。禁止理由が空でも、開始許可・機体状態・安全性全体の保証にはならない。

受信側の単調時計で受信から3秒以上を期限切れとする。3秒は暫定値であり、対象機の報告間隔と受信遅延を読み取り専用で確認してから決める。未受信時はブロックしたままにし、このモジュールからメッセージ配信設定を要求しない。

## 次回の接続点

1. 既存MAVLink受信ループと同じ経路で `EkfHealthMonitor(system_id=BOAT_SYSID, component_id=BOAT_COMPID)` を使う。受信直後の `time.monotonic()` を `observe(msg, received_at=...)` に渡す。監視器はメッセージ種別と送信元IDを照合するが、IDは認証ではない。接続が変わったら旧サンプルを破棄する。`EKF_STATUS_REPORT` に送信元時刻がないため、ローカル受信時刻だけではバッファ滞留や再送を区別できない。受信キューの滞留を制限し、再接続時には旧データを使わない。
2. v2相当の `current_position()` による3D Fix、衛星数、HDOP、GPS/位置更新、位置有効性をそのまま適用する。`block_reasons(monotonic_now=..., gnss_reason=None if position is not None else gps_reason)` に結果を渡す。位置が無効なら必ず禁止理由を渡し、EKF正常でGNSS異常を相殺しない。
3. COEXの `CoexTargetGate` で送信元・セッション・連番・鮮度を別に検証する。両方の入力の期限を目標到着時だけでなく定期的に評価し、HEARTBEAT・ARM・モード・既存安全ラッチ・操作者の許可を含む全条件が揃うまで目標生成へ進めない。COEXの `sent_at` だけは壁時計で比較し、EKFの受信時刻と直接比較しない。
4. 異常を検知したら統合側が必要なラッチを維持する。異常サンプルが同じ受信バッチ内の正常サンプルで上書きされる前に評価する。パケット受信やEKF回復だけで、既存ラッチを解除したり旧目標で再開したりしない。このモジュールは健全性評価のみで、ラッチの解除や再許可は扱わない。

## ログと未決事項

- `block_reasons()` は、新しい禁止理由を `WARNING` に出力する。同じ理由の繰り返しは抑制する。統合側で通常の `logging` ハンドラーを設定する。
- `blueboat_ekf_health` ロガーを `DEBUG` にすると、報告ごとに送信元、受信単調時刻、フラグと、`velocity_variance`・`pos_horiz_variance`・`pos_vert_variance`・`compass_variance`・`terrain_alt_variance`・`airspeed_variance` の受信値を記録する。欠落フィールドは `None`。値の大小は今回の判定条件にしない。
- Rover 4.7.0のEKF3では、一部の `*_variance` にイノベーション整合性のテスト比の平方根が入る。フィールド名だけで位置誤差の分散（m²）と解釈しない。MAVLink定義と機体の実装・ログを照合し、通常時と異常時の分布を確認したうえで、閾値・継続時間・欠損/非有限値の扱いを設計する。

参照: [MAVLink ArduPilotMega: EKF_STATUS_REPORT](https://mavlink.io/en/messages/ardupilotmega.html#EKF_STATUS_REPORT)、[ArduPilot Rover 4.7.0: EKF3出力](https://github.com/ArduPilot/ardupilot/blob/Rover-4.7.0/libraries/AP_NavEKF3/AP_NavEKF3_Outputs.cpp)。

オフライン試験:

```powershell
.\.venv\Scripts\python.exe -m unittest -v test_blueboat_ekf_health test_coex_target test_coex_target_receiver
```

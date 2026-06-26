# STAGE 1 設計: 「Susumuに脳をつける」LLMエージェント・コア

> 話しかけたら、その意図を理解して安全に動き・喋り・見るロボット。
> 以降の STAGE 2（賢い目）/ STAGE 3（追従・案内）/ STAGE 4（巡回）は、
> すべてこの STAGE 1 のエージェントに **ツールを足すだけ** で積み上がる。

---

## 0. このドキュメントの位置づけ

4ステージ・ロードマップの背骨である STAGE 1 の具体設計。
方針: 既存資産を**作り直さず配線する**。新規実装は「足りないツール」と「安全な配線」だけに絞る。

### 全体ロードマップ（再掲）

```
        ┌──────────── クラウドLLM（Gemini/Claude on Vertex AI）= 脳 ────────────┐
        │   意図理解 ・ 行動判断 ・ 見たものの説明 ・ 会話                       │
        └──┬───────────┬──────────────────┬──────────────────┬────────────────┘
     耳(ASR)│       口(TTS)│              目(D435i+VLM)│              足(Nav2)│
   Google   │   AivisSpeech│        物体/人/表情認識      │     巡回・追従・案内 │
```

- **STAGE 1**（本書）: 耳・口・足（基本移動）・目（オンデマンド撮影）を脳に配線
- **STAGE 2**: 目を常時化（`susumu_object_perception` を脳のツールに）
- **STAGE 3**: 追従・案内（`hri_face_detect` 等 + Nav2 のゴール送信ツール）
- **STAGE 4**: 自律巡回 + 異常通知

---

## 1. 現状の事実（調査で確定）

### 1.1 既にある資産（再利用する）

| 資産 | 場所 | STAGE 1 での役割 |
|---|---|---|
| `susumu_agent` | `src/susumu_agent` | **エージェント本体**。`/from_human`→LLM→`/cmd_vel`+`/to_human`。Vertex AI 接続済み（`.env` あり） |
| `RobotTools` | `susumu_agent/agent/tools.py` | move/rotate/curve/execute_sequence/observe/macro 等の function calling ツール群 |
| `ROS2Robot` | `susumu_agent/robot/ros2_robot.py` | 実機向け cmd_vel publisher。TwistStamped 対応・stop_event 対応 |
| Google Cloud ASR | `src/susumu_asr` | 耳。`stt_option.launch.py`（Silero VAD + google_cloud） |
| AivisSpeech TTS | `tts_option.launch.py` / `audio_option.launch.py` | 口。`to_human_2_speak_ros` が `/to_human`→`/speak` 変換 |
| `twist_mux` | `twist_mux.launch.py` | **優先度レーン定義済み**: `/joy_cmd_vel`(10) / `/nav_cmd_vel`(5) / `/llm_cmd_vel`(3) → `/input_twist` |
| `laserscan_filter_node` | `susumu_robo/` | 安全層。`/input_twist`→（障害物で直進ブロック）→`/cmd_vel` |
| Nav2 | `nav2.launch.py` | 足の本命。STAGE 1 では「地点へ行く」を足す土台 |
| `agent_option.launch.py` | `launch/` | エージェント起動 launch（既存。STAGE 1 で改修） |

### 1.2 配線ギャップ（STAGE 1 で埋める）

```
【今】 susumu_agent ──/cmd_vel──▶ （安全層・mux を素通り）

【あるべき】
  susumu_agent ──/llm_cmd_vel──▶ twist_mux ──/input_twist──▶ laserscan_filter ──/cmd_vel──▶ モータ
                                    ▲                              （障害物で直進停止）
                          joy(10) > nav(5) > llm(3)
```

ギャップ一覧:

1. **エージェントの出力先が `/cmd_vel` 固定** → `/llm_cmd_vel` に変えて mux + 安全層を通す
   （`config.yaml` の `robot.cmd_vel_topic` を `/llm_cmd_vel` にするだけで可能）
2. **移動が時間ベースのオープンループのみ** → 「キッチンへ行って」等の **地点ナビ（navigate_to）ツールが無い**
3. **`observe`（カメラ）が実機未配線** → `image_topic` を D435i (`/camera/...`) に合わせる
4. **耳・口・足・脳がバラバラ launch** → STAGE 1 用の統合 launch が無い
5. **緊急停止（「ストップ」）が mux 最上位を取れていない** → 安全上、停止は最優先レーンへ

---

## 2. STAGE 1 のゴール（受け入れ条件）

実機で、マイクに話しかけて以下が成立する:

1. **会話**: 「こんにちは」→ AivisSpeech で自然に返事
2. **基本移動（安全付き）**: 「ゆっくり前進して」→ 前進。ただし**前に障害物があれば laserscan_filter が直進を止める**
3. **見る**: 「何が見える？」→ D435i 画像を LLM が解説して喋る
4. **緊急停止**: 「ストップ」→ 即時停止が**他のどのレーンより優先**される
5. **（足の入口）地点ナビ**: 「元の場所に戻って」程度の単純な navigate_to が1つ動く（地図ありの場合）

> 5 は STAGE 3 への布石。STAGE 1 では「ツールの口を1つ開ける」ところまで。

---

## 3. アーキテクチャ（STAGE 1 完成形）

```
 ┌── 耳 ──────────┐   ┌── 脳: susumu_agent ────────────────┐   ┌── 口 ───────────────┐
 │ susumu_asr     │   │  /from_human ─▶ LLM(function call) │   │ to_human_2_speak_ros│
 │ Silero VAD     ├──▶│   tools:                            ├──▶│  /to_human ─▶ /speak │
 │ + Google ASR   │   │    move/rotate/curve/observe        │   │  ─▶ AivisSpeech      │
 │ ─▶ /from_human │   │    navigate_to (新) / cancel_nav(新)│   └─────────────────────┘
 └────────────────┘   │    stop(緊急)                       │
                      │   出力:                              │
                      │    /llm_cmd_vel   (基本移動)         │
                      │    /nav2 goal     (地点ナビ・新)     │
                      │    /to_human      (発話)             │
                      └───────┬──────────────┬──────────────┘
                              │              │
                   /llm_cmd_vel(prio3)   NavigateToPose action
                              ▼              ▼
                      ┌── twist_mux ──┐   ┌── Nav2 ──┐
                      │ joy>nav>llm   │   │ 出力:     │
                      │ ─▶/input_twist│◀──┤/nav_cmd_vel(prio5)
                      └──────┬────────┘   └──────────┘
                             ▼
                  ┌── laserscan_filter（安全層）──┐
                  │ 障害物で直進ブロック          │
                  │ ─▶ /cmd_vel                   │
                  └──────────┬───────────────────┘
                             ▼
                          モータ（botwheel）
```

ポイント:
- **基本移動（llm_cmd_vel）も自律ナビ（nav_cmd_vel）も、最終的に同じ安全層（laserscan_filter）を必ず通る**
- joy（ゲームパッド）が最優先 = 人が常にオーバーライドできる
- 緊急停止は専用の最優先レーン（後述 §5）

---

## 4. 実装タスク（STAGE 1）

優先度順。各タスクは独立にテスト可能。

### T1. エージェント出力を安全レーンに通す ★最優先・最小
- `susumu_agent/config.yaml`: `robot.cmd_vel_topic: "/llm_cmd_vel"`、`cmd_vel_stamped: true`
- 確認: `ros2 topic pub /from_human ... "ゆっくり前進"` → `/llm_cmd_vel` に出る → mux 経由で `/input_twist` → 障害物を置くと `/cmd_vel` の直進が止まる
- **これだけで「安全付きで喋って動く」が成立**（observe/nav なしでもデモ可能）

### T2. observe を D435i に配線
- `config.yaml`: `image_topic` を D435i のカラー画像トピックに（`d435i.launch.py` の出力名を確認して合わせる）
- `interface.camera_send_to_cloud: true`（既定 true）
- 確認: 「何が見える？」で前方の物体を説明する

### T3. 統合 launch を作る（耳・脳・口・足を1コマンドで）
- 新規 `launch/agent_full.launch.py`（または `agent_option.launch.py` を拡張）:
  - include: `stt_option`（耳）/ `audio_option` or `tts_option`（口）/ `agent`（脳）
  - 実機の足は `robo_indoor.launch.py` 側で起動済み前提（mux・安全層・モータ）
  - 起動順は TimerAction で TTS を最後に（既存 `audio_option_debug` の知見に倣う）
- 確認: 1 launch + robo_indoor で、話しかけ→動く・喋る が成立

### T4. navigate_to ツールを追加（足の入口）★STAGE 3 への布石
- `RobotTools` に `navigate_to(place: str)` を追加し `get_all_tools()` に登録
- 実装: 名前付き地点テーブル（`config/places.yaml`: `{キッチン: {x,y,yaw}, 玄関: {...}}`）を引いて
  Nav2 の `NavigateToPose` アクションへゴール送信
- `cancel_navigation()` も追加（「やめて」で中断）
- 地図が無い/Nav2 未起動なら `report_unsupported` 相当を返す（graceful degradation）
- 確認: 地図ありで「玄関に行って」→ Nav2 がゴールへ走り、laserscan_filter で安全担保

### T5. 緊急停止を最優先レーンに（安全強化）
- 「ストップ」は LLM を経由しない即時停止（README に既述）。これを twist_mux の
  最優先トピック（例 `/estop_cmd_vel` priority 100）か、laserscan_filter 直前のラッチ停止に流す
- 確認: 走行中に「ストップ」で即停止、かつ他レーンより必ず勝つ

---

## 5. 安全設計（必須・妥協しない）

- **二重の安全**: ① エージェント側 `compliance_mode` / `human_presence_max_speed` / watchdog ② ロボ側 `laserscan_filter`（ハード障害物停止）。STAGE 1 では②を必ず経由させる（T1）
- **人がいつでも勝てる**: ゲームパッド joy が mux 最優先。手動オーバーライド可能
- **緊急停止の独立性**: 「ストップ」は LLM 応答を待たず、最優先レーンで即時（T5）
- **クラウド前提のフェイルセーフ**: LLM タイムアウト/通信断時は move を発火させない。watchdog で cmd_vel が途切れたら停止（mux timeout 2.0s が既に効く）
- **画像送信の明示**: `camera_send_to_cloud` がクラウドへ画像を送ることを認識（プライバシー）。オフ運用も可能に

---

## 6. クラウド方針（ユーザー選択: クラウドOK）

- LLM: Vertex AI（Gemini 2.5 Flash/Pro）。`config.yaml` で切替可。Claude on Vertex も選択肢
- ASR: Google Cloud Speech-to-Text（既存）
- VLM: observe は LLM のマルチモーダル入力に画像を渡す方式（既存実装済み）
- コスト管理: `cost_control`（daily_command_limit / observe_limit / alert）が既にある。デモ運用で有効化

---

## 7. デモ・シナリオ（見せ場 = ユーザー目的）

STAGE 1 完成時の 90 秒デモ台本案:

1. 「Susumu、こんにちは」→ 振り向き or 一言返事（口）
2. 「ゆっくり前に来て」→ 近づく。途中で手をかざす → **障害物検知で安全停止**（安全層の見せ場）
3. 「何が見える？」→ 「机の上にマグカップがありますね」（目の見せ場）
4. 「ストップ」→ 即停止（安全の信頼感）
5. （地図あれば）「元の場所に戻って」→ 自走で帰還（足の予告編 → STAGE 3 へ続く）

---

## 8. 次の STAGE への接続点

| STAGE | 足すもの | STAGE 1 のどこに乗るか |
|---|---|---|
| 2 賢い目 | `susumu_object_perception` を購読し「今見えてる物」を常時 state 化 | observe ツールを「常時認識結果の参照」に拡張 |
| 3 追従/案内 | `hri_face_detect`/`hri_engagement` で人を追う + navigate_to を本格化 | T4 の navigate_to / Nav2 配線をそのまま使う |
| 4 巡回 | 名前付き地点を巡回するループ + 異常を observe で検知し通知 | T4 の places.yaml + STAGE 2 の目 |

---

## 付録: 確認した主要ファイル

- `susumu_agent/agent/tools.py` — function calling ツール定義
- `susumu_agent/robot/ros2_robot.py` — 実機 cmd_vel 実装（時間ベース・オープンループ）
- `susumu_agent/config.yaml` — robot/llm/interface/auth/cost_control 設定
- `susumu_robo/launch/twist_mux.launch.py` — llm/nav/joy 優先度レーン（既定義）
- `susumu_robo/launch/agent_option.launch.py` — エージェント起動 launch
- `susumu_robo/susumu_robo/laserscan_filter_node.py` — 安全層（/input_twist→/cmd_vel）

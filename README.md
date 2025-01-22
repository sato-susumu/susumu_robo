# susumu_robo
## 方向性
- ROS2搭載でパワフルながら静かに動く小型移動ロボット
- なるべく市販品を使う、扱いやすく安全な電源を使う
## TODO
- [x] タッチ対応コマンド実行メニュー
- [x] 機器を接続する
- [x] 起動後にすぐに動かせるようにする
- [x] 3Dプリンターでカバーを作る
- [x] ジョイパッドによる操作
- [x] LiDARを使って障害物を検知し、停止する
## TODO ?
- [x] デプスカメラ
- [x] 音声合成
- [ ] 複数cmd_velの受付 (cmd_vel_mux)
- [x] ReSpeakerマイク
- [ ] 音声認識
- [ ] ウェイクワード判定
- [ ] 画像認識、追跡
- [ ] 地図作成、ナビゲーション
- [ ] LLM
- [x] アバター
- [ ] リップシンク
- [ ] 表情認識
- [x] LED
- [ ] ARuCoマーカー認識

## ハードウェア
### 機器
| Item                                                                                                                                                                                                                                                   | 数量 | 重量                                                   | 補足・備考                                                                             |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----|------------------------------------------------------|-----------------------------------------------------------------------------------|
| [Prime Power Bank (27650mAh, 250W)](https://www.ankerjapan.com/products/a1340)                                                                                                                                                                         | 1  | [約665g](https://www.ankerjapan.com/products/a1340)   | USB-C1 / C2 出力：5V⎓3A / 9V⎓3A / 12V⎓1.5A / 15V⎓3A / 20V⎓5A / 28V⎓5A (最大140W)       |
| [DP100 安定化電源](https://www.switch-science.com/products/9414)                                                                                                                                                                                            | 1  | [約95g](https://www.switch-science.com/products/9414) | 入出力電源スペック: 入力 100-240V AC, 出力 0-30V DC / 0-5A                                     |
| [DDT M0601C_111](https://www.switch-science.com/products/764) ＋ タイヤ           | 2  | 約765g                                                | モーター。[DDSM115 ダイレクトドライブサーボモーター](https://www.switch-science.com/products/9628)と一緒？ |
| [8HP-CAPLCD Monitor](https://www.waveshare.com/8hp-caplcd-monitor.htm)                                                                                                                                                                                 | 1  | 623g                                                 | 8インチ静電容量式タッチディスプレイ、1280×800、HDMI/Type-Cディスプレイインターフェース SKU:24483                   |
| [プラス・マイナス分岐ターミナル](https://www.amon.jp/products2/detail.php?product_code=3360)                                                                                                                                                                          | 1  | -                                                    | DC12V車専用、使用可能電流:5A、使用可能電力:60W以下、適合コードサイズ:0.12〜1.25sq相当(AWG26〜16)                  |
| [Waveshare USB TO RS485](https://www.waveshare.com/usb-to-rs485.htm)                                                                                                                                                                                   | 1  | -                                                    | USB to RS485インターフェースコンバーター                                                        |
| [Livox Mid-360](https://www.livoxtech.com/jp/mid-360)                                                                                                                                                                                                  | 1  | [約265g](https://www.livoxtech.com/jp/mid-360)        | 3D LiDAR。水平360°、垂直59°（-7°〜52°）                                                    |
| [TRIGKEY G5 Mini PC](https://trigkey.com/products/trigkey-g5-mini-pc-w11-desktop-12th-gen-intel-n1004core-up-to-3-4ghz-16g-ddr4-500g-pcie1-ssd-dual-ethernet-mini-computer-support-micro-computer-w10-pro-wifi-6-bt5-2-dual-hdmi-triple-screen-output) | 1  | 約800g    | 付属ACアダプタは12V3A                                                                    |
| [PS5 DualSense](https://www.playstation.com/ja-jp/accessories/dualsense-wireless-controller/) | 1 | - | 無線ゲームパッド                                                                          |
| [自作ケーブル](https://www.sato-susumu.com/entry/mid360_cable)| 1 | - | Livox Mid-360とPCを接続するためのケーブル                                                      |
| USBハブ | 1 | - | USBハブ                                                                             |
| [ReSpeaker USB Mic Array](https://wiki.seeedstudio.com/ReSpeaker-USB-Mic-Array/) | 1 | - |                                                                                   |
| USBスピーカー | 1 | - | DAISO USB ミニスピーカー 3W x 2。周波数帯域 35Hz - 200Hz。インピーダンス６Ω                       |
| BlinkStick Strip | 1 | - | USB LED                                                                           |
| TP-Link UB500/A| 1 | - | USB Bluetoothアダプタ                                                                 |
| TP-Link TL-WN725N | 1 | - | USB WiFiアダプタ                                                                      |
| D435i | 1 | - | カメラ                                                                               |


### フレーム部分
合計約2.2Kg
| Item                                                                                                     | 数量 | 重量         | 合計重量     | 備考・補足                                                    |
|----------------------------------------------------------------------------------------------------------|------|------------|----------|----------------------------------------------------------|
| [HSMC50-R](https://jp.misumi-ec.com/vona2/detail/110300477340/?ProductCode=HSMC50-R)                    | 2    | 462g/1個    | 924g     | アルミフレーム用キャスタ、コーナー取付タイプ、軽荷重用                              |
| [HFSF5-2020-210](https://jp.misumi-ec.com/vona2/detail/110302683920/?ProductCode=HFSF5-2020-210)        | 4    | 約111g/本 | 約444g | アルミフレーム 5シリーズ、正方形、20×20mm、1列溝、3面溝、長さ: 210mm、重量: 0.53kg/m |
| [HFSF5-2020-230](https://jp.misumi-ec.com/vona2/detail/110302683920/?ProductCode=HFSF5-2020-230)        | 4    | 約122g/本 | 約488g | アルミフレーム 5シリーズ、正方形、20×20mm、1列溝、3面溝、長さ: 230mm、重量: 0.53kg/m |
| [HFSR5-2020-184](https://jp.misumi-ec.com/vona2/detail/110302685570/?ProductCode=HFSR5-2020-184)        | 4    | 約077g/本 | 約309g | アルミフレーム 5シリーズ、R形状、20×20mm、長さ: 184mm、重量: 0.42kg/m         |
| [HBLSS5](https://jp.misumi-ec.com/vona2/detail/110300438930/?ProductCode=HBLSS5)                        |12    | 約3.8g/1個 | 約45.6g | ５シリーズ（溝幅６ｍｍ）用薄型ブラケット |
| [HFS5-2020-210](https://jp.misumi-ec.com/vona2/detail/110302683830/?ProductCode=HFS5-2020-210)          | 1    | 約105g/本 | 約105g | アルミフレーム 5シリーズ、正方形、20×20mm、1列溝、4面溝、長さ: 210mm、重量: 0.5kg/m |
| 上記以外にも、フレームキャップ、ストッパナット、ボルトが必要| - | - | - |


## 配線図
```mermaid
flowchart TB
    %% 電源供給ライン
    subgraph Power["Power"]
        A[Prime Power Bank]
        B[DP100]
        C[分岐ターミナル]
    end

    subgraph Motor["Motor"]
        D[DDSM115]
        E[DDSM115]
    end

    subgraph MiniPC["Mini PC関連"]
        F[Mini PC]
        K[USBハブ]
        H[USB TO RS485]
        L[PS5 DualSense]
        I[Monitor]
        M[マイクアレイ]
        N[UBSスピーカー]
        O[USB LED]
        P[Bluetoothアダプタ]
        Q[WiFiアダプタ]
        R[D435i]
        S[ReSpeaker USB Mic Array]
    end

    subgraph LiDAR["LiDAR関連"]
        G[Livox Mid-360]
        J[自作ケーブル]
    end

    %% 電源供給ライン
    A -->|20V出力| B
    B -->|12V出力| C
    C -->|12V出力| D
    C -->|12V出力| E
    C -->|12V出力| F
    C -->|12V出力| J

    %% Mini PCとの接続
    F -->|USB| K
    F -->|USB| R
    H -->|RS485| D
    H -->|RS485| E
    F -->|Bluetooth| L
    F -->|Type-C| I
    F -->|イーサネット| J

    %% USB Hubとの接続
    K -->|USB| H
    K -->|USB| M
    K -->|USB| N
    K -->|USB| O
    K -->|USB| P
    K -->|USB| Q
    K -->|USB| S

    %% その他
    S -->|オーディオケーブル φ3.5mmステレオミニプラグ | N
    J -->|12V出力＋イーサネット| G

    %% クラス定義 (色分け)
    classDef power fill:#FFDD00,stroke:#333,stroke-width:2px,color:#000;     
    classDef motor fill:#FF7373,stroke:#333,stroke-width:2px,color:#000;     
    classDef miniPC fill:#00A0E9,stroke:#333,stroke-width:2px,color:#000;   
    classDef lidar fill:#96C291,stroke:#333,stroke-width:2px,color:#000;     

    %% クラス適用
    class A,B,C power;
    class D,E motor;
    class F,K,H,L,I,M,N,O,P,Q,R,S miniPC;
    class G,J lidar;
```

## 制限事項
- オーディオ関連ノードが自動起動できない

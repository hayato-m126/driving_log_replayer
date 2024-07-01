# 再設計

driving_log_replayerを再設計する。
ROS 2 dashingのときに開発が始まってフレームワークも作る側も未成熟なままとりあえず動くものを作った。
フレームワークも作る側の理解も進歩して、現状の設計だと良くないところ、気になる部分が出てきている。

課題に思ってること、出来たらいいことを上げて、検証して、新しい設計を作る。

## 使いづらいポイント

### CLIが必要。コマンドが長い

- dlrコマンドを利用する。ros2 launchコマンドにしたい
- そもそもlaunchの引数が多すぎて人間が書くには長過ぎる

```shell
dlr simulaiton run -p perception

# 上記で実際に叩かれるているコマンド
ros2 launch driving_log_replayer perception.launch.py map_path:=/home/hyt/map/oss vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit vehicle_id:=default scenario_path:=/home/hyt/dlr_data/auto/perception/sample/scenario.yaml result_json_path:=/home/hyt/out/auto/perception/2024-0619-160250/sample/sample_dataset/result.json input_bag:=/home/hyt/dlr_data/auto/perception/sample/t4_dataset/sample_dataset/input_bag result_bag_path:=/home/hyt/out/auto/perception/2024-0619-160250/sample/sample_dataset/result_bag t4_dataset_path:=/home/hyt/dlr_data/auto/perception/sample/t4_dataset/sample_dataset result_archive_path:=/home/hyt/out/auto/perception/2024-0619-160250/sample/sample_dataset/result_archive sensing:=False
```

### フォルダ構成に柔軟性がない

- dlr cli専用のフォルダ構成を要求する。
- bagだけとt4_datasetを使うものでフォルダ構成が違う
- webauto cliでDLしてきたリソースは、webautoのフォルダ構成になっている。
- webautoでDLしたきたリソースを使って少し条件を書き換えたシナリオを実行するということが難しい

<https://tier4.github.io/driving_log_replayer/ja/overview/setup/#_3>

```shell
~/.webauto/simulation/data
❯ tree -d
.
├── bag
│   └── x2_dev
│       ├── 58b869c6-3ca1-4fa9-aa61-e267b368ea27
│       │   └── input_bag
│       └── efb573eb-fc86-4242-9ee1-da2b1b2cfa1c
│           └── input_bag
├── map
│   └── x2_dev
│       └── 329
│           └── 329-20230508115514620986
│               └── pointcloud_map.pcd
└── t4_dataset
    ├── prd_jt
    │   └── 3be686d3-765e-4f93-88ea-b881d89f4fce
    │       └── 3
    │           └── 3f7e8145-1ee0-44cd-82e6-f78786ea80f9
    │               └── 0
    │                   ├── annotation
    │                   ├── data
    │                   │   ├── CAM_BACK
    │                   │   ├── CAM_BACK_LEFT
    │                   │   ├── CAM_BACK_RIGHT
    │                   │   ├── CAM_FRONT
    │                   │   ├── CAM_FRONT_LEFT
    │                   │   ├── CAM_FRONT_RIGHT
    │                   │   └── LIDAR_CONCAT
    │                   ├── input_bag
    │                   └── map
    │                       └── pointcloud_map.pcd
    └── x2_dev
        └── 84eb6a44-1078-4945-b4d1-9c7b2055215a
            └── 3
                └── 2299887d-bde7-4675-8d7d-65b0767a2339
                    └── 0
                        ├── annotation
                        ├── data
                        │   ├── CAM_BACK
                        │   ├── CAM_BACK_LEFT
                        │   ├── CAM_BACK_RIGHT
                        │   ├── CAM_FRONT
                        │   ├── CAM_FRONT_LEFT
                        │   ├── CAM_FRONT_RIGHT
                        │   └── LIDAR_CONCAT
                        ├── input_bag
                        └── map
                            └── pointcloud_map.pcd
```

### 1回のシミュレーションでどれか1個の評価しか出来ない

perceptionとannotationless_perceptionは少なくとも同時に評価出来ていい。
同じperceptionのコンポーネントが出すtopicをsubscribeして結果を書き出している。
2回バラバラにやるより、1回で両方評価出来たほうが検査にかかる時間が短縮できる。

評価ノードはsubscribeして、topicの内容解析して結果をファイルに書き出しているだけ。
現状result.jsonl固定で出力しているが、ユースケース毎に別の名前でファイルで出力するように直せばよい。
result_bagはそれぞれの評価で必要なtopicをマージしてrecorderに渡せばいい。

### シナリオフォーマットがわかりにくい

開発開始時にScenario Simulatorのフォーマットを真似してCamelCaseで記載することとしたが、perception_evalに渡す設定をシナリオフォーマットの中に入れるようになり、snake_caseが混ざるようになった。
Pydanticをシナリオの構造チェックに使っていて、Modelを定義する都合上、snake_caseで書いて統一したほうがわかりやすそう。シナリオ変換ツール作る。

## サンプル実装で実現可能を確認した

- ros1のときみたいに、bag playの方にrequiredをつけたい。今はノード側でclockが止まったら終了を判断している。
- cliをなくす。わかりにくい。シナリオをパースして必要なargを作ってるし、シナリオファイル自体も引数に指定しているこれは無駄。webauto側もlaunch引数が変わることで、I/Fが変わってしまう。それを吸収するためにwasimやdlr_cliがいるのは違う。launchの中でyamlを処理する仕組みに変更
- ros2 launch driving_log_replayer driving_log_replayer.launch.py scenario_path:=${scenario_path} evaluations:="['perception', "annotationless_perception']" 2つ必須の想定
- 全部t4_datasetベースの設計に変えたい。scenarioにdataset_pathを記述(絶対パス/相対パスの両方の指定が可能)、dataset_path/input_bagを指定すればbag playできる。
- 今のディレクトリ固定の構成をやめたい。webautoのディレクトリにあるt4_datasetをシナリオから指定してdlr simulation runとかもやりたい。↑でできる。
- 複数の評価を一変に回したい。引数でlaunchするevaluator_nodeを動的に変えるのは出来た。evaluator_node毎にresult.jsonlのファイル名か変えるなど追加で必要だがとりあえずできそう。

## やりたい

- 現状はbagの長さを切らないと評価の時間の指定は出来ないけど、dlrの評価によらない共通機能として、評価開始時間と終了時間を指定できるようにしたい。これによって共通のbagの使いまわしを楽にしたい。（なくてもいいかも）
- clock止まったあとに、最終メトリクス出したりしているから自分のノードで終了判定できるのも悪くないけど、後処理をOnExitで別途起動できるようにしておけばいいような。
- 後処理が自由にかけるなら、ndt_convergenceの評価は単にbag作ってるだけだから、後処理だけ書いてもらればいい。
- AWSIMを使ってシミュレーションを実行しながら評価する方法もサポートしたい
- 現状1topicで１レコードの結果を出しているだけだから、メトリクスを出す処理をlaunchが終了したあとのあと処理として実行すれば問題なくいけるのでは？
- scenario format yaml を pydanticでスキーマをガチガチに固める。やればできる
- result format jsonl を pydanticでスキーマをガチガチに固める。やればできる

### GUI

- foxgloveを前提にしたい。webでも同じように見れるように。mcapにする
- rvizのコピーメンテするの面倒臭すぎるので、Autoware本体に更新に左右されないようにしたい。

## 最新追従の課題

rosのdependencyはバージョン指定しているわけじゃないから勝手にライブラリのバージョン上がることがある。

pytestが上がって、colcon testが動かなくなった例
https://github.com/ros2/launch/pull/766

numpy 1.24.0でnp.floatがなくなったせいで動かなくなった例
https://github.com/Box-Robotics/ros2_numpy/pull/6

上記のように、依存ライブラリのアップデートがあるから、依存に合わせた修正を入れないと動かなくなることがある。
基本的にrosのパッケージは最新を追ってないといつか動かなくなる。
何もしてないのに壊れたのパターン。何もしてないからこそ壊れている。

だけど、最新化するときに、プロダクトで使ってないやつの更新も入ってたり、影響があるのかないのか判定するのも難しい。

## あったらいいかなくらい

ユースケース毎に依存が異なる。であれば、使うユースケース毎にパッケージを分割して、メタリポ構造にする。
依存をユースケースに閉じ込める。
本体は最低限のインターフェースだけにする。

ただし、管理が煩雑になりそう。

## Autoware構成

各コンポーネントの繋がり。

https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/
https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/node-diagram/
https://www.youtube.com/watch?v=XdYMcmwVv8g

各コンポーネントの入出力をbagに入れておいて、launchの引数に合わせて、remapする。
これによって、モジュールの出力固定で評価と、変更して評価が切り替えられる。

x2 perceptionはフィルタ処理がLiDAR1個ずつになっていて、sensing:=trueじゃなきゃ動かない。
こういうプロダクト固有の問題を吸収できるか。

## webautoの連携

driving_log_replayerの本体がwebautoとのインターフェースになっている。
driving_log_replayerのユースケースが増えてたりしても、webauto側に手を入れることなく実行できるようにしたい。
作っても、サポートずっとされずにローカルで回し続けるのはきつい。

logsimのときはlogsimだけしかなかったが、GPUが必要なインスタンスが必要になったり、とかそのへんが理由？
webauto-ci.ymlでsimulator_type毎に設定かけるようにするために現状1ユースケースが当てられてるいるが、この辺、GPUありなしだけで分けられないものか。
ラベル機能があれば分けられる？

## 試す

- [x] 親のlaunchにyamlを渡して、yamlをパースして、評価用のlaunchを呼べるか
- [x] パラメータを後ろから付け足したら上書きできるか確認
- [x] bagだけのやつをt4_dataset想定に変更
- [x] bag playに OnProcessExit仕掛けられるか確認する。https://ubuntu.com/blog/ros2-launch-required-nodes
- [x] launchの中でデータ出力先のディレクトクトリを作成する

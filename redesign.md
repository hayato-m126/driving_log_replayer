# 再設計

driving_log_replayerを再設計する。
ROS 2 dashingのときに開発が始まってフレームワークも作る側も未成熟なままとりあえず動くものを作った。
フレームワークも作る側の理解も進歩して、現状の設計だと良くないところ、気になる部分が出てきている。

課題に思ってること、出来たらいいことを上げて、検証して、新しい設計を作る。

## 課題

rosのdependencyはバージョン指定しているわけじゃないから勝手にライブラリのバージョン上がることがある。

pytestが上がって、colcon testが動かなくなった例
https://github.com/ros2/launch/pull/766

numpy 1.24.0でnp.floatがなくなったせいで動かなくなった例
https://github.com/Box-Robotics/ros2_numpy/pull/6

上記のように、依存ライブラリのアップデートがあるから、依存に合わせた修正を入れないと動かなくなることがある。
基本的にrosのパッケージは最新を追ってないといつか動かなくなる。
何もしてないのに壊れたのパターン。何もしてないからこそ壊れている。

だけど、最新化するときに、プロダクトで使ってないやつの更新も入ってたり、影響があるのかないのか判定するのも難しい。

ユースケース毎に依存が異なる。であれば、使うユースケース毎にパッケージを分割して、依存をユースケースに閉じ込める。
本体は最低限のインターフェースだけにする。

## やりたい

- 複数の評価を一変に回したい。perceptionとannotationless_perceptionは一緒でいいだろうという気持ちがある。どっちかだけの切り替えも可能にしたい
- scenario format yaml を pyadanticでスキーマーガチガチに固める
- result format jsonl を pydanticでスキーマーガチガチに固める
- 現状はbagの長さを切らないと評価の時間の指定は出来ないけど、dlrの評価によらない共通機能として、評価開始時間と終了時間を指定できるようにしたい。これによって共通のbagの使いまわしを楽にしたい。（なくてもいいかも）
- clock止まったあとに、最終メトリクス出したりしているから自分のノードで終了判定できるのも悪くないけど、後処理をOnExitで別途起動できるようにしておけばいいような。
- 後処理が自由にかけるなら、ndt_convergenceの評価は単にbag作ってるだけだから、後処理だけ書いてもらればいい。
- AWSIMを使ってシミュレーションを実行しながら評価する方法もサポートしたい

### GUI

- foxgloveを前提にしたい。webでも同じように見れるように。mcapにする
- rvizのコピーメンテするの面倒臭すぎるので、Autoware本体に更新に左右されないようにしたい。

## サンプル実装で実現可能を確認した

- ros1のときみたいに、bag playの方にrequiredをつけたい。今はノード側でclockが止まったら終了を判断している。
- cliをなくす。わかりにくい。シナリオをパースして必要なargを作ってるし、シナリオファイル自体も引数に指定しているこれは無駄。webauto側もlaunch引数が変わることで、I/Fが変わってしまう。それを吸収するためにwasimやdlr_cliがいるのは違う。launchの中でyamlを処理する仕組みに変更
- ros2 launch driving_log_replayer driving_log_replayer.launch.py scenario_path:=${scenario_path} evaluations:="['perception', "annotationless_perception']" 2つ必須の想定
- 全部t4_datasetベースの設計に変えたい。scenarioにdataset_pathを記述(絶対パス/相対パスの両方の指定が可能)、dataset_path/input_bagを指定すればbag playできる。
- 今のディレクトリ固定の構成をやめたい。webautoのディレクトリにあるt4_datasetをシナリオから指定してdlr simulation runとかもやりたい。↑でできる。

## あったらいいかなくらい

- メタレポ構成にして必要な評価機能だけ入れる

管理が煩雑になりそう。

## 構成

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

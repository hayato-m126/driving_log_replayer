# 再設計

rosのdependencyはバージョン指定しているわけじゃないから勝手にライブラリのバージョン上がることがある。

pytestが上がって、colcon testが動かなくなった例
https://github.com/ros2/launch/pull/766

numpy 1.24.0でnp.floatがなくなったせいで動かなくなった例
https://github.com/Box-Robotics/ros2_numpy/pull/6

上記のように、依存ライブラリのアップデートがあるから、依存に合わせた修正を入れないと動かなくなることがある。
基本的にrosのパッケージは最新を追ってないといつか動かなくなる。
何もしてないのに壊れたのパターン。何もしてないからこそ壊れている。

だけど、最新化するときに、プロダクトで使ってないやつの更新も入ってたり、影響があるのかないのか判定するのも難しい。
eveのケースとか
lanelet2_extension_pythonとか

## やりたい

- 全部t4_datasetベースの設計に変えたい。
- scenario format スキーマーでガチガチに固める
- result format スキーマーでガチガチに固める
- 今のディレクトリ固定の構成をやめたい。webautoのディレクトリにあるt4_datasetをシナリオから指定してdlr simulation runとかもやりたい

## あったらいいかなくらい

- メタレポ構成にして必要な評価機能だけ入れる

管理が煩雑になりそう。

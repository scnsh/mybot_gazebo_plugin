# Overview

## 自作モデルのSDFを使って、Gazebo上で動かす

  - 以下のチュートリアルをmybotモデル用に変更を加えたもの
  - http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5

Version
----

0.0.1

Requirements
-------------

* Ubuntu 16.04 
* [Gazebo] - **Version 7.11**
* [libgazebo7-dev] -- **Version 7.11**

Building/Compiling
-------------

cmakeを使ったビルドが必要

```sh
:mybot_gazebo_plugin/build$ cmake ..
:mybot_gazebo_plugin/build$ make
```

以下の環境変数で作成したbuildディレクトリを追加

```sh
export GAZEBO_PLUGIN_PATH=<path_to_dir>/mybot_gazebo_plugin/build:$GAZEBO_PLUGIN_PATH
export GAZEBO_MODEL_PATH=<path_to_dir>/mybot_gazebo_plugin/models:$GAZEBO_MODEL_PATH
```

Basic Usage
--------------

### gazeboの起動
```sh
gazebo --verbose mybot.world
```

git cloneしたディレクトリに移動後、実行

### モデルの操作

`$ build/pos 20`

`$ build/vel 20`

`$ build/force 20`


実行形式に引数で値を指定するとJointに位置、速度、力の指示を渡す

[gazebo]: http://gazebosim.org
[libgazebo7-dev]: https://packages.debian.org/sid/libgazebo7-dev
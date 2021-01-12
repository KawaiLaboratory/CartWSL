# cartbot
## 概要
### プロジェクトの目標
- 病院内で任意の人を追跡する荷物運搬ロボットを作る。ここでは、レーザーレンジファインダー(以下LRF)による外界の検知を行うことができるプログラムを組む。
- 人や壁などを検知するセンサーはLRFのみ、Raspberry Piで動くようにして欲しい(難しそう)。

## 注意
- Raspberry PIなどの3GB以下のメモリを持つコンピュータでコンパイルを行う場合、メモリ+swap領域で合計3GB用意してください。
- コマンドラインに「Failed to find match for field 'rgb'」と出るが、これは点群にRGBに値が入っていないという警告。別に問題ないので無視。
- コマンドラインに「[pcl::KdTreeFLANN::setInputCloud] Cannot create a KDTree with an empty input cloud!」と出ることがあるが、点群データが存在しないからクラスタリングできないという警告。LRFが起動していない場合やLRFの検知範囲上に何もないとこうなる。

## 起動前の環境設定
### LRF
- LRFを動かす前にシリアル通信を行うttyACM0のアクセス権限を使用するユーザーが使えること。できなければ、sudo gpasswd -a ユーザ名 dialout を入力する。

### フォルダのための環境変数
- 「CARTBOT_FEATURES_DATA_FOLDER」という、cartbot/features_dataフォルダまでの絶対パスを作っておいてください。データを読み込むのに必要です。
- Ubuntuでの例：export CARTBOT_FEATURES_DATA_FOLDER=/home/hogehoge/catkin_ws/src/cartbot/features_data

## cartbotで使うroslaunch  
- roslaunchはroscoreと起動したいrosのnodeをコマンド一つでしてくれるもの。
- cartbot内にあるroslaunchは、rosのnodeで起動する手間を省いてくれる。
- rvizを起動し、任意のFixed FrameとTopicを入力することで、どんな値が取れているのか確認できる。
- 注意:センサーからのデータを配信するノードは各自自分で起動すること。(こちらの環境では起こらなかったが、fixed frame等の配信の値が違う場合があるかもしれません。その場合は、各cppファイルを書き換える必要があります。)
- cartbotで使うroslaunchは以下の通り
    - cartbot detection_torso.launch
        - 胴体を検知し、カルマンフィルターで追跡するもの。先行研究。
        - 追跡対象者の初期予想位置はロボットの前方0.5m
        - 2秒間追跡が不可能な場合、ロボットは停止し追跡対象者の予想位置は初期予想位置に戻る。
        - 配信されるものは「cartbotで使うrosのnode」の項の「cartbot main」と「cartbot detection_torso.py」を参照。
    - cartbot detection_hip.launch
        - 腰を検知し、カルマンフィルターで追跡するもの。「cartbot detection_torso.launch」の腰バージョン。
        - 追跡対象者の初期予想位置はロボットの前方0.5m
        - 2秒間追跡が不可能な場合、ロボットは停止し追跡対象者の予想位置は初期予想位置に戻る。
        - 配信されるものは「cartbotで使うrosのnode」の項の「cartbot main」と「cartbot detection_hip.py」を参照。
    - cartbot detection_leg_kf.launch
        - 足を検知し、カルマンフィルターで追跡するもの。独自(先行研究ありそう)。
        - 追跡対象者の初期予想位置はロボットの前方0.5m
        - 2秒間追跡が不可能な場合、ロボットは停止し追跡対象者の予想位置は初期予想位置に戻る。
        - 配信されるものは「cartbotで使うrosのnode」の項の「cartbot main」と「cartbot detection_leg_kf.py」を参照。
    - cartbot detection_leg.launch
        - 足を検知し、歩行モデルで追跡するもの。先行研究。
        - 追跡対象者の初期予測位置の範囲はロボットの前方0.5mから0.85mまで。
        - 歩行モデルはカルマンフィルターと違って、追跡対象者の予測位置を点で表すことはできないため、追跡対象者の予測位置を示すことができない。
        - 歩行モデルの仕様上、直角に曲がるなどの急な旋回は検知を困難にする。
        - 1秒間追跡が不可能な場合、ロボットは停止し追跡対象者の予想位置は初期予測位置の範囲に戻る。
        - 配信されるものは「cartbotで使うrosのnode」の項の「cartbot main」と「cartbot detection_leg.py」を参照。
    - cartbot observation.launch(非推奨)
        - 取得データの加工、特徴の抽出などが行える。
        - コマンド押した後に出てくる表示が出ない恐れあり。rosrunでしたほうが良い。
    - turtlebot_bringup minimal.launch
        - turtlebotを動作させるためのlaunch。これがないとturtlebotは動かない。
        - turtlebotを動かす場合、これを他のrosのnodeやroslaunchと併用すること。

## cartbotで使うrosのnode
- rosrunで動くrosのnode。roslaunchで動かす場合は特に動かさなくていい。
- cartboで関係のあるrosのnodeは以下の通り
    - rosrun urg_node urg_node _serial_port:=/dev/ttyACM0
        - LRFを動かすためのnode。外部からインストールする必要あり。
        - rvizでFixed Frameは「/laser」、LaserScanのTopicは「/scan」でLRFから配信されているデータを表示する。
    - rviz rviz
        - LRFで受け取った値がどんなものなのかグラフィカルに表示してくれる。
    - cartbot main
        - LRFで受け取った点群から特徴を抽出してくれる。
        - rvizでfixed Frameを「/laser」、PointCloud2のTopicは「/lrf_clustring」で点群をクラスタリングしたものを表示する。
        - ~~黒い点群はクラスタリングで分割された点群以外の点群、分割された点群は色分けされる。(分割された点群が10以上の場合、10以降は白色になる。)~~ メモリ節約のため、色が表示されることはない。
    - cartbot detection_hip.py
        - LRFで受け取った値から、SVMとカルマンフィルタで追跡対象者の位置を割り出す。
        - 腰対応。
        - これよりも先に作った「cartbot detection_torso.py」と中身はほとんど同じ、唯一違うのは特徴データファイルの参照先ぐらい。
        - Float32MultiArray型の「status」を配信する。statusは3つの要素を含む一次配列で、[検知できているかいないかの判定,x座標,y座標]という形になっている。
        - rvizでfixed Frameを「/laser」、PointCloud2のTopicは「/center_point」で検知した胴体(緑)と追跡対象者の予測地点(黄色、検知中は赤にもなる)を表示する。
    - cartbot detection_leg_kf.py
        - LRFで受け取った値から、SVMとカルマンフィルタで追跡対象者の位置を割り出す。
        - 足対応。
        - Float32MultiArray型の「status」を配信する。statusは3つの要素を含む一次配列で、[検知できているかいないかの判定,x座標,y座標]という形になっている。
        - rvizでfixed Frameを「/laser」、PointCloud2のTopicは「/center_point」で検知した足(紫)と人の中心点(緑)と追跡対象者の予測地点(黄色、検知中は赤にもなる)を表示する。
    - cartbot detection_leg.py
        - LRFで受け取った値から、SVMと歩行モデルで追跡対象者の位置を割り出す。
        - 足対応。
        - Float32MultiArray型の「status」を配信する。statusは3つの要素を含む一次配列で、[検知できているかいないかの判定,x座標,y座標]という形になっている。
        - rvizでfixed Frameを「/laser」、PointCloud2のTopicは「/center_point」で検知中追跡対象者(赤)で表示する。
    - cartbot detection_torso.py
        - LRFで受け取った値から、SVMとカルマンフィルタで追跡対象者の位置を割り出す。
        - 胴体対応。
        - Float32MultiArray型の「status」を配信する。statusは3つの要素を含む一次配列で、[検知できているかいないかの判定,x座標,y座標]という形になっている。
        - rvizでfixed Frameを「/laser」、PointCloud2のTopicは「/center_point」で検知した胴体(緑)と追跡対象者の予測地点(黄色、検知中は赤にもなる)を表示する。
    - cartbot res_plot.py
        - 学習データの分布を見ることができる。
    - cartbot observation
        - 取得データの加工、特徴の抽出などが行える。
    - cartbot lisner_sample
        - 配信されているデータを取得するためのサンプルプログラム。最初の時点ではFloat32MultiArray型の「status」を受信する。

## cartbotの機能
### 実演
- 「cartbotで使うroslaunch」の項にあるcartbot detection_XX.launch(XXにはアンダーバーや英数字が入る)の中から動かしたいものを動かす。
- これを使うと、Float32MultiArray型の「status」が配信される。statusは3つの要素を含む一次配列で、[検知できているかいないかの判定,x座標,y座標]という形になっている。
- rviz向けにも点群が表示される。
- この配列を受信するためのサンプルプログラムは、lisner_sample.cppである。

### 学習データの構築
- 「rosrun cartbot observation」と「rosrun urg_node urg_node _serial_port:=/dev/ttyACM0」を動作させる。「rviz」も起動させることで、LRFの取得データを見ることができる.
- rvizで見る場合は、fixed Frameを「/laser」に、PointCloud2のTopicは「/lrf_observation」にする。
- コマンドラインから、観測データ(pcdデータ)の保存や特徴データ(SVMで特徴の学習を行うのに必要)の保存を行うことができる。
- 以下にコマンドの一覧を示す。
    - c: 連続保存の開始。連続保存はdを押すまで続く。続いている間は、LRFで受け取った値の加工手法を施した観測データが保存される。観測データはfeatures_data/observation/連続保存開始前までは存在しなかった最小番号フォルダに保存される。
    - d: コマンドc、vを終了する。
    - e: 観測データから学習データを生成する。対象となる観測データがおいてあるフォルダを指定後、選択中の特徴データの保存ファイルに特徴データが保存される。尚、観測データはo_lrf_X.pcd(Xには整数が入る)で連続していなければならない。
    - m: LRFで取得するデータの距離範囲を指定できる。入力時は、最小値x 最大値x 最小値y 最大値y(例:0 2.0 -0.5 1)と入力すれば良い。初期値は0 3.0 -0.5 0.5となっている。
    - o: LRFで受け取った値の加工手法の変更。
        - 0はLRFで得た点群をクラスタリング後、最も点の多い塊を保存対象物とする。rvizでは赤色点群となる。初期値。
        - 1はLRFで得た点群をクラスタリングされた点群をすべて保存する。rvizでは黄色点群となる。
        - 2はLRFで得た点群をそのまま保存する。rvizでは緑色点群となる。
    - p: 特徴データの保存ファイルの選択。0の場合は胴(torso_features_data.csv)の、1の場合は足(leg_features_data.csv)の、1の場合は腰(hip_features_data.csv)の保存先となる。初期値は2。
    - q: 終了。
    - r: 選択中の特徴データの保存ファイルの削除。
    - v: 観測データを閲覧できる。閲覧したい観測データを含んだフォルダを指定後、o_lrf_X.pcd(Xには整数が入る)とある時、「,」でX-1、「.」でX+1のファイルを閲覧できる。
- 観測データはなるべく歩いている最中のものを記録するべきである。
- 観測中は対象者の足又は胴体以外映らないようにすること。
- 作成した特徴データの保存ファイルのコピーは、cartbot/features_data/back_upフォルダに入れれおくと良い。ちなみに、そのフォルダ内にある特徴データの保存ファイルはcartbot作成者のデータである。

## 複数のマシン間でROSを動かす
- 以下では2つのマシン間(ここではマシン1とマシン2として、どちらもLinuxを使う、できればUbuntu系統)で動かすとする。
- 準備
    - どちらのマシンも「ufw disable」でファイアウォールを切っておく。
- ネットワークの設定
    - マシン1もマシン2も固定IPを持っておく。当然、固定IPはユニークである。ここではマシン1の固定IPが192.168.1.10、マシン2の固定IPが192.168.1.11とする。
    - roscoreをマシン1で動かすとする。その場合、.bashrcファイルに以下の内容を追記する。
        - マシン1の.bashrc  
        export ROS_MASTER_URI=http://192.168.1.10:11311  
        export ROS_HOSTNAME=192.168.1.10  
        - マシン2の.bashrc  
        export ROS_MASTER_URI=http://192.168.1.10:11311  
        export ROS_HOSTNAME=192.168.1.11  
- 動作確認
    - 最初にマシン1でroscoreを動かす。
    - マシン1でrosrun rospy_tutorials listener.py、マシン2でtalker.pyを動かす。
    - 両方のノードに反応があれば成功。


# cartbotで思いついたことのメモ(cartbotの動作に関係なし)
## 動作メモ
- 初期計測位置はx=0.5m、y=0.0。その位置で人のような物を認識したら動き始める。
- 予測地点0.1mまでで、一番近い候補が選ばれる。0.1mに候補がない場合、その地点までロボットが進む。
    - 進む際、ロボットの予測地点も動く(絶対距離ではなく、相対距離である。)
    - プログラムするのであれば、対象が見つからない->対象が最後に居た位置まで進む->そこで待機が妥当?　当然対象が見つからない間も予測地点を更新し続けるべき。この場合、絶対座標が必要になる。

## 思った事
- よくよく考えたら、足の動きとかを予測する深層学習モデル使えば良くない?
- カルマンフィルターでは、対象者の近場にいる人間を誤検知する可能性が十分にある。
- それなら、足の動いた距離をベクトルにしてやれば行ける?この場合、回帰ではなく分類問題になりそう。
- 動きを捕捉する論文はレーザーレンジファインダーに絞らなくても問題ないかな。
- 行動と外界に関する研究であるので、強化学習についても学習するべき。

## マイリスト論文
### 点群とCNN
- PointNet
    - https://www.slideshare.net/FujimotoKeisuke/point-net
    - これをLRFに用いれば、人物特定だけで済んだりしたりして...(外界から得られるデータが少ないため無理だと思われる)
    - もし、これで人物の特定ができれば、カルマンフィルターもSVMもいらない

### Movement prediction
- Object tracking with movement prediction algorithms
    - https://ieeexplore.ieee.org/document/8262952/
    - IEEE conf, 2016
    - 適当にとってきた

- 環境に応じた人間の移動予測に基づく移動ロボットの人物回避
    - http://www.robot.t.u-tokyo.ac.jp/~yamashita/paper/A/A039Final.pdf
    - 日本機械学会, 2012
    - LRFを使ってる

- カルマンフィルタを用いた足位置予測に基づく人物追跡自律移動ロボットの研究
    - https://ipsj.ixsq.nii.ac.jp/ej/index.php?action=pages_view_main&active_action=repository_action_common_download&item_id=52431&item_no=1&attribute_id=1&file_no=1&page_id=13&block_id=8
    - 情報処理学会, 2004

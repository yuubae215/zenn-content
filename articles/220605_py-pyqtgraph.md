---
title: "QtDesignerにPyQtGraphのGraphicsLayoutWidgetを埋め込む"
emoji: "📈"
type: "tech" # tech: 技術記事 / idea: アイデア
topics: ["python", "Qt", "PyQt", "PyQtGraph"]
published: true
---

## PyQtGraph
PyQtGraphはNumpyとQtのGraphicsViewを使用することによって、Pythonでも高速な数値計算とGUIでのグラフ描画ができるライブラリです。
Pythonでのグラフ描画はmatplotlibが主流だとは思いますが、データを取得しつつオンラインで描画処理ができるため私は愛用しています。

ただ、PyQtGraphの情報が少ないのが難点で私みたいにソフトウェアの知識が乏しい人間には学習が大変でした。
Qtも全くさわったことなかったので、勉強から始めましたが以下の記事がとても理解の助けになりました(内容はPyQtです)。
https://www.slideshare.net/RansuiIso/pyqtgui
Main Window？Widget？とかちんぷんかんぷんだったのですが、絵つきでとてもわかりやすく解説してくださっています。
p.23あたりからコンポーネントの説明になっています。それまでは環境構築の内容になっているので、環境構築がすでに完了しているかたはp.23まで飛ばしても良いかもしれません。

### インストール方法
GitHubのリポジトリは以下です。
https://github.com/pyqtgraph/pyqtgraph

PyQtGraphのpipでのインストールは以下です。
```shell
pip install pyqtgraph
```

PyQtGraphは公式のExampleが豊富にあり、以下のコマンドによりすぐに実行できるようになっています。
```shell
python -m pyqtgraph.examples
```
👇実行するとこんな感じです(右側のグラフは`Basic Plotting`を実行したものです)
![](/images/art06_py-pyqtgraph/gif_220605234012.gif)
私はこれだけで、「え、なにこれすご…！」ってなりました。

## QtDesigner
今回はUIの作成に`QtDesigner`を利用します。
`QtDesigner`はQtで作成するアプリケーションのUI部分のみを作成するものです。
GUI上でウィンドウやボタン、グラフ描画画面などの配置ができます。
`QtDesigner`で作成した`.ui`ファイルをQtで読み込んで使用することができます。

ちょっと癖はありますが、見た目部分を`.ui`ファイルで管理できるので私は好きです。
ちょこっと見た目を変えたい時、コードだとすぐに見た目につながりませんが`QtDesginer`であればGUI操作で変更できるため、プロトタイピングには大変便利です。

### インストール方法
以下のサイトから`QtDesigner`をダウンロードし、PCにインストールしてください。
https://build-system.fman.io/qt-designer-download

### QtDesignerでカスタムWidgetを使用する方法
pyqtgraphのグラフ描画のためのGraphicsLayoutWidgetは`QtDesigner`で素のままでは使用できないため、カスタムWidgetとしてGraphicsViewからpromote(格上げ)して使用します。

以下は`QtDesigner`にカスタムWidgetを組み込む方法です。
https://pyqtgraph.readthedocs.io/en/latest/how_to_use.html#embedding-widgets-inside-pyqt-applications

まずは、Main WindowでもWidgetでも良いので新規作成します。
下図のようにGraphics Viewを配置してください。
![](/images/art06_py-pyqtgraph/ss_2022-06-06-001021.png)

次にGraphics Viewを右クリックし、`Promote to...`を選択します。
![](/images/art06_py-pyqtgraph/ss_2022-06-06-001151.png)

`Promoted class name`に格上げするクラスを、`Header file`にインクルードするライブラリ名を入れます。
![](/images/art06_py-pyqtgraph/ss_2022-06-06-001411.png)

これでグラフ描画するための.uiファイルの完成です。

## GUI制御スクリプト
次は、表示するデータを作ったり、GUIを更新するタイマーを動かしたりするためのPythonファイルを記述します。
以下は、Exampleの`Basic Plotting`内中段一番右のプロットを抜き出して.uiから読み込んだグラフウィンドウにはめ込んでみました。

```python: sample.py
from pyqtgraph.Qt import QtCore
import numpy as np
import pyqtgraph as pg

from PyQt5 import uic

app = pg.mkQApp("Plotting Example")

win = uic.loadUi("sample.ui")
#win = pg.GraphicsLayoutWidget(show=True, title="Basic plotting examples")
win.resize(1000,600)
win.setWindowTitle('pyqtgraph example: Plotting')

# Enable antialiasing for prettier plots
pg.setConfigOptions(antialias=True)

def buttonClicked():
    print("button is clicked.")

win.pushButton.clicked.connect(buttonClicked)

p6 = win.graphicsView.addPlot(title="Updating plot")
curve = p6.plot(pen='y')
data = np.random.normal(size=(10,1000))
ptr = 0
def update():
    global curve, data, ptr, p6
    curve.setData(data[ptr%10])
    if ptr == 0:
        p6.enableAutoRange('xy', False)  ## stop auto-scaling after the first data set is plotted
    ptr += 1
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)

win.show()

if __name__ == '__main__':
    pg.exec()
```

`pg.GraphicsLayoutWidget()`のところを、.uiファイルを参照するように`uic.loadUi("sample.ui")`でウィンドウオブジェクトを生成しました。
あとは、`pg.GraphicsLayoutWidget()`でもともとshowしていたのをコメントアウトしてしまったので、別の場所でshowしてます。

👇.uiファイルを同じ階層に置いてPythonファイルを起動すると、こんな感じの見た目になります。
![](/images/art06_py-pyqtgraph/gif_220606235753.gif)

ついでに、PushButtonを下段に置いてみました。
ボタンを押すと、`button is clicked.`とターミナルのほうにログが出力されるかと思います。

## まとめ
QtDesignerで作成した.uiファイルを参照してPyQtGraphを使用してみました。
小ネタですが、格上げの機能に行きつけなかっため意外と苦労しました。

本記事で作成したスクリプトはリポジトリにアップしました。
https://github.com/yuubae215/pyqt-qtdesigner_sample

見ていただいたかたありがとうございました😊
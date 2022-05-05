---
title: "PythonでOPC-UA"
emoji: "🎉"
type: "tech" # tech: 技術記事 / idea: アイデア
topics: [Python, opcua]
published: true
---
# やりたいこと
クライアントPC側PythonからOPC-UAサーバに接続したい。

## 背景
仕事でフィールドバスを使った開発環境をそろえる必要があるのですが、
EtherCATやPROFINETなどの産業用フィールドバスを利用した通信は、環境をそろえるのがとてもしんどいです。何か簡単に構築できる環境はないものかと思っていました。

そこでOPC-UAの登場です。
前述のフィールドバスのプロトコルをOPC-UAがサポートしているため、OPC-UAサーバが構築できればフィールドバス通信をクライアント側がする必要がなくなります。
また、このサーバはRaspberry Piでも構築可能な点もナイスです[^1]。
簡易的な環境であれば安価に構築できそうということがかなり大きなモチベになり、やってみようと思い立ちました。

# 環境
- OS:Windows11
- Python:3.8.10

# サーバー側
まずは、サーバー側の準備をします。
サーバーには[Unified-Automation](https://www.unified-automation.com/downloads.html)のDemo Serverを使ってみます。
サイト上側タブの「Downloads」から、「OPC UA Servers」を選び、OPC UA C++ Demo Server (Windows)からダウンロードできます。

:::message
Demo Serverのダウンロードにはユーザ登録が必要です。
:::

ダウンロード・インストール出来たらサーバアプリを立ち上げます。
おそらくこの時点で、`opc.tcp://localhost:48010`というサーバが立ち上がっています。
ターミナルが出てきて以下が表示されるはずです。
```:UaCPPServer
***************************************************
 Server opened endpoints for following URLs:
     opc.tcp://host_name:48010
***************************************************
***************************************************
 Press x to shutdown server
***************************************************
```
👆host_nameは実際は自分のPC名になっていると思います。
これでサーバー側の準備は完了です。


# クライアント側
次にクライアント側の準備をします。
クライアントにはpythonの[freeopcua](https://github.com/FreeOpcUa/python-opcua)ライブラリを利用します。
pipであれば以下でインストールできます。  
```
pip install opcua
```

:::details 動作確認用クライアント側アプリ
最初はどういった代物なのかわからなかったので、ネットにあるアプリを頼りました。
クライアント側のアプリでUaExpertというアプリがあります。
これも[Unified-Automation](https://www.unified-automation.com/downloads.html)のサイトから「OPC UA Client」→「UaExpert」でダウンロードできます。(なお、後述のコード中にあるノードの調べ方がわからなかったので今回はこのアプリで調べました。)

[追記]
freeopcuaがgithubに載せている、`Simple OPC-UA GUI client`というアプリもありました。
`pip install opcua-client`でインストールし、`opcua-client`で起動できます。
これの便利なところは、簡単にデータをグラフ表示できるところです。
ノードリストで右クリックをするとメニューにグラフ追加が表示されます。
更新間隔は`Interval`で調整可能です。
:::

## スクリプト
freeopcuaのgithubにあるexampleから、`client-minimal.py`をいじります。

```py:client-minimal_demo.py
import sys
sys.path.insert(0, "..")

from opcua import Client

try:
	client = Client("opc.tcp://localhost:48010") # connect to demo server
	client.connect()

	root = client.get_root_node()
	print("Objects node is: ", root)
	print("Children of root are: ", root.get_children())
	var = root.get_child([
		"0:Objects", 
		"0:Server", 
		"3:AreaAirConditioner",
		"3:AirConditioner_1",
		"3:Temperature",
	])
	print("var is: ", var)
	print(var.get_data_value())
finally:
	client.disconnect()
```
`Client`でクライアントクラスをインスタンス化し、`connect`で接続します。
`root = client.get_root_node()`でrootのノードを取得しています。
`root.get_children()`はrootの子ノードです。
`var = root.get_child`で直接取得したいノードを指定することができます。
ここで`var`はデモサーバの中にある、`AirConditioner`モデルの温度(`Temperature`)を取得しています。
`var.get_data_value()`とすることで`var`のデータを取得しています。
(`var.get_value()`でノードの値だけを取り出すこともできます。)

それでは、このスクリプトを実行してみましょう。
# 実行結果
以下の出力が得られました。
```py
cryptography is not installed, use of crypto disabled
cryptography is not installed, use of crypto disabled
Objects node is:  i=84
Children of root are:  [Node(TwoByteNodeId(i=87)), Node(TwoByteNodeId(i=85)), Node(TwoByteNodeId(i=86))]
var is:  ns=3;s=AirConditioner_1.Temperature
DataValue(Value:Variant(val:71.26239999999964,type:VariantType.Double), StatusCode:StatusCode(Good), SourceTimestamp:2022-05-01 03:53:33.253081)
```
最初のほうの出力でNode idが取得できていて、最後のほうでは変数データに値するノードとその値が取得できていることがわかります。
:::message
`AirConditioner_1`の`Tempurature`が`71.26...`となっていて、エアコン温度を示していることがわかりますね(たぶんこれ摂氏じゃなくて、華氏でしょう)。
:::

> cryptography is not installed, use of crtpto disabled

というログが出力されていますが、これは暗号化プロトコルを使用していないから出力されてるログだと思います。使用する際には証明書などが必要になるはずです。

# まとめ・感想
- Demo Serverを使用してOPC-UAテスト用サーバを立てました。
- freeopcuaライブラリを使用してテスト用クライアントスクリプトを記述し実行しました。

スクリプト自体は思ったより簡単に書けましたし、githubのexampleにアンコメントしたらすぐに動くようなスクリプトが残していてくれているのでドキュメントを探し回らなくても直感的に何書いたら良いのかがわかりました。
データ取得するだけならすぐに何かしらできそうですね。

次はRaspberry PiにOPC-UAサーバを構築して、Pythonから通信してみようと思います。


[^1]: https://www.slideshare.net/ssuserd477fa/ethercatprofinet-opc-ua
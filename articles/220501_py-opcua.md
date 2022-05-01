---
title: "PythonでOPC-UA"
emoji: "🎉"
type: "tech" # tech: 技術記事 / idea: アイデア
topics: [Python, opcua]
published: true
---
# やりたいこと
PythonからOPC-UAに接続したい。

## 背景
EtherCATやPROFINETなどの産業用フィールドバスを利用した通信は、環境をそろえるのがとてもしんどいです。
OPC-UAを使うと、前述のフィールドバスのプロトコルをOPC-UAがサポートしているので、サーバの役割をしてくれてフィールドバス専用機器をそろえる必要がなくなります。
さらに、Raspberry Piでも動作可能という情報があったので、簡易的な環境であれば安価に構築できそうということがかなり大きなモチベーションになりました。
https://www.slideshare.net/ssuserd477fa/ethercatprofinet-opc-ua

# サーバー側
まずは、サーバー側の準備をします。
サーバーには[Unified-Automation](https://www.unified-automation.com/downloads.html)のDemo Serverを使ってみます。(ダウンロードにはユーザ登録が必要です。)
サイト上側タブの「Downloads」から、「OPC UA Servers」を選び、OPC UA C++ Demo Server (Windows)からダウンロードできます。

ダウンロード・インストール出来たらサーバアプリを立ち上げます。
おそらくこの時点で、`opc.tcp://localhost:48010`というサーバが立ち上がっています。

これでサーバー側の準備は完了です。

# クライアント側
次にクライアント側の準備をします。
クライアントにはpythonの[freeopcua](https://github.com/FreeOpcUa/python-opcua)ライブラリを利用します。
pipであれば以下でインストールできます。  
`pip install opcua`


:::details 動作確認用クライアント側アプリ
最初はどういった代物なのかわからなかったので、ネットにあるアプリを頼りました。
クライアント側のアプリでUaExpertというアプリがあります。
これも[Unified-Automation](https://www.unified-automation.com/downloads.html)のサイトから「OPC UA Client」→「UaExpert」でダウンロードできます。
(なお、後述のコード中にあるノードの調べ方がわからなかったので今回はこのアプリで調べました。)

[追記]
freeopcuaがgithubに載せている、`Simple OPC-UA GUI client`というアプリもありました。
`pip install opcua-client`でインストールし、`opcua-client`で起動できます。
これの便利なところは、簡単にデータをグラフ表示できるところです。
ノードリストで右クリックをするとメニューにグラフ追加が表示されます。
更新間隔は`Interval`で調整可能です。
:::

### スクリプト
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
	myvar = root.get_child([
		"0:Objects", 
		"0:Server", 
		"3:AreaAirConditioner",
		"3:AirConditioner_1",
		"3:Temperature",
	])
	print("myvar is: ", myvar)
	print(myvar.get_data_value())
finally:
	client.disconnect()
```
`Client`でクライアントクラスをインスタンス化し、`connect`で接続します。
`root = client.get_root_node()`でrootのノードを取得しています。
`root.get_children()`はrootの子ノードです。
`myvar = root.get_child`で直接取得したいノードを指定することができます。
ここで`myvar`はデモサーバの中にある、`AirConditioner`モデルの温度(`Temperature`)を取得しています。
`myvar.get_data_value()`とすることで`myvar`のデータを取得しています。
(`myvar.get_value()`でノードの値だけを取り出すこともできます。)

それでは、このスクリプトを実行してみます。
# 実行結果
特にエラーなければ以下の出力が得られるはずです。
```py
cryptography is not installed, use of crypto disabled
cryptography is not installed, use of crypto disabled
Objects node is:  i=84
Children of root are:  [Node(TwoByteNodeId(i=87)), Node(TwoByteNodeId(i=85)), Node(TwoByteNodeId(i=86))]
myvar is:  ns=3;s=AirConditioner_1.Temperature
DataValue(Value:Variant(val:71.26239999999964,type:VariantType.Double), StatusCode:StatusCode(Good), SourceTimestamp:2022-05-01 03:53:33.253081)
```
最初のほうの出力でNode idが取得できていて、最後のほうでは変数データに値するノードとその値が取得できている(`AirConditioner_1`の`Tempurature`が`71.26...`)ことがわかります。

> cryptography is not installed, use of crtpto disabled

これは暗号化プロトコルを使用していないから出力されてるログだと思います。  
使用する際には証明書などが必要になるはずです。

# まとめ・感想
スクリプト自体は思ったより簡単に書けましたし、githubのexampleにアンコメントしたらすぐに動くようなスクリプトが残していてくれているのでドキュメントを探し回らなくても直感的に何書いたら良いのかがわかりました。
データ取得するだけならすぐに何かしらできそうですね。

次はRaspberry PiにOPC-UAサーバを構築して、Pythonから通信してみようと思います。
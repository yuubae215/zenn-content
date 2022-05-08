---
title: "jupyterlab variable inspectorのインストール"
emoji: "🔅"
type: "tech" # tech: 技術記事 / idea: アイデア
topics: [jupyterlab]
published: false
---
## jupyterlab_variableinspector
コードを書くとき普段はVSCodeを愛用していますが、何やらこのような記事を発見しました。
https://hk29.hatenablog.jp/entry/2020/09/26/164521
記事によると、Jupyter Lab上でコードを書きつつ、変数の中身を表示できるとのこと。
神機能ではないですか。。。！

早速導入を試みようと思います。

## はじめに
まだJupyter Lab自体未導入の方は以下から導入してみてください。
https://jupyter.org/install
Extensionにはnodejsに依存しているパッケージが多いらしいので、nodejsも入れておきます。
https://inabower.hateblo.jp/entry/2019/01/07/215923

## インストール
以下が公式です。
https://github.com/lckr/jupyterlab-variableInspector
これを見るといくつかインストール方法をがありそうです。

まあmain wayと書いてあるpipで行ってみましょう。
```powershell
pip install lckr-jupyterlab-variableinspector
```
これで入ったら楽勝ですね。

さあどうでしょうか。
```powershell
      error: could not create 'build\bdist.win-amd64\wheel\lckr_jupyterlab_variableinspector-3.0.9.data\data\share\jupyter\labextensions\@lckr\jupyterlab_variableinspector': 指定されたパスが見つかりません。
      [end of output]

  note: This error originates from a subprocess, and is likely not a problem with pip.
  ERROR: Failed building wheel for lckr-jupyterlab-variableinspector
Failed to build lckr-jupyterlab-variableinspector
ERROR: Could not build wheels for lckr-jupyterlab-variableinspector, which is required to install pyproject.toml-based projects
```
そう簡単にはいかないやつですか。。。
エラーログ長かったので最後のほうだけ抜粋しました。

エラーログのここが気になったので調べてみました。
> ERROR: Could not build wheels for lckr-jupyterlab-variableinspector, which is required to install pyproject.toml-based projects

ビルドにはpyproject.tomlベースのプロジェクトが要る？
(ただのpip install...じゃあかんのか。)

ネットを探すとこちらの記事にそれらしいことが記載されていました。
https://orolog.hatenablog.jp/entry/2019/03/24/223531
どうやら`pip install git+https://github...`でいけそうな気がしてきました。

ということで、以下コマンドで再チャレンジしてみます。
```powershell
pip install git+https://github.com/lckr/jupyterlab-variableInspector.git
```
おお。。。！通りますね。
```powershell
Successfully built lckr-jupyterlab-variableinspector
Installing collected packages: lckr-jupyterlab-variableinspector
Successfully installed lckr-jupyterlab-variableinspector-3.0.9
```
無事にインストールできたそうです。

では、Jupyter Labを起動してみましょう。
```powershell
Jupyter-lab
```

あれ。。。起動したら`Build Recommended`と出てしまいました(なんで？)。

ビルドボタンが出てたので押してみるも、以下のビルドエラーが。

```powershell
Build Failed
Build failed with 500.

        If you are experiencing the build failure after installing an extension (or trying to include previously installed extension after updating JupyterLab) please check the extension repository for new installation instructions as many extensions migrated to the prebuilt extensions system which no longer requires rebuilding JupyterLab (but uses a different installation procedure, typically involving a package manager such as 'pip' or 'conda').

        If you specifically intended to install a source extension, please run 'jupyter lab build' on the server for full output.
```

ログに`jupyter lab build`と出ているのでそのまま打ってみました。

```powershell
jupyter lab build
```
しかし、またエラーが。。。
```powershell
An error occurred.
RuntimeError: JupyterLab failed to build
See the log file for details:  C:\Users\toitoy8\AppData\Local\Temp\jupyterlab-debug-gj6pp_oo.log
```
エラーログの途中で、メモリ不足の可能性もあるからオプション付けてビルドしてみてと書いてあったので、それも試してみましたが同じエラーでした。

このやり方は詰まったので、別の方法を試してみます。
はじめのサイトに`jupyter labextension install`でのインストール方法があったので、それでインストールしてみることに。

コマンドは以下です。
```powershell
jupyter labextension install @lckr/jupyterlab_variableinspector
```
うーん、これも通りませんね。
```powershell    
Building jupyterlab assets (production, minimized)
An error occurred.
RuntimeError: JupyterLab failed to build
See the log file for details:  C:\Users\toitoy8\AppData\Local\Temp\jupyterlab-debug-2z8ki8ah.log
```

つまりにつまった挙句、以下のteratailでQAを見つけました。
https://teratail.com/questions/348800
どうやら、公式のver.3.0.9が怪しいとのことです。

3.0.7でインストールする方法があったので、以下で試してみることに。
```powershell
jupyter labextension install @lckr/jupyterlab_variableinspector@3.0.7
```
なにもログ出ませんでしたが、なんか通ったみたいです。

再度Jupyter Labを起動してみます。

![image](https://user-images.githubusercontent.com/53713805/167282128-4902e4b2-aaa7-49b1-bedc-3eb913a7b719.png)

左側の拡張機能のタブから検索すると、variableinspectorがインストールされていることが確認できました！

## まとめ
![image](https://user-images.githubusercontent.com/53713805/167302296-511594dd-af99-4e34-a540-49e39bf754cd.png)
Jupyter Labの拡張機能variable inspectorを導入してみました。
結局やったことは、
・Jupyter Labのインストール
・nodejsのインストール
・`jupyter labextension install @lckr/jupyterlab_variableinspector@3.0.7`の実行
です。
あとは`Jupyter-lab`で起動して、画面内の右クリックメニューから`Open variable inspector`を選択すれば変数ウォッチ画面が表示されます。
配列変数だと変数の隣の虫眼鏡から配列の中身が見えるので便利そうです。
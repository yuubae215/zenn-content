# Zennコンテンツ作成・管理ガイド

## 概要

[Zenn](https://zenn.dev/)は、プログラミングやIT技術に関する記事や書籍を公開できるプラットフォームです。このガイドでは、Zenn CLIを使用したコンテンツの作成・管理方法について説明します。

## リポジトリ構成

現在のリポジトリには以下のディレクトリとファイルが含まれています：

- `articles/` - 個別の記事が格納されるディレクトリ
- `books/` - 本（複数の章で構成される長いコンテンツ）が格納されるディレクトリ
- `images/` - 記事や本で使用する画像ファイルを格納するディレクトリ
- `package.json` - プロジェクトの依存関係情報
- `README.md` - リポジトリの基本情報

## 記事の作成方法

### 1. 新しい記事ファイルの作成

Zenn CLIを使用して新しい記事を作成するには：

```bash
npx zenn new:article
```

または、既存のリポジトリに手動でマークダウンファイルを作成することもできます：

```bash
touch articles/my-new-article.md
```

### 2. 記事のフロントマター設定

記事ファイルの先頭には、以下のようなフロントマターを設定します：

```markdown
---
title: "記事のタイトル"
emoji: "😸" # アイキャッチとして使われる絵文字
type: "tech" # tech: 技術記事 / idea: アイデア記事
topics: ["JavaScript", "TypeScript", "React"] # タグ（5つまで）
published: true # 公開設定（falseにすると下書き）
---

ここから本文を書きます...
```

### 3. 記事の執筆と編集

マークダウン形式で記事を執筆します。Zennでは以下の記法が使用できます：

- 通常のマークダウン記法（見出し、リスト、リンクなど）
- コードブロック（シンタックスハイライト対応）
- 数式（KaTeX）
- メッセージボックス
- アコーディオン（トグル）

### 4. プレビュー

```bash
npx zenn preview
```

このコマンドを実行すると、ローカルサーバーが起動し、ブラウザでプレビューが表示されます（デフォルトでは http://localhost:8000）。

## 本の作成方法

### 1. 新しい本の作成

```bash
npx zenn new:book
```

### 2. 本の設定

作成された `books/book-slug/config.yaml` ファイルを編集します：

```yaml
title: "本のタイトル"
summary: "本の簡単な説明"
topics: ["JavaScript", "React"]
published: true
price: 0 # 有料の場合は価格（200〜5000）
chapters:
  - introduction
  - chapter1
  - chapter2
```

### 3. 各章の作成

各章はマークダウンファイルとして作成します：

```bash
touch books/book-slug/introduction.md
touch books/book-slug/chapter1.md
touch books/book-slug/chapter2.md
```

## 画像の追加

画像ファイルは `images` ディレクトリに配置し、以下のように参照します：

```markdown
![画像の説明](/images/example.png)
```

## GitHubへのプッシュと公開

1. 変更をコミットします：

```bash
git add .
git commit -m "コミットメッセージ"
```

2. リモートリポジトリへプッシュします：

```bash
git push origin main
```

3. GitHubにプッシュすると、自動的にZennに反映されます（GitHub連携が設定されている場合）。

## 既存の記事一覧

現在のリポジトリには以下の記事が含まれています：

- `.keep` - ディレクトリを維持するための空ファイル
- `220501_py-opcua.md` - OPC UAに関する記事
- `220508_jup-extend01.md` - Jupyterの拡張に関する記事
- `220514_py-roslibpy_rosbridge.md` - ROS関連の記事
- `220519_codesys.md` - CODESYSに関する記事
- `220530_codesys-ethercat.md` - CODESYS EtherCATに関する記事
- `220605_py-pyqtgraph.md` - PyQtGraphに関する記事
- `220701_phone-change_to_android.md` - Androidへの乗り換えに関する記事
- `ai-programming-guide.md` - AIを活用したプログラミング効率化ガイド
- `javascript-typescript-trends.md` - JavaScriptとTypeScriptの最新トレンドと実践的活用法
- `microservices-guide-2025.md` - 実践的なマイクロサービス開発ガイド2025年版

## 既存の本

- `books/programming-intro/` - プログラミング入門に関する本
  - `config.yaml` - 本の設定ファイル
  - `introduction.md` - 導入章
  - `javascript-fundamentals.md` - JavaScript基礎
  - `programming-basics.md` - プログラミングの基礎

## 参考リンク

- [Zenn CLIの公式ガイド](https://zenn.dev/zenn/articles/zenn-cli-guide)
- [ZennのMarkdown記法一覧](https://zenn.dev/zenn/articles/markdown-guide)
- [本の作成ガイド](https://zenn.dev/zenn/articles/zenn-book-guide)
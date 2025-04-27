---
title: "JavaScript基礎"
---

# JavaScript基礎

## JavaScriptとは

JavaScriptは、1995年にNetscape社のブレンダン・アイク（Brendan Eich）によって開発されたプログラミング言語です。当初はウェブページに動的な要素を追加するための言語として設計されましたが、現在ではウェブブラウザだけでなく、サーバー、モバイルアプリ、デスクトップアプリなど、様々な環境で利用されています。

### JavaScriptの特徴

1. **インタープリタ言語**: JavaScriptはコンパイルが不要で、ブラウザが直接コードを解釈・実行します
2. **動的型付け**: 変数の型は実行時に自動的に決定され、変更可能です
3. **オブジェクト指向**: プロトタイプベースのオブジェクト指向プログラミングをサポート
4. **関数型プログラミング**: 関数を第一級オブジェクトとして扱います
5. **イベント駆動**: ユーザーの操作や他のイベントに応じてコードを実行できます

### JavaScriptの位置づけ

ウェブ開発における3つの主要技術：

- **HTML**: ウェブページの構造を定義（骨格）
- **CSS**: ウェブページの見た目をスタイリング（外見）
- **JavaScript**: ウェブページに動的な機能を追加（動作）

## JavaScriptの基本構文

### コードの配置と実行

HTMLファイル内でJavaScriptを実行する方法は主に3つあります：

#### 1. インラインスクリプト

HTMLタグの属性としてJavaScriptを記述する方法：

```html
<button onclick="alert('ボタンがクリックされました')">クリック</button>
```

#### 2. 内部スクリプト

HTML文書内の`<script>`タグ内にJavaScriptを記述する方法：

```html
<!DOCTYPE html>
<html>
<head>
  <title>JavaScriptの例</title>
</head>
<body>
  <h1>こんにちは！</h1>
  
  <script>
    // JavaScriptコードをここに記述
    alert('ページが読み込まれました');
  </script>
</body>
</html>
```

#### 3. 外部スクリプト

別ファイル（.jsファイル）にJavaScriptを記述し、HTMLから参照する方法：

```html
<!DOCTYPE html>
<html>
<head>
  <title>外部JavaScriptの例</title>
  <!-- 外部JavaScriptファイルの読み込み -->
  <script src="script.js"></script>
</head>
<body>
  <h1>こんにちは！</h1>
</body>
</html>
```

script.js:
```javascript
// 外部JavaScriptファイル
alert('外部ファイルから実行されました');
```

### コメント

JavaScriptでは2種類のコメントが使用できます：

```javascript
// 単一行コメント（行末まで）

/*
  複数行コメント
  複数行に渡って
  コメントを書くことができます
*/
```

コメントはコードの理解や保守性を高めるために非常に重要です。

### 変数と定数

JavaScriptでの変数宣言には主に3つの方法があります：

#### 1. var（古い方法）

```javascript
var name = "田中";
var age = 25;
```

`var`は古い宣言方法で、一部の問題（スコープの問題など）があるため、モダンなJavaScriptでは通常使用しません。

#### 2. let（推奨）

```javascript
let name = "田中";
let age = 25;
```

`let`は再代入可能な変数を宣言します。モダンなJavaScriptでは`var`の代わりにこちらを使用します。

#### 3. const（定数）

```javascript
const PI = 3.14159;
const TAX_RATE = 0.1;
```

`const`は再代入不可能な定数を宣言します。値が変更されるべきでない場合は`const`を使用します。

### データ型とリテラル

JavaScriptの主要なデータ型を詳しく見ていきましょう：

#### 基本データ型（プリミティブ型）

1. **数値型（Number）**
   ```javascript
   let integer = 42;        // 整数
   let float = 3.14;        // 浮動小数点数
   let exponential = 2.5e5; // 指数表記（250000）
   let hexadecimal = 0xFF;  // 16進数（255）
   let binary = 0b1010;     // 2進数（10）
   let octal = 0o747;       // 8進数（487）
   let infinity = Infinity; // 無限大
   let notANumber = NaN;    // 非数（Not a Number）
   ```

2. **文字列型（String）**
   ```javascript
   let singleQuotes = 'こんにちは';     // シングルクォート
   let doubleQuotes = "JavaScript";    // ダブルクォート
   let backticks = `${singleQuotes}、${doubleQuotes}`; // テンプレートリテラル
   ```

   テンプレートリテラル（バッククォート）では変数の埋め込みや複数行の文字列が簡単に作成できます。

3. **論理型（Boolean）**
   ```javascript
   let isActive = true;
   let isLoggedIn = false;
   ```

4. **undefined**
   ```javascript
   let undefinedVar;
   console.log(undefinedVar); // undefined（値が代入されていない）
   ```

5. **null**
   ```javascript
   let emptyValue = null; // 意図的に「値がない」ことを表す
   ```

6. **シンボル（Symbol）** - ES6で導入された新しい型
   ```javascript
   const uniqueKey = Symbol('description');
   ```

#### 参照型（オブジェクト型）

1. **オブジェクト（Object）**
   ```javascript
   let person = {
     name: "佐藤",
     age: 30,
     isStudent: false,
     greet: function() {
       return `こんにちは、${this.name}です。`;
     }
   };
   
   // アクセス方法
   console.log(person.name);        // ドット記法
   console.log(person["age"]);      // ブラケット記法
   console.log(person.greet());     // メソッド呼び出し
   ```

2. **配列（Array）**
   ```javascript
   let fruits = ["りんご", "バナナ", "オレンジ"];
   
   // アクセス方法
   console.log(fruits[0]);           // りんご
   console.log(fruits.length);       // 3
   
   // 配列メソッド
   fruits.push("ぶどう");            // 末尾に追加
   fruits.pop();                     // 末尾から削除
   fruits.unshift("いちご");         // 先頭に追加
   fruits.shift();                   // 先頭から削除
   ```

3. **関数（Function）**
   ```javascript
   // 関数宣言
   function add(a, b) {
     return a + b;
   }
   
   // 関数式
   const multiply = function(a, b) {
     return a * b;
   };
   
   // アロー関数（ES6）
   const subtract = (a, b) => a - b;
   ```

### 型変換

JavaScriptでは自動的に型変換（暗黙的型変換）が行われる場合がありますが、明示的に型を変換することもできます。

#### 暗黙的型変換（自動）

```javascript
let num = 5;
let str = "10";

console.log(num + str);      // "510" (数値が文字列に変換されて連結)
console.log(str - num);      // 5 (文字列が数値に変換されて減算)
console.log("10" == 10);     // true (値の比較、型変換あり)
console.log("10" === 10);    // false (値と型の比較、型変換なし)
```

#### 明示的型変換（手動）

```javascript
// 文字列 → 数値
let str = "42";
let num1 = Number(str);      // 42
let num2 = parseInt(str);    // 42 (整数として解析)
let num3 = parseFloat(str);  // 42.0 (浮動小数点として解析)

// 数値 → 文字列
let num = 42;
let str1 = String(num);      // "42"
let str2 = num.toString();   // "42"

// 他の型 → 論理値
let bool1 = Boolean("");     // false (空文字列はfalse)
let bool2 = Boolean(0);      // false (0はfalse)
let bool3 = Boolean("hello"); // true (空でない文字列はtrue)
let bool4 = Boolean(42);     // true (0以外の数値はtrue)
```

### 演算子

JavaScriptのさらに多くの演算子を見ていきましょう：

#### 1. 算術演算子

```javascript
// 基本演算子
let a = 10;
let b = 3;

console.log(a + b);  // 加算: 13
console.log(a - b);  // 減算: 7
console.log(a * b);  // 乗算: 30
console.log(a / b);  // 除算: 3.3333...
console.log(a % b);  // 剰余: 1

// インクリメント/デクリメント
let c = 5;
c++;                 // 後置インクリメント
console.log(c);      // 6
++c;                 // 前置インクリメント
console.log(c);      // 7
c--;                 // 後置デクリメント
console.log(c);      // 6
--c;                 // 前置デクリメント
console.log(c);      // 5

// 指数演算子（ES2016）
console.log(2 ** 3); // 8 (2の3乗)
```

#### 2. 代入演算子

```javascript
let x = 10;

x += 5;          // x = x + 5 と同じ (15)
x -= 3;          // x = x - 3 と同じ (12)
x *= 2;          // x = x * 2 と同じ (24)
x /= 4;          // x = x / 4 と同じ (6)
x %= 4;          // x = x % 4 と同じ (2)
x **= 3;         // x = x ** 3 と同じ (8)
```

#### 3. 比較演算子

```javascript
let a = 5;
let b = "5";
let c = 10;

// 等価演算子（値の比較、型変換あり）
console.log(a == b);     // true
console.log(a != c);     // true

// 厳密等価演算子（値と型の比較、型変換なし）
console.log(a === b);    // false
console.log(a !== b);    // true

// 関係演算子
console.log(a < c);      // true
console.log(a > c);      // false
console.log(a <= c);     // true
console.log(a >= c);     // false
```

#### 4. 論理演算子

```javascript
let x = true;
let y = false;

console.log(x && y);     // 論理AND: false
console.log(x || y);     // 論理OR: true
console.log(!x);         // 論理NOT: false

// ショートサーキット評価
console.log(x && "Yes"); // "Yes"（xがtrueなのでyを評価）
console.log(y && "Yes"); // false（yがfalseなので"Yes"は評価されない）
console.log(x || "Yes"); // true（xがtrueなので"Yes"は評価されない）
console.log(y || "Yes"); // "Yes"（yがfalseなので"Yes"を評価）
```

#### 5. 三項演算子

```javascript
let age = 20;
let status = (age >= 18) ? "大人" : "子供";
console.log(status);     // "大人"
```

#### 6. その他の演算子

```javascript
// スプレッド演算子（ES6）
let arr1 = [1, 2, 3];
let arr2 = [...arr1, 4, 5]; // [1, 2, 3, 4, 5]

// nullish合体演算子（ES2020）
let user = null;
let username = user ?? "ゲスト"; // "ゲスト"

// オプショナルチェーン（ES2020）
let obj = { a: { b: 1 } };
console.log(obj?.c?.d);  // undefined (エラーにならない)
```

## 制御構造

JavaScriptでプログラムの流れを制御する方法を詳しく見ていきましょう：

### 条件分岐

#### 1. if文

```javascript
let hour = 14;

if (hour < 12) {
  console.log("おはようございます");
} else if (hour < 18) {
  console.log("こんにちは");
} else {
  console.log("こんばんは");
}
```

#### 2. switch文

```javascript
let day = "月";
let schedule;

switch (day) {
  case "月":
    schedule = "プログラミング学習";
    break;
  case "水":
    schedule = "ミーティング";
    break;
  case "金":
    schedule = "プロジェクト作業";
    break;
  default:
    schedule = "予定なし";
}

console.log(`今日の予定: ${schedule}`);
```

### ループ（繰り返し）

#### 1. for文

```javascript
// 基本的なforループ
for (let i = 0; i < 5; i++) {
  console.log(`カウント: ${i}`);
}

// 配列の走査
let colors = ["赤", "青", "緑"];
for (let i = 0; i < colors.length; i++) {
  console.log(`色: ${colors[i]}`);
}
```

#### 2. for...of文（ES6）

```javascript
// 配列の要素を直接反復
let fruits = ["りんご", "バナナ", "オレンジ"];
for (let fruit of fruits) {
  console.log(`フルーツ: ${fruit}`);
}

// 文字列の各文字を反復
for (let char of "こんにちは") {
  console.log(char);
}
```

#### 3. for...in文

```javascript
// オブジェクトのプロパティを反復
let person = {
  name: "山田",
  age: 25,
  job: "エンジニア"
};

for (let key in person) {
  console.log(`${key}: ${person[key]}`);
}
```

#### 4. while文

```javascript
// 条件が真である限り繰り返す
let i = 0;
while (i < 5) {
  console.log(`while: ${i}`);
  i++;
}
```

#### 5. do...while文

```javascript
// 最低1回は実行し、その後条件が真である限り繰り返す
let j = 0;
do {
  console.log(`do-while: ${j}`);
  j++;
} while (j < 5);
```

### ループ制御

```javascript
// break: ループを即座に終了
for (let i = 0; i < 10; i++) {
  if (i === 5) {
    break;
  }
  console.log(`break例: ${i}`);
}

// continue: 現在の反復をスキップし、次の反復に進む
for (let i = 0; i < 5; i++) {
  if (i === 2) {
    continue;
  }
  console.log(`continue例: ${i}`);
}
```

## 関数

関数はJavaScriptのコードを再利用可能なブロックにまとめる方法です。

### 関数の定義と呼び出し

```javascript
// 基本的な関数宣言
function greet(name) {
  return `こんにちは、${name}さん！`;
}

// 関数の呼び出し
let message = greet("田中");
console.log(message);  // こんにちは、田中さん！

// デフォルトパラメータ（ES6）
function greetWithDefault(name = "ゲスト") {
  return `こんにちは、${name}さん！`;
}

console.log(greetWithDefault());  // こんにちは、ゲストさん！
console.log(greetWithDefault("佐藤"));  // こんにちは、佐藤さん！
```

### 関数式とアロー関数

```javascript
// 関数式
const add = function(a, b) {
  return a + b;
};

// アロー関数（ES6）
const multiply = (a, b) => a * b;

// 複数行のアロー関数
const divide = (a, b) => {
  if (b === 0) {
    throw new Error("0で割ることはできません");
  }
  return a / b;
};

// 即時実行関数（IIFE）
(function() {
  console.log("この関数は定義と同時に実行されます");
})();
```

### スコープと変数の可視性

```javascript
// グローバルスコープ
let globalVar = "グローバル変数";

function testScope() {
  // 関数スコープ
  let functionVar = "関数内変数";
  
  if (true) {
    // ブロックスコープ（ES6 letとconst）
    let blockVar = "ブロック内変数";
    var oldVar = "varはブロックスコープを無視";
    
    console.log(globalVar);    // アクセス可能
    console.log(functionVar);  // アクセス可能
    console.log(blockVar);     // アクセス可能
  }
  
  console.log(globalVar);      // アクセス可能
  console.log(functionVar);    // アクセス可能
  // console.log(blockVar);    // エラー: blockVarは定義されていません
  console.log(oldVar);         // "varはブロックスコープを無視"（アクセス可能）
}

testScope();
console.log(globalVar);        // アクセス可能
// console.log(functionVar);   // エラー: functionVarは定義されていません
// console.log(blockVar);      // エラー: blockVarは定義されていません
// console.log(oldVar);        // エラー: oldVarは定義されていません
```

### コールバック関数

コールバック関数は、他の関数に引数として渡される関数です。

```javascript
// コールバック関数を受け取る関数
function processData(data, callback) {
  // データ処理ロジック
  const processedData = `処理済み: ${data}`;
  
  // 処理が完了したらコールバックを実行
  callback(processedData);
}

// コールバック関数を定義して渡す
processData("サンプルデータ", function(result) {
  console.log(result);  // 処理済み: サンプルデータ
});

// アロー関数でコールバックを渡す
processData("別のデータ", result => {
  console.log(result);  // 処理済み: 別のデータ
});
```

## オブジェクトと配列の詳細

### オブジェクトの操作

```javascript
// オブジェクトの作成
const user = {
  firstName: "太郎",
  lastName: "山田",
  age: 30,
  email: "taro@example.com",
  isActive: true,
  address: {
    city: "東京",
    postalCode: "123-4567"
  },
  hobbies: ["読書", "プログラミング", "旅行"]
};

// プロパティへのアクセス
console.log(user.firstName);        // 太郎
console.log(user["lastName"]);      // 山田
console.log(user.address.city);     // 東京
console.log(user.hobbies[1]);       // プログラミング

// プロパティの追加と更新
user.phone = "090-1234-5678";       // プロパティの追加
user.age = 31;                       // プロパティの更新

// プロパティの削除
delete user.isActive;

// プロパティの存在確認
console.log("email" in user);       // true
console.log(user.hasOwnProperty("phone")); // true

// オブジェクトのメソッド
const calculator = {
  add: function(a, b) {
    return a + b;
  },
  // メソッド短縮記法（ES6）
  subtract(a, b) {
    return a - b;
  }
};

console.log(calculator.add(5, 3));      // 8
console.log(calculator.subtract(10, 4)); // 6
```

### オブジェクトの操作（ES6以降）

```javascript
// スプレッド演算子（オブジェクトのコピーと拡張）
const baseUser = { id: 1, name: "山田" };
const extendedUser = { ...baseUser, email: "yamada@example.com" };

// オブジェクトの分割代入
const { name, email, id } = extendedUser;
console.log(name, email, id);  // 山田 yamada@example.com 1

// 残余パラメータ（レスト演算子）
const { id: userId, ...userInfo } = extendedUser;
console.log(userId, userInfo);  // 1 { name: "山田", email: "yamada@example.com" }

// Object.keysとObject.values（ES2017）
console.log(Object.keys(baseUser));     // ["id", "name"]
console.log(Object.values(baseUser));   // [1, "山田"]
console.log(Object.entries(baseUser));  // [["id", 1], ["name", "山田"]]
```

### 配列の操作

```javascript
// 配列の作成
const fruits = ["りんご", "バナナ", "オレンジ"];

// 要素へのアクセス
console.log(fruits[0]);           // りんご
console.log(fruits[fruits.length - 1]);  // オレンジ

// 配列の結合
const moreFruits = ["いちご", "ぶどう"];
const allFruits = fruits.concat(moreFruits);
// または spread 演算子（ES6）
const allFruitsSpread = [...fruits, ...moreFruits];

// 配列の分割代入（ES6）
const [first, second, ...rest] = allFruits;
console.log(first, second, rest);  // りんご バナナ ["オレンジ", "いちご", "ぶどう"]

// 配列操作メソッド
fruits.push("メロン");           // 末尾に追加
fruits.pop();                    // 末尾から削除して返す
fruits.unshift("パイナップル");   // 先頭に追加
fruits.shift();                  // 先頭から削除して返す
fruits.splice(1, 1, "キウイ");    // 指定位置の要素を削除/置換
const citrus = fruits.slice(1, 3); // 指定範囲の要素を新しい配列として取得

// 配列の検索
console.log(fruits.indexOf("りんご"));  // 0 (見つからない場合は -1)
console.log(fruits.includes("キウイ"));  // true

// 配列の反復処理
fruits.forEach((fruit, index) => {
  console.log(`${index}: ${fruit}`);
});

// 配列の変換
const upperFruits = fruits.map(fruit => fruit.toUpperCase());
const longFruits = fruits.filter(fruit => fruit.length > 3);
const allLengths = fruits.reduce((total, fruit) => total + fruit.length, 0);

// 配列の並べ替え
fruits.sort();                  // アルファベット順
fruits.reverse();               // 逆順
```

### 配列の高度な操作（ES6以降）

```javascript
// find と findIndex
const users = [
  { id: 1, name: "田中" },
  { id: 2, name: "鈴木" },
  { id: 3, name: "佐藤" }
];

const suzuki = users.find(user => user.name === "鈴木");
const suzukiIndex = users.findIndex(user => user.name === "鈴木");

// Array.from（イテラブルから配列を作成）
const name = "JavaScript";
const chars = Array.from(name);  // ["J", "a", "v", "a", "S", "c", "r", "i", "p", "t"]

// Array.of（指定した値から配列を作成）
const numbers = Array.of(1, 2, 3, 4, 5);  // [1, 2, 3, 4, 5]

// flat と flatMap（ES2019）
const nestedArray = [1, [2, 3], [4, [5, 6]]];
const flatArray = nestedArray.flat();     // [1, 2, 3, 4, [5, 6]]
const deepFlatArray = nestedArray.flat(2); // [1, 2, 3, 4, 5, 6]
```

## 練習問題

以下の練習問題に挑戦して、この章で学んだJavaScriptの基本を強化しましょう：

1. 以下の条件に基づいて、点数（0〜100）から成績（A, B, C, D, F）を返す関数を作成してください：
   - 90以上：A
   - 80〜89：B
   - 70〜79：C
   - 60〜69：D
   - 60未満：F

2. 配列内の数値の平均値を計算する関数を作成してください。

3. オブジェクトの配列から特定のプロパティの値のみを抽出した新しい配列を返す関数を作成してください。（例：`extract(users, 'name')` は `["田中", "鈴木", "佐藤"]` を返す）

4. 文字列が回文（前から読んでも後ろから読んでも同じ）かどうかをチェックする関数を作成してください。

5. 与えられた文章内で最も頻出する単語を見つける関数を作成してください。

次の章では、JavaScriptの制御構造（条件分岐とループ）について詳しく学びます。
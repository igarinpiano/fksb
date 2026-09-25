# 缶サット fks-B-II(全国大会仕様) ソフトウェアレポジトリ
マイコン:TWELITE

## ディレクトリ説明
- **MWSTAGE/MWSDK/Act-samples**　:\
    TWELITEマイコンの組み込みソフトのc++群\
    MWSTAGEをPCにinstallしてできるディレクトリの`Act-samples/`だけおいてある(他は重いので)\
    `honban/`は打ち上げ前のfks-B実機に書き込んだもの\
    `receive/`は地上局の受信機のmonostickに書き込むもの．ただただ受信したものをserial通信で垂れ流すだけのプログラム\
    ほかはテストにつかったいろいろ

- **ana/**　:\
    本番データ分析ディレクトリ．大会当日に作った

- **cal/**　:\
    機体やRWの慣性モメントを求めるのに使った.pyファイル．これはそのまま実行できず，.FCStdをFreeCADでひらいてpython consoleで実行した

- **godot_ps/**　:\
    姿勢制御シムのgodotプロジェクトディレクトリ

- **godot_tr/**　:\
    地上局アプリのディレクトリ\
    `extension/`はgodotでTWELITE MONOSTICKと通信するための拡張．libftdiをc++で使うために作った\
    `fks_gd/`はgodotのプロジェクトディレクトリ

- **record/**　:\
    地上局アプリのlogを記録するディレクトリ．\
    `flash/`はフラッシュメモリのデータ抽出のデータ．\
    `realtime/`受信したデータ\
    `tugounoiide-ta/`あ　まずい　消せ消せ消せ(消しわすれただけ．そんな不正みたいなことはしてないですからね!？)\

- **soundRecord/**　:\
    なんだこれ…

あとは割とどうでもいいものです．



# C++を用いた経路計画の勉強用リポジトリ
##環境
- Visual Studio 2019
- OpenGL（描画）


## RRT
障害物は円を想定、円の半径を膨張させ、新しい点候補が円内に含まれるかで衝突判定を行っている。
RRTは最適な経路を保証しない
### RRTデモ映像
https://github.com/user-attachments/assets/ba4186f5-fc0d-4ef8-93af-5e56d280ddbb

※青い線が導出した経路、黒点は経路算出時の候補、灰色の丸は障害物




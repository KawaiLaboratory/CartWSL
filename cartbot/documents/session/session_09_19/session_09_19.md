# 打ち合わせ 2018 9/19
## 活動状況
- 胴体とカルマンフィルターで追跡する手法と足と歩行モデルで追跡する手法を使ったものを作った。
- 上記のものに加えて、足とカルマンフィルターで追跡するものを作った。
- サンプル1000個、10m直進、等速歩行中の検知率は以下の通り

| 足とカルマンフィルター | 胴体とカルマンフィルター | 足と歩行モデル |
| - | - | - |
| 66.3% | 61.5% | 87.9% |

- TurtleBotが遅すぎるため、実際の歩行速度に合わせて追跡できるかまだ確認できていない。

## 今後の予定
- 検知率改善案を検討中
- TurtleBot以外のロボットの使用の検討
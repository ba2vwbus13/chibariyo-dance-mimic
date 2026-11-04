#!/usr/bin/env python3
"""crowd_test.world から「服(色帯)だけを変えた」検証ワールドを生成する。

crowd_test.world では通行人(person2)だけが胸(床から0.98-1.33m)に赤帯を着けている。
Re-IDの色領域(床から0.95-1.35m)はこの配置を見て決めたため、別の服装でも効くかを確かめる。
人の動き・部屋・センサは crowd_test.world と同一で、帯の有無・高さ・色だけが違う。

使い方(このファイルのあるディレクトリで): python3 make_clothing_variants.py
生成物は crowd_<variant>.world。crowd_test.world を直したら再生成すること。
"""
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
RED = (0.8, 0.05, 0.05)
LIGHT_BLUE = (0.25, 0.55, 0.95)
NAVY = (0.05, 0.1, 0.4)

# 帯: (床からの下端[m], 上端[m], RGB)。人モデルのリンク原点は床から0.8m
TORSO = (0.975, 1.325)
VARIANTS = {
    # 本人が赤帯、通行人は無地(「赤=他人」に依存していないか)
    'swap': {'person': [(*TORSO, RED)], 'person2': []},
    # 通行人の赤帯が肩寄り(色領域と一部だけ重なる)
    'high': {'person': [], 'person2': [(1.20, 1.50, RED)]},
    # 通行人の赤帯がズボンの位置(色領域の外。失敗が予想される限界の確認)
    'pants': {'person': [], 'person2': [(0.45, 0.85, RED)]},
    # 色相が近く明るさが違う服(水色と紺)。H-Sヒストグラムは明度を見ない
    'similar': {'person': [(*TORSO, LIGHT_BLUE)], 'person2': [(*TORSO, NAVY)]},
    # 同じ服(見分けようがない対照条件)
    'identical': {'person': [(*TORSO, RED)], 'person2': [(*TORSO, RED)]},
}


def band_xml(name, z0, z1, rgb):
    center = (z0 + z1) / 2 - 0.8  # リンク原点(床から0.8m)基準
    c = ' '.join(f'{v:g}' for v in rgb)
    return f"""        <visual name="{name}">
          <pose>0 0 {center:.4f} 0 0 0</pose>
          <geometry><cylinder><radius>0.19</radius><length>{z1 - z0:.4f}</length></cylinder></geometry>
          <material>
            <ambient>{c} 1</ambient>
            <diffuse>{c} 1</diffuse>
          </material>
        </visual>
"""


def main():
    src = open(os.path.join(HERE, 'crowd_test.world')).read()
    # 元の赤帯(person2)を取り除いて、両者とも無地の状態にする
    base, n = re.subn(r'\s*<!-- 赤い帯[^\n]*-->\s*<visual name="shirt_band">.*?</visual>\n',
                      '\n', src, flags=re.S)
    assert n == 1, 'crowd_test.world の赤帯が見つからない'
    for variant, bands in VARIANTS.items():
        out = base
        for model, specs in bands.items():
            start = out.index(f'<model name="{model}">')
            end = out.index('</link>', start)
            end = out.rindex('\n', 0, end) + 1  # </link> の行頭に挿入する
            xml = ''.join(band_xml(f'band{i}', *s) for i, s in enumerate(specs))
            out = out[:end] + xml + out[end:]
        header = (f'<!-- make_clothing_variants.py で crowd_test.world から生成 ({variant})。'
                  f'直接編集しないこと -->\n')
        out = out.replace('<sdf', header + '<sdf', 1)
        path = os.path.join(HERE, f'crowd_{variant}.world')
        with open(path, 'w') as f:
            f.write(out)
        print('wrote', path)


if __name__ == '__main__':
    main()

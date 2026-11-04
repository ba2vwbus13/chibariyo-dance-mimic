# 公開してはいけない情報（IP アドレス、Wi-Fi の名前、MAC アドレス、メールアドレス、手元のフォルダの場所）が
# リポジトリに入っていないかを調べる。見つかったら場所を表示して、終了コード 1 で終わる。
#   python3 tools/check_secrets.py          … コミットの前に手元で実行する
#   GitHub Actions でも、プルリクエストのたびに実行する（.github/workflows/check.yml）
# PDF は pdftotext で文字を取り出して調べる（pdftotext がなければ飛ばす）
import os
import re
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TEXT_EXT = {'.md', '.html', '.py', '.svg', '.sh', '.bash', '.yaml', '.yml', '.xml', '.cfg', '.txt', '.css', '.csv',
            '.json', '.toml', '.launch', ''}
SKIP_DIRS = {'.git', 'build', 'install', 'log', '__pycache__', 'bags', 'videos'}
PATTERNS = [
    ('IP アドレス', re.compile(r'\b(192\.168|10\.100|172\.(1[6-9]|2\d|3[01]))\.\d+\.\d+\b')),
    ('Wi-Fi の名前', re.compile(r'Buffalo-[0-9A-Za-z-]+')),
    ('MAC アドレス', re.compile(r'\b([0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}\b')),
    ('メールアドレス', re.compile(r'[A-Za-z0-9._%+-]+@(gmail\.com|[A-Za-z0-9-]+\.ac\.jp|[A-Za-z0-9-]+\.kosen-ac\.jp)')),
    ('手元のフォルダの場所', re.compile(r'/Users/[A-Za-z0-9_.-]+|/home/(?!ubuntu\b)[a-z][a-z0-9_-]+')),
]
SELF = os.path.relpath(os.path.abspath(__file__), ROOT)


def texts():
    for root, dirs, files in os.walk(ROOT):
        dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
        for f in files:
            p = os.path.join(root, f)
            rel = os.path.relpath(p, ROOT)
            if rel == SELF or f.endswith('.local.md') or rel.startswith('local' + os.sep):
                continue
            ext = os.path.splitext(f)[1].lower()
            if ext == '.pdf':
                if shutil.which('pdftotext'):
                    yield rel, subprocess.run(['pdftotext', p, '-'], capture_output=True, text=True).stdout
            elif ext in TEXT_EXT:
                with open(p, encoding='utf-8', errors='ignore') as fp:
                    yield rel, fp.read()


def main():
    found = []
    for rel, text in texts():
        for i, line in enumerate(text.splitlines(), 1):
            for what, pat in PATTERNS:
                if pat.search(line):
                    found.append(f'{rel}:{i}: {what}: {line.strip()[:100]}')
    if found:
        print('公開してはいけない情報が入っているかもしれません（<ロボットのIP> のような書き方に直す）:')
        print('\n'.join(found[:100]))
        sys.exit(1)
    print('問題は見つかりませんでした')


if __name__ == '__main__':
    main()

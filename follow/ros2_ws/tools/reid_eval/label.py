"""collect の log.jsonl に、画像時刻での真値から person/person2 ラベルを付け直す"""
import bisect, json, math, sys, collections
def load(d):
    gt = collections.defaultdict(list)
    for l in open(f'{d}/gt.jsonl'):
        n, t, x, y, yaw = json.loads(l); gt[n].append((t, x, y, yaw))
    for n in gt: gt[n].sort()
    return gt
def at(seq, t):
    ts = [e[0] for e in seq]; i = bisect.bisect_left(ts, t)
    c = [seq[j] for j in (i-1, i) if 0 <= j < len(seq)]
    return min(c, key=lambda e: abs(e[0]-t))
def labeled(d, b_tol=0.10, d_tol=0.5):
    gt = load(d); out = []
    for l in open(f'{d}/log.jsonl'):
        x = json.loads(l); t = x['stamp_img']
        _, rx, ry, yaw = at(gt['robot'], t)
        cx, cy = rx + 0.255*math.cos(yaw), ry + 0.255*math.sin(yaw)
        hits = []; rel = {}
        for n in ('person', 'person2'):
            _, px, py, _ = at(gt[n], t)
            b = math.atan2(py-cy, px-cx) - yaw; b = math.atan2(math.sin(b), math.cos(b))
            dd = math.hypot(px-cx, py-cy); rel[n] = (dd, b)
            if abs(b - x['bearing']) < b_tol and abs(dd - 0.15 - x['dist']) < d_tol: hits.append(n)
        x['label2'] = hits[0] if len(hits) == 1 else ('both' if hits else '?')
        # 2人が方位で近いフレームは重なり(遮蔽)の可能性があるので印を付ける
        x['overlap'] = abs(rel['person'][1] - rel['person2'][1]) < 0.3
        out.append(x)
    return out
if __name__ == '__main__':
    for d in sys.argv[1:]:
        r = labeled(d)
        print(d.split('/')[-1], collections.Counter(x['label2'] for x in r), 'overlap', sum(x['overlap'] for x in r))

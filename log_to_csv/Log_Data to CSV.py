import re, csv, os, glob
from datetime import datetime
from collections import defaultdict, deque

# ── Config ─────────────────────────────────────────────────────────────────────
NOTEBOOK_DIR = r'C:\Users\hp\Desktop\BIAL Logs'
log_files    = sorted(glob.glob(os.path.join(NOTEBOOK_DIR, '*.txt')))

if not log_files:
    raise FileNotFoundError(f'No .txt files found in {NOTEBOOK_DIR}')

last_log = log_files[-1]
CSV_FILE = os.path.join(NOTEBOOK_DIR, os.path.splitext(os.path.basename(last_log))[0] + '.csv')

print(f'Found {len(log_files)} log file(s):')
for f in log_files:
    print(f'  {os.path.basename(f)}')
print(f'Output CSV -> {CSV_FILE}\n')

# ── Regexes ────────────────────────────────────────────────────────────────────
ts_re = re.compile(r'^(\d{4}-\d{2}-\d{2})\s+(\d{2}:\d{2}:\d{2})\s+-\s+')

tray_re = re.compile(
    r"sending data to message broker data:.*?"
    r"'action':\s*'(retrieve|store)'.*?"
    r"'status':\s*'(start|stop)'.*?"
    r"'tray_id':\s*'(TID-\d+)'.*?"
    r"'slot_id':\s*'(S-[\d\-]+)'",
    re.IGNORECASE | re.DOTALL
)

move_re = re.compile(
    r"sending data to message broker data:.*?"
    r"'action':\s*'(move)'.*?"
    r"'status':\s*'(start|stop)'.*?"
    r"'row':\s*(\d+).*?"
    r"'rack':\s*(\d+).*?"
    r"'slot':\s*(\d+)",
    re.IGNORECASE | re.DOTALL
)

def parse_ts(line):
    m = ts_re.match(line)
    if not m:
        return None
    return datetime.strptime(m.group(1) + ' ' + m.group(2), '%Y-%m-%d %H:%M:%S')

# ── Accumulators ───────────────────────────────────────────────────────────────
starts       = defaultdict(deque)
move_starts  = deque()
totals       = defaultdict(lambda: defaultdict(float))
counts       = defaultdict(lambda: defaultdict(int))
slots_seen   = defaultdict(set)
first_seen   = {}
last_seen    = {}
move_total_s = 0.0
move_count   = 0

# ── Parse all files ────────────────────────────────────────────────────────────
for log_path in log_files:
    fname = os.path.basename(log_path)
    matched = 0
    with open(log_path, 'r', encoding='utf-8', errors='replace') as f:
        for line in f:
            ts = parse_ts(line)
            if not ts:
                continue

            m = tray_re.search(line)
            if m:
                action, status, tray_id, slot_id = (
                    m.group(1).lower(), m.group(2).lower(), m.group(3), m.group(4)
                )
                first_seen.setdefault(tray_id, ts)
                last_seen[tray_id] = ts
                slots_seen[tray_id].add(slot_id)
                key = (tray_id, action)
                if status == 'start':
                    starts[key].append((ts, slot_id))
                else:
                    if starts[key]:
                        start_ts, _ = starts[key].popleft()
                        totals[tray_id][action] += (ts - start_ts).total_seconds()
                        counts[tray_id][action] += 1
                matched += 1
                continue

            m = move_re.search(line)
            if m:
                status = m.group(2).lower()
                if status == 'start':
                    move_starts.append((ts, m.group(3), m.group(4), m.group(5)))
                else:
                    if move_starts:
                        start_ts, *_ = move_starts.popleft()
                        move_total_s += (ts - start_ts).total_seconds()
                        move_count   += 1
                matched += 1

    print(f'  Parsed {fname}  ->  {matched} broker events matched')

# ── Sort trays CHRONOLOGICALLY by first_seen ───────────────────────────────────
sorted_trays = sorted(totals.keys(), key=lambda t: first_seen.get(t, datetime.min))

# ── Print table ────────────────────────────────────────────────────────────────
print()
HDR = f"{'Tray ID':<14} {'Slot IDs Visited':<36} {'Retr_s':>8} {'Rn':>4} {'Store_s':>8} {'Sn':>4} {'Total_s':>9}  {'First Seen':<20} Last Seen"
SEP = '-' * len(HDR)
print(SEP)
print(HDR)
print(SEP)

for tray_id in sorted_trays:               # <-- chronological
    r   = totals[tray_id].get('retrieve', 0.0)
    s   = totals[tray_id].get('store',    0.0)
    rn  = counts[tray_id].get('retrieve', 0)
    sn  = counts[tray_id].get('store',    0)
    sls = ' | '.join(sorted(slots_seen[tray_id]))
    fs  = str(first_seen.get(tray_id, ''))
    ls  = str(last_seen.get(tray_id,  ''))
    print(f"{tray_id:<14} {sls:<36} {r:>8.1f} {rn:>4} {s:>8.1f} {sn:>4} {r+s:>9.1f}  {fs:<20} {ls}")

print(SEP)
print(f"{'[MOVE]':<14} {'(robot moves - no tray/slot)':<36} {'':>8} {'':>4} {'':>8} {'':>4} {move_total_s:>9.1f}  count={move_count}")
print(SEP)
print(f"Total trays : {len(totals)}")
print(f"Move ops    : {move_count}  ({move_total_s:.1f}s total)")

# ── Write CSV ──────────────────────────────────────────────────────────────────
with open(CSV_FILE, 'w', newline='', encoding='utf-8') as out:
    w = csv.writer(out)
    w.writerow(['tray_id', 'slot_ids_visited',
                'retrieve_total_s', 'retrieve_count',
                'store_total_s',    'store_count',
                'total_s', 'first_seen', 'last_seen'])
    for tray_id in sorted_trays:           # <-- chronological
        r   = totals[tray_id].get('retrieve', 0.0)
        s   = totals[tray_id].get('store',    0.0)
        sls = ' | '.join(sorted(slots_seen[tray_id]))
        w.writerow([tray_id, sls,
                    round(r, 2), counts[tray_id].get('retrieve', 0),
                    round(s, 2), counts[tray_id].get('store',    0),
                    round(r+s, 2),
                    first_seen.get(tray_id), last_seen.get(tray_id)])
    w.writerow(['[MOVE]', '(robot moves)', '', '', '', '',
                round(move_total_s, 2), f'count={move_count}', ''])

print(f'\nCSV saved -> {CSV_FILE}')
print(f'File exists: {os.path.exists(CSV_FILE)}')

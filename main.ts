// Push-optimal Sokoban solver: each room solved on its own (a box pushed out of a
// room is lost, so a level's optimum is the sum of its rooms'), A* over push-states
// with a Hungarian box-to-goal assignment heuristic, dead-square and frozen-2x2
// pruning, and a push-BFS fallback. Budgets are work counts, never the clock.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
const IN = process.env.MAKEFUN_INPUT_DIR ?? '/input', OUT = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';
const D = [[0, -1, 'u', 'U'], [0, 1, 'd', 'D'], [-1, 0, 'l', 'L'], [1, 0, 'r', 'R']];
const AM = 100000, BM = 250000, WM = 3000000;
let W = 0;
const less = (a, b) => a[0] < b[0] || a[0] === b[0] && (a[1] < b[1] || a[1] === b[1] && a[2] < b[2]);
function hp(h, e) { let i = h.length; h.push(e); while (i > 0) { const p = i - 1 >> 1; if (!less(h[i], h[p])) break; [h[i], h[p]] = [h[p], h[i]]; i = p; } }
function hq(h) { const t = h[0], l = h.pop(); if (h.length) { h[0] = l; for (let i = 0;;) { let c = 2 * i + 1; if (c >= h.length) break; if (c + 1 < h.length && less(h[c + 1], h[c])) c++; if (!less(h[c], h[i])) break; [h[i], h[c]] = [h[c], h[i]]; i = c; } } return t; }
function parse(t) {
  const r = t.split('\n').filter((x) => x.length), h = r.length, w = Math.max(...r.map((x) => x.length));
  const wall = new Uint8Array(w * h).fill(1), g = [], b = [];
  let p = -1;
  r.forEach((row, y) => { for (let x = 0; x < row.length; x++) { const c = row[x], i = y * w + x; if (c === '#') continue; wall[i] = 0; if ('.*+'.includes(c)) g.push(i); if ('$*'.includes(c)) b.push(i); if ('@+'.includes(c)) p = i; } });
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) if (!x || !y || x === w - 1 || y === h - 1) wall[y * w + x] = 1;
  b.sort((a, b) => a - b);
  return { w, h, wall, goals: g.filter((x) => !wall[x]), boxes: b.filter((x) => !wall[x]), player: p };
}
function live(w, wall, gs) {
  const v = new Uint8Array(wall.length), q = [...gs];
  for (const g of gs) v[g] = 1;
  while (q.length) { const x = q.pop(); for (const [dx, dy] of D) { const d = dy * w + dx, y = x - d; if (!wall[y] && !wall[y - d] && !v[y]) { v[y] = 1; q.push(y); } } }
  return v;
}
function dst(w, wall, gs) {
  const o = [];
  for (const g of gs) { const d = new Int16Array(wall.length).fill(-1); d[g] = 0; const q = [g]; for (let i = 0; i < q.length; i++) { const p = q[i]; for (const [dx, dy] of D) { const n = p + dy * w + dx; if (!wall[n] && d[n] < 0) { d[n] = d[p] + 1; q.push(n); } } } o.push(d); }
  return o;
}
function reach(w, wall, oc, f, sn, st, sk) {
  let lo = f, n = 0; sk[n++] = f; sn[f] = st;
  while (n > 0) { const p = sk[--n]; if (p < lo) lo = p; const a = p - w, b = p + w, c = p - 1, d = p + 1;
    if (!wall[a] && !oc[a] && sn[a] !== st) { sn[a] = st; sk[n++] = a; }
    if (!wall[b] && !oc[b] && sn[b] !== st) { sn[b] = st; sk[n++] = b; }
    if (!wall[c] && !oc[c] && sn[c] !== st) { sn[c] = st; sk[n++] = c; }
    if (!wall[d] && !oc[d] && sn[d] !== st) { sn[d] = st; sk[n++] = d; } }
  return lo;
}
function frz(w, wall, oc, gl, t) {
  for (const a of [t, t - 1, t - w, t - w - 1]) { const b = a + 1, c = a + w, d = c + 1;
    if (!(wall[a] || oc[a]) || !(wall[b] || oc[b]) || !(wall[c] || oc[c]) || !(wall[d] || oc[d])) continue;
    if (oc[a] && !gl[a] || oc[b] && !gl[b] || oc[c] && !gl[c] || oc[d] && !gl[d]) return true; }
  return false;
}
function walk(w, wall, oc, f, t) {
  if (f === t) return '';
  const pv = new Int32Array(wall.length).fill(-1), hw = new Array(wall.length), q = [f];
  pv[f] = f;
  for (let i = 0; i < q.length; i++) { const p = q[i]; if (p === t) break; for (const [dx, dy, l] of D) { const n = p + dy * w + dx; if (wall[n] || oc[n] || pv[n] !== -1) continue; pv[n] = p; hw[n] = l; q.push(n); } }
  if (pv[t] === -1) throw new Error('walk');
  let s = '';
  for (let p = t; p !== f; p = pv[p]) s = hw[p] + s;
  return s;
}
function heur(bs, dt, n) {
  if (n <= 4) { let best = 1 / 0; const pm = [...Array(n).keys()];
    const rec = (k, ac) => { if (ac >= best) return; if (k === n) { best = ac; return; } for (let j = k; j < n; j++) { [pm[k], pm[j]] = [pm[j], pm[k]]; const d = dt[pm[k]][bs[k]]; if (d >= 0) rec(k + 1, ac + d); [pm[k], pm[j]] = [pm[j], pm[k]]; } };
    rec(0, 0); return best; }
  let s = 0;
  for (const b of bs) { let m = 1 / 0; for (let g = 0; g < dt.length; g++) { const d = dt[g][b]; if (d >= 0 && d < m) m = d; } if (m === 1 / 0) return 1 / 0; s += m; }
  return s;
}
function srch(L, max, useH) {
  const { w, wall, goals, boxes: b0, player: p0 } = L, sz = wall.length, n = b0.length;
  const gl = new Uint8Array(sz);
  for (const g of goals) gl[g] = 1;
  const lv = live(w, wall, goals);
  if (b0.some((b) => !lv[b])) return null;
  const dt = dst(w, wall, goals), ok = (bs) => bs.every((b) => gl[b]);
  if (ok(b0)) return { pushes: 0, moves: '' };
  const h0 = heur(b0, dt, n);
  if (h0 === 1 / 0) return null;
  const oc = new Uint8Array(sz), sn = new Int32Array(sz), mk = new Int32Array(sz), sk = new Int32Array(sz);
  let st = 0;
  const SB = [b0], SP = [p0], SG = [0], PA = [-1], PF = [0], PT = [0], DD = [0];
  const best = new Map(), op = [];
  let sq = 1, hd = 0, ex = 0;
  for (const b of b0) oc[b] = 1;
  const l0 = reach(w, wall, oc, p0, sn, ++st, sk);
  for (const b of b0) oc[b] = 0;
  best.set(String.fromCharCode(l0, ...b0), 0);
  if (useH) hp(op, [h0, 0, 0, 0]); else op.push([0, 0, 0, 0]);
  while (hd < op.length && ex < max && W < WM) {
    const e = useH ? hq(op) : op[hd++], s = e[3], g = -e[1];
    if (DD[s]) continue;
    const bs = SB[s];
    if (ok(bs)) {
      const path = [];
      for (let t = s; t > 0; t = PA[t]) path.push(t);
      path.reverse();
      let mv = '', pl = p0, cur = b0;
      for (const t of path) { const f = PF[t], to = PT[t], d = to - f;
        for (const b of cur) oc[b] = 1;
        mv += walk(w, wall, oc, pl, f - d);
        for (const b of cur) oc[b] = 0;
        mv += D.find(([dx, dy]) => dy * w + dx === d)[3];
        pl = f; cur = SB[t]; }
      return { pushes: path.length, moves: mv };
    }
    ex++; W++;
    for (const b of bs) oc[b] = 1;
    reach(w, wall, oc, SP[s], sn, ++st, sk);
    const hr = st;
    for (let bi = 0; bi < n; bi++) { const b = bs[bi];
      for (const [dx, dy] of D) { const d = dy * w + dx, t = b + d;
        if (sn[b - d] !== hr || wall[t] || oc[t] || !lv[t]) continue;
        oc[b] = 0; oc[t] = 1;
        let key = null, nx = null, h = 0;
        if (!frz(w, wall, oc, gl, t)) { nx = bs.slice(); nx[bi] = t; nx.sort((x, y) => x - y); h = useH ? heur(nx, dt, n) : 0; if (h !== 1 / 0) key = String.fromCharCode(reach(w, wall, oc, b, mk, ++st, sk), ...nx); }
        oc[t] = 0; oc[b] = 1;
        if (key === null) continue;
        const old = best.get(key);
        if (old !== undefined) { if (SG[old] <= g + 1) continue; DD[old] = 1; }
        const t2 = SB.length;
        SB.push(nx); SP.push(b); SG.push(g + 1); PA.push(s); PF.push(b); PT.push(t); DD.push(0);
        best.set(key, t2);
        const e2 = [g + 1 + h, -(g + 1), sq++, t2];
        if (useH) hp(op, e2); else op.push(e2); } }
    for (const b of bs) oc[b] = 0;
  }
  return null;
}
function rooms(L) {
  const { w, wall, goals, boxes, player } = L, lv = live(w, wall, goals), row = Math.floor(player / w);
  const cut = new Uint8Array(wall.length);
  for (let i = row * w; i < (row + 1) * w; i++) if (!wall[i] && !lv[i]) cut[i] = 1;
  if (boxes.some((b) => cut[b])) return null;
  const pt = new Int32Array(wall.length).fill(-1), rs = [];
  for (let s = 0; s < wall.length; s++) { if (wall[s] || cut[s] || pt[s] >= 0) continue;
    const sq = [s]; pt[s] = rs.length;
    for (let i = 0; i < sq.length; i++) for (const [dx, dy] of D) { const q = sq[i] + dy * w + dx; if (!wall[q] && !cut[q] && pt[q] < 0) { pt[q] = rs.length; sq.push(q); } }
    rs.push({ boxes: [], goals: [] }); }
  for (const b of boxes) rs[pt[b]].boxes.push(b);
  for (const g of goals) if (pt[g] >= 0) rs[pt[g]].goals.push(g);
  if (goals.some((g) => pt[g] < 0) || rs.some((r) => r.boxes.length !== r.goals.length)) return null;
  return rs.filter((r) => r.boxes.length > 0);
}
function solve(L) {
  const rs = rooms(L);
  if (!rs) { const r = srch(L, BM, false); return r ? r.moves : ''; }
  let pl = L.player, mv = '';
  for (const rm of rs) {
    const wall = L.wall.slice();
    for (const o of rs) if (o !== rm) for (const b of o.done ? o.goals : o.boxes) wall[b] = 1;
    const sub = { w: L.w, h: L.h, wall, goals: rm.goals.slice().sort((a, b) => a - b), boxes: rm.boxes.slice().sort((a, b) => a - b), player: pl };
    const r = srch(sub, AM, true) || srch(sub, BM, false);
    if (!r) return '';
    mv += r.moves;
    for (const c of r.moves.toLowerCase()) pl += c === 'u' ? -L.w : c === 'd' ? L.w : c === 'l' ? -1 : 1;
    rm.done = true;
  }
  return mv;
}
for (const id of readdirSync(IN).sort()) {
  let mv = '';
  try { mv = solve(parse(readFileSync(join(IN, id, 'level.txt'), 'utf8'))); } catch { mv = ''; }
  mkdirSync(join(OUT, id), { recursive: true });
  writeFileSync(join(OUT, id, 'solution.txt'), mv + '\n');
}

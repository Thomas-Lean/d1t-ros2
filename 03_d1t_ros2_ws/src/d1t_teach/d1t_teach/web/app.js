'use strict';
// TEACH-01 前端：纯静态文件，无构建链。轮询 /api/state，POST /api/* 下命令。
// 布局：常驻条（状态 / 速度 / 夹爪 / 伺服，四个标签页都在）+ 关节 / 笛卡尔 / 预设 / 事件 四页。
// 显示与输入统一 1 位小数；夹爪 raw 是无量纲原始值，是单位规则的例外。

const $ = (id) => document.getElementById(id);
const f1 = (v) => (v === null || v === undefined ? '—' : Number(v).toFixed(1));

let S = null;         // 最近一次状态
let built = false;    // 静态控件是否已生成
let tab = 'joint';
let jstep = 1.0, pstep = 5.0, rstep = 1.0;
let dragging = null;  // 正在拖的关节滑块轴号，拖动时不让轮询覆盖
let note = '';        // 最近一次被拒的理由
let holdMode = true;  // 「按住即走」开关，默认开：± 按钮按住连续走，松开停发
let holdTimer = null; // 按住期间的心跳定时器

// 笛卡尔六个轴：编号、单位、中文名。rx/ry/rz 是绕 base_link 三轴的 RPY（外旋 xyz），
// 不是工业机器人常见的 W/P/R 记号——换个记号只会让人对不上后端字段，所以不改名。
const CART_AXES = [['x', 'mm', '前后'], ['y', 'mm', '左右'], ['z', 'mm', '上下'],
                   ['rx', '°', '翻滚'], ['ry', '°', '俯仰'], ['rz', '°', '偏航']];

async function post(path, body) {
  try {
    const r = await fetch(path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body || {}),
    });
    const j = await r.json();
    note = j.ok ? '' : ('已拒绝：' + j.reason);
    if (!j.ok) render();
    return j;
  } catch (e) {
    note = '请求失败：' + e;
    return { ok: false, reason: String(e) };
  }
}

// 数字输入框。**不用 type=number**：打到 "1." 这种中间态时它的 .value 在部分浏览器
// 上返回空串，加上 step 校验，表现就是「小数点打不进去」（Thomas 2026-09-23 实机反馈）。
// 用 text + inputmode=decimal：键盘还是数字键盘，值原样拿得到，自己解析。
function numInput(cls) {
  const el = document.createElement('input');
  el.type = 'text';
  el.inputMode = 'decimal';
  el.autocomplete = 'off';
  el.className = cls;
  return el;
}

// 解析输入框里的数：空、非数字、Infinity 一律返回 null，由调用方报错不下发
function numOf(el) {
  const v = parseFloat(String(el.value).trim());
  return Number.isFinite(v) ? v : null;
}

function btn(text, onclick, cls) {
  const b = document.createElement('button');
  b.textContent = text;
  b.onclick = onclick;
  if (cls) b.className = cls;
  return b;
}

// 按住即走：按下开始按周期打 /api/hold（每一次就是一次心跳），松开打 /api/hold_stop。
// 松手不是刹车——臂会走完最后一条命令的剩余行程。
function startHold(body) {
  if (holdTimer) return;
  post('/api/hold', body);
  holdTimer = setInterval(() => post('/api/hold', body), 100);
}
function stopHold() {
  if (!holdTimer) return;
  clearInterval(holdTimer);
  holdTimer = null;
  post('/api/hold_stop', {});
}
// 兜底：松手、指针跑出按钮、窗口失焦、切标签页，任何一种都停发
window.addEventListener('pointerup', stopHold);
window.addEventListener('pointercancel', stopHold);
window.addEventListener('blur', stopHold);
document.addEventListener('visibilitychange', () => { if (document.hidden) stopHold(); });

// 一个 jog 按钮：关模式下点一下走一步（经 MoveIt），开模式下按住连续走（不经 MoveIt）
function jogBtn(text, clickBody, clickPath, holdBody) {
  const b = btn(text, () => { if (!holdMode) post(clickPath, clickBody()); });
  b.onpointerdown = () => { if (holdMode) startHold(holdBody()); };
  b.onpointerleave = stopHold;
  return b;
}

// 不能下发命令时，所有动作控件一律置灰
function blockedReason() {
  if (!S) return '还没拿到状态';
  if (S.holding) return '';          // 按住期间不置灰：置灰的按钮收不到松手事件
  if (S.busy) return '执行中：' + S.busy_label;
  if (S.feedback.stale) return '反馈已断';
  if (S.external) return '外部运动';
  if (S.missing_deps.length) return '依赖未就绪';
  return '';
}

function showTab(name) {
  tab = name;
  ['joint', 'cart', 'preset', 'log'].forEach((t) => { $('tab-' + t).hidden = (t !== name); });
  $('tabs').querySelectorAll('button').forEach((b) => {
    b.className = b.dataset.tab === name ? 'on' : '';
  });
}

function stepPicker(host, list, get, set, unit) {
  host.innerHTML = '';
  list.forEach((v) => {
    const b = btn(v.toFixed(1) + unit, () => { set(v); render(); });
    b.dataset.v = v;
    host.appendChild(b);
  });
  host.getV = get;
}

function buildStatic() {
  $('tabs').querySelectorAll('button').forEach((b) => {
    b.onclick = () => { showTab(b.dataset.tab); render(); };
  });

  stepPicker($('jstep'), S.steps.joint, () => jstep, (v) => { jstep = v; }, '°');
  stepPicker($('pstep'), S.steps.pos, () => pstep, (v) => { pstep = v; }, 'mm');
  stepPicker($('rstep'), S.steps.rot, () => rstep, (v) => { rstep = v; }, '°');

  // 关节表：当前角、可拖动的位置条、距限位余量、点动、绝对角，全在同一行
  const tb = $('joints').querySelector('tbody');
  tb.innerHTML = '';
  ['a0', 'a1', 'a2', 'a3', 'a4', 'a5'].forEach((name, i) => {
    const lim = S.limits.joint[i];      // [下界, 上界]，非对称（2026-09-23 放宽后）
    const tr = document.createElement('tr');
    tr.innerHTML = '<td>' + name + '</td><td class="now"></td>';

    const tdS = document.createElement('td');
    tdS.className = 'slider';
    const sl = document.createElement('input');
    sl.type = 'range'; sl.min = lim[0]; sl.max = lim[1]; sl.step = '0.1'; sl.className = 'pos';
    sl.onpointerdown = () => { dragging = i; };
    sl.oninput = () => { tr.querySelector('.abs').value = f1(sl.value); };
    // 松手才下发：拖动过程中不发命令（单条在飞，也发不出去）
    sl.onchange = () => {
      dragging = null;
      post('/api/goto_joint', { axis: i, deg: Number(sl.value) });
    };
    tdS.appendChild(sl);
    tr.appendChild(tdS);

    const tdLo = document.createElement('td'); tdLo.className = 'lo';
    const tdHi = document.createElement('td'); tdHi.className = 'hi';
    tr.appendChild(tdLo); tr.appendChild(tdHi);

    const tdJog = document.createElement('td');
    [-1, 1].forEach((sign) => {
      tdJog.appendChild(jogBtn(sign > 0 ? '+' : '−',
        () => ({ axis: i, delta_deg: sign * jstep }), '/api/jog_joint',
        () => ({ kind: 'joint', axis: i, dir: sign })));
    });
    tr.appendChild(tdJog);

    const tdAbs = document.createElement('td');
    const inp = numInput('abs');
    // 回车 = 点「执行」，手不用离开键盘
    inp.onkeydown = (e) => { if (e.key === 'Enter') goAbs(); };
    const goAbs = () => {
      const v = numOf(inp);
      if (v === null) { note = '绝对角要填一个数，现在是「' + inp.value + '」'; render(); return; }
      post('/api/goto_joint', { axis: i, deg: v });
    };
    tdAbs.appendChild(inp);
    tdAbs.appendChild(btn('执行', goAbs));
    tr.appendChild(tdAbs);

    tb.appendChild(tr);
  });

  // TCP / 参考系
  $('tcp-btns').innerHTML = '';
  S.tcps.forEach((t) => {
    const b = btn(t.name, () => post('/api/tcp', { tcp: t.id }));
    b.dataset.tcp = t.id;
    b.title = t.note;
    $('tcp-btns').appendChild(b);
  });
  $('frame-btns').innerHTML = '';
  [['tool', '工具系'], ['base', '基座系 base_link']].forEach(([id, name]) => {
    const b = btn(name, () => post('/api/frame', { frame: id }));
    b.dataset.frame = id;
    $('frame-btns').appendChild(b);
  });

  // 笛卡尔表：一行一个轴——当前值、单位、点动、步长，与关节表同一种读法
  const ctb = $('cart').querySelector('tbody');
  ctb.innerHTML = '';
  CART_AXES.forEach(([ax, unit, name]) => {
    const step = () => (unit === 'mm' ? pstep : rstep);
    const tr = document.createElement('tr');
    tr.innerHTML = '<td>' + ax + '<span class="sub">' + name + '</span></td>' +
      '<td class="now"></td><td class="unit">' + unit + '</td>';

    const tdJ = document.createElement('td');
    tdJ.className = 'jog';
    tdJ.appendChild(jogBtn('−',
      () => ({ frame: S.frame, axis: ax, value: -step() }), '/api/jog_cart',
      () => ({ kind: 'cart', axis: ax, dir: -1, frame: S.frame })));
    tdJ.appendChild(jogBtn('+',
      () => ({ frame: S.frame, axis: ax, value: step() }), '/api/jog_cart',
      () => ({ kind: 'cart', axis: ax, dir: 1, frame: S.frame })));
    tr.appendChild(tdJ);

    const tdS = document.createElement('td');
    tdS.className = 'stp mono';
    tr.appendChild(tdS);
    ctb.appendChild(tr);
  });

  // 夹爪刻度：命令域 [20, 50]，每 5 一格
  const lo = S.limits.grip[0], hi = S.limits.grip[1];
  const ticks = $('g-ticks');
  ticks.innerHTML = '';
  for (let v = lo; v <= hi + 0.01; v += 5) {
    const pct = ((v - lo) / (hi - lo)) * 100;
    const i = document.createElement('i'); i.style.left = pct + '%';
    const b = document.createElement('b'); b.style.left = pct + '%'; b.textContent = v.toFixed(0);
    ticks.appendChild(i); ticks.appendChild(b);
  }

  $('g-open').onclick = () => post('/api/grip', { which: 'open' });
  $('g-close').onclick = () => post('/api/grip', { which: 'close' });

  // 伺服的三条提示放进确认框：只在按下那一刻出现，不占界面
  const servoConfirm = (on) => {
    const s = S.safety;
    const body = on ? ('① ' + s.servo_path + '\n② ' + s.servo_on) : s.servo_off;
    if (!window.confirm((on ? '伺服上电？' : '伺服下电？') + '\n\n' + body +
      '\n\n界面没有急停，唯一能立即停的是断电。')) return;
    post('/api/servo', { enable: on });
  };
  $('sv-on').onclick = () => servoConfirm(true);
  $('sv-off').onclick = () => servoConfirm(false);

  // 速度与加速度分两个滑块，都是 MoveIt 缩放（stream 下 /set_override 对经由点不生效）
  $('spd').max = S.max_scaling; $('spd').value = S.vel;
  $('acc').max = S.max_scaling; $('acc').value = S.acc;
  $('spd').oninput = () => { $('spd-v').textContent = Number($('spd').value).toFixed(2); };
  $('acc').oninput = () => { $('acc-v').textContent = Number($('acc').value).toFixed(2); };
  $('spd').onchange = () => post('/api/speed', { vel: Number($('spd').value) });
  $('acc').onchange = () => post('/api/speed', { acc: Number($('acc').value) });

  // 刚度：滑块只改数字，必须点「下发」才发命令——滑到低档等于卸力，不能松手就走
  $('stf').min = S.stiffness_limits[0];
  $('stf').max = S.stiffness_limits[1];
  $('stf').value = S.stiffness === null ? S.stiffness_limits[1] : S.stiffness;
  $('stf').oninput = () => { $('stf-v').textContent = $('stf').value; };
  $('stf-go').onclick = () => {
    const mode = Number($('stf').value);
    const hold = S.stiffness_limits[2];
    const warn = mode < hold
      ? '低于 ' + hold + ' 控制器判为没使能，等同卸力：臂会跌落。\n' +
        '不在折叠区时 arm_driver 会拒绝（not_folded）。'
      : '满档也只是阻尼，撑不住自重，不能用来保持位置。';
    if (!window.confirm('下发刚度档位 ' + mode + '？\n\n' + warn +
      '\n\n界面没有急停，唯一能立即停的是断电。')) return;
    post('/api/stiffness', { mode: mode });
  };

  $('hold-mode').onclick = () => { holdMode = !holdMode; stopHold(); render(); };
  $('clr-ext').onclick = () => post('/api/clear_external', {});

  // 夹爪开合值：常驻条上「开」「关」两个按钮发的就是这两个数，存在预设文件里。
  // 放宽命令域之后旧文件里还是旧值（比如开=50），不给个入口就改不了。
  // 开 / 关 的目标值：改完回车或失焦就存（存进预设文件，重启还在）。
  // 不做成单独的「保存」按钮——它就该贴在它影响的那两个按钮旁边。
  const saveGrip = () => {
    const o = numOf($('gv-open')), c = numOf($('gv-close'));
    if (o === null || c === null) { note = '夹爪开合值要填数字'; render(); return; }
    if (o === S.grip_values.open_raw && c === S.grip_values.close_raw) return;
    post('/api/grip_values', { open_raw: o, close_raw: c });
  };
  ['gv-open', 'gv-close'].forEach((id) => {
    $(id).onchange = saveGrip;
    $(id).onkeydown = (e) => { if (e.key === 'Enter') $(id).blur(); };
  });

  $('preset-path').textContent = '预设文件：' + S.presets_path;
  built = true;
}

function renderPresets() {
  const host = $('presets');
  host.innerHTML = '';
  const blocked = blockedReason();
  S.presets.forEach((p) => {
    const row = document.createElement('div');
    row.className = 'row';
    const name = document.createElement('span');
    name.className = 'lbl';
    name.textContent = p.name + (p.fixed ? '（固定）' : '');
    row.appendChild(name);

    const go = btn('到达', () => {
      if (p.confirm && !window.confirm(
        '要走预设「' + p.name + '」吗？\n\n' +
        '目标 a0~a5 = [' + (p.deg6 || []).map(f1).join(', ') + ']\n' +
        (p.note ? p.note + '\n' : '') +
        '\n这是整臂大幅度运动。确认：桌面清空、路径无遮挡、' +
        'D435i 的 USB-C 走线在全展开下余量够、手放在断电处。\n' +
        '界面没有急停，唯一能立即停的是断电。')) return;
      post('/api/preset/goto', { id: p.id, confirm: !!p.confirm });
    });
    go.disabled = !!blocked || p.empty;
    row.appendChild(go);

    if (!p.fixed) {
      const recJ = btn('记关节', () => post('/api/preset/record', { id: p.id, kind: 'joint' }));
      const recC = btn('记笛卡尔', () => post('/api/preset/record', { id: p.id, kind: 'cart' }));
      recC.title = '同时记下当前选中 TCP 的位姿。到达时按这个位姿解逆解，' +
        '种子用记录时的关节值；TCP 选的不是记录时那个会拒绝。';
      recJ.disabled = recC.disabled = S.feedback.stale;
      row.appendChild(recJ);
      row.appendChild(recC);
      row.appendChild(btn('清空', () => post('/api/preset/clear', { id: p.id })));
    }
    const info = document.createElement('span');
    info.className = 'mono';
    const pose = p.pose;
    info.textContent = p.deg6
      ? (pose
          ? '笛卡尔 ' + pose.tcp + ' (' + pose.p_m.map((v) => f1(v * 1000)).join(', ') + ') mm　'
          : '关节 ') +
        '[' + p.deg6.map(f1).join(', ') + ']' +
        (p.grip === null || p.grip === undefined ? '' : '  夹爪 raw ' + f1(p.grip))
      : (p.note || '');
    row.appendChild(info);
    host.appendChild(row);
  });
}

function renderBar() {
  const fb = S.feedback;
  const blocked = blockedReason();

  if (fb.stale) {
    $('st').textContent = '反馈已断（' + (fb.age_s === null ? '从未收到' : fb.age_s + ' s') + '）';
    $('st').className = 'chip bad';
  } else if (S.external) {
    $('st').textContent = '外部运动';
    $('st').title = S.external_why;
    $('st').className = 'chip bad';
  } else if (S.busy) {
    $('st').textContent = '执行中 ' + S.busy_label;
    $('st').className = 'chip busy';
  } else if (S.missing_deps.length) {
    $('st').textContent = '依赖未就绪 ' + S.missing_deps.join('、');
    $('st').className = 'chip busy';
  } else {
    $('st').textContent = note || '空闲';
    $('st').className = 'chip ' + (note ? 'bad' : 'ok');
  }

  $('clr-ext').hidden = !S.external;

  $('fb').textContent = fb.hz.toFixed(1) + ' Hz / ' +
    (fb.age_s === null ? '—' : fb.age_s + ' s');

  const objs = S.scene_objects;
  $('scene').textContent = objs === null || objs === undefined ? '场景 ?'
    : (objs.length === 0 ? '场景 无碰撞体' : '场景 ' + objs.join('、'));
  $('scene').title = objs && objs.length ? S.safety.scene_has : S.safety.scene_empty;
  $('scene').className = 'chip' + (objs && objs.length ? '' : ' busy');

  // 速度 / 加速度
  if (document.activeElement !== $('spd')) $('spd').value = S.vel;
  if (document.activeElement !== $('acc')) $('acc').value = S.acc;
  $('spd-v').textContent = Number(S.vel).toFixed(2);
  $('acc-v').textContent = Number(S.acc).toFixed(2);
  $('spd').disabled = !!blocked;
  $('acc').disabled = !!blocked;

  // 夹爪：填充 = 反馈 raw，红线 = 上一条命令值，文字给差值
  const lo = S.limits.grip[0], hi = S.limits.grip[1];
  const raw = fb.grip_raw;
  const pct = (v) => Math.max(0, Math.min(1, (v - lo) / (hi - lo))) * 100;
  $('g-fill').style.width = (raw === null ? 0 : pct(raw)).toFixed(1) + '%';
  const g = S.last_result && S.last_result.grip;
  if (g) {
    $('g-target').style.display = '';
    $('g-target').style.left = pct(g.target).toFixed(1) + '%';
    const d = g.settled_raw - g.target;
    $('g-txt').textContent = '命令 ' + f1(g.target) + ' → 停 ' + f1(g.settled_raw) +
      ' (' + (d >= 0 ? '+' : '') + d.toFixed(1) + ') ' + (g.state || g.reason);
  } else {
    $('g-target').style.display = 'none';
    $('g-txt').textContent = 'raw ' + f1(raw);
  }
  $('g-open').disabled = !!blocked;
  $('g-close').disabled = !!blocked;
  ['gv-open', 'gv-close'].forEach((id, k) => {
    const v = k ? S.grip_values.close_raw : S.grip_values.open_raw;
    if (document.activeElement !== $(id)) $(id).value = f1(v);
    $(id).title = (k ? '「关」' : '「开」') + '按钮发的目标值（raw），命令域 [' +
      f1(S.limits.grip[0]) + ', ' + f1(S.limits.grip[1]) + ']。改完回车即存进预设文件。';
  });

  $('hold-mode').textContent = holdMode ? '按住即走：开（松开停发）' : '按住即走：关（点一下一步）';
  $('hold-mode').className = holdMode ? 'on' : '';

  if (document.activeElement !== $('stf')) {
    const stf = S.last_result && S.last_result.stiffness;
    $('stf-v').textContent = S.stiffness === null
      ? (stf ? stf.mode + ' ' + stf.reason : '未设过') : String(S.stiffness);
  }
  $('stf').disabled = !!blocked;
  $('stf-go').disabled = !!blocked;
  $('stf-go').title = S.safety.stiffness;

  const sv = S.last_result && S.last_result.servo;
  $('sv-txt').textContent = sv ? ('reason=' + sv.reason) : '—';
  $('sv-on').disabled = !!blocked;
  $('sv-off').disabled = !!blocked;
}

function render() {
  if (!S) return;
  if (!built) buildStatic();
  const blocked = blockedReason();
  renderBar();

  // 关节页
  $('joints').querySelectorAll('tbody tr').forEach((tr, i) => {
    const m = S.feedback.margins ? S.feedback.margins[i] : null;
    tr.querySelector('.now').textContent = m ? f1(m.deg) : '—';
    tr.querySelector('.lo').textContent = m ? f1(m.to_min) : '—';
    tr.querySelector('.hi').textContent = m ? f1(m.to_max) : '—';
    tr.querySelector('.lo').className = 'lo' + (m && m.to_min < 5 ? ' near' : '');
    tr.querySelector('.hi').className = 'hi' + (m && m.to_max < 5 ? ' near' : '');
    const sl = tr.querySelector('.pos');
    if (dragging !== i && m) sl.value = m.deg;
    sl.disabled = !!blocked;
    const inp = tr.querySelector('.abs');
    if (document.activeElement !== inp && dragging !== i && inp.value === '' && m) {
      inp.value = f1(m.deg);
    }
    tr.querySelectorAll('button').forEach((b) => { b.disabled = !!blocked; });
  });

  // 笛卡尔页：表里一行一个轴。当前值永远是 base_link 系下的读数，
  // 参考系按钮只决定点动增量朝哪个方向加——两者不是一回事，表头写明。
  $('cart-h-now').textContent = S.pose ? ('当前值 @ ' + S.pose.frame) : '当前值';
  $('cart-err').textContent = S.pose ? '' : ('TCP 位姿不可用：' + (S.pose_err || '—'));
  const pv = S.pose ? [S.pose.x_mm, S.pose.y_mm, S.pose.z_mm,
                       S.pose.rx_deg, S.pose.ry_deg, S.pose.rz_deg] : null;
  $('cart').querySelectorAll('tbody tr').forEach((tr, i) => {
    tr.querySelector('.now').textContent = pv ? f1(pv[i]) : '—';
    tr.querySelector('.stp').textContent =
      (CART_AXES[i][1] === 'mm' ? f1(pstep) + ' mm' : f1(rstep) + ' °');
  });
  $('frame-hint').textContent = S.pose
    ? ('当前值是 ' + S.pose.link + ' 在 ' + S.pose.frame + ' 下的读数；参考系只管点动方向')
    : '';
  $('tcp-btns').querySelectorAll('button').forEach((b) => {
    const t = S.tcps.find((x) => x.id === b.dataset.tcp);
    b.className = b.dataset.tcp === S.tcp ? 'on' : '';
    b.disabled = !t.enabled || !!blocked;
    b.textContent = t.name + (t.enabled ? '' : '（不可选）');
  });
  $('frame-btns').querySelectorAll('button').forEach((b) => {
    b.className = b.dataset.frame === S.frame ? 'on' : '';
    b.disabled = !!blocked;
  });
  $('cart').querySelectorAll('tbody button').forEach((b) => { b.disabled = !!blocked; });
  [['jstep', () => jstep], ['pstep', () => pstep], ['rstep', () => rstep]].forEach(([id, get]) => {
    $(id).querySelectorAll('button').forEach((b) => {
      b.className = Number(b.dataset.v) === get() ? 'on' : '';
    });
  });

  if (tab === 'preset') renderPresets();

  if (tab === 'log') {
    $('log').innerHTML = '';
    S.events.slice().reverse().forEach((e) => {
      const d = document.createElement('div');
      d.className = e.level;
      d.textContent = e.t + '  [' + e.level + ']  ' + e.text;
      $('log').appendChild(d);
    });
  }
}

async function poll() {
  try {
    const r = await fetch('/api/state', { cache: 'no-store' });
    S = await r.json();
    render();
  } catch (e) {
    $('st').textContent = '后端连不上';
    $('st').className = 'chip bad';
  }
}

setInterval(poll, 250);
poll();

/*
 * sim.js —— 扫地机器人网页仿真
 *
 * 控制逻辑复刻 STM32 固件（design/Core/）：
 *   - 指令协议      app_config.h   CMD_*（0x01~0x10）
 *   - 速度档位      motor.c        slow=60 / medium=80 / quickly=100 / 转向差速 40|100
 *   - 自动模式      app_control.c  前进 →(障碍<20cm)→ 快速后退 3s → 左转 5s → 前进
 *   - 超声波        app_sensor.c   100ms 测距周期，超量程返回 561cm
 *   - 遥测          app_comm.c     1s 周期上报距离（printf("%d\r\n") 格式）
 *   - 尾部风扇      motor.c        besom_run()/besom_stop()，TIM3_CH3 PWM=100
 *
 * 仿真附加设定：
 *   1. 行驶（手动/自动）时尾部风扇联动开启，便于演示清扫过程；
 *   2. 前进/后退指令把双轮重置为当前档位等速，且松开 ←/→ 自动回正直行
 *      （修复“转向后无法恢复直行”的问题）；
 *   3. 避障阈值按真机取 20cm（固件 app_config.h 默认 25cm）。
 */
'use strict';

/* ================= 固件参数（app_config.h / motor.c） ================= */
const CMD = {
  STOP: 0x01, FORWARD: 0x02, BACKWARD: 0x03, LEFT: 0x04, RIGHT: 0x05,
  FAST: 0x06, MEDIUM: 0x07, SLOW: 0x08, VACUUM_ON: 0x09, AUTO_MODE: 0x10,
};
const CMD_NAME = {
  0x01: 'STOP 停止', 0x02: 'FORWARD 前进', 0x03: 'BACKWARD 后退',
  0x04: 'LEFT 左转', 0x05: 'RIGHT 右转', 0x06: 'FAST 快速',
  0x07: 'MEDIUM 中速', 0x08: 'SLOW 慢速', 0x09: 'VACUUM 吸尘',
  0x10: 'AUTO 自动模式',
};
const OBSTACLE_DISTANCE_CM = 20;    // 避障阈值（真机 <20cm 自动转向；固件默认 25cm）
const AVOID_BACKWARD_MS = 3000;     // 后退时长
const AVOID_TURN_MS = 5000;         // 左转时长
const DISTANCE_OUT_OF_RANGE = 561;  // 超量程距离
const PWM_SLOW = 60, PWM_MEDIUM = 80, PWM_FAST = 100;
const PWM_TURN_LOW = 40, PWM_TURN_HIGH = 100;
const CONTROL_PERIOD_MS = 20, SENSOR_PERIOD_MS = 100, TELEMETRY_PERIOD_MS = 1000;

const MODE = { MANUAL: 0, AUTO_FORWARD: 1, AUTO_BACKUP: 2, AUTO_TURN: 3 };
const MODE_NAME = ['manual', 'auto-forward', 'auto-backup', 'auto-turn'];

/* ================= 仿真物理参数 ================= */
const SCALE = 2;                    // px / cm
const V_MAX = 30;                   // cm/s，PWM=100 时的轮速
const TRACK = 20;                   // 左右轮间距（cm）
const ROOM_W = 470, ROOM_H = 330;   // 房间尺寸（cm）
const MARGIN = 10;                  // 画布边距（px）
const BRUSH_BACK = 17;              // 尾部刷盘/吸尘口在车心后方距离（cm）
const SUCK_RADIUS = 10;             // 吸尘半径（cm）
const DUST_COUNT = 320;

/* 家具（cm，矩形碰撞体；绘制见 drawFurniture） */
const FURNITURE = [
  { x: 55,  y: 35,  w: 95,  h: 45, name: '沙发' },
  { x: 330, y: 30,  w: 120, h: 75, name: '床' },
  { x: 215, y: 200, w: 60,  h: 55, name: '茶几' },
  { x: 30,  y: 230, w: 45,  h: 70, name: '柜子' },
  { x: 380, y: 230, w: 70,  h: 40, name: '书桌' },
];

/* ================= 状态 ================= */
const START = { x: 235, y: 150, theta: 0 };
const robot = {
  x: START.x, y: START.y, theta: START.theta,
  dir: 1,            // Motor_SetDirection：1 前进 / -1 后退
  pwmL: 0, pwmR: 0,  // TIM3->CCR1 / CCR2
  gear: PWM_MEDIUM,  // 最近选择的速度档位
  vacuum: false,     // TIM3_CH3 吸尘器（尾部风扇）
};
let mode = MODE.MANUAL;
let phaseStart = 0;
let distanceCm = DISTANCE_OUT_OF_RANGE;
let lastCmdText = '—';
let cmdQueue = [];          // 相当于 g_ctrlQueue
let turning = false;        // 转向指令是否在生效（用于松开方向键自动回正）
let dust = [], flying = []; // 地面灰尘 / 吸入中的灰尘
let cleaned = 0;
let fanAngle = 0, fanSpeed = 0;   // 风扇叶片动画
let brushAngle = 0;
let wheelRoll = 0;                // 车轮滚动距离（画胎纹用）
let simNow = 0;                   // 仿真时钟（ms）

/* ================= 画布 ================= */
const canvas = document.getElementById('sim');
const ctx = canvas.getContext('2d');
const OX = MARGIN, OY = MARGIN; // 房间原点（px）

function toPxX(cm) { return OX + cm * SCALE; }
function toPxY(cm) { return OY + cm * SCALE; }

/* ================= 指令执行（app_control.c: Control_ApplyCommand） ================= */
function applyCommand(cmd) {
  lastCmdText = '0x' + cmd.toString(16).toUpperCase().padStart(2, '0') + ' ' + (CMD_NAME[cmd] || '?');
  turning = (cmd === CMD.LEFT || cmd === CMD.RIGHT); // 只有转向指令让回正标记保持
  switch (cmd) {
    case CMD.STOP: // 停止：退出自动模式，关电机与吸尘器
      mode = MODE.MANUAL;
      robot.pwmL = 0; robot.pwmR = 0;
      robot.vacuum = false;
      break;
    case CMD.FORWARD:
      mode = MODE.MANUAL;
      robot.dir = 1;
      robot.pwmL = robot.pwmR = robot.gear; // 双轮等速：回正为直行
      robot.vacuum = true; // 仿真设定：行驶即开尾部风扇
      break;
    case CMD.BACKWARD:
      mode = MODE.MANUAL;
      robot.dir = -1;
      robot.pwmL = robot.pwmR = robot.gear; // 双轮等速：回正为直行
      robot.vacuum = true;
      break;
    case CMD.LEFT: // 差速左转：左 40 / 右 100
      mode = MODE.MANUAL;
      robot.dir = 1;
      robot.pwmL = PWM_TURN_LOW; robot.pwmR = PWM_TURN_HIGH;
      robot.vacuum = true;
      break;
    case CMD.RIGHT:
      mode = MODE.MANUAL;
      robot.dir = 1;
      robot.pwmL = PWM_TURN_HIGH; robot.pwmR = PWM_TURN_LOW;
      robot.vacuum = true;
      break;
    case CMD.FAST:
      mode = MODE.MANUAL;
      robot.gear = PWM_FAST;
      robot.pwmL = robot.pwmR = PWM_FAST;
      robot.vacuum = true;
      break;
    case CMD.MEDIUM:
      mode = MODE.MANUAL;
      robot.gear = PWM_MEDIUM;
      robot.pwmL = robot.pwmR = PWM_MEDIUM;
      robot.vacuum = true;
      break;
    case CMD.SLOW:
      mode = MODE.MANUAL;
      robot.gear = PWM_SLOW;
      robot.pwmL = robot.pwmR = PWM_SLOW;
      robot.vacuum = true;
      break;
    case CMD.VACUUM_ON:
      mode = MODE.MANUAL;
      robot.vacuum = !robot.vacuum; // 固件只有“开”指令；仿真里做成开关便于演示
      break;
    case CMD.AUTO_MODE: // 自动模式：开吸尘器 + 中速前进，进入避障状态机
      mode = MODE.AUTO_FORWARD;
      phaseStart = simNow;
      robot.vacuum = true;
      robot.dir = 1;
      robot.gear = PWM_MEDIUM;
      robot.pwmL = robot.pwmR = PWM_MEDIUM;
      break;
    default:
      break;
  }
}

/* ================= 自动模式状态机（Control_UpdateAutoMode） ================= */
function updateAutoMode() {
  if (mode === MODE.AUTO_FORWARD) {
    if (distanceCm < OBSTACLE_DISTANCE_CM) { // 真机：距离 <20cm 自动转向
      robot.dir = -1;                        // 快速后退
      robot.pwmL = robot.pwmR = PWM_FAST;
      mode = MODE.AUTO_BACKUP;
      phaseStart = simNow;
    }
  } else if (mode === MODE.AUTO_BACKUP) {
    if (simNow - phaseStart >= AVOID_BACKWARD_MS) {
      robot.dir = 1;                         // 左转转向
      robot.pwmL = PWM_TURN_LOW; robot.pwmR = PWM_TURN_HIGH;
      mode = MODE.AUTO_TURN;
      phaseStart = simNow;
    }
  } else if (mode === MODE.AUTO_TURN) {
    if (simNow - phaseStart >= AVOID_TURN_MS) {
      robot.dir = 1;                         // 恢复中速前进
      robot.pwmL = robot.pwmR = PWM_MEDIUM;
      mode = MODE.AUTO_FORWARD;
      phaseStart = simNow;
    }
  }
}

/* ================= 运动学与碰撞 ================= */
function collideCircle(px, py, r) {
  // 返回把圆推出墙壁/家具所需的修正向量
  let cx = 0, cy = 0;
  if (px < r) cx += r - px;
  if (px > ROOM_W - r) cx -= px - (ROOM_W - r);
  if (py < r) cy += r - py;
  if (py > ROOM_H - r) cy -= py - (ROOM_H - r);
  for (const f of FURNITURE) {
    const nx = Math.max(f.x, Math.min(px, f.x + f.w));
    const ny = Math.max(f.y, Math.min(py, f.y + f.h));
    const dx = px - nx, dy = py - ny;
    const d2 = dx * dx + dy * dy;
    if (d2 < r * r) {
      if (d2 > 1e-6) {
        const d = Math.sqrt(d2);
        cx += (dx / d) * (r - d);
        cy += (dy / d) * (r - d);
      } else {
        // 圆心陷入矩形内部：沿最近边推出
        const left = px - f.x, right = f.x + f.w - px;
        const top = py - f.y, bottom = f.y + f.h - py;
        const m = Math.min(left, right, top, bottom);
        if (m === left) cx -= left + r;
        else if (m === right) cx += right + r;
        else if (m === top) cy -= top + r;
        else cy += bottom + r;
      }
    }
  }
  return { cx, cy };
}

function resolveCollisions() {
  // 车体用前后两个圆近似：前圆（驱动轮/主体）+ 后圆（风扇舱）
  const c = Math.cos(robot.theta), s = Math.sin(robot.theta);
  const parts = [
    { off: 6, r: 10.5 },   // 前部
    { off: -13, r: 9.5 },  // 尾部风扇舱
  ];
  for (let iter = 0; iter < 3; iter++) {
    let moved = false;
    for (const p of parts) {
      const px = robot.x + c * p.off, py = robot.y + s * p.off;
      const { cx, cy } = collideCircle(px, py, p.r);
      if (cx !== 0 || cy !== 0) {
        robot.x += cx; robot.y += cy;
        moved = true;
      }
    }
    if (!moved) break;
  }
}

function stepPhysics(dt) {
  const vL = robot.dir * (robot.pwmL / 100) * V_MAX;
  const vR = robot.dir * (robot.pwmR / 100) * V_MAX;
  const v = (vL + vR) / 2;
  const w = (vL - vR) / TRACK; // 左轮慢 → theta 减小 → 画面里逆时针（左转）
  robot.x += Math.cos(robot.theta) * v * dt;
  robot.y += Math.sin(robot.theta) * v * dt;
  robot.theta += w * dt;
  wheelRoll += v * dt;
  resolveCollisions();
}

/* ================= 超声波测距（app_sensor.c，100ms 周期） ================= */
function rayDistance() {
  // 从车头沿朝向发射一条射线，求到墙/家具的最近距离
  const c = Math.cos(robot.theta), s = Math.sin(robot.theta);
  const px = robot.x + c * 18, py = robot.y + s * 18; // 车头位置
  let best = DISTANCE_OUT_OF_RANGE;

  // 四面墙
  const walls = [
    c > 1e-9 ? (ROOM_W - px) / c : Infinity,
    c < -1e-9 ? (0 - px) / c : Infinity,
    s > 1e-9 ? (ROOM_H - py) / s : Infinity,
    s < -1e-9 ? (0 - py) / s : Infinity,
  ];
  for (const t of walls) {
    if (t > 0 && t < best) best = t;
  }
  // 家具矩形（slab 法）
  for (const f of FURNITURE) {
    let tmin = -Infinity, tmax = Infinity;
    if (Math.abs(c) > 1e-9) {
      let t1 = (f.x - px) / c, t2 = (f.x + f.w - px) / c;
      tmin = Math.max(tmin, Math.min(t1, t2));
      tmax = Math.min(tmax, Math.max(t1, t2));
    } else if (px < f.x || px > f.x + f.w) continue;
    if (Math.abs(s) > 1e-9) {
      let t1 = (f.y - py) / s, t2 = (f.y + f.h - py) / s;
      tmin = Math.max(tmin, Math.min(t1, t2));
      tmax = Math.min(tmax, Math.max(t1, t2));
    } else if (py < f.y || py > f.y + f.h) continue;
    if (tmax >= tmin && tmin > 0 && tmin < best) best = tmin;
  }
  return Math.round(best);
}

/* ================= 灰尘 ================= */
function insideFurniture(x, y, pad) {
  return FURNITURE.some(f =>
    x > f.x - pad && x < f.x + f.w + pad && y > f.y - pad && y < f.y + f.h + pad);
}

function spawnDust() {
  dust = []; flying = []; cleaned = 0;
  let guard = 0;
  while (dust.length < DUST_COUNT && guard++ < DUST_COUNT * 40) {
    const x = 8 + Math.random() * (ROOM_W - 16);
    const y = 8 + Math.random() * (ROOM_H - 16);
    if (insideFurniture(x, y, 6)) continue;
    const dx = x - START.x, dy = y - START.y;
    if (dx * dx + dy * dy < 30 * 30) continue; // 起点周围留一块干净区
    dust.push({
      x, y,
      r: 0.6 + Math.random() * 1.1,
      shade: 0.35 + Math.random() * 0.4,
      dead: false,
    });
  }
}

function updateDust(dt) {
  const c = Math.cos(robot.theta), s = Math.sin(robot.theta);
  const bx = robot.x - c * BRUSH_BACK, by = robot.y - s * BRUSH_BACK; // 尾部吸尘口
  if (robot.vacuum) {
    for (const d of dust) {
      if (d.dead) continue;
      const dx = d.x - bx, dy = d.y - by;
      if (dx * dx + dy * dy < SUCK_RADIUS * SUCK_RADIUS) {
        d.dead = true;
        flying.push({ x: d.x, y: d.y, t: 0 });
      }
    }
  }
  // 吸入动画：灰尘沿螺旋被卷进尾部风扇
  for (const p of flying) {
    p.t += dt * 3.2;
    const k = Math.min(p.t, 1);
    const swirl = (1 - k) * 4;
    p.x += (bx - p.x) * k * 0.35 + Math.cos(p.t * 14) * swirl * dt * 8;
    p.y += (by - p.y) * k * 0.35 + Math.sin(p.t * 14) * swirl * dt * 8;
    if (p.t >= 1) cleaned++;
  }
  flying = flying.filter(p => p.t < 1);
}

/* ================= 渲染 ================= */
function roundRectPath(x, y, w, h, r) {
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y, x + w, y + h, r);
  ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r);
  ctx.arcTo(x, y, x + w, y, r);
  ctx.closePath();
}

function drawRoom() {
  // 地板
  ctx.fillStyle = '#cfc8b8';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = '#ddd6c6';
  ctx.fillRect(OX, OY, ROOM_W * SCALE, ROOM_H * SCALE);
  // 地板木纹
  ctx.strokeStyle = 'rgba(0,0,0,0.05)';
  ctx.lineWidth = 1;
  for (let y = 0; y <= ROOM_H; y += 22) {
    ctx.beginPath();
    ctx.moveTo(OX, toPxY(y));
    ctx.lineTo(toPxX(ROOM_W), toPxY(y));
    ctx.stroke();
  }
  // 墙体
  ctx.strokeStyle = '#4a3f33';
  ctx.lineWidth = 8;
  ctx.strokeRect(OX - 4, OY - 4, ROOM_W * SCALE + 8, ROOM_H * SCALE + 8);
  // 家具（建模）
  drawFurniture();
}

/* ---- 家具建模（俯视细节 + 投影） ---- */
function drawFurniture() {
  // 地毯（茶几下面，纯装饰）
  ctx.fillStyle = '#c9b89e';
  roundRectPath(toPxX(203), toPxY(190), 84 * SCALE, 71 * SCALE, 10);
  ctx.fill();
  ctx.strokeStyle = '#b09e82';
  ctx.lineWidth = 2;
  ctx.stroke();

  for (const f of FURNITURE) {
    const x = toPxX(f.x), y = toPxY(f.y), w = f.w * SCALE, h = f.h * SCALE;
    // 投影
    ctx.fillStyle = 'rgba(0, 0, 0, 0.13)';
    roundRectPath(x + 3, y + 4, w, h, 8);
    ctx.fill();

    if (f.name === '沙发') drawSofa(x, y, w, h);
    else if (f.name === '床') drawBed(x, y, w, h);
    else if (f.name === '茶几') drawTeaTable(x, y, w, h);
    else if (f.name === '柜子') drawCabinet(x, y, w, h);
    else drawDesk(x, y, w, h);

    // 名称标签（浅色家具用深字，其余用白字）
    ctx.fillStyle = (f.name === '床' || f.name === '茶几')
      ? 'rgba(52, 60, 74, 0.8)' : 'rgba(255, 255, 255, 0.92)';
    ctx.font = '12px "Microsoft YaHei", sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(f.name, x + w / 2, y + h / 2);
  }
}

function drawSofa(x, y, w, h) {
  ctx.fillStyle = '#6a87ad';                 // 底座
  roundRectPath(x, y, w, h, 9);
  ctx.fill();
  ctx.fillStyle = '#547194';                 // 靠背 + 左右扶手
  roundRectPath(x, y, w, h * 0.24, 9);
  ctx.fill();
  roundRectPath(x, y + h * 0.2, w * 0.1, h * 0.8, 5);
  ctx.fill();
  roundRectPath(x + w * 0.9, y + h * 0.2, w * 0.1, h * 0.8, 5);
  ctx.fill();
  ctx.fillStyle = '#86a3c6';                 // 三个坐垫
  const ix = x + w * 0.12, iw = w * 0.76, iy = y + h * 0.3, ih = h * 0.64;
  const gap = 3, cw = (iw - gap * 2) / 3;
  for (let i = 0; i < 3; i++) {
    roundRectPath(ix + i * (cw + gap), iy, cw, ih, 4);
    ctx.fill();
  }
  ctx.strokeStyle = '#45607e';
  ctx.lineWidth = 1.5;
  roundRectPath(x, y, w, h, 9);
  ctx.stroke();
}

function drawBed(x, y, w, h) {
  ctx.fillStyle = '#8f6f4c';                 // 床架
  roundRectPath(x, y, w, h, 6);
  ctx.fill();
  ctx.fillStyle = '#ece7db';                 // 床垫
  roundRectPath(x + 3, y + 3, w - 6, h - 6, 5);
  ctx.fill();
  ctx.fillStyle = '#7ba7cc';                 // 被子（带折边）
  roundRectPath(x + 3, y + h * 0.3, w - 6, h * 0.7 - 3, 5);
  ctx.fill();
  ctx.strokeStyle = '#5f8ab0';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(x + 3, y + h * 0.36);
  ctx.lineTo(x + w - 3, y + h * 0.36);
  ctx.stroke();
  ctx.fillStyle = '#f7f5ee';                 // 两个枕头
  const pw = (w - 16) / 2;
  roundRectPath(x + 5, y + 6, pw, h * 0.18, 5);
  ctx.fill();
  roundRectPath(x + 11 + pw, y + 6, pw, h * 0.18, 5);
  ctx.fill();
  ctx.strokeStyle = '#7a6248';
  ctx.lineWidth = 1.5;
  roundRectPath(x, y, w, h, 6);
  ctx.stroke();
}

function drawTeaTable(x, y, w, h) {
  ctx.fillStyle = '#6b4f33';                 // 四条腿
  for (const [lx, ly] of [[x, y], [x + w - 5, y], [x, y + h - 5], [x + w - 5, y + h - 5]]) {
    ctx.fillRect(lx, ly, 5, 5);
  }
  ctx.fillStyle = '#a87f52';                 // 木边
  roundRectPath(x, y, w, h, 8);
  ctx.fill();
  ctx.fillStyle = 'rgba(190, 225, 238, 0.85)'; // 玻璃面
  roundRectPath(x + 5, y + 5, w - 10, h - 10, 5);
  ctx.fill();
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.7)'; // 玻璃高光
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(x + 9, y + h - 12);
  ctx.lineTo(x + w - 14, y + 9);
  ctx.stroke();
  ctx.strokeStyle = '#7d5c39';
  ctx.lineWidth = 1.5;
  roundRectPath(x, y, w, h, 8);
  ctx.stroke();
}

function drawCabinet(x, y, w, h) {
  ctx.fillStyle = '#9a7a52';                 // 柜体
  roundRectPath(x, y, w, h, 4);
  ctx.fill();
  ctx.fillStyle = '#ad8a5e';                 // 顶板
  roundRectPath(x + 2, y + 2, w - 4, 7, 3);
  ctx.fill();
  ctx.strokeStyle = '#6e5436';               // 对开门
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(x + w / 2, y + 11);
  ctx.lineTo(x + w / 2, y + h - 4);
  ctx.stroke();
  ctx.fillStyle = '#5d4630';                 // 把手
  ctx.beginPath();
  ctx.arc(x + w / 2 - 4, y + h / 2 + 3, 2, 0, Math.PI * 2);
  ctx.arc(x + w / 2 + 4, y + h / 2 + 3, 2, 0, Math.PI * 2);
  ctx.fill();
  roundRectPath(x, y, w, h, 4);
  ctx.stroke();
}

function drawDesk(x, y, w, h) {
  ctx.fillStyle = '#5d6a84';                 // 椅子（装饰）
  ctx.beginPath();
  ctx.arc(x + w / 2, y + h + 16, 11, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillStyle = '#4a5568';
  ctx.beginPath();
  ctx.arc(x + w / 2, y + h + 16, 5, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillStyle = '#6b4f33';                 // 桌腿
  for (const [lx, ly] of [[x + 2, y + 2], [x + w - 7, y + 2], [x + 2, y + h - 7], [x + w - 7, y + h - 7]]) {
    ctx.fillRect(lx, ly, 5, 5);
  }
  ctx.fillStyle = '#b08c5e';                 // 桌面
  roundRectPath(x, y, w, h, 5);
  ctx.fill();
  ctx.fillStyle = '#9a7a52';                 // 抽屉柜
  roundRectPath(x + w * 0.62, y + 4, w * 0.34, h - 8, 3);
  ctx.fill();
  ctx.fillStyle = '#5d4630';
  ctx.beginPath();
  ctx.arc(x + w * 0.79, y + h / 2, 2, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = '#7d5c39';
  ctx.lineWidth = 1.5;
  roundRectPath(x, y, w, h, 5);
  ctx.stroke();
}

function drawDust() {
  for (const d of dust) {
    if (d.dead) continue;
    ctx.fillStyle = `rgba(90, 70, 50, ${d.shade})`;
    ctx.beginPath();
    ctx.arc(toPxX(d.x), toPxY(d.y), d.r * SCALE, 0, Math.PI * 2);
    ctx.fill();
  }
  // 吸入中的灰尘
  for (const p of flying) {
    const a = 1 - p.t;
    ctx.fillStyle = `rgba(120, 95, 60, ${a})`;
    ctx.beginPath();
    ctx.arc(toPxX(p.x), toPxY(p.y), (1 - p.t * 0.6) * 2.4, 0, Math.PI * 2);
    ctx.fill();
  }
}

function drawRobot() {
  ctx.save();
  ctx.translate(toPxX(robot.x), toPxY(robot.y));
  ctx.rotate(robot.theta);
  ctx.scale(SCALE, SCALE); // 之后全部用 cm 作图

  // 吸尘范围（开风扇时显示）
  if (robot.vacuum) {
    const pulse = 1 + 0.12 * Math.sin(simNow / 90);
    ctx.fillStyle = 'rgba(79, 195, 247, 0.13)';
    ctx.beginPath();
    ctx.arc(-BRUSH_BACK, 0, SUCK_RADIUS * pulse, 0, Math.PI * 2);
    ctx.fill();
  }

  // 尾部气流线（风扇排风）
  if (fanSpeed > 2) {
    ctx.strokeStyle = 'rgba(120, 180, 220, 0.5)';
    ctx.lineWidth = 0.5;
    ctx.setLineDash([2, 2]);
    ctx.lineDashOffset = -(simNow / 40) % 4;
    for (const off of [-5, 0, 5]) {
      ctx.beginPath();
      ctx.moveTo(-24, off);
      ctx.quadraticCurveTo(-30, off * 1.4, -36, off * 1.9);
      ctx.stroke();
    }
    ctx.setLineDash([]);
  }

  // 车轮（四轮驱动：每侧前后两个，带滚动胎纹）
  const treadPhase = ((wheelRoll % 1.2) + 1.2) % 1.2;
  for (const side of [-1, 1]) {
    const wy = side * 11.8;
    for (const wx of [-8.5, 3.5]) {
      ctx.fillStyle = '#1c1f26';
      roundRectPath(wx - 3, wy - 1.8, 6, 3.6, 1.6);
      ctx.fill();
      ctx.strokeStyle = '#3a3f4c';
      ctx.lineWidth = 0.35;
      ctx.stroke();
      // 胎纹
      ctx.strokeStyle = '#4c5260';
      ctx.lineWidth = 0.45;
      for (const off of [-1.6, 0, 1.6]) {
        const tx = wx + off + treadPhase;
        ctx.beginPath();
        ctx.moveTo(tx, wy - 1.5);
        ctx.lineTo(tx, wy + 1.5);
        ctx.stroke();
      }
      // 轮毂
      ctx.fillStyle = '#6b7280';
      ctx.beginPath();
      ctx.arc(wx, wy, 0.9, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  // 尾部风扇舱（圆形舱体）
  ctx.fillStyle = 'rgba(200, 214, 228, 0.55)';
  ctx.beginPath();
  ctx.arc(-13, 0, 9.5, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = '#55606f';
  ctx.lineWidth = 0.5;
  ctx.stroke();

  // 主舱体（胶囊形，半透明外壳，呼应结构图）
  ctx.fillStyle = 'rgba(214, 226, 238, 0.5)';
  roundRectPath(-13.5, -10, 27, 20, 8);
  ctx.fill();
  ctx.strokeStyle = '#4c5666';
  ctx.lineWidth = 0.5;
  ctx.stroke();

  // 电池（中部蓝色方块）
  ctx.fillStyle = 'rgba(70, 120, 200, 0.85)';
  roundRectPath(-6, -5, 11, 10, 1.5);
  ctx.fill();
  ctx.strokeStyle = '#2c4a80';
  ctx.lineWidth = 0.35;
  ctx.stroke();

  // 顶部提手/框架（前部线框）
  ctx.strokeStyle = 'rgba(90, 100, 115, 0.8)';
  ctx.lineWidth = 0.4;
  ctx.strokeRect(3, -6, 9, 12);
  for (let i = 1; i < 4; i++) {
    ctx.beginPath();
    ctx.moveTo(3 + i * 2.25, -6);
    ctx.lineTo(3 + i * 2.25, 6);
    ctx.stroke();
  }

  // 车头超声波传感器（HC-SR04 双眼）
  ctx.fillStyle = '#dfe7ef';
  ctx.strokeStyle = '#333';
  ctx.lineWidth = 0.3;
  for (const ey of [-3.5, 3.5]) {
    ctx.beginPath();
    ctx.arc(13.2, ey, 1.7, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = '#9aa7b5';
    ctx.beginPath();
    ctx.arc(13.2, ey, 0.8, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = '#dfe7ef';
  }

  // 尾部旋转刷（黄色三臂刷盘）
  ctx.save();
  ctx.translate(-BRUSH_BACK - 2.5, 0);
  ctx.rotate(brushAngle);
  ctx.strokeStyle = '#d8b93a';
  ctx.lineWidth = 1.1;
  ctx.lineCap = 'round';
  for (let i = 0; i < 3; i++) {
    ctx.rotate((Math.PI * 2) / 3);
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.quadraticCurveTo(2.5, 1.5, 4.5, 0.4);
    ctx.stroke();
  }
  ctx.fillStyle = '#b89a20';
  ctx.beginPath();
  ctx.arc(0, 0, 1.2, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();

  // 尾部风扇（三叶片，运行时高速旋转）
  ctx.save();
  ctx.translate(-13, 0);
  ctx.rotate(fanAngle);
  ctx.fillStyle = robot.vacuum ? 'rgba(60, 90, 120, 0.9)' : 'rgba(90, 100, 115, 0.8)';
  for (let i = 0; i < 3; i++) {
    ctx.rotate((Math.PI * 2) / 3);
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.quadraticCurveTo(3.5, -1.2, 7, -3.2);
    ctx.quadraticCurveTo(7.8, -0.6, 4.5, 1.4);
    ctx.closePath();
    ctx.fill();
  }
  ctx.fillStyle = '#39424f';
  ctx.beginPath();
  ctx.arc(0, 0, 1.6, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();

  // 风扇舱防护圈
  ctx.strokeStyle = 'rgba(85, 96, 111, 0.9)';
  ctx.lineWidth = 0.45;
  ctx.beginPath();
  ctx.arc(-13, 0, 8, 0, Math.PI * 2);
  ctx.stroke();

  ctx.restore();

  // 朝向指示 + 超声波射线（自动模式时）
  const c = Math.cos(robot.theta), s = Math.sin(robot.theta);
  const nx = toPxX(robot.x + c * 18), ny = toPxY(robot.y + s * 18);
  if (mode !== MODE.MANUAL) {
    const dpx = Math.min(distanceCm, 120) * SCALE;
    ctx.strokeStyle = distanceCm < OBSTACLE_DISTANCE_CM
      ? 'rgba(248, 113, 113, 0.8)' : 'rgba(52, 211, 153, 0.55)';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([5, 4]);
    ctx.beginPath();
    ctx.moveTo(nx, ny);
    ctx.lineTo(nx + c * dpx, ny + s * dpx);
    ctx.stroke();
    ctx.setLineDash([]);
  }
}

function render() {
  drawRoom();
  drawDust();
  drawRobot();
}

/* ================= HUD ================= */
const el = id => document.getElementById(id);
const hud = {
  mode: el('hud-mode'), gear: el('hud-gear'),
  pwml: el('hud-pwml'), pwmr: el('hud-pwmr'),
  fan: el('hud-fan'), dist: el('hud-dist'), cmd: el('hud-cmd'),
  bar: el('hud-bar'), clean: el('hud-clean'), console: el('console'),
};

function gearName() {
  if (robot.gear === PWM_FAST) return '快速 (100)';
  if (robot.gear === PWM_SLOW) return '慢速 (60)';
  return '中速 (80)';
}

function updateHud() {
  hud.mode.textContent = MODE_NAME[mode] + (mode !== MODE.MANUAL
    ? `（${((simNow - phaseStart) / 1000).toFixed(1)}s）` : '');
  hud.gear.textContent = gearName();
  hud.pwml.textContent = robot.pwmL;
  hud.pwmr.textContent = robot.pwmR;
  hud.fan.textContent = robot.vacuum ? 'ON' : 'OFF';
  hud.fan.className = 'badge ' + (robot.vacuum ? 'on' : 'off');
  hud.dist.textContent = distanceCm >= DISTANCE_OUT_OF_RANGE
    ? '> ' + DISTANCE_OUT_OF_RANGE + ' cm（超量程）' : distanceCm + ' cm';
  hud.dist.style.color = distanceCm < OBSTACLE_DISTANCE_CM ? '#f87171' : '';
  hud.cmd.textContent = lastCmdText;
  const total = dust.length;
  const pct = total ? Math.round((cleaned / total) * 100) : 0;
  hud.clean.textContent = `${cleaned} / ${total} (${pct}%)`;
  hud.bar.style.width = pct + '%';
}

function telemetryLog() {
  // 与固件 printf("%lu\r\n", distance) 一致：只输出距离数值
  const line = document.createElement('div');
  line.textContent = String(distanceCm);
  hud.console.appendChild(line);
  while (hud.console.childNodes.length > 40) {
    hud.console.removeChild(hud.console.firstChild);
  }
  hud.console.scrollTop = hud.console.scrollHeight;
}

/* ================= 键盘 → 指令队列 ================= */
const KEY_MAP = {
  w: CMD.FORWARD, arrowup: CMD.FORWARD,
  s: CMD.BACKWARD, arrowdown: CMD.BACKWARD,
  a: CMD.LEFT, arrowleft: CMD.LEFT,
  d: CMD.RIGHT, arrowright: CMD.RIGHT,
  1: CMD.SLOW, 2: CMD.MEDIUM, 3: CMD.FAST,
  f: CMD.VACUUM_ON,
  g: CMD.AUTO_MODE, enter: CMD.AUTO_MODE,
  ' ': CMD.STOP,
};

function isTurnKey(key) {
  return key === 'a' || key === 'arrowleft' || key === 'd' || key === 'arrowright';
}

window.addEventListener('keydown', e => {
  const key = e.key.toLowerCase();
  if (key === 'r') { resetSim(); return; }
  const cmd = KEY_MAP[key];
  if (cmd === undefined) return;
  e.preventDefault();
  if (cmd === CMD.LEFT || cmd === CMD.RIGHT) turning = true; // 先打标记，快速点按也能回正
  // 队列深度 8，与 g_ctrlQueue 一致；按住重复触发时去重
  if (cmdQueue.length < 8 && cmdQueue[cmdQueue.length - 1] !== cmd) {
    cmdQueue.push(cmd);
  }
});

window.addEventListener('keyup', e => {
  // 松开转向键：只要转向仍生效（含已入队未执行），就回正为直行
  if (isTurnKey(e.key.toLowerCase()) && turning && mode === MODE.MANUAL &&
      cmdQueue.length < 8) {
    cmdQueue.push(CMD.FORWARD);
  }
});

/* ================= 重置 ================= */
function resetSim() {
  robot.x = START.x; robot.y = START.y; robot.theta = START.theta;
  robot.dir = 1; robot.pwmL = 0; robot.pwmR = 0;
  robot.gear = PWM_MEDIUM; robot.vacuum = false;
  mode = MODE.MANUAL;
  turning = false;
  distanceCm = DISTANCE_OUT_OF_RANGE;
  lastCmdText = '—';
  cmdQueue = [];
  spawnDust();
}

/* ================= 主循环（控制 20ms / 测距 100ms / 上报 1s） ================= */
let acc = 0, lastFrame = performance.now();
let sensorTimer = 0, telemetryTimer = 0;

function frame(now) {
  let dtMs = now - lastFrame;
  lastFrame = now;
  if (dtMs > 200) dtMs = 200; // 掉帧保护
  acc += dtMs;

  while (acc >= CONTROL_PERIOD_MS) {
    acc -= CONTROL_PERIOD_MS;
    simNow += CONTROL_PERIOD_MS;

    while (cmdQueue.length > 0) applyCommand(cmdQueue.shift());
    updateAutoMode();
    stepPhysics(CONTROL_PERIOD_MS / 1000);
    updateDust(CONTROL_PERIOD_MS / 1000);

    // 风扇/刷盘动画速度（惯性启停）
    const targetFan = robot.vacuum ? 26 : 0;
    fanSpeed += (targetFan - fanSpeed) * 0.08;
    fanAngle += fanSpeed * (CONTROL_PERIOD_MS / 1000);
    brushAngle -= (robot.vacuum ? 7 : 0.6) * (CONTROL_PERIOD_MS / 1000);

    sensorTimer += CONTROL_PERIOD_MS;
    if (sensorTimer >= SENSOR_PERIOD_MS) {
      sensorTimer = 0;
      distanceCm = rayDistance();
    }
    telemetryTimer += CONTROL_PERIOD_MS;
    if (telemetryTimer >= TELEMETRY_PERIOD_MS) {
      telemetryTimer = 0;
      telemetryLog();
    }
  }

  render();
  updateHud();
  requestAnimationFrame(frame);
}

spawnDust();
requestAnimationFrame(frame);

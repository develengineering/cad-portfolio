// Kinematics of the R4 diesel (port of engine_scene.body_T): pose({A}) -> {M: {component: 4x4 row-major}} for crank angle A
// (degrees, 0 = TDC compression of cylinder 1, cycle 720). World: cylinders up, crank axis along Z at y = 202, mm.
export function makeKin(meta) {
  const K = meta.kin, parts = meta.parts, names = Object.keys(parts);
  const { A0, PH0, R_CR, L_ROD, LIFT_Y } = K, AX = K.axes;
  const rad = (d) => d * Math.PI / 180, deg = (r) => r * 180 / Math.PI;
  const I4 = () => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
  const mul = (A, B) => {
    const M = new Array(16);
    for (let r = 0; r < 4; r++) for (let c = 0; c < 4; c++)
      M[r * 4 + c] = A[r * 4] * B[c] + A[r * 4 + 1] * B[4 + c] + A[r * 4 + 2] * B[8 + c] + A[r * 4 + 3] * B[12 + c];
    return M;
  };
  const trans = (v) => [1, 0, 0, v[0], 0, 1, 0, v[1], 0, 0, 1, v[2], 0, 0, 0, 1];
  const rotz = (c, d) => {            // rotation about a Z-parallel axis through world xy point c
    const a = rad(d), ca = Math.cos(a), sa = Math.sin(a);
    return [ca, -sa, 0, c[0] - ca * c[0] + sa * c[1], sa, ca, 0, c[1] - sa * c[0] - ca * c[1], 0, 0, 1, 0, 0, 0, 0, 1];
  };
  const w2 = (xy) => [-xy[0], -xy[1] + LIFT_Y];
  const pinY = (a) => -(R_CR * Math.cos(rad(a)) + Math.sqrt(L_ROD * L_ROD - (R_CR * Math.sin(rad(a))) ** 2));
  const throwA = (cyl, A) => A + ((cyl === 2 || cyl === 3) ? 180 : 0);
  const tappetLift = (k, A) => { const d = ((A - K.mid[k] + 360) % 720 + 720) % 720 - 360; return Math.max(0, 10 * Math.cos(rad(d / 2)) - 6); };
  const valveLift = (k, A) => { const L = tappetLift(k, A); return 1.2742 * L - 0.01229 * L * L; };
  const th = (L) => -deg(Math.asin(Math.min(1, L / 35.8)));
  const STROKE_OFF = { 1: 0, 3: 180, 4: 360, 2: 540 };

  function pose(q) {
    const A = q.A, dA = A - A0;
    const M = {}; for (const n of names) M[n] = I4();
    const put = (n, m) => { if (n in M) M[n] = m; };
    for (const n of ['crank4-1', 'gear_crank-1', 'crank_pulley-1', 'flywheel-1']) put(n, rotz(AX.O, dA));
    put('gear_idler-1', rotz(AX.IDLER, -dA * 24 / 36));
    const cam = rotz(AX.CAM, (A - PH0) / 2); put('gear_cam-1', cam); put('camshaft-1', cam);
    const pump = rotz(AX.PUMP, dA / 2); put('gear_cam-2', pump); put('pump_shaft-1', pump);
    put('water_pump_fan-1', rotz(AX.FAN, dA * 140 / 84));
    put('alternator-1', rotz(AX.ALT, dA * 140 / 60));
    for (const cyl of [1, 2, 3, 4]) {
      const a = throwA(cyl, A), y = pinY(a), r0 = K.rod0[cyl], y0 = r0.p0[1];
      const d = trans([0, -(y - y0), 0]); put(`piston2-${cyl}`, d); put(`pin-${cyl}`, d);
      const c = [R_CR * Math.sin(rad(a)), -R_CR * Math.cos(rad(a))], p = [0, y];
      const ang = deg(Math.atan2(p[1] - c[1], p[0] - c[0]) - Math.atan2(r0.p0[1] - r0.c0[1], r0.p0[0] - r0.c0[0]));
      const Mr = rotz(w2(r0.c0), ang), wc = w2(c), wc0 = w2(r0.c0);
      put(`rod-${cyl}`, mul(trans([wc[0] - wc0[0], wc[1] - wc0[1], 0]), Mr));
    }
    for (let k = 0; k < 8; k++) {
      const lt = tappetLift(k, A), lt0 = tappetLift(k, PH0), lv = valveLift(k, A), lv0 = valveLift(k, PH0);
      const up = trans([0, lt - lt0, 0]); put(`tappet-${k + 1}`, up); put(`pushrod-${k + 1}`, up);
      put(K.valve_comp[k], trans([0, -(lv - lv0), 0]));
      put(`rocker-${k + 1}`, rotz(AX.ROCKER, th(lv) - th(lv0)));
    }
    for (const c in K.coil) { const [k, fr] = K.coil[c]; put(c, trans([0, -fr * (valveLift(k, A) - valveLift(k, PH0)), 0])); }
    return { M };
  }
  function strokes(A) {               // per cylinder: current stroke name and its phase 0..1
    const out = {};
    for (const cyl of [1, 2, 3, 4]) {
      const ph = ((A - STROKE_OFF[cyl]) % 720 + 720) % 720;
      out[cyl] = ['Power', 'Exhaust', 'Intake', 'Compression'][Math.floor(ph / 180)];
    }
    return out;
  }
  const lifts = (A) => Array.from({ length: 8 }, (_, k) => valveLift(k, A));
  return { pose, trans, mul, names, strokes, lifts };
}

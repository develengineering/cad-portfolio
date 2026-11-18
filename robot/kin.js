// Kinematics of robot_v2 (port of rig.py): pose(q) -> {component: 4x4 row-major}. Assembly frame: J1 axis = Y, forward +Z, floor Y = 0, mm.
// Every gear train follows its joint with the real ratio: planetary stages, timing belts, bevel pair, gripper screw and links.
export function makeKin(meta) {
  const Q0 = meta.q0, C = meta.const;
  const X = [1, 0, 0], Y = [0, 1, 0], NY = [0, -1, 0];
  const A4 = C.A4, P2 = C.P2, P3 = C.P3, WC = C.WC, PX = C.PX, G4 = C.G4, c1 = C.c1;
  const parts = meta.parts, names = Object.keys(parts);
  const fr = { 1: meta.fr['1'], 2: meta.fr['2'], 3: meta.fr['3'] };
  const linkFinger = { 1: meta.link_finger['1'], 2: meta.link_finger['2'], 3: meta.link_finger['3'] };
  const I4 = () => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
  const mul = (A, B) => {
    const M = new Array(16);
    for (let r = 0; r < 4; r++) for (let c = 0; c < 4; c++)
      M[r * 4 + c] = A[r * 4] * B[c] + A[r * 4 + 1] * B[4 + c] + A[r * 4 + 2] * B[8 + c] + A[r * 4 + 3] * B[12 + c];
    return M;
  };
  const rot = (p, d, deg) => {
    const L = Math.hypot(d[0], d[1], d[2]); const x = d[0] / L, y = d[1] / L, z = d[2] / L;
    const a = deg * Math.PI / 180, s = Math.sin(a), c = 1 - Math.cos(a);
    const R = [1 + c * (x * x - 1), -z * s + c * x * y, y * s + c * x * z,
               z * s + c * x * y, 1 + c * (y * y - 1), -x * s + c * y * z,
               -y * s + c * x * z, x * s + c * y * z, 1 + c * (z * z - 1)];
    const t = [p[0] - (R[0] * p[0] + R[1] * p[1] + R[2] * p[2]), p[1] - (R[3] * p[0] + R[4] * p[1] + R[5] * p[2]), p[2] - (R[6] * p[0] + R[7] * p[1] + R[8] * p[2])];
    return [R[0], R[1], R[2], t[0], R[3], R[4], R[5], t[1], R[6], R[7], R[8], t[2], 0, 0, 0, 1];
  };
  const trans = (v) => [1, 0, 0, v[0], 0, 1, 0, v[1], 0, 0, 1, v[2], 0, 0, 0, 1];
  const add = (a, b, k = 1) => [a[0] + k * b[0], a[1] + k * b[1], a[2] + k * b[2]];
  const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const gripGeo = (th) => {
    const P = [20, 84], a = 13, l = 13, phi = (215 - th) * Math.PI / 180;
    const Lp = [P[0] + a * Math.cos(phi), P[1] + a * Math.sin(phi)];
    const Cc = [12, Lp[1] - Math.sqrt(l * l - (12 - Lp[0]) ** 2)];
    return [P, Lp, Cc];
  };

  function pose(qIn) {
    const q = Object.assign({}, Q0, qIn || {});
    const d = {}; for (const k in Q0) d[k] = q[k] - Q0[k];
    const L = [I4()];
    L.push(rot([0, 0, 0], Y, d.J1));
    L.push(mul(L[1], rot(P2, X, d.J2)));
    L.push(mul(L[2], rot(P3, X, d.J3)));
    L.push(mul(L[3], rot(WC, A4, d.J4)));
    L.push(mul(L[4], rot(WC, X, d.J5)));
    L.push(mul(L[5], rot(WC, NY, d.J6)));
    const M = {};
    for (const n of names) M[n] = L[parts[n].link];
    const planetary = (pre, link, c, a, th, kind) => {
      const B = L[link];
      const put = (name, Ml) => { const k = pre ? pre + ' ' + name : name; if (k in M) M[k] = mul(B, Ml); };
      const planets = (pname, thCar, zr, zp) => {
        for (const i of [1, 2, 3]) {
          const k = pre ? `${pre} ${pname}-${i}` : `${pname}-${i}`;
          if (k in M) M[k] = mul(B, mul(rot(c, a, thCar), rot(parts[k].cen, a, -thCar * zr / zp)));
        }
      };
      if (kind === 'h') {
        put('pg23_carrier-1', rot(c, a, th));
        for (const s of ['pg23_sun_z12-1', 'pg23_coupling-1']) put(s, rot(c, a, 10 * th));
        planets('pg23_planet_z48', th, 108, 48);
      } else if (kind === 'i') {
        put('pg23g50_carrier2-1', rot(c, a, th));
        for (const s of ['pg23g50_sun2_z18-1', 'pg23g50_carrier1-1']) put(s, rot(c, a, 5 * th));
        planets('pg23g50_planet2_z27', th, 72, 27);
        for (const s of ['pg23_sun_z12-1', 'pg23_coupling-1']) put(s, rot(c, a, 50 * th));
        planets('pg23_planet_z48', 5 * th, 108, 48);
      } else if (kind === 'j') {
        put('pg34_carrier2-1', rot(c, a, th));
        for (const s of ['pg34_sun2_z18-1', 'pg34_carrier1-1']) put(s, rot(c, a, 5 * th));
        planets('pg_planet_z27', th, 72, 27);
        for (const s of ['pg34_sun1_z12-1', 'pg_coupling-1']) put(s, rot(c, a, 50 * th));
        planets('pg34_planet1_z48', 5 * th, 108, 48);
      } else if (kind === '6') {
        put('j6_carrier-1', rot(c, a, th));
        for (const s of ['j6_sun_z12-1', 'j6_coupling-1']) put(s, rot(c, a, 10 * th));
        planets('j6_planet_z48', th, 108, 48);
      }
    };
    planetary('pg23_gearbox_10to1-1', 0, c1, Y, 3 * d.J1, 'h');
    M['j1_pulley_20t-1'] = mul(L[0], rot(c1, Y, 3 * d.J1));
    planetary('pg34_gearbox_50to1-1', 1, P2, X, d.J2, 'j');
    planetary('pg23g50_gearbox_50to1-1', 2, P3, X, d.J3, 'i');
    planetary('pg23_gearbox_10to1-2', 3, G4, A4, 3.6 * d.J4, 'h');
    M['j1_pulley_20t-2'] = mul(L[3], rot(G4, A4, 3.6 * d.J4));
    planetary('pg23_gearbox_10to1-3', 3, WC, A4, d.J4 + d.J5, 'h');
    for (const n of ['j5_shaft-1', 'j5_bevel_m2_z20-1']) M[n] = mul(L[4], rot(WC, A4, d.J5));
    for (const n of ['j5_xshaft-1', 'j5_bevel_m2_z20-2', 'j1_pulley_20t-3']) M[n] = mul(L[4], rot(PX, X, d.J5));
    planetary('', 5, WC, NY, d.J6, '6');
    // gripper
    const th0 = Q0.F, th = q.F;
    const [P_, L0, C0] = gripGeo(th0), [, L1, C1] = gripGeo(th);
    const down = NY, dyc = C1[1] - C0[1];
    M['grip_crosshead-1'] = mul(L[6], trans([0, -dyc, 0]));
    M['grip_screw_tr8-1'] = mul(L[6], rot(WC, down, dyc / 2 * 360));
    const flangeY = 600.2;
    const pt = (i, rv) => add(add([WC[0], flangeY, WC[2]], fr[i], rv[0]), down, rv[1]);
    for (const i of [1, 2, 3]) {
      const nAx = cross(fr[i], down);
      M[`finger_v2-${i}`] = mul(L[6], rot(pt(i, P_), nAx, -(th - th0)));
    }
    for (const i of [1, 2, 3]) {
      const k = linkFinger[i], nAx = cross(fr[k], down);
      const a0 = Math.atan2(L0[1] - C0[1], L0[0] - C0[0]) * 180 / Math.PI, a1 = Math.atan2(L1[1] - C1[1], L1[0] - C1[0]) * 180 / Math.PI;
      M[`grip_link-${i}`] = mul(L[6], mul(trans(sub(pt(k, C1), pt(k, C0))), rot(pt(k, C0), nAx, a1 - a0)));
    }
    return { M, L };
  }
  return { pose, trans, mul, names };
}

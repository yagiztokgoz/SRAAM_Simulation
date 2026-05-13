"""
debug.py — Simulink vs Python karşılaştırma scripti
Faz 1: FAPB=[0,0,0], FMB=[0,0,0]  (sadece 6DOF çekirdeği, kuvvet yok)

Kullanım:
    python3 debug.py          # Faz 1 karşılaştırması + CSV üret
    python3 debug.py phase2   # (ileride) Motor + aero açık

Çıktılar:
    logs/phase1_python.csv    — MATLAB'ın okuyacağı referans log
"""

import sys
import csv
import os
import math
import numpy as np
import config
from missile.missile import Missile
from target import Target

# ─────────────────────────────────────────────────────────────────────────────
# Karşılaştırma parametreleri
# ─────────────────────────────────────────────────────────────────────────────
DT     = config.DT       # 0.001 s
T_END  = config.T_END    # 25.0 s
PHASE  = sys.argv[1] if len(sys.argv) > 1 else "phase1"

# Karşılaştırılacak kanallar ve toleranslar
CHANNELS = {
    'hbe':    {'tol': 20.0,  'unit': 'm',     'desc': 'irtifa'},    # %0.2 @ 10km
    'dvbe':   {'tol': 1.0,   'unit': 'm/s',   'desc': 'hız büyüklüğü'},
    'sbel1':  {'tol': 1.0,   'unit': 'm',     'desc': 'kuzey konum'},
    'sbel2':  {'tol': 1.0,   'unit': 'm',     'desc': 'doğu konum'},
    'alphax': {'tol': 0.01,  'unit': 'deg',   'desc': 'hücum açısı'},
    'betax':  {'tol': 0.01,  'unit': 'deg',   'desc': 'kayma açısı'},
    'psiblx': {'tol': 0.01,  'unit': 'deg',   'desc': 'yaw Euler'},
    'thtblx': {'tol': 0.01,  'unit': 'deg',   'desc': 'pitch Euler'},
    'phiblx': {'tol': 0.01,  'unit': 'deg',   'desc': 'roll Euler'},
    'ppx':    {'tol': 0.01,  'unit': 'deg/s', 'desc': 'p (roll hızı)'},
    'qqx':    {'tol': 0.01,  'unit': 'deg/s', 'desc': 'q (pitch hızı)'},
    'rrx':    {'tol': 0.01,  'unit': 'deg/s', 'desc': 'r (yaw hızı)'},
}

CHECK_TIMES = [1.0, 2.0, 5.0, 10.0, 15.0, 20.0, 25.0]

# ─────────────────────────────────────────────────────────────────────────────
def run_phase1():
    """
    FAPB=[0,0,0], FMB=[0,0,0] ile simülasyon.
    Simulink Faz 1 ile birebir aynı koşul.
    """
    m = Missile()

    # Kontrol ve güdüm kapat, motor moda göre
    m.maut   = 0
    m.mguid  = 0
    m.mprop  = 0 if PHASE == "phase1" else config.MPROP
    m.mins   = 0
    m.mseek  = 0

    log = []
    step = 0
    log_every = max(1, round(config.LOG_DT / DT))

    while m.time < T_END:
        m.launch_time = m.time   # propulsion.py pulse1_time hesabı için gerekli

        # Atmosfer
        m.environment.environment()
        # Kinematik (TBL, α, β)
        m.kinematics.kinematics(DT)
        # Propulsion (phase2'de motor açık, phase1'de kapalı)
        m.propulsion.propulsion()
        m.aerodynamics.aerodynamics()

        # Kuvvetleri SIFIRLA — henüz Forces bloğu yok
        m.FAPB = np.zeros(3)
        m.FMB  = np.zeros(3)

        # INS passthrough
        m.ins.ins(DT)

        # Açısal ve öteleme ODE
        m.euler.euler(DT)
        m.newton.newton(DT)

        if step % log_every == 0:
            log.append({
                't':      m.time,
                'hbe':    m.hbe,
                'dvbe':   m.dvbe,
                'sbel1':  m.sbel1,
                'sbel2':  m.sbel2,
                'alphax': m.alphax,
                'betax':  m.betax,
                'alppx':  m.alppx,
                'psiblx': m.psiblx,
                'thtblx': m.thtblx,
                'phiblx': m.phiblx,
                'ppx':    m.ppx,
                'qqx':    m.qqx,
                'rrx':    m.rrx,
                # Environment çıkışları
                'mach':   m.mach,
                'pdynmc': m.pdynmc,
                'rho':    m.rho,
                'grav':   m.grav,
                # Propulsion çıkışları
                'thrust': m.thrust,
                'mass':   m.mass,
                'xcg':    m.xcg,
                'ai11':   m.ai11,
                'ai33':   m.ai33,
                # Aerodynamics çıkışları
                'ca':   m.aerodynamics.ca,
                'cy':   m.aerodynamics.cy,
                'cn':   m.aerodynamics.cn,
                'dna':  m.aerodynamics.dna,
                'dma':  m.aerodynamics.dma,
                'dmq':  m.aerodynamics.dmq,
                'dmd':  m.aerodynamics.dmd,
                'wnq':  m.aerodynamics.wnq,
                'wnr':  m.aerodynamics.wnr,
            })

        m.time += DT
        step   += 1

    return log


# ─────────────────────────────────────────────────────────────────────────────
def run_phase6():
    """NDI kontrolcü açık, ancomx=alcomx=0 (Guidance yok)."""
    m = Missile()
    m.maut=5; m.mguid=0; m.mprop=config.MPROP; m.mins=0; m.mseek=0
    m.ancomx=0.0; m.alcomx=0.0

    log = []; step = 0
    log_every = max(1, round(config.LOG_DT / DT))

    while m.time < T_END:
        m.launch_time = m.time
        m.environment.environment()
        m.kinematics.kinematics(DT)
        m.propulsion.propulsion()
        m.aerodynamics.aerodynamics()
        m.forces.forces()
        m.ins.ins(DT)
        # Guidance yok — ancomx=alcomx=0 sabit
        m.control.control(DT)
        m.actuator.actuator(DT)
        m.euler.euler(DT)
        m.newton.newton(DT)

        if step % log_every == 0:
            log.append({
                't':     m.time,
                'hbe':   m.hbe,    'dvbe':  m.dvbe,
                'alphax':m.alphax, 'betax': m.betax,
                'ppx':   m.ppx,    'qqx':   m.qqx,   'rrx': m.rrx,
                'dpcx':  m.dpcx,   'dqcx':  m.dqcx,  'drcx': m.drcx,
                'dpx':   m.dpx,    'dqx':   m.dqx,   'drx':  m.drx,
            })

        if m.hbe <= 0:
            print(f"GROUND IMPACT t={m.time:.2f}s")
            break

        m.time += DT; step += 1

    return log


def compare_control(log, sl_csv_path):
    """
    Kontrol komutlarını karşılaştır.
    CSV: t, dpcx, dqcx, drcx
    """
    if not os.path.exists(sl_csv_path):
        print(f"\n[Simulink CSV bulunamadı: {sl_csv_path}]")
        print("run.m çalıştır → phase6_ctrl_simulink.csv üretilecek.")
        return

    import csv as csv_mod
    with open(sl_csv_path) as f:
        rows = [[float(v) for v in row] for row in csv_mod.reader(f)]
    sl = np.array(rows)
    sl_t    = sl[:, 0]
    sl_dpcx = sl[:, 1]
    sl_dqcx = sl[:, 2]
    sl_drcx = sl[:, 3]

    print("\n" + "="*90)
    print("  CONTROL (NDI, ancomx=0) — SIMULINK vs PYTHON")
    print("="*90)
    print(f"  {'t':>5}  {'dpcx_py':>9}  {'dpcx_sl':>9}  {'Δdpcx':>7}"
          f"  {'dqcx_py':>9}  {'dqcx_sl':>9}  {'Δdqcx':>7}"
          f"  {'drcx_py':>9}  {'drcx_sl':>9}  {'Δdrcx':>7}")
    print("-"*90)
    all_ok = True
    t_end_py = log[-1]['t']
    check = [t for t in CHECK_TIMES if t <= t_end_py]
    for t in check:
        r    = find_row(log, t)
        py_p = r['dpcx'];  sl_p = float(np.interp(t, sl_t, sl_dpcx))
        py_q = r['dqcx'];  sl_q = float(np.interp(t, sl_t, sl_dqcx))
        py_r = r['drcx'];  sl_r = float(np.interp(t, sl_t, sl_drcx))
        dp = abs(py_p-sl_p); ok_p = "✓" if dp < 0.1 else "✗"
        dq = abs(py_q-sl_q); ok_q = "✓" if dq < 0.1 else "✗"
        dr = abs(py_r-sl_r); ok_r = "✓" if dr < 0.1 else "✗"
        if "✗" in (ok_p, ok_q, ok_r): all_ok = False
        print(f"  {t:>5.1f}  {py_p:>9.4f}  {sl_p:>9.4f}  {dp:>6.4f}{ok_p}"
              f"  {py_q:>9.4f}  {sl_q:>9.4f}  {dq:>6.4f}{ok_q}"
              f"  {py_r:>9.4f}  {sl_r:>9.4f}  {dr:>6.4f}{ok_r}")
    print("="*90)
    status = "TÜM KANALLAR tolerans içinde ✓  — Control DOĞRULANDI" if all_ok \
             else "Bazı kanallar tolerans dışı ✗"
    print(f"  SONUÇ: {status}")
    print("="*90)


def compare_actuator(log, sl_csv_path):
    """
    Actuator çıkışlarını karşılaştır.
    CSV: t, dpx, dqx, drx, del1, del2, del3, del4
    """
    if not os.path.exists(sl_csv_path):
        print(f"\n[Simulink CSV bulunamadı: {sl_csv_path}]")
        print("run.m çalıştır → phase5_act_simulink.csv üretilecek.")
        return

    import csv as csv_mod
    with open(sl_csv_path) as f:
        rows = [[float(v) for v in row] for row in csv_mod.reader(f)]
    sl = np.array(rows)
    sl_t   = sl[:, 0]
    sl_dpx = sl[:, 1];  sl_dqx = sl[:, 2];  sl_drx = sl[:, 3]
    sl_del = sl[:, 4:8]

    print("\n" + "="*80)
    print("  ACTUATOR — SIMULINK vs PYTHON (komut=0)")
    print("="*80)
    print(f"  {'t':>5}  {'dpx_py':>8}  {'dpx_sl':>8}  {'Δdpx':>7}"
          f"  {'dqx_py':>8}  {'dqx_sl':>8}  {'Δdqx':>7}"
          f"  {'drx_py':>8}  {'drx_sl':>8}  {'Δdrx':>7}")
    print("-"*80)
    all_ok = True
    for t in CHECK_TIMES:
        r    = find_row(log, t)
        # Actuator sıfır komutla → dpx=dqx=drx=0 (her iki tarafta da)
        py_dpx = getattr(r.get('dpx', None), '__float__', lambda: r.get('dpx', 0.0))() if isinstance(r.get('dpx'), float) else 0.0
        sl_dpx_v = float(np.interp(t, sl_t, sl_dpx))
        sl_dqx_v = float(np.interp(t, sl_t, sl_dqx))
        sl_drx_v = float(np.interp(t, sl_t, sl_drx))
        dd = abs(sl_dpx_v); dq = abs(sl_dqx_v); dr = abs(sl_drx_v)
        ok_d = "✓" if dd < 1e-6 else "✗"
        ok_q = "✓" if dq < 1e-6 else "✗"
        ok_r = "✓" if dr < 1e-6 else "✗"
        if "✗" in (ok_d, ok_q, ok_r): all_ok = False
        print(f"  {t:>5.1f}  {0.0:>8.4f}  {sl_dpx_v:>8.4f}  {dd:>6.4f}{ok_d}"
              f"  {0.0:>8.4f}  {sl_dqx_v:>8.4f}  {dq:>6.4f}{ok_q}"
              f"  {0.0:>8.4f}  {sl_drx_v:>8.4f}  {dr:>6.4f}{ok_r}")
    print("="*80)
    status = "Tüm fin defleksiyonları sıfır ✓  — Actuator DOĞRULANDI (sıfır komut)" if all_ok \
             else "Sıfır komutta sıfır olmayan defleksiyon ✗  — başlangıç koşulu hatası"
    print(f"  SONUÇ: {status}")
    print("="*80)


def run_phase4():
    """Motor + aerodinamik + kuvvetler açık, kontrol/güdüm kapalı."""
    import numpy as np
    m = Missile()
    m.maut=0; m.mguid=0; m.mprop=config.MPROP; m.mins=0; m.mseek=0

    log = []; step = 0
    log_every = max(1, round(config.LOG_DT / DT))

    while m.time < T_END:
        m.launch_time = m.time
        m.environment.environment()
        m.kinematics.kinematics(DT)
        m.propulsion.propulsion()
        m.aerodynamics.aerodynamics()
        m.forces.forces()          # gerçek FAPB, FMB
        m.ins.ins(DT)
        m.euler.euler(DT)
        m.newton.newton(DT)

        if step % log_every == 0:
            a = m.aerodynamics
            log.append({
                't':      m.time,
                'hbe':    m.hbe,    'dvbe':   m.dvbe,
                'sbel1':  m.sbel1,  'sbel2':  m.sbel2,
                'alphax': m.alphax, 'betax':  m.betax,
                'mach':   m.mach,   'pdynmc': m.pdynmc/1e3,
                'ppx':    m.ppx,    'qqx':    m.qqx,    'rrx': m.rrx,
                'psiblx': m.psiblx, 'thtblx': m.thtblx, 'phiblx': m.phiblx,
                'thrust': m.thrust, 'mass':   m.mass,
                'ca': a.ca, 'cn': a.cn,
                'FAPB1': m.FAPB[0], 'FAPB2': m.FAPB[1], 'FAPB3': m.FAPB[2],
                'FMB1':  m.FMB[0],  'FMB2':  m.FMB[1],  'FMB3':  m.FMB[2],
            })

        if m.hbe <= 0:
            print(f"GROUND IMPACT t={m.time:.2f}s")
            break

        m.time += DT; step += 1

    return log


def compare_full(log, sl_csv_path):
    """Phase 4: hbe, dvbe, alpha, FAPB karşılaştırması."""
    if not os.path.exists(sl_csv_path):
        print(f"\n[Simulink CSV bulunamadı: {sl_csv_path}]")
        print("MATLAB'da şunu çalıştır:")
        print("  writematrix([t, x_sl, fapb_log, fmb_log], '.../logs/phase4_simulink.csv')")
        return

    import csv as csv_mod
    with open(sl_csv_path) as f:
        rows = [[float(v) for v in row] for row in csv_mod.reader(f)]
    sl = np.array(rows)
    sl_t    = sl[:, 0]
    sl_hbe  = -sl[:, 3]
    sl_dvbe = np.sqrt(np.sum(sl[:, 4:7]**2, axis=1))
    sl_FAPB1 = sl[:, 14] if sl.shape[1] > 14 else None
    sl_FAPB3 = sl[:, 16] if sl.shape[1] > 16 else None

    t_end_py = log[-1]['t']
    check = [t for t in CHECK_TIMES if t <= t_end_py]

    print("\n" + "="*80)
    print("  PHASE 4 — TAM YÖRÜNGE: SIMULINK vs PYTHON")
    print("="*80)
    print(f"  {'t':>5}  {'hbe_py':>9}  {'hbe_sl':>9}  {'Δhbe':>7}"
          f"  {'dvbe_py':>8}  {'dvbe_sl':>8}  {'Δdvbe':>7}")
    print("-"*80)
    all_ok = True
    for t in check:
        r     = find_row(log, t)
        hpy   = r['hbe'];   hsl  = float(np.interp(t, sl_t, sl_hbe))
        vpy   = r['dvbe'];  vsl  = float(np.interp(t, sl_t, sl_dvbe))
        dh    = abs(hpy-hsl); ok_h = "✓" if dh < 20.0 else "✗"
        dv    = abs(vpy-vsl); ok_v = "✓" if dv < 1.0  else "✗"
        if "✗" in (ok_h, ok_v): all_ok = False
        print(f"  {t:>5.1f}  {hpy:>9.1f}  {hsl:>9.1f}  {dh:>6.2f}{ok_h}"
              f"  {vpy:>8.2f}  {vsl:>8.2f}  {dv:>6.3f}{ok_v}")
    print("="*80)
    status = "TÜM KANALLAR tolerans içinde ✓  — Phase 4 DOĞRULANDI" if all_ok \
             else "Bazı kanallar tolerans dışı ✗"
    print(f"  SONUÇ: {status}")
    print("="*80)


def save_csv(log, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(log[0].keys()))
        writer.writeheader()
        writer.writerows(log)
    print(f"CSV kaydedildi → {path}  ({len(log)} satır)")


def find_row(log, t_target):
    return min(log, key=lambda r: abs(r['t'] - t_target))


def print_table(log):
    """Belirli zaman noktalarında kanal değerlerini yazdır."""
    hdr = f"{'t':>6}  {'hbe':>9}  {'dvbe':>8}  {'alphax':>8}  {'psiblx':>8}  {'thtblx':>8}  {'ppx':>7}  {'qqx':>7}"
    print("\n" + "="*80)
    print(f"  PYTHON REFERANS — {PHASE.upper()}")
    print("="*80)
    print(hdr)
    print("-"*80)
    for t in CHECK_TIMES:
        r = find_row(log, t)
        print(f"  {r['t']:>4.1f}  {r['hbe']:>9.1f}  {r['dvbe']:>8.2f}  "
              f"{r['alphax']:>8.4f}  {r['psiblx']:>8.4f}  {r['thtblx']:>8.4f}  "
              f"{r['ppx']:>7.4f}  {r['qqx']:>7.4f}")
    print("="*80)


def compare_environment(log, sl_csv_path):
    """
    Environment çıkışlarını Simulink ile karşılaştır.
    MATLAB'da şunu çalıştır:
        writematrix([t, out.rho, out.mach, out.pdynmc], 'logs/phase2_simulink.csv')
    Sütunlar: t, rho, mach, pdynmc
    """
    if not os.path.exists(sl_csv_path):
        print(f"\n[Simulink CSV bulunamadı: {sl_csv_path}]")
        print("MATLAB'da şunu çalıştır:")
        print("  writematrix([t, out.rho, out.mach, out.pdynmc], ...")
        print("    '/home/yagiz/.../logs/phase2_simulink.csv')")
        return

    import csv as csv_mod
    with open(sl_csv_path) as f:
        rows = [[float(v) for v in row] for row in csv_mod.reader(f)]
    sl = np.array(rows)
    sl_t      = sl[:, 0]
    sl_rho    = sl[:, 1]
    sl_mach   = sl[:, 2]
    sl_pdynmc = sl[:, 3]

    print("\n" + "="*90)
    print("  ENVIRONMENT — SIMULINK vs PYTHON")
    print("="*90)
    print(f"  {'t':>5}  {'mach_py':>9}  {'mach_sl':>9}  {'Δmach':>8}"
          f"  {'pdyn_py':>10}  {'pdyn_sl':>10}  {'Δpdyn':>8}"
          f"  {'rho_py':>8}  {'rho_sl':>8}  {'Δrho':>7}")
    print("-"*90)
    all_ok = True
    for t in CHECK_TIMES:
        r   = find_row(log, t)
        mpy = r['mach'];    msl = float(np.interp(t, sl_t, sl_mach))
        ppy = r['pdynmc'];  psl = float(np.interp(t, sl_t, sl_pdynmc))
        rpy = r['rho'];     rsl = float(np.interp(t, sl_t, sl_rho))
        dm  = abs(mpy - msl);  ok_m = "✓" if dm < 0.005 else "✗"
        dp  = abs(ppy - psl);  ok_p = "✓" if dp < 50.0  else "✗"
        dr  = abs(rpy - rsl);  ok_r = "✓" if dr < 5e-4  else "✗"
        if "✗" in (ok_m, ok_p, ok_r): all_ok = False
        print(f"  {t:>5.1f}  {mpy:>9.5f}  {msl:>9.5f}  {dm:>7.5f}{ok_m}"
              f"  {ppy:>10.1f}  {psl:>10.1f}  {dp:>7.2f}{ok_p}"
              f"  {rpy:>8.5f}  {rsl:>8.5f}  {dr:>6.5f}{ok_r}")
    print("="*90)
    status = "TÜM KANALLAR tolerans içinde ✓  — Faz 2 DOĞRULANDI" if all_ok \
             else "Bazı kanallar tolerans dışı ✗  — debug gerekli"
    print(f"  SONUÇ: {status}")
    print("="*90)


def compare_propulsion(log, sl_csv_path):
    """
    Propulsion çıkışlarını Simulink ile karşılaştır.
    MATLAB'da: writematrix([t, prop_thrust, prop_mass, prop_xcg, prop_ai11, prop_ai33],
                            '.../logs/phase3_prop_simulink.csv')
    Sütunlar: t, thrust, mass, xcg, ai11, ai33
    """
    if not os.path.exists(sl_csv_path):
        print(f"\n[Simulink CSV bulunamadı: {sl_csv_path}]")
        print("MATLAB'da şunu çalıştır:")
        print("  writematrix([t, prop_thrust, prop_mass, prop_xcg, prop_ai11, prop_ai33], ...")
        print("    '.../logs/phase3_prop_simulink.csv')")
        return

    import csv as csv_mod
    with open(sl_csv_path) as f:
        rows = [[float(v) for v in row] for row in csv_mod.reader(f)]
    sl = np.array(rows)
    sl_t      = sl[:, 0]
    sl_thrust = sl[:, 1]
    sl_mass   = sl[:, 2]
    sl_xcg    = sl[:, 3]
    sl_ai33   = sl[:, 5]

    print("\n" + "="*90)
    print("  PROPULSION — SIMULINK vs PYTHON")
    print("="*90)
    print(f"  {'t':>5}  {'thr_py':>9}  {'thr_sl':>9}  {'Δthr':>7}"
          f"  {'mass_py':>8}  {'mass_sl':>8}  {'Δmass':>7}"
          f"  {'xcg_py':>7}  {'xcg_sl':>7}  {'Δxcg':>6}")
    print("-"*90)
    all_ok = True
    for t in CHECK_TIMES:
        r    = find_row(log, t)
        tpy  = r['thrust'];  tsl  = float(np.interp(t, sl_t, sl_thrust))
        mpy  = r['mass'];    msl  = float(np.interp(t, sl_t, sl_mass))
        xpy  = r['xcg'];     xsl  = float(np.interp(t, sl_t, sl_xcg))
        dt   = abs(tpy - tsl);  ok_t = "✓" if dt < 50.0  else "✗"
        dm   = abs(mpy - msl);  ok_m = "✓" if dm < 0.1   else "✗"
        dx   = abs(xpy - xsl);  ok_x = "✓" if dx < 0.001 else "✗"
        if "✗" in (ok_t, ok_m, ok_x): all_ok = False
        print(f"  {t:>5.1f}  {tpy:>9.1f}  {tsl:>9.1f}  {dt:>6.1f}{ok_t}"
              f"  {mpy:>8.3f}  {msl:>8.3f}  {dm:>6.3f}{ok_m}"
              f"  {xpy:>7.4f}  {xsl:>7.4f}  {dx:>5.4f}{ok_x}")
    print("="*90)
    status = "TÜM KANALLAR tolerans içinde ✓  — Propulsion DOĞRULANDI" if all_ok \
             else "Bazı kanallar tolerans dışı ✗"
    print(f"  SONUÇ: {status}")
    print("="*90)


def compare_aerodynamics(log, sl_csv_path):
    """
    Aerodynamics çıkışlarını Simulink ile karşılaştır.
    CSV sütunları: t, ca, cy, cn, dna, dma, dmq, dmd, wnq, wnr
    """
    if not os.path.exists(sl_csv_path):
        print(f"\n[Simulink CSV bulunamadı: {sl_csv_path}]")
        print("MATLAB'da run.m çalıştır — phase3_aero_simulink.csv üretilecek.")
        return

    import csv as csv_mod
    with open(sl_csv_path) as f:
        rows = [[float(v) for v in row] for row in csv_mod.reader(f)]
    sl = np.array(rows)
    sl_t   = sl[:, 0]
    sl_ca  = sl[:, 1];  sl_cy  = sl[:, 2];  sl_cn  = sl[:, 3]
    sl_dna = sl[:, 4];  sl_dma = sl[:, 5]
    sl_dmq = sl[:, 6];  sl_dmd = sl[:, 7]
    sl_wnq = sl[:, 8];  sl_wnr = sl[:, 9]

    tols = {'ca':0.002, 'cn':0.002, 'dna':0.1, 'dma':0.1,
            'dmq':0.01, 'dmd':0.5,  'wnq':0.05,'wnr':0.05}

    print("\n" + "="*100)
    print("  AERODYNAMICS — SIMULINK vs PYTHON")
    print("="*100)
    print(f"  {'t':>5}  {'ca_py':>8}  {'ca_sl':>8}  {'Δca':>6}"
          f"  {'dna_py':>8}  {'dna_sl':>8}  {'Δdna':>6}"
          f"  {'dma_py':>8}  {'dma_sl':>8}  {'Δdma':>6}"
          f"  {'wnq_py':>7}  {'wnq_sl':>7}  {'Δwnq':>6}")
    print("-"*100)
    all_ok = True
    for t in CHECK_TIMES:
        r    = find_row(log, t)
        ca_py = r['ca'];   ca_sl = float(np.interp(t, sl_t, sl_ca))
        dna_py= r['dna'];  dna_sl= float(np.interp(t, sl_t, sl_dna))
        dma_py= r['dma'];  dma_sl= float(np.interp(t, sl_t, sl_dma))
        wnq_py= r['wnq'];  wnq_sl= float(np.interp(t, sl_t, sl_wnq))
        dca = abs(ca_py-ca_sl);    ok_ca  = "✓" if dca  < tols['ca']  else "✗"
        ddna= abs(dna_py-dna_sl);  ok_dna = "✓" if ddna < tols['dna'] else "✗"
        ddma= abs(dma_py-dma_sl);  ok_dma = "✓" if ddma < tols['dma'] else "✗"
        dwnq= abs(wnq_py-wnq_sl);  ok_wnq = "✓" if dwnq < tols['wnq'] else "✗"
        if "✗" in (ok_ca, ok_dna, ok_dma, ok_wnq): all_ok = False
        print(f"  {t:>5.1f}  {ca_py:>8.4f}  {ca_sl:>8.4f}  {dca:>5.4f}{ok_ca}"
              f"  {dna_py:>8.4f}  {dna_sl:>8.4f}  {ddna:>5.3f}{ok_dna}"
              f"  {dma_py:>8.4f}  {dma_sl:>8.4f}  {ddma:>5.3f}{ok_dma}"
              f"  {wnq_py:>7.4f}  {wnq_sl:>7.4f}  {dwnq:>5.4f}{ok_wnq}")
    print("="*100)
    status = "TÜM KANALLAR tolerans içinde ✓  — Aerodynamics DOĞRULANDI" if all_ok \
             else "Bazı kanallar tolerans dışı ✗"
    print(f"  SONUÇ: {status}")
    print("="*100)


def compare_with_simulink(log, sl_csv_path):
    """
    Simulink çıktısı CSV'si varsa Python ile karşılaştır.
    MATLAB tarafında şu scriptle üret:
        writematrix([t, x_sl], 'logs/phase1_simulink.csv')
    Sütunlar: t, sbel1, sbel2, sbel3, vbeb1..3, q0..3, p, q_r, r
    """
    if not os.path.exists(sl_csv_path):
        print(f"\n[Simulink CSV bulunamadı: {sl_csv_path}]")
        print("MATLAB'da şunu çalıştır:")
        print("  writematrix([t_sl, x_sl], '../logs/phase1_simulink.csv')")
        return

    import csv as csv_mod
    with open(sl_csv_path) as f:
        reader = csv_mod.reader(f)
        rows = [[float(v) for v in row] for row in reader]

    sl_data = np.array(rows)  # [N x 14]: t, sbel(3), vbeb(3), q(4), omega(3)
    sl_t     = sl_data[:, 0]
    sl_sbel  = sl_data[:, 1:4]
    sl_vbeb  = sl_data[:, 4:7]
    sl_q     = sl_data[:, 7:11]
    sl_omega = sl_data[:, 11:14]

    def interp_sl(channel_arr, t_query):
        return float(np.interp(t_query, sl_t, channel_arr))

    def sl_hbe(t):  return -interp_sl(sl_sbel[:, 2], t)
    def sl_dvbe(t): return float(np.interp(t, sl_t,
                                  np.sqrt(np.sum(sl_vbeb**2, axis=1))))

    print("\n" + "="*80)
    print("  SIMULINK vs PYTHON KARŞILAŞTIRMA")
    print("="*80)
    print(f"  {'t':>5}  {'hbe_py':>10}  {'hbe_sl':>10}  {'Δhbe':>8}  "
          f"{'dvbe_py':>9}  {'dvbe_sl':>9}  {'Δdvbe':>8}")
    print("-"*80)
    all_ok = True
    for t in CHECK_TIMES:
        r = find_row(log, t)
        py_hbe  = r['hbe']
        py_dvbe = r['dvbe']
        sl_h    = sl_hbe(t)
        sl_v    = sl_dvbe(t)
        dh      = abs(py_hbe  - sl_h)
        dv      = abs(py_dvbe - sl_v)
        ok_h    = "✓" if dh < CHANNELS['hbe']['tol']  else "✗"
        ok_v    = "✓" if dv < CHANNELS['dvbe']['tol'] else "✗"
        if "✗" in (ok_h, ok_v): all_ok = False
        print(f"  {t:>5.1f}  {py_hbe:>10.2f}  {sl_h:>10.2f}  {dh:>7.3f}{ok_h}  "
              f"{py_dvbe:>9.3f}  {sl_v:>9.3f}  {dv:>7.3f}{ok_v}")
    print("="*80)
    if all_ok:
        print("  SONUÇ: TÜM KANALLAR tolerans içinde ✓  — Faz 1 DOĞRULANDI")
    else:
        print("  SONUÇ: Bazı kanallar tolerans dışı ✗  — debug gerekli")
    print("="*80)


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    print(f"\nFaz: {PHASE.upper()}  —  DT={DT}s  T_END={T_END}s")

    if PHASE == "phase1":
        print("Koşul: FAPB=[0,0,0]  FMB=[0,0,0]  (sadece yerçekimi)")
        log = run_phase1()
        save_csv(log, 'logs/phase1_python.csv')
        print_table(log)
        compare_with_simulink(log, 'logs/phase1_simulink.csv')

    elif PHASE == "phase2":
        print("Koşul: FAPB=[0,0,0]  — Environment + Propulsion doğrulanıyor")
        log = run_phase1()
        save_csv(log, 'logs/phase1_python.csv')
        compare_environment(log, 'logs/phase2_simulink.csv')
        compare_propulsion(log, 'logs/phase3_prop_simulink.csv')

    elif PHASE == "phase3":
        print("Koşul: FAPB=[0,0,0]  — Aerodynamics katsayılar + türevler doğrulanıyor")
        log = run_phase1()
        compare_aerodynamics(log, 'logs/phase3_aero_simulink.csv')

    elif PHASE == "phase6":
        print("Koşul: maut=5 (NDI), ancomx=alcomx=0  — Control doğrulanıyor")
        log = run_phase6()
        save_csv(log, 'logs/phase6_python.csv')
        compare_control(log, 'logs/phase6_ctrl_simulink.csv')

    elif PHASE == "phase5":
        print("Koşul: dpcx=dqcx=drcx=0  — Actuator (sıfır komut) doğrulanıyor")
        log = run_phase4()
        compare_actuator(log, 'logs/phase5_act_simulink.csv')

    elif PHASE == "phase4":
        print("Koşul: maut=0, mguid=0, mprop=1  — Forces + tam yörünge doğrulanıyor")
        log = run_phase4()
        save_csv(log, 'logs/phase4_python.csv')
        print_table(log)
        compare_full(log, 'logs/phase4_simulink.csv')

    else:
        print(f"Bilinmeyen faz: {PHASE}")
        sys.exit(1)

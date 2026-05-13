function [dqcx, drcx] = control_ndi_cop( ...
    ancomx, alcomx, ...
    alphax, betax, dvbe, ...
    qqx, rrx, ...
    fspb2, fspb3, ...
    dna, dma, dmq, dmd, dnd, dyb, dlnb, dlnr, dlnd)
%CONTROL_NDI_COP  NDI autopilot reformulated about the Center of Percussion.
%
% Kaynak: missile/control.py — control_ndi_cop()  (maut=7)
%
% Standart NDI'dan tek fark dış döngüdedir:
%   Standart NDI : alpha_cmd = g*an / dna          (CG referanslı → NMP zero var)
%   NDI-CoP      : alpha_cmd = g*an / dna_cop       (CoP referanslı → NMP zero yok)
%
% CoP konumu (CG'nin önünde, +burun yönü) — m cinsinden:
%   x_cop = -dnd_rad / dmd       dnd_rad = dnd * DEG  [m/s²/deg → m/s²/rad]
%
% CoP'ta efektif kuvvet türevi:
%   dna_cop = dna + x_cop * dma
%   dyb_cop = dyb + x_cop * dlnb
%
% Orta ve iç döngüler standart NDI ile aynıdır.
%
% qqx, rrx: deg/s — kinBus'tan direkt.
%#codegen

AGRAV  = 9.80665;
DEG    = 180.0 / pi;
RAD    = pi / 180.0;
SMALL  = 1e-7;

alimit   = ALIMIT;
alplimx  = ALP_LIMX;
wblimx   = WBLIMX;
dqlimx   = DQLIMX;
drlimx   = DRLIMX;
wn_ndi_q = WN_NDI_Q;
wn_ndi_r = WN_NDI_R;
k_alpha  = K_ALPHA;
k_beta   = K_BETA;

% ── Yapısal ivme sınırlayıcı ─────────────────────────────────────────────────
aa = sqrt(ancomx^2 + alcomx^2);
if aa > alimit;  aa = alimit;  end
if abs(ancomx) < SMALL && abs(alcomx) < SMALL
    phi_c = 0.0;
else
    phi_c = atan2(ancomx, alcomx);
end
alcomx_l = aa * cos(phi_c);
ancomx_l = aa * sin(phi_c);

% ── Center of Percussion ─────────────────────────────────────────────────────
% dnd [m/s²/deg] → [m/s²/rad]: aynı fin açısı için dmd ile birim uyumu sağlanır
dnd_rad = dnd * DEG;

dmd_s  = dmd;  if abs(dmd_s)  < SMALL; dmd_s  = SMALL; end
dlnd_s = dlnd; if abs(dlnd_s) < SMALL; dlnd_s = SMALL; end

% CoP konumu ±3 m ile sınırlandırıldı (düşük q'da sayısal kararlılık)
x_cop_q = max(min(-dnd_rad / dmd_s,  3.0), -3.0);
x_cop_r = max(min(-dnd_rad / dlnd_s, 3.0), -3.0);

% CoP'ta efektif kuvvet türevleri
dna_cop = dna + x_cop_q * dma;
dyb_cop = dyb + x_cop_r * dlnb;

% ── ADIM 1: Kuvvet inversiyonu (CoP referanslı — NMP free) ───────────────────
dna_s = dna_cop; if abs(dna_s) < SMALL; dna_s = SMALL; end
dyb_s = dyb_cop; if abs(dyb_s) < SMALL; dyb_s = SMALL; end

alpha_cmd = AGRAV * ancomx_l / dna_s;
beta_cmd  = AGRAV * alcomx_l / dyb_s;

alp_lim   = alplimx * RAD;
alpha_cmd = max(min(alpha_cmd, alp_lim), -alp_lim);
beta_cmd  = max(min(beta_cmd,  alp_lim), -alp_lim);

% ── ADIM 2: Kinematik inversiyon ──────────────────────────────────────────────
% α̇ = q − (g/V)·aₙ_CG  kimliği CG referanslıdır — an_meas (CG) doğrudur.
alpha   = alphax * RAD;
beta    = betax  * RAD;
qq      = qqx * RAD;    % deg/s → rad/s
rr      = rrx * RAD;
V_s     = max(dvbe, 30.0);
an_meas = -fspb3 / AGRAV;
al_meas =  fspb2 / AGRAV;

q_cmd = k_alpha * (alpha_cmd - alpha) + AGRAV * an_meas / V_s;
r_cmd = k_beta  * (beta - beta_cmd)   + AGRAV * al_meas / V_s;

rate_lim = wblimx * RAD;
q_cmd = max(min(q_cmd, rate_lim), -rate_lim);
r_cmd = max(min(r_cmd, rate_lim), -rate_lim);

% ── ADIM 3: Moment inversiyonu (standart NDI ile aynı) ───────────────────────
q_dot_des   = wn_ndi_q * (q_cmd - qq);
r_dot_des   = wn_ndi_r * (r_cmd - rr);
q_dot_model = dma * alpha + dmq * qq;
r_dot_model = dlnb * beta + dlnr * rr;

dqc  = (q_dot_des - q_dot_model) / dmd_s;
drc  = (r_dot_des - r_dot_model) / dlnd_s;

dqcx = max(min(dqc * DEG, dqlimx), -dqlimx);
drcx = max(min(drc * DEG, drlimx), -drlimx);
end

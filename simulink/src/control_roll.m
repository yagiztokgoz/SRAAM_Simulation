function dpcx = control_roll(pdynmc, phiblx, ppx, dlp, dld)
% ppx: roll hızı deg/s (kinBus'tan direkt gelir)
%CONTROL_ROLL  Yuvarlanma pozisyon kontrolcüsü.
% Kaynak: missile/control.py — control_roll()
%#codegen

RAD   = pi / 180.0;
DEG   = 180.0 / pi;
SMALL = 1e-7;

zrcl     = ZRCL;
factwrcl = FACT_WRCL;
dplimx   = DPLIMX;
phicomx  = 0.0;   % banka açısı komutu — Guidance eklenince değişir

% Değişken roll bant genişliği
wrcl  = (0.00024 * pdynmc + 10.0) * (1.0 + factwrcl);

% Kazançlar
dld_s = dld; if abs(dld_s) < SMALL; dld_s = SMALL; end
gkp   = (2.0 * zrcl * wrcl + dlp) / dld_s;
gkphi = wrcl * wrcl / dld_s;

% Roll pozisyon komutu (pp = ppx*RAD, gkp pp üzerinden çalışır)
pp   = ppx * RAD;
dpc  = gkphi * (phicomx - phiblx) * RAD - gkp * pp;
dpcx = max(min(dpc * DEG, dplimx), -dplimx);
end

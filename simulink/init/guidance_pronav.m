function [ancomx, alcomx, dtbc, tgoc] = guidance_pronav( ...
    STEL, VTEL, SBELC, VBELC, TBL)
%GUIDANCE_PRONAV  Orantılı navigasyon güdümü (midcourse pronav).
% Kaynak: missile/guidance.py — _pronav_mid()
%
% Girişler:
%   STEL  [3x1] : hedef konumu NED - m
%   VTEL  [3x1] : hedef hızı NED - m/s
%   SBELC [3x1] : füze konumu (INS tahmini) NED - m
%   VBELC [3x1] : füze hızı (INS tahmini) NED - m/s
%   TBL   [3x3] : gövde-yerel DCM (INS tahmini)
%
% Çıkışlar:
%   ancomx : normal ivme komutu - g
%   alcomx : yanal ivme komutu - g
%   dtbc   : hedefe mesafe - m
%   tgoc   : tahmini uçuş süresi - s
%#codegen

AGRAV = 9.80665;
SMALL = 1e-7;
gnav  = GNAV;     % workspace parametresi
alimit = ALIMIT;  % workspace parametresi

ancomx = 0.0;
alcomx = 0.0;
dtbc   = 0.0;
tgoc   = 0.0;

% ── Hedef-füze görüş vektörü ──────────────────────────────────────────────────
STBLC = STEL - SBELC;
dtbc  = norm(STBLC);

if dtbc < SMALL
    return
end

UTBLC = STBLC / dtbc;

% ── Görüş açısı (diagnostik) ─────────────────────────────────────────────────
% UTBBC = TBL * UTBLC;  (ileride kullanılabilir)

% ── Göreceli hız (hedef - füze) ───────────────────────────────────────────────
VTBLC = VTEL - VBELC;

% ── Kapanma hızı ─────────────────────────────────────────────────────────────
dvtbc = abs(dot(UTBLC, VTBLC));
if dvtbc < SMALL; dvtbc = SMALL; end

% ── Tahmini uçuş süresi ───────────────────────────────────────────────────────
tgoc = dtbc / dvtbc;

% ── Eylemsiz LOS açısal hızı ─────────────────────────────────────────────────
WOELC = cross(UTBLC, VTBLC) / dtbc;

% ── ProNav ivme komutu (gövde ekseni) - g ────────────────────────────────────
ACBX = TBL * cross(WOELC, UTBLC) * gnav * dvtbc / AGRAV;

% ── Yanal ve normal bileşenler ────────────────────────────────────────────────
all_acc =  ACBX(2);
ann_acc = -ACBX(3);

% ── Dairesel sınırlayıcı ─────────────────────────────────────────────────────
aa = sqrt(all_acc^2 + ann_acc^2);
if aa > alimit;  aa = alimit;  end
if abs(ann_acc) < SMALL && abs(all_acc) < SMALL
    phi = 0.0;
else
    phi = atan2(ann_acc, all_acc);
end

alcomx = aa * cos(phi);
ancomx = aa * sin(phi);
end

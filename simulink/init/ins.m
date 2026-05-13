function [SBELC, VBELC, TBLC, ...
          ppxc, qqxc, rrxc, fspb2c, fspb3c, ...
          alphaxc, betaxc, dvbec, hbem, ...
          thtblcx, phiblcx, psivlcx, thtvlcx, ...
          RECED, EVBED, ESTTCD] = ins( ...
    TBL, ppx, qqx, rrx, ...
    SBEL, VBEL, dvbe, hbe, fspb1, fspb2, fspb3, ...
    RECE, EVBE, ESTTC, noise_g, noise_a, noise_h, ...
    EBIASG, EBIASA, ESCALG, EMISG, EMISA, EUNBG, BIASAL, ESCALA, MINS)
%INS  Inertial Navigation System — MINS=0:ideal, MINS=1:gercek IMU.
%   Kaynak: missile/ins.py
%#codegen

DEG   = 180.0 / pi;
RAD   = pi / 180.0;
SMALL = 1e-8;

% ── Tüm çıkışları önceden boyutlandır (Simulink derleme zorunluluğu) ──────────
SBELC   = zeros(3,1);
VBELC   = zeros(3,1);
TBLC    = zeros(3,3);
ppxc    = 0.0;
qqxc    = 0.0;
rrxc    = 0.0;
fspb2c  = 0.0;
fspb3c  = 0.0;
alphaxc = 0.0;
betaxc  = 0.0;
dvbec   = 0.0;
hbem    = 0.0;
thtblcx = 0.0;
phiblcx = 0.0;
psivlcx = 0.0;
thtvlcx = 0.0;
RECED   = zeros(3,1);
EVBED   = zeros(3,1);
ESTTCD  = zeros(3,1);

% ── Gerçek gövde vektörleri ───────────────────────────────────────────────────
WBEB = [ppx; qqx; rrx] * RAD;
FSPB = [fspb1; fspb2; fspb3];

% ── Ideal başlangıç değerleri (MINS=0 için doğrudan kullanılır) ───────────────
SBELC  = [SBEL(1); SBEL(2); SBEL(3)];
VBELC  = [VBEL(1); VBEL(2); VBEL(3)];
TBLC   = TBL;
WBECB  = [WBEB(1); WBEB(2); WBEB(3)];
FSPCB  = [FSPB(1); FSPB(2); FSPB(3)];
dvbec  = dvbe;
hbem   = hbe;

if MINS == 1

    % ── Jiroskop hata modeli ──────────────────────────────────────────────────
    EGB   = diag(ESCALG) + skew3(EMISG);
    EWBEB = EBIASG + EGB * WBEB + EUNBG .* FSPB + noise_g;
    WBECB = WBEB + EWBEB;

    % ── İvmeölçer hata modeli ─────────────────────────────────────────────────
    EAB   = diag(ESCALA) + skew3(EMISA);
    EFSPB = EBIASA + EAB * FSPB;
    FSPCB = FSPB + EFSPB + noise_a;

    % ── Eğim hatası dinamiği: dRECE/dt = TLB * EWBEB ─────────────────────────
    RECED = TBL' * EWBEB;

    % INS DCM: TBLC ≈ TBL * (I + skew(RECE))
    RERE = skew3(RECE);
    TBLC = TBL * (eye(3) + RERE);

    % ── Hız hatası dinamiği ───────────────────────────────────────────────────
    TLCB  = TBLC';
    EVBED = TLCB * EFSPB - RERE * TLCB * FSPCB;

    % ── Konum hatası dinamiği: dESTTC/dt = EVBE ──────────────────────────────
    ESTTCD = EVBE;

    % ── Tahmin birleştirme ────────────────────────────────────────────────────
    SBELC = ESTTC + SBEL;
    VBELC = EVBE  + VBEL;
    dvbec = norm(VBELC);
    hbem  = hbe + BIASAL + noise_h;

end

% ── Türetilmiş tahmin nicelikleri (her iki modda ortak) ───────────────────────
v1 = VBELC(1);  v2 = VBELC(2);  v3 = VBELC(3);
if abs(v1) < SMALL && abs(v2) < SMALL
    psivlc = 0.0;
    thtvlc = 0.0;
else
    psivlc = atan2(v2, v1);
    thtvlc = atan2(-v3, sqrt(v1*v1 + v2*v2));
end

VBEBC  = TBLC * VBELC;
vb1 = VBEBC(1);  vb2 = VBEBC(2);  vb3 = VBEBC(3);
dvbebc = sqrt(vb1*vb1 + vb2*vb2 + vb3*vb3);
if dvbebc < 1e-6;  dvbebc = 1e-6;  end

alphac = atan2(vb3, vb1);
betac  = asin(max(-1.0, min(1.0, vb2 / dvbebc)));

tblc13 = TBLC(1,3);
if abs(tblc13) < 1.0
    thtblc = asin(-tblc13);
elseif tblc13 > 0.0
    thtblc = -pi / 2.0;
else
    thtblc =  pi / 2.0;
end
phiblc = atan2(TBLC(2,3), TBLC(3,3));

% ── Çıkış atamaları ──────────────────────────────────────────────────────────
ppxc    = WBECB(1) * DEG;
qqxc    = WBECB(2) * DEG;
rrxc    = WBECB(3) * DEG;
fspb2c  = FSPCB(2);
fspb3c  = FSPCB(3);
alphaxc = alphac * DEG;
betaxc  = betac  * DEG;
thtblcx = thtblc * DEG;
phiblcx = phiblc * DEG;
psivlcx = psivlc * DEG;
thtvlcx = thtvlc * DEG;

end

function S = skew3(v)
S = [  0.0,  -v(3),  v(2); ...
      v(3),   0.0,  -v(1); ...
     -v(2),   v(1),  0.0 ];
end

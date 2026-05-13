function [TBL, alpha_deg, beta_deg, alpp_deg, dvbe, hbe, ...
          psiblx, thtblx, phiblx, VBEL, psivlx, thtvlx, anx, ayx, ...
          ppx, qqx, rrx, SBEL] = ...
          state_to_outputs(x)
%STATE_TO_OUTPUTS  x[13] vektöründen ikincil çıkışları hesaplar.
% Algebraic — integratör yok. Simulink MATLAB Function bloğuna koy.
%
% Kaynak: kinematics.py:144-235, newton.py:107-157
%#codegen

EPS   = 1e-10;
DEG   = 180.0 / pi;
SMALL = 1e-7;

% Durumu aç
SBEL = x(1:3);
VBEB = x(4:6);
q    = x(7:10);
om   = x(11:13);

q0=q(1); q1=q(2); q2=q(3); q3=q(4);

% ── DCM ──────────────────────────────────────────────────────────────────────
TBL = [ q0^2+q1^2-q2^2-q3^2,  2*(q1*q2+q0*q3),  2*(q1*q3-q0*q2);
        2*(q1*q2-q0*q3),  q0^2-q1^2+q2^2-q3^2,  2*(q2*q3+q0*q1);
        2*(q1*q3+q0*q2),  2*(q2*q3-q0*q1),  q0^2-q1^2-q2^2+q3^2];

% ── Euler Açıları (kinematics.py:159-185) ────────────────────────────────────
tbl13 = TBL(1,3);
if abs(tbl13) < 1.0
    thtbl = asin(-tbl13);
else
    thtbl = pi/2 * sign(-tbl13);
end
cthtbl = max(cos(thtbl), EPS);

cpsi = TBL(1,1) / cthtbl;
cpsi = max(min(cpsi, 1-EPS), -(1-EPS));
cphi = TBL(3,3) / cthtbl;
cphi = max(min(cphi, 1-EPS), -(1-EPS));

psibl = acos(cpsi) * sign(TBL(1,2));
phibl = acos(cphi) * sign(TBL(2,3));

psiblx = psibl * DEG;
thtblx = thtbl * DEG;
phiblx = phibl * DEG;

% ── NED Hızı ve Hız ──────────────────────────────────────────────────────────
VBEL = TBL' * VBEB;
vbel1 = VBEL(1);  vbel2 = VBEL(2);  vbel3 = VBEL(3);
dvbe  = norm(VBEL);
if dvbe < EPS; dvbe = EPS; end

% ── Uçuş yolu açıları ────────────────────────────────────────────────────────
if abs(vbel1) < SMALL && abs(vbel2) < SMALL
    psivlx = 0.0;
else
    psivlx = atan2(vbel2, vbel1) * DEG;
end
thtvlx = atan2(-vbel3, sqrt(vbel1^2 + vbel2^2)) * DEG;

% ── İrtifa ────────────────────────────────────────────────────────────────────
hbe = -SBEL(3);

% ── Hücum ve kayma açıları (kinematics.py:187-215) ───────────────────────────
vbeb1 = VBEB(1);  vbeb2 = VBEB(2);  vbeb3 = VBEB(3);

alpha_rad = atan2(vbeb3, vbeb1);
beta_rad  = asin(max(-1.0, min(1.0, vbeb2 / dvbe)));

dum = vbeb1 / dvbe;
dum = max(min(dum, 1-EPS), -(1-EPS));
alpp_rad = acos(dum);

alpha_deg = alpha_rad * DEG;
beta_deg  = beta_rad  * DEG;
alpp_deg  = alpp_rad  * DEG;

% ── Diagnostic ivmeler g cinsinden (newton.py:127-128) ───────────────────────
FSPB = zeros(3,1);   % çağıran blok FAPB/mass'i geçirecek — burada placeholder
% anx/ayx: Forces bloğu bağlanınca güncellenecek
anx = 0.0;
ayx = 0.0;

% ── Açısal hızlar (x(11:13) rad/s → deg/s) ───────────────────────────────────
ppx = x(11) * DEG;
qqx = x(12) * DEG;
rrx = x(13) * DEG;

% ── Konum (Guidance için) ─────────────────────────────────────────────────────
SBEL = x(1:3);
end

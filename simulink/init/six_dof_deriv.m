function xdot = six_dof_deriv(x, FAPB, FMB, mass, grav, ai11, ai33)
%SIX_DOF_DERIV  6-DoF durum türev fonksiyonu — Simulink MATLAB Function bloğuna koy.
%
% Girişler:
%   x     [13x1]  Durum vektörü: [SBEL(3); VBEB(3); q(4); omega(3)]
%   FAPB  [3x1]   Yerçekimi dışı toplam kuvvet, gövde ekseni - N
%   FMB   [3x1]   Toplam moment, gövde ekseni - N·m
%   mass  [1]     Araç kütlesi - kg
%   grav  [1]     Yerçekimi ivmesi - m/s²
%   ai11  [1]     Yuvarlanma atalet momenti - kg·m²
%   ai33  [1]     Yunuslama/yalpalama atalet momenti - kg·m²
%
% Çıkışlar:
%   xdot  [13x1]  Durum türevi: dx/dt
%
% Kaynak Python modülleri:
%   newton.py  : konum ve hız türevleri (satır 84-104)
%   kinematics.py : quaternion türevi (satır 127-135)
%   euler.py   : açısal hız türevi (satır 45-51)
%
%#codegen

xdot = zeros(13, 1);   % boyut bildirimi — Simulink codegen için gerekli

% ── Durumu aç ────────────────────────────────────────────────────────────────
SBEL = x(1:3);   % konum NED - m         (kullanılmıyor, sadece türevi var)
VBEB = x(4:6);   % hız gövde ekseni - m/s
q    = x(7:10);  % quaternion [q0;q1;q2;q3]
om   = x(11:13); % açısal hız [p;q;r] - rad/s

pp = om(1);  qq_r = om(2);  rr = om(3);

% ── Quaternion → DCM (kinematics.py:145-149) ─────────────────────────────────
q0 = q(1);  q1 = q(2);  q2 = q(3);  q3 = q(4);
TBL = [ q0^2+q1^2-q2^2-q3^2,  2*(q1*q2+q0*q3),  2*(q1*q3-q0*q2);
        2*(q1*q2-q0*q3),  q0^2-q1^2+q2^2-q3^2,  2*(q2*q3+q0*q1);
        2*(q1*q3+q0*q2),  2*(q2*q3-q0*q1),  q0^2-q1^2-q2^2+q3^2];

% ── Konum türevi: dSBEL/dt = TBL' * VBEB (newton.py:101) ────────────────────
dSBEL = TBL' * VBEB;

% ── Hız türevi: Newton 2. kanun dönen çerçevede (newton.py:84-94) ────────────
FSPB  = FAPB / mass;                       % spesifik kuvvet - m/s²
ATB   = cross(om, VBEB);                   % Coriolis: ω × V
dVBEB = FSPB - ATB + TBL * [0; 0; grav];  % grav NED aşağı pozitif

% ── Quaternion türevi (kinematics.py:127-135) ─────────────────────────────────
ck = 50.0;                                 % ortogonalite düzeltme kazancı
ortho_err = 1.0 - dot(q, q);
Omega = [  0,   -pp, -qq_r, -rr;
           pp,   0,   rr,  -qq_r;
           qq_r, -rr,  0,    pp;
           rr,   qq_r, -pp,   0 ];
dq = 0.5 * Omega * q + ck * ortho_err * q;

% ── Açısal hız türevi: Euler denklemleri (euler.py:45-47) ────────────────────
dpp = FMB(1) / ai11;
dqq = ((ai33 - ai11) * pp * rr + FMB(2)) / ai33;
drr = (-(ai33 - ai11) * pp * qq_r + FMB(3)) / ai33;

% ── Paketle ───────────────────────────────────────────────────────────────────
xdot = [dSBEL; dVBEB; dq; dpp; dqq; drr];
end

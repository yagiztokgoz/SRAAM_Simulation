function [xdot, dpx, dqx, drx, del1, del2, del3, del4] = ...
         actuator_deriv(x_act, dpcx, dqcx, drcx)
%ACTUATOR_DERIV  2. mertebe fin aktüatör dinamikleri.
% Kaynak: missile/actuator.py — actuator_scnd()
%
% Durum vektörü x_act [8x1]:
%   x_act(1:4)  = [del1, del2, del3, del4]   fin konumları - deg
%   x_act(5:8)  = [del1d, del2d, del3d, del4d] fin hızları - deg/s
%
% Girişler: dpcx, dqcx, drcx — kontrolcü komutları - deg
% Çıkışlar: xdot[8], dpx, dqx, drx — gerçek defleksiyonlar - deg
%#codegen

xdot = zeros(8, 1);   % boyut bildirimi

wnact  = WNACT;    % workspace parametresi - rad/s
zetact = ZETACT;   % workspace parametresi
dlimx  = DLIMX;    % konum sınırı - deg
ddlimx = DDLIMX;   % hız sınırı - deg/s

wn2 = wnact * wnact;

% ── Komut karışımı (actuator.py:63-66) ───────────────────────────────────────
delcx1 = -dpcx + dqcx - drcx;
delcx2 = -dpcx + dqcx + drcx;
delcx3 = +dpcx + dqcx - drcx;
delcx4 = +dpcx + dqcx + drcx;

delcx = [delcx1; delcx2; delcx3; delcx4];

% ── Durum ─────────────────────────────────────────────────────────────────────
del  = x_act(1:4);    % konum - deg
deld = x_act(5:8);    % hız - deg/s

% ── 2. mertebe ODE türevleri (actuator.py:134) ───────────────────────────────
deldd = wn2 * (delcx - del) - 2.0 * zetact * wnact * deld;

% ── Hız sınırlayıcı (actuator.py:141-142) ────────────────────────────────────
% Not: Simulink integratörü sınırla → Rate Limiter bloğuyla veya
%      türevi burada klip yap (yaklaşık)
deld_lim = max(min(deld, ddlimx), -ddlimx);

% ── Türev vektörü ─────────────────────────────────────────────────────────────
% dkonum/dt = hız (sınırlı)
% dhız/dt   = ikinci türev
xdot = [deld_lim; deldd];

% ── Konum sınırlayıcı (çıkış klibi) ─────────────────────────────────────────
del_out = max(min(del, dlimx), -dlimx);

% ── Geri dönüşüm: fin defleksiyon → roll/pitch/yaw (actuator.py:87-89) ───────
dpx = (-del_out(1) - del_out(2) + del_out(3) + del_out(4)) / 4.0;
dqx = (+del_out(1) + del_out(2) + del_out(3) + del_out(4)) / 4.0;
drx = (-del_out(1) + del_out(2) - del_out(3) + del_out(4)) / 4.0;

del1 = del_out(1);
del2 = del_out(2);
del3 = del_out(3);
del4 = del_out(4);
end

function acc = target_accel(t, pos, vel, scenario)
%TARGET_ACCEL  Hedef ivme vektörü — 10 senaryo.
% Kaynak: missile/target.py — Target._accel()
%
% Girişler:
%   t        : simülasyon zamanı - s
%   pos      : hedef konumu NED - m  (kullanılmıyor, ileriki senaryolar için)
%   vel      : hedef hızı NED - m/s
%   scenario : senaryo numarası (1-10)
%
% Çıkış:
%   acc [3x1]: hedef ivmesi NED - m/s²
%#codegen

G = 9.80665;
acc = zeros(3,1);

vn = vel(1);  ve = vel(2);  vd = vel(3);

% ── Yardımcı: saat yönünde dik vektör (yatay düzlemde) ───────────────────────
vm = sqrt(vn^2 + ve^2);
if vm < 1.0
    cw_n = 0.0;  cw_e = 0.0;
else
    cw_n =  ve / vm;
    cw_e = -vn / vm;
end

if scenario == 1
    % S1: Sabit hız, düz uçuş
    acc = zeros(3,1);

elseif scenario == 2
    % S2: t>1.5s'de 5g sağ kırılma (santripetal)
    if t >= 1.5
        acc = 5.0 * G * [cw_n; cw_e; 0.0];
    end

elseif scenario == 3
    % S3: Sinüsoidal manevra (3g yanal + 2g dikey)
    acc = [0.0; 3.0*G*sin(1.2*t); -2.0*G*cos(1.8*t + pi/3)];

elseif scenario == 4
    % S4: t>2s 8g sağ kırılma, t>3.5s 6g yukarı çekme
    if t >= 4.8
        acc = zeros(3,1);
    elseif t >= 3.5
        acc = [0.0; 0.0; -6.0*G];
    elseif t >= 2.0
        acc = 8.0 * G * [cw_n; cw_e; 0.0];
    end

elseif scenario == 5
    % S5: Sabit hız (beaming)
    acc = zeros(3,1);

elseif scenario == 6
    % S6: t>3s 7g sağ kırılma, t>8s düzleşme
    if t >= 8.0
        acc = zeros(3,1);
    elseif t >= 3.0
        acc = 7.0 * G * [cw_n; cw_e; 0.0];
    end

elseif scenario == 7
    % S7: Sabit hız (look-down)
    acc = zeros(3,1);

elseif scenario == 8
    % S8: t>4s ~9g kombine kırılma
    if t >= 4.0
        acc = [0.0; 6.364*G; -6.364*G];
    end

elseif scenario == 9
    % S9: Afterburner tırmanma (~3g)
    vtot = norm(vel);
    if vtot > 1.0
        vhat = vel / vtot;
        a_drag = -0.7 * G * vhat;
        up = [0.0; 0.0; -1.0];
        perp = up - dot(up, vhat) * vhat;
        pm = norm(perp);
        if pm > 1e-6
            a_lift = 3.0 * G * perp / pm;
        else
            a_lift = zeros(3,1);
        end
        acc = a_drag + a_lift;
    end

elseif scenario == 10
    % S10: 4g koordineli dönüş
    vm2 = sqrt(vn^2 + ve^2);
    if vm2 > 0.1
        ax = -vel(2) / vm2 * 4.0 * G;
        ay =  vel(1) / vm2 * 4.0 * G;
        acc = [ax; ay; 0.0];
    end

elseif scenario == 11
    % S11: 12km + t>7s geç 6g sağ kırılma
    if t >= 7.0
        acc = 6.0 * G * [cw_n; cw_e; 0.0];
    end

elseif scenario == 12
    % S12: 15km + t>5s sinüsoidal jink (3g yanal + 2g dikey)
    if t >= 5.0
        tau = t - 5.0;
        acc = [0.0; 3.0*G*sin(1.0*tau); -2.0*G*cos(1.5*tau)];
    end

elseif scenario == 13
    % S13: Tail-chase + t>4s 7g yukarı + t>6s 7g sağ kırılma
    if t >= 6.0
        acc = 7.0 * G * [cw_n; cw_e; 0.0];
    elseif t >= 4.0
        acc = [0.0; 0.0; -7.0*G];
    end

elseif scenario == 14
    % S14: Uzun beam + t>6s 8g sharp break
    if t >= 6.0
        acc = 8.0 * G * [cw_n; cw_e; 0.0];
    end
end
end

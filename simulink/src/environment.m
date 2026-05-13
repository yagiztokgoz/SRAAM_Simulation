function [rho, press, tempk, vsound, grav, mach, pdynmc] = environment(hbe, dvbe)
%ENVIRONMENT  US 1976 Standart Atmosfer + yerçekimi + Mach + dinamik basınç.
% Kaynak: missile/environment.py
%#codegen

% Fiziksel sabitler (environment.py:4-8)
REARTH     = 6.3781e6;
G_CONST    = 6.674e-11;
EARTH_MASS = 5.9722e24;
R_AIR      = 287.058;
G0         = 9.80665;

h = max(hbe, 0.0);

% ── US76 Standart Atmosfer (environment.py:18-57) ────────────────────────────
if h <= 11000.0
    T = 288.15 - 6.5e-3 * h;
    P = 101325.0 * (T / 288.15)^5.25588;

elseif h <= 20000.0
    T = 216.65;
    P = 22632.1 * exp(-G0 / (R_AIR * T) * (h - 11000.0));

elseif h <= 32000.0
    T = 216.65 + 1.0e-3 * (h - 20000.0);
    P = 5474.89 * (T / 216.65)^(-G0 / (R_AIR * 1.0e-3));

elseif h <= 47000.0
    T = 228.65 + 2.8e-3 * (h - 32000.0);
    P = 868.019 * (T / 228.65)^(-G0 / (R_AIR * 2.8e-3));

elseif h <= 51000.0
    T = 270.65;
    P = 110.906 * exp(-G0 / (R_AIR * T) * (h - 47000.0));

elseif h <= 71000.0
    T = 270.65 - 2.8e-3 * (h - 51000.0);
    P = 66.9389 * (T / 270.65)^(G0 / (R_AIR * 2.8e-3));

else
    T = 214.65 - 2.0e-3 * (h - 71000.0);
    P = 3.95642 * (T / 214.65)^(G0 / (R_AIR * 2.0e-3));
end

rho    = P / (R_AIR * T);
press  = P;
tempk  = T;

% ── Ses hızı, Mach, dinamik basınç ───────────────────────────────────────────
vsound = sqrt(1.4 * R_AIR * T);
mach   = abs(dvbe / vsound);
pdynmc = 0.5 * rho * dvbe * dvbe;

% ── Yerçekimi (ters kare kanunu) ─────────────────────────────────────────────
rad  = REARTH + hbe;
grav = G_CONST * EARTH_MASS / (rad * rad);
end

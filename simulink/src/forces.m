function [FAPB, FMB, fspb1, fspb2, fspb3] = forces(ca, cy, cn, cll, clm, cln, thrust, pdynmc, mass)
%FORCES  Toplam kuvvet ve moment vektörleri — gövde ekseni.
% Kaynak: missile/forces.py
%#codegen

refa = REFA;   % workspace parametresi
refl = REFL;   % workspace parametresi

% Aerodinamik kuvvetler + itki (forces.py:33-45)
FAPB = [-pdynmc * refa * ca + thrust;
         pdynmc * refa * cy;
        -pdynmc * refa * cn];

% Aerodinamik momentler (forces.py:51-55)
FMB = pdynmc * refa * refl * [cll; clm; cln];

% Spesifik kuvvet (newton.py: FSPB = FAPB/mass)
fspb1 = FAPB(1) / mass;    % eksenel   (INS ivmeölçer modeli için)
fspb2 = FAPB(2) / mass;    % yanal     (control_ndi için)
fspb3 = FAPB(3) / mass;    % normal    (control_ndi için)
end

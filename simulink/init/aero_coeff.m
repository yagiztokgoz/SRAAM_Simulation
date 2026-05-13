function [ca, cy, cn, cll, clm, cln, ...
          dna, dma, dmq, dmd, dnd, ...
          dyb, dlnb, dlnr, dlnd, dlp, dld, ...
          wnq, wnr, zetq, zetr] = ...
    aero_coeff( ...
        ca0, caa, cad, ca0b, ...
        cy0, cydr, ...
        cn0, cndq, ...
        cll0, cllp, clldp, ...
        clm0, clmq, clmdq, ...
        cln0, clnr, clndr, ...
        cn0p, cn0m, clm0p, clm0m, ...
        cy0p, cy0m, cln0p, cln0m, ...
        alpp, alpm, betp, betm, ...
        deff, ...
        dpx, dqx, drx, ppx, qqx, rrx, ...
        dvbe, pdynmc, mass, xcg, ai11, ai33, mprop, thrust)
%AERO_COEFF  Katsayılar + boyutsal türevler — tüm girişler LUT çıkışı skalardır.
% Kaynak: missile/aerodynamics.py
%#codegen

SMALL  = 1e-7;
DEG    = 180.0 / pi;
RAD    = pi / 180.0;

% Workspace parametreleri (Symbols → Parameter)
refl   = REFL;
refa   = REFA;
xcgref = XCG_REF;

% ── Eksenel kuvvet ────────────────────────────────────────────────────────────
ca  = ca0 + caa + cad * deff^2;
if thrust > 0;  ca = ca + ca0b;  end  % Python: if tsl != 0.0

% ── Yan ve normal kuvvetler ───────────────────────────────────────────────────
cy  = cy0  + cydr  * drx;
cn  = cn0  + cndq  * dqx;

% ── Momentler ─────────────────────────────────────────────────────────────────
cll = cllp * RAD * ppx * refl / (2.0*dvbe) + cll0 + clldp * dpx;
clm = clm0 + clmq * RAD * qqx * refl / (2.0*dvbe) + clmdq * dqx ...
      - cn/refl * (xcgref - xcg);
cln = cln0 + clnr * RAD * rrx * refl / (2.0*dvbe) + clndr * drx ...
      - cy/refl * (xcgref - xcg);

% ── ND türevler ───────────────────────────────────────────────────────────────
da  = alpp - alpm;
cna  = DEG * (cn0p  - cn0m)  / (da + SMALL);
clma = DEG * (clm0p - clm0m) / (da + SMALL) - cna/refl*(xcgref-xcg);

db   = betp - betm;
cyb  = DEG * (cy0p  - cy0m)  / (db + SMALL);
clnb = DEG * (cln0p - cln0m) / (db + SMALL) - cyb/refl*(xcgref-xcg);

% ── Boyutsal türevler ─────────────────────────────────────────────────────────
dna  = (pdynmc*refa/mass)                           * cna;
dma  = (pdynmc*refa*refl/ai33)                      * clma;
dmq  = DEG*(pdynmc*refa*refl/ai33)*(refl/(2*dvbe))  * clmq * RAD;
dmd  = DEG*(pdynmc*refa*refl/ai33)                  * clmdq;
dnd  = (pdynmc*refa/mass)                           * cndq;
dyb  = (pdynmc*refa/mass)                           * cyb;
dlnb = (pdynmc*refa*refl/ai33)                      * clnb;
dlnr = DEG*(pdynmc*refa*refl/ai33)*(refl/(2*dvbe))  * clnr * RAD;
dlnd = DEG*(pdynmc*refa*refl/ai33)                  * clndr;
dlp  = DEG*(pdynmc*refa*refl/ai11)*(refl/(2*dvbe))  * cllp * RAD;
dld  = DEG*(pdynmc*refa*refl/ai11)                  * clldp;

% ── Dinamik kökler ────────────────────────────────────────────────────────────
a11q = dmq;  a12q = dma/(dna+SMALL);
a21q = dna;  a22q = -dna/(dvbe+SMALL);
argq = (a11q+a22q)^2 - 4*(a11q*a22q - a12q*a21q);
if argq >= 0;  wnq=0.0; zetq=0.0;
else;  wnq=sqrt(-(argq)/4.0 + ((a11q+a22q)/2)^2 - (a11q+a22q)^2/4);
       wnq=sqrt(max(a11q*a22q - a12q*a21q, 0));
       zetq=-(a11q+a22q)/(2*max(wnq,SMALL)); end

a11r = dlnr;  a12r = dlnb/(dyb+SMALL);
a21r = -dyb;  a22r = dyb/(dvbe+SMALL);
argr = (a11r+a22r)^2 - 4*(a11r*a22r - a12r*a21r);
if argr >= 0;  wnr=0.0; zetr=0.0;
else;  wnr=sqrt(max(a11r*a22r - a12r*a21r, 0));
       zetr=-(a11r+a22r)/(2*max(wnr,SMALL)); end

end

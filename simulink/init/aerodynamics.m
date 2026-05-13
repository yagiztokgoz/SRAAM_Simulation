function [ca, cy, cn, cll, clm, cln, ...
          dna, dma, dmq, dmd, dnd, dyb, dlnb, dlnr, dlnd, dlp, dld, ...
          wnq, wnr, zetq, zetr] = ...
    aerodynamics(mach, alphax, betax, pdynmc, dvbe, ...
                 dpx, dqx, drx, ppx, qqx, rrx, ...
                 mass, xcg, ai11, ai33, mprop)
%AERODYNAMICS  Aerodinamik katsayılar + boyutsal türevler.
% Kaynak: missile/aerodynamics.py
%
% Girişler (sinyal portları — 16):
%   mach, alphax, betax, pdynmc, dvbe
%   dpx, dqx, drx, ppx, qqx, rrx
%   mass, xcg, ai11, ai33, mprop
%
% LUT verileri ve sabitler workspace'den parametre olarak okunur
% (MATLAB Function bloğunda Symbols → scope: Parameter)

% ── Workspace parametreleri ───────────────────────────────────────────────────
% Bu değişkenler MATLAB Function bloğu Symbols panelindan "Parameter" olarak
% tanımlanmalı — SRAAM6_params çalıştırıldığında workspace'den okunur.
xcgref = XCG_REF;
refl   = REFL;
refa   = REFA;
alplimx = ALP_LIMX;

coder.extrinsic('interp1', 'interpn');

DEG   = 180.0 / pi;
RAD   = pi / 180.0;
SMALL = 1e-7;

% ── 1D interpolasyon (Mach tabanlı) ──────────────────────────────────────────
ca0_v   = interp1(mach_bp, ca0_vs_mach,  mach, 'linear', 'extrap');
cad_v   = interp1(mach_bp, cad_vs_mach,  mach, 'linear', 'extrap');
cllp_v  = interp1(mach_bp, cllp_vs_mach, mach, 'linear', 'extrap') * RAD;
clmq_v  = interp1(mach_bp, clmq_vs_mach, mach, 'linear', 'extrap') * RAD;
clnr_v  = interp1(mach_bp, clnr_vs_mach, mach, 'linear', 'extrap') * RAD;
ca0b_v  = interp1(mach_bp, ca0b_vs_mach, mach, 'linear', 'extrap');

% ── 3D interpolasyon (Mach, Alpha, Beta) ─────────────────────────────────────
ac = max(min(alphax, alpha_bp(end)), alpha_bp(1));
bc = max(min(betax,  beta_bp(end)),  beta_bp(1));
mc = max(min(mach,   mach_bp(end)),  mach_bp(1));

caa_v   = interpn(mach_bp,alpha_bp,beta_bp, caa_table,   mc,ac,bc,'linear');
cy0_v   = interpn(mach_bp,alpha_bp,beta_bp, cy0_table,   mc,ac,bc,'linear');
cydr_v  = interpn(mach_bp,alpha_bp,beta_bp, cydr_table,  mc,ac,bc,'linear');
cn0_v   = interpn(mach_bp,alpha_bp,beta_bp, cn0_table,   mc,ac,bc,'linear');
cndq_v  = interpn(mach_bp,alpha_bp,beta_bp, cndq_table,  mc,ac,bc,'linear');
cll0_v  = interpn(mach_bp,alpha_bp,beta_bp, cll0_table,  mc,ac,bc,'linear');
clldp_v = interpn(mach_bp,alpha_bp,beta_bp, clldp_table, mc,ac,bc,'linear');
clm0_v  = interpn(mach_bp,alpha_bp,beta_bp, clm0_table,  mc,ac,bc,'linear');
clmdq_v = interpn(mach_bp,alpha_bp,beta_bp, clmdq_table, mc,ac,bc,'linear');
cln0_v  = interpn(mach_bp,alpha_bp,beta_bp, cln0_table,  mc,ac,bc,'linear');
clndr_v = interpn(mach_bp,alpha_bp,beta_bp, clndr_table, mc,ac,bc,'linear');

% ── Katsayılar ────────────────────────────────────────────────────────────────
deff = (abs(dqx) + abs(drx)) / 2.0;

ca  = ca0_v + caa_v + cad_v * deff^2;
if mprop > 0;  ca = ca + ca0b_v;  end

cy  = cy0_v  + cydr_v  * drx;
cn  = cn0_v  + cndq_v  * dqx;
cll = cllp_v * ppx * refl / (2.0*dvbe) + cll0_v + clldp_v * dpx;
clm = clm0_v + clmq_v * qqx * refl / (2.0*dvbe) + clmdq_v * dqx ...
      - cn/refl * (xcgref - xcg);
cln = cln0_v + clnr_v * rrx * refl / (2.0*dvbe) + clndr_v * drx ...
      - cy/refl * (xcgref - xcg);

% ── Boyutsal türevler ─────────────────────────────────────────────────────────
alpha = abs(alphax);
alpp  = max(alpha+3.0, 3.0);  alpm = max(alpha-3.0, 0.0);
apc   = max(min(alpp, alpha_bp(end)), alpha_bp(1));
amc   = max(min(alpm, alpha_bp(end)), alpha_bp(1));

cn0p  = interpn(mach_bp,alpha_bp,beta_bp, cn0_table,  mc,apc,bc,'linear');
cn0m  = interpn(mach_bp,alpha_bp,beta_bp, cn0_table,  mc,amc,bc,'linear');
cna   = DEG * (cn0p-cn0m) / (alpp-alpm);

clm0p = interpn(mach_bp,alpha_bp,beta_bp, clm0_table, mc,apc,bc,'linear');
clm0m = interpn(mach_bp,alpha_bp,beta_bp, clm0_table, mc,amc,bc,'linear');
clma  = DEG*(clm0p-clm0m)/(alpp-alpm) - cna/refl*(xcgref-xcg);

beta  = abs(betax);
betp  = max(beta+3.0, 3.0);   betm = max(beta-3.0, 0.0);
bpc   = max(min(betp, beta_bp(end)), beta_bp(1));
bmc   = max(min(betm, beta_bp(end)), beta_bp(1));

cy0p  = interpn(mach_bp,alpha_bp,beta_bp, cy0_table,  mc,ac,bpc,'linear');
cy0m  = interpn(mach_bp,alpha_bp,beta_bp, cy0_table,  mc,ac,bmc,'linear');
cyb   = DEG * (cy0p-cy0m) / (betp-betm);

cln0p = interpn(mach_bp,alpha_bp,beta_bp, cln0_table, mc,ac,bpc,'linear');
cln0m = interpn(mach_bp,alpha_bp,beta_bp, cln0_table, mc,ac,bmc,'linear');
clnb  = DEG*(cln0p-cln0m)/(betp-betm) - cyb/refl*(xcgref-xcg);

dna  = (pdynmc*refa/mass)                  * cna;
dma  = (pdynmc*refa*refl/ai33)             * clma;
dmq  = DEG*(pdynmc*refa*refl/ai33)*(refl/(2.0*dvbe)) * clmq_v;
dmd  = DEG*(pdynmc*refa*refl/ai33)         * clmdq_v;
dnd  = (pdynmc*refa/mass)                  * cndq_v;
dyb  = (pdynmc*refa/mass)                  * cyb;
dlnb = (pdynmc*refa*refl/ai33)             * clnb;
dlnr = DEG*(pdynmc*refa*refl/ai33)*(refl/(2.0*dvbe)) * clnr_v;
dlnd = DEG*(pdynmc*refa*refl/ai33)         * clndr_v;
dlp  = DEG*(pdynmc*refa*refl/ai11)*(refl/(2.0*dvbe)) * cllp_v;
dld  = DEG*(pdynmc*refa*refl/ai11)         * clldp_v;

% ── Dinamik kökler ────────────────────────────────────────────────────────────
a11q = dmq;   a12q = dma/(dna+SMALL);
a21q = dna;   a22q = -dna/(dvbe+SMALL);
argq = (a11q+a22q)^2 - 4*(a11q*a22q - a12q*a21q);
if argq >= 0;  wnq=0.0; zetq=0.0;
else;  wnq=sqrt(a11q*a22q-a12q*a21q); zetq=-(a11q+a22q)/(2*wnq); end

a11r = dlnr;  a12r = dlnb/(dyb+SMALL);
a21r = -dyb;  a22r = dyb/(dvbe+SMALL);
argr = (a11r+a22r)^2 - 4*(a11r*a22r - a12r*a21r);
if argr >= 0;  wnr=0.0; zetr=0.0;
else;  wnr=sqrt(a11r*a22r-a12r*a21r); zetr=-(a11r+a22r)/(2*wnr); end

end

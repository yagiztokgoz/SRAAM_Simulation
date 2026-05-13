% SRAAM6_params.m

%% ── Dizinler ─────────────────────────────────────────────────────────────────
SRAAM6_ROOT   = fileparts(fileparts(mfilename('fullpath')));
AERO_MAT_FILE = fullfile(SRAAM6_ROOT, 'data', 'aero_tables.mat');
%% ── Simülasyon ───────────────────────────────────────────────────────────────
DT     = 0.001;    % integrasyon adımı - s
T_END  = 25.0;     % max simülasyon süresi - s
LOG_DT = 0.01;     % loglama örnek aralığı - s  (decimation = LOG_DT/DT = 10)
HIT_R  = 0;      % isabet yarıçapı - m  (0 → saf CPA modu)
%% ── Fiziksel Sabitler ────────────────────────────────────────────────────────
AGRAV = 9.80665;   % yerçekimi ivmesi - m/s²

%% ── Başlangıç (fırlatma) Durumu ──────────────────────────────────────────────
% NED çerçevesi: [Kuzey; Doğu; Aşağı]  — irtifa = -sbel3
SBEL_INIT     = [0.0; 0.0; -10000.0];   % NED konum - m
ATTITUDE_INIT = [0.0; 0.0; 0.0];        % [psi; theta; phi] - deg
ALPHA0_DEG    = 0.0;                     % başlangıç hücum açısı - deg
BETA0_DEG     = 0.0;                     % başlangıç kayma açısı - deg
SPEED_INIT    = 250.0;                   % başlangıç hava hızı - m/s

% Başlangıç gövde ekseni hızı (VBEB_IC hesabı)
a0 = ALPHA0_DEG * pi/180;
b0 = BETA0_DEG  * pi/180;
VBEB_IC = SPEED_INIT * [cos(a0)*cos(b0); sin(b0); sin(a0)*cos(b0)];

% Başlangıç quaternion (birim — kimlik dönüşümü, psi=tht=phi=0)
Q_IC = [1; 0; 0; 0];   % [q0; q1; q2; q3]

% Başlangıç açısal hız
OMEGA_IC = [0; 0; 0];  % [p; q; r] - rad/s

% Tam durum vektörü başlangıç koşulu x[13]
X_IC = [SBEL_INIT; VBEB_IC; Q_IC; OMEGA_IC];

%% ── Aerodinamik Gövde ────────────────────────────────────────────────────────
ALP_LIMX = 46.0;      % toplam hücum açısı sınırı - deg
REFL     = 0.1524;    % moment türevleri için referans uzunluk - m
REFA     = 0.01824;   % aerodinamik katsayılar için referans alan - m²
XCG_REF  = 1.2994;    % yanma sonu ağırlık merkezi, burundan - m

%% ── İtki / Kütle / Atalet ────────────────────────────────────────────────────
MPROP    = 1;          % 0:kapalı  1:açık  2:2.pulse  3:sabit itki
AEXIT    = 0.0125;     % meme çıkış alanı - m²
MASS_IC  = 91.95;      % başlangıç araç kütlesi - kg
XCG_IC   = 1.536;      % başlangıç ağırlık merkezi, burundan - m
AI11_IC  = 0.308;      % yuvarlanma atalet momenti - kg·m²
AI33_IC  = 59.80;      % yunuslama/yalpalama atalet momenti - kg·m²

% İtki tabloları aero_tables.mat'tan yükleniyor
if exist(AERO_MAT_FILE, 'file')
    load(AERO_MAT_FILE, ...
        'prop_thrust_vs_time_t', 'prop_thrust_vs_time_v', ...
        'prop_mass_vs_time_t',   'prop_mass_vs_time_v', ...
        'prop_cg_vs_time_t',     'prop_cg_vs_time_v', ...
        'prop_moipitch_vs_time_t','prop_moipitch_vs_time_v', ...
        'prop_moiroll_vs_time_t', 'prop_moiroll_vs_time_v');

    % LUT bloklarında kullanılan kısa isimler
    prop_thrust_v   = prop_thrust_vs_time_v(:);
    prop_thrust_t   = prop_thrust_vs_time_t(:);
    prop_mass_v     = prop_mass_vs_time_v(:);
    prop_mass_t     = prop_mass_vs_time_t(:);
    prop_cg_v       = prop_cg_vs_time_v(:);
    prop_cg_t       = prop_cg_vs_time_t(:);
    prop_moipitch_v = prop_moipitch_vs_time_v(:);
    prop_moipitch_t = prop_moipitch_vs_time_t(:);
    prop_moiroll_v  = prop_moiroll_vs_time_v(:);
    prop_moiroll_t  = prop_moiroll_vs_time_t(:);
else
    error('SRAAM6_params: aero_tables.mat bulunamadı → %s', AERO_MAT_FILE);
end

%% ── Aktüatör ─────────────────────────────────────────────────────────────────
MACT    = 2;       % 0:dinamik yok  2:2.mertebe
DLIMX   = 28.0;    % kanat açısı sınırı - deg
DDLIMX  = 600.0;   % kanat hız sınırı - deg/s
WNACT   = 251.0;   % aktüatör doğal frekansı - rad/s
ZETACT  = 0.7;     % aktüatör sönüm oranı - boyutsuz

%% ── Otopilot Seçimi ──────────────────────────────────────────────────────────
MAUT      = 5;

ALIMIT    = 50.0;  % toplam yapısal ivme sınırı - g
DQLIMX    = 28.0;  % yunuslama kanat komut sınırı - deg
DRLIMX    = 28.0;  % yalpalama kanat komut sınırı - deg
DPLIMX    = 28.0;  % yuvarlanma komut sınırı - deg
WBLIMX    = 30.0;  % gövde açısal hız sınırı (q,r komutu) - deg/s

%% ── Yuvarlanma Kontrolcüsü ───────────────────────────────────────────────────
ZRCL      = 0.9;   % yuvarlanma kapalı döngü sönümü
FACT_WRCL = 0.0;   % yuvarlanma bant genişliği faktörü

%% ── Hız Kontrolcüsü (maut=2) ────────────────────────────────────────────────
ZETLAGR   = 0.6;   % kapalı hız döngüsü sönümü

%% ── İvme Kontrolcüsü (maut=3) ───────────────────────────────────────────────
TWCL      = 0.4;   % wacl yumuşatıcı zaman sabiti - s
WACL_BIAS = 3.0;   % wacl bias - rad/s
FACT_WACL = -0.45; % yunuslama/yalpalama kapalı döngü frekans faktörü
PACL      = 14.0;  % kapalı döngü reel pol
ZACL      = 0.7;   % ivme kapalı döngü karmaşık pol sönümü

%% ── NDI / INDI (maut=5 / 6) ─────────────────────────────────────────────────
WN_NDI_Q      = 50.0;   % iç moment döngüsü bant genişliği (yunuslama) - rad/s
WN_NDI_R      = 50.0;   % iç moment döngüsü bant genişliği (yalpalama) - rad/s
K_ALPHA       = 10.0;   % orta alpha döngüsü kazancı - 1/s
K_BETA        = 10.0;   % orta beta döngüsü kazancı - 1/s
WN_INDI_FILT  = 50.0;   % INDI eşleştirilmiş LPF bant genişliği - rad/s

%% ── Güdüm ────────────────────────────────────────────────────────────────────
% mguid = |orta|terminal|  iki basamaklı bayrak
%   orta    = 0:kapalı  3:pronav  2:çizgi-A2G  5:çizgi-A2A
%   terminal = 0:kapalı  6:kompanse-pronav (seeker kilidi gerektirir)
GNAV      = 4.0;   % orantılı navigasyon kazancı
MNAV      = 0;     % 0:doğrudan hedef verisi

%% ── INS ──────────────────────────────────────────────────────────────────────
MINS      = 0;     % 0:ideal  1:gerçek IMU hataları
INS_SEED  = 42;    % tekrarlanabilir rastgele çekimler için tohum

% IMU hata gerçekleştirmeleri — uçuş başında bir kez çekilir.
% MINS=0 durumunda sıfır vektörler kullanılır (workspace değişkenleri
% her iki modda da tanımlı olmalı — Simulink Parameter çözümlemesi için).
% ── IMU Sınıf Seçimi ─────────────────────────────────────────────────────────
% IMU_GRADE: 1=taktik(askeri yüksek kalite)  2=endüstriyel  3=düşük kalite
IMU_GRADE = 3;

if MINS == 1
    rng(INS_SEED);

    if IMU_GRADE == 1
        % ── Taktik sınıf (yüksek kalite askeri) ─────────────────────────────
        % Jiroskop
        sg_bias  = 3.2e-6;    % rad/s   (~0.66 deg/hr)
        sg_scale = 2.5e-5;    % boyutsuz (25 ppm)
        sg_mis   = 1.1e-4;    % rad
        sg_walk  = 1.0e-6;    % rad/√s
        % İvmeölçer
        sa_bias  = 3.56e-3;   % m/s²    (~363 μg)
        sa_scale = 5.0e-4;    % boyutsuz (500 ppm)
        sa_mis   = 1.1e-4;    % rad
        sa_walk  = 1.0e-4;    % (m/s)/√s

    elseif IMU_GRADE == 2
        % ── Endüstriyel sınıf (düşük kalite askeri / ticari) ─────────────────
        % Jiroskop
        sg_bias  = 1.0e-3;    % rad/s   (~206 deg/hr)     ×300 taktikten
        sg_scale = 5.0e-3;    % boyutsuz (5000 ppm = 0.5%) ×200
        sg_mis   = 1.0e-2;    % rad      (~0.57°)           ×91
        sg_walk  = 1.0e-4;    % rad/√s                      ×100
        % İvmeölçer
        sa_bias  = 5.0e-1;    % m/s²    (~51 mg)            ×140
        sa_scale = 5.0e-2;    % boyutsuz (5%)                ×100
        sa_mis   = 1.0e-2;    % rad      (~0.57°)            ×91
        sa_walk  = 1.0e-2;    % (m/s)/√s                    ×100

    else
        % ── Düşük kalite (tüketici sınıfı, kötü senaryo) ─────────────────────
        % Jiroskop
        sg_bias  = 1.0e-2;    % rad/s   (~2063 deg/hr ≈ 2000 deg/hr)
        sg_scale = 5.0e-2;    % boyutsuz (5%)
        sg_mis   = 3.0e-2;    % rad      (~1.7°)
        sg_walk  = 1.0e-3;    % rad/√s
        % İvmeölçer
        sa_bias  = 2.0;       % m/s²    (~204 mg ≈ 200 mg)
        sa_scale = 2.0e-1;    % boyutsuz (20%)
        sa_mis   = 3.0e-2;    % rad      (~1.7°)
        sa_walk  = 5.0e-2;    % (m/s)/√s
    end

    % Hata gerçekleştirmeleri — uçuş başında bir kez çekilir
    EBIASG = randn(3,1) * sg_bias;
    ESCALG = randn(3,1) * sg_scale;
    EMISG  = randn(3,1) * sg_mis;
    EUNBG  = zeros(3,1);
    EBIASA = randn(3,1) * sa_bias;
    ESCALA = randn(3,1) * sa_scale;
    EMISA  = randn(3,1) * sa_mis;
    BIASAL = 0.0;

    % Band-Limited White Noise PSD değerleri
    INS_NOISE_PWR_G = sg_walk^2;   % (rad/s)²/Hz
    INS_NOISE_PWR_A = sa_walk^2;   % (m/s²)²/Hz
    INS_NOISE_PWR_H = 0.0;

else
    EBIASG = zeros(3,1);  ESCALG = zeros(3,1);
    EMISG  = zeros(3,1);  EUNBG  = zeros(3,1);
    EBIASA = zeros(3,1);  ESCALA = zeros(3,1);
    EMISA  = zeros(3,1);  BIASAL = 0.0;
    INS_NOISE_PWR_G = 0.0;
    INS_NOISE_PWR_A = 0.0;
    INS_NOISE_PWR_H = 0.0;
end

%% ── Seeker (kinematik IR) ────────────────────────────────────────────────────
MSEEK     = 0;
RACQ      = 5000.0;   % edinme menzili - m
DTIMAC    = 0.2;      % edinme bekleme süresi - s
DBLIND    = 100.0;    % kör menzil - m

%% ── TVC (varsayılan kapalı) ──────────────────────────────────────────────────
MTVC   = 0;
PARM   = 0.0;
GTVC   = 0.0;

% Aero LUT dizileri (aerodynamics.m MATLAB Function bloğuna parametre olarak geçilir)
load(AERO_MAT_FILE, 'mach_bp', 'alpha_bp', 'beta_bp', ...
    'ca0_vs_mach', 'caa_vs_mach_alpha_beta', 'cad_vs_mach', ...
    'cy0_vs_mach_alpha_beta', 'cydr_vs_mach_alpha_beta', ...
    'cn0_vs_mach_alpha_beta', 'cndq_vs_mach_alpha_beta', ...
    'cll0_vs_mach_alpha_beta', 'cllp_vs_mach', 'clldp_vs_mach_alpha_beta', ...
    'clm0_vs_mach_alpha_beta', 'clmq_vs_mach', 'clmdq_vs_mach_alpha_beta', ...
    'cln0_vs_mach_alpha_beta', 'clnr_vs_mach', 'clndr_vs_mach_alpha_beta', ...
    'ca0b_vs_mach');

% aerodynamics.m içinde kısa isimlerle kullanılıyor
caa_table   = caa_vs_mach_alpha_beta;
cy0_table   = cy0_vs_mach_alpha_beta;
cydr_table  = cydr_vs_mach_alpha_beta;
cn0_table   = cn0_vs_mach_alpha_beta;
cndq_table  = cndq_vs_mach_alpha_beta;
cll0_table  = cll0_vs_mach_alpha_beta;
clldp_table = clldp_vs_mach_alpha_beta;
clm0_table  = clm0_vs_mach_alpha_beta;
clmdq_table = clmdq_vs_mach_alpha_beta;
cln0_table  = cln0_vs_mach_alpha_beta;
clndr_table = clndr_vs_mach_alpha_beta;

%% ── Senaryo açıklamaları ─────────────────────────────────────────────────────
SCENARIO_NAMES = {
    'S01 | Constant-speed straight flight  |  6 km range, 72deg aspect, no maneuver';
    'S02 | Accelerating escape turn        |  5g right break at t=1.5s (centripetal)';
    'S03 | Sinusoidal jink                 |  3g lateral + 2g vertical weave';
    'S04 | Sharp L-maneuver               |  8g right break t>2s, 6g pull-up t>3.5s';
    'S05 | Beaming / notch                |  90deg crossing geometry, max LOS rate';
    'S06 | Head-on + break                |  closing at Mach~1, 7g break at t=3s';
    'S07 | Look-down engage               |  target at 7500m, missile at 10000m';
    'S08 | Last-ditch break               |  9g combined break at t=4s (endgame)';
    'S09 | Energy-bleed climb             |  afterburner ~3g climbing turn';
    'S10 | Sustained coordinated turn     |  4g horizontal turn, ~8.8s flight';
    'S11 | Long-range late break          |  12 km, 6g right break at t=7s, ~10s';
    'S12 | Very long range sinusoidal     |  15 km, 3g+2g jink at t=5s, ~14s';
    'S13 | Tail-chase escape              |  target fleeing, 7g pull-up+break, ~10s';
    'S14 | Long-range beam + hard break   |  10 km beam, 8g break at t=6s, ~11s';
};
%% ── Senaryo başlangıç koşulları ────────
SCENARIO = 12;   % ← buradan senaryo seç (1-14)
SCENARIO_NAME = SCENARIO_NAMES{SCENARIO};

TGET_POS = [ 6000,  1500, -10000;   % S1
             6000,  1500, -10000;   % S2
             6000,  1500, -10000;   % S3
             6000,  1500, -10000;   % S4
             5500, -2000, -10000;   % S5
             7000,     0, -10000;   % S6
             5500,  1000,  -7500;   % S7
             6000,  1500, -10000;   % S8
             6500,   800, -10000;   % S9
            10000, -1000, -12000;   % S10
            12000,  2000, -10000;   % S11 uzun menzil (~10s)
            15000,  3000, -10000;   % S12 çok uzun menzil (~14s)
             8000,     0, -10000;   % S13 tail-chase (~10s)
            10000, -3000, -10000];  % S14 uzun beam (~10s)

TGET_VEL = [-200,   0,  0;   % S1
            -200,   0,  0;   % S2
            -200,   0,  0;   % S3
            -200,   0,  0;   % S4
               0, 300,  0;   % S5
            -300,   0,  0;   % S6
            -220,   0,  0;   % S7
            -200,   0,  0;   % S8
            -250,   0,  0;   % S9
            -180, 120,  0;   % S10
            -200,   0,  0;   % S11
            -200,   0,  0;   % S12
             200,   0,  0;   % S13 kaçıyor
               0, 250,  0];  % S14

TGET_POS_IC = TGET_POS(SCENARIO, :)';   % [3x1] NED - m
TGET_VEL_IC = TGET_VEL(SCENARIO, :)';   % [3x1] NED - m/s

fprintf('SRAAM6_params: yüklendi. MAUT=%d  MINS=%d  MSEEK=%d  SCENARIO=%d\n', MAUT, MINS, MSEEK, SCENARIO);

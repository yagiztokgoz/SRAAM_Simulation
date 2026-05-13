function plot_results(simOut, SCENARIO, draw)
%PLOT_RESULTS  SRAAM6 simulation results — three diagnostic figures.

BLUE   = [0.259 0.545 0.792];
RED    = [0.851 0.325 0.310];
GREEN  = [0.361 0.722 0.361];
ORANGE = [0.941 0.678 0.306];
PURPLE = [0.580 0.404 0.741];
TEAL   = [0.090 0.635 0.722];
FIN_LIM = 28.0;  ALIMIT = 50.0;

%% ── Extract ──────────────────────────────────────────────────────────────────
t    = simOut.tout;
x_sl = simOut.xout;
mis_N = x_sl(:,1);  mis_E = x_sl(:,2);  mis_h = -x_sl(:,3);

kin_alpha = simOut.kin_log.alpha_deg.Data;
kin_beta  = simOut.kin_log.beta_deg.Data;
kin_alpp  = simOut.kin_log.alpp_deg.Data;
kin_dvbe  = simOut.kin_log.dvbe.Data;
kin_psibl = simOut.kin_log.psiblx.Data;
kin_thtbl = simOut.kin_log.thtblx.Data;
kin_phibl = simOut.kin_log.phiblx.Data;
kin_psivl = simOut.kin_log.psivlx.Data;
kin_thtvl = simOut.kin_log.thtvlx.Data;
kin_ppx   = simOut.kin_log.ppx.Data;
kin_qqx   = simOut.kin_log.qqx.Data;
kin_rrx   = simOut.kin_log.rrx.Data;

env_mach   = simOut.env_log.mach.Data;
env_pdynmc = simOut.env_log.pdynmc.Data;

prop_thrust = simOut.prop_log.thrust.Data;
prop_mass   = simOut.prop_log.mass.Data;
prop_xcg    = simOut.prop_log.xcg.Data;

fm_FAPB_raw = simOut.fm_log.FAPB.Data;
fm_FAPB = squeeze(fm_FAPB_raw)';
if size(fm_FAPB,2) ~= 3; fm_FAPB = fm_FAPB'; end
anx     = -fm_FAPB(:,3) ./ (prop_mass * 9.80665);
ayx     =  fm_FAPB(:,2) ./ (prop_mass * 9.80665);
G_total =  sqrt(anx.^2 + ayx.^2);

ctrl_dpcx = simOut.control_log.dpcx.Data;
ctrl_dqcx = simOut.control_log.dqcx.Data;
ctrl_drcx = simOut.control_log.drcx.Data;
act_del1  = simOut.act_log.del1.Data;
act_del2  = simOut.act_log.del2.Data;
act_del3  = simOut.act_log.del3.Data;
act_del4  = simOut.act_log.del4.Data;

guid_ancomx = simOut.guidance_log.ancomx.Data;
guid_alcomx = simOut.guidance_log.alcomx.Data;
guid_dtbc   = simOut.guidance_log.dtbc.Data;
guid_tgoc   = simOut.guidance_log.tgoc.Data;

tgt_raw  = simOut.target_log.STEL.Data;
% Robust extraction: handle [3×1×N], [3×N], or [N×3] shapes
tgt_raw  = squeeze(tgt_raw);
if size(tgt_raw,1) == 3 && size(tgt_raw,2) ~= 3
    tgt_raw = tgt_raw';   % → [N×3]
elseif size(tgt_raw,1) ~= 3 && size(tgt_raw,2) == 3
    % already [N×3]
elseif size(tgt_raw,1) == 3 && size(tgt_raw,2) == 3
    % ambiguous — assume first dim is components if Time length matches
    if length(simOut.target_log.STEL.Time) == 3
        % very short sim, leave as is
    else
        tgt_raw = tgt_raw';
    end
end
tgt_N = tgt_raw(:,1);  tgt_E = tgt_raw(:,2);  tgt_h = -tgt_raw(:,3);

% ── Tüm sinyalleri ortak t vektörüne hizala ──────────────────────────────────
% t ve x_sl simOut.tout/xout'tan gelir (reference time)
n = length(t);

% Yardımcı: sinyal vektörünü t'ye yeniden örnekle
resamp = @(sig, sig_t) interp1(sig_t, sig, t, 'linear', 'extrap');

% Guidance sinyallerinin kendi zaman vektörü
t_guid = simOut.guidance_log.dtbc.Time;
t_kin  = simOut.kin_log.alpha_deg.Time;
t_env  = simOut.env_log.mach.Time;
t_prop = simOut.prop_log.thrust.Time;
t_act  = simOut.act_log.del1.Time;
t_ctrl = simOut.control_log.dpcx.Time;
t_tgt  = simOut.target_log.STEL.Time;
t_fm   = simOut.fm_log.FAPB.Time;

% Hedef konumu yeniden örnekle
tgt_STEL_rs = [interp1(t_tgt,tgt_N,t,'linear','extrap'), ...
               interp1(t_tgt,tgt_E,t,'linear','extrap'), ...
               interp1(t_tgt,-tgt_h,t,'linear','extrap')];
tgt_N = tgt_STEL_rs(:,1); tgt_E = tgt_STEL_rs(:,2);
tgt_h = -tgt_STEL_rs(:,3);

% Kinematik
kin_alpha = resamp(kin_alpha, t_kin);  kin_beta  = resamp(kin_beta,  t_kin);
kin_alpp  = resamp(kin_alpp,  t_kin);  kin_dvbe  = resamp(kin_dvbe,  t_kin);
kin_psibl = resamp(kin_psibl, t_kin);  kin_thtbl = resamp(kin_thtbl, t_kin);
kin_phibl = resamp(kin_phibl, t_kin);  kin_psivl = resamp(kin_psivl, t_kin);
kin_thtvl = resamp(kin_thtvl, t_kin);
kin_ppx   = resamp(kin_ppx,   t_kin);  kin_qqx   = resamp(kin_qqx,   t_kin);
kin_rrx   = resamp(kin_rrx,   t_kin);

% Environment
env_mach   = resamp(env_mach,   t_env);
env_pdynmc = resamp(env_pdynmc, t_env);

% Propulsion
prop_thrust = resamp(prop_thrust, t_prop);
prop_mass   = resamp(prop_mass,   t_prop);
prop_xcg    = resamp(prop_xcg,    t_prop);

% Forces → G
fm_FAPB_rs = [interp1(t_fm,fm_FAPB(:,1),t,'linear','extrap'), ...
              interp1(t_fm,fm_FAPB(:,2),t,'linear','extrap'), ...
              interp1(t_fm,fm_FAPB(:,3),t,'linear','extrap')];
fm_FAPB = fm_FAPB_rs;
anx     = -fm_FAPB(:,3) ./ (prop_mass * 9.80665);
ayx     =  fm_FAPB(:,2) ./ (prop_mass * 9.80665);
G_total =  sqrt(anx.^2 + ayx.^2);

% Control + Actuator
ctrl_dpcx = resamp(ctrl_dpcx, t_ctrl);
ctrl_dqcx = resamp(ctrl_dqcx, t_ctrl);
ctrl_drcx = resamp(ctrl_drcx, t_ctrl);
act_del1  = resamp(act_del1,  t_act);
act_del2  = resamp(act_del2,  t_act);
act_del3  = resamp(act_del3,  t_act);
act_del4  = resamp(act_del4,  t_act);

% Guidance
guid_ancomx = resamp(guid_ancomx, t_guid);
guid_alcomx = resamp(guid_alcomx, t_guid);
guid_dtbc   = resamp(guid_dtbc,   t_guid);
guid_tgoc   = resamp(guid_tgoc,   t_guid);

% ── Miss distance: guid_dtbc kullan (zaten mesafe) ───────────────────────────
[miss_dist, idx_cpa] = min(guid_dtbc);
t_cpa = t(idx_cpa);
% Senaryo açıklamasını workspace'den oku
if evalin('base','exist(''SCENARIO_NAME'',''var'')')
    scen_desc = evalin('base','SCENARIO_NAME');
else
    scen_desc = sprintf('Scenario %d', SCENARIO);
end
title_main = sprintf('%s\nmiss = %.2f m  |  t_{CPA} = %.3f s', ...
    scen_desc, miss_dist, t_cpa);

% Stamp indices (en az 4 nokta, idx_cpa sınırı içinde)
if idx_cpa > 4
    stamp_idx = round(linspace(1, idx_cpa, 6));
    stamp_idx = stamp_idx(2:end-1);
else
    stamp_idx = [];
end

% Axis bounds
all_N = [mis_N; tgt_N];  all_E = [mis_E; tgt_E];  all_h = [mis_h; tgt_h];
cN = mean([max(all_N) min(all_N)]);  cE = mean([max(all_E) min(all_E)]);
cH = mean([max(all_h) min(all_h)]);
rng3 = max([max(all_N)-min(all_N), max(all_E)-min(all_E), ...
            max(all_h)-min(all_h)]) * 0.55 + 500;
if draw
    %% ════════════════════════════════════════════════════════════════════════════
    %  FIGURE 1 — TRAJECTORY
    %% ════════════════════════════════════════════════════════════════════════════
    figure('Name',sprintf('S%d – Trajectory',SCENARIO), ...
        'Color','white','Position',[40 40 1500 660]);
    sgtitle(title_main,'FontSize',13,'FontWeight','bold','Color',[0.1 0.1 0.1]);
    
    % — 3D —
    ax3d = subplot(2,3,[1 4]);  hold(ax3d,'on');
    h_floor = min(all_h) - 400;
    % Ground shadows (RGBA kullanmadan, açık renkle)
    BLUE_L = BLUE*0.4 + 0.6;   % lighter shade for shadow
    RED_L  = RED*0.4  + 0.6;
    plot3(ax3d, mis_N, mis_E, repmat(h_floor,size(mis_N)), ...
        'Color',BLUE_L,'LineWidth',1.2,'LineStyle',':','HandleVisibility','off');
    plot3(ax3d, tgt_N, tgt_E, repmat(h_floor,size(tgt_N)), ...
        'Color',RED_L, 'LineWidth',1.0,'LineStyle',':','HandleVisibility','off');
    % Vertical drop at CPA
    plot3(ax3d, [mis_N(idx_cpa) mis_N(idx_cpa)], ...
               [mis_E(idx_cpa) mis_E(idx_cpa)], ...
               [h_floor mis_h(idx_cpa)], '--','Color',[0.6 0.6 0.6], ...
               'LineWidth',0.8,'HandleVisibility','off');
    % Glow
    BLUE_G = BLUE*0.3 + 0.7;
    RED_G  = RED*0.3  + 0.7;
    plot3(ax3d, mis_N, mis_E, mis_h,'Color',BLUE_G,'LineWidth',8, ...
        'HandleVisibility','off');
    plot3(ax3d, tgt_N, tgt_E, tgt_h,'Color',RED_G, 'LineWidth',6, ...
        'HandleVisibility','off');
    % Main paths — hedef solid kırmızı, füze solid mavi
    plot3(ax3d, tgt_N, tgt_E, tgt_h,'Color',RED, 'LineWidth',2.5,'DisplayName','Target');
    plot3(ax3d, mis_N, mis_E, mis_h,'Color',BLUE,'LineWidth',2.0,'DisplayName','Missile');
    % Time stamps — missile
    scatter3(ax3d, mis_N(stamp_idx),mis_E(stamp_idx),mis_h(stamp_idx), ...
        55,'filled','MarkerFaceColor',BLUE*0.7,'MarkerEdgeColor','w', ...
        'HandleVisibility','off');
    for k = stamp_idx
        text(ax3d, mis_N(k),mis_E(k),mis_h(k)+200, sprintf('%.1fs',t(k)), ...
            'FontSize',7,'Color',BLUE*0.6,'HorizontalAlignment','center', ...
            'HandleVisibility','off');
    end
    % Time stamps — target (same indices)
    scatter3(ax3d, tgt_N(stamp_idx),tgt_E(stamp_idx),tgt_h(stamp_idx), ...
        40,'filled','MarkerFaceColor',RED*0.7,'MarkerEdgeColor','w', ...
        'HandleVisibility','off');
    % Launch markers
    scatter3(ax3d, mis_N(1),mis_E(1),mis_h(1), 90,BLUE,'filled','s','DisplayName','Launch');
    scatter3(ax3d, tgt_N(1),tgt_E(1),tgt_h(1), 90,RED, 'filled','s', ...
        'DisplayName','Target start');
    scatter3(ax3d, mis_N(idx_cpa),mis_E(idx_cpa),mis_h(idx_cpa), ...
        180,BLUE,'p','filled','MarkerEdgeColor','k','LineWidth',0.8,'DisplayName','CPA');
    scatter3(ax3d, tgt_N(idx_cpa),tgt_E(idx_cpa),tgt_h(idx_cpa), ...
        180,RED,'x','LineWidth',2.5,'DisplayName','Target @ CPA');
    xlabel(ax3d,'North [m]','FontSize',9); ylabel(ax3d,'East [m]','FontSize',9);
    zlabel(ax3d,'Altitude [m]','FontSize',9);
    xlim(ax3d,[cN-rng3 cN+rng3]); ylim(ax3d,[cE-rng3 cE+rng3]);
    zlim(ax3d,[cH-rng3 cH+rng3]);
    title(ax3d,'3D Engagement Geometry','FontWeight','bold','FontSize',11);
    legend(ax3d,'Location','best','FontSize',8);
    grid(ax3d,'on'); view(ax3d,35,20); ax3d.Color = [0.97 0.97 0.97];
    ax3d.DataAspectRatio = [1 1 1];   % eşit ölçek
    
    % — Top view —
    ax2 = subplot(2,3,2);  hold(ax2,'on');
    plot(ax2, mis_N,mis_E,'Color',BLUE_G,'LineWidth',8,'HandleVisibility','off');
    plot(ax2, tgt_N,tgt_E,'Color',RED_G, 'LineWidth',6,'HandleVisibility','off');
    plot(ax2, tgt_N,tgt_E,'Color',RED, 'LineWidth',2.5,'DisplayName','Target');
    plot(ax2, mis_N,mis_E,'Color',BLUE,'LineWidth',2.0,'DisplayName','Missile');
    plot(ax2, [mis_N(1) tgt_N(1)],[mis_E(1) tgt_E(1)],':', ...
        'Color',[0.6 0.6 0.6],'LineWidth',1.0,'HandleVisibility','off');
    scatter(ax2, mis_N(1),mis_E(1), 80,BLUE,'filled','s','HandleVisibility','off');
    scatter(ax2, tgt_N(1),tgt_E(1), 80,RED, 'filled','s','HandleVisibility','off');
    scatter(ax2, mis_N(idx_cpa),mis_E(idx_cpa),120,BLUE,'p','filled','DisplayName','CPA');
    xlabel(ax2,'North [m]','FontSize',9); ylabel(ax2,'East [m]','FontSize',9);
    title(ax2,'Top View  (North – East)','FontWeight','bold','FontSize',11);
    axis(ax2,'equal'); grid(ax2,'on'); legend(ax2,'Location','best','FontSize',8);
    ax2.Color = [0.97 0.97 0.97];
    
    % — Altitude profile —
    ax3 = subplot(2,3,3);  hold(ax3,'on');
    fill(ax3,[mis_N; flipud(mis_N)],[mis_h; zeros(size(mis_h))],BLUE, ...
        'FaceAlpha',0.07,'EdgeColor','none','HandleVisibility','off');
    plot(ax3, tgt_N,tgt_h,'Color',RED, 'LineWidth',2.5,'DisplayName','Target');
    plot(ax3, mis_N,mis_h,'Color',BLUE,'LineWidth',2.0,'DisplayName','Missile');
    scatter(ax3, mis_N(idx_cpa),mis_h(idx_cpa),120,BLUE,'p','filled','DisplayName','CPA');
    xlabel(ax3,'North [m]','FontSize',9); ylabel(ax3,'Altitude [m]','FontSize',9);
    title(ax3,'Side View  (North – Altitude)','FontWeight','bold','FontSize',11);
    grid(ax3,'on'); legend(ax3,'Location','best','FontSize',8);
    ax3.Color = [0.97 0.97 0.97];
    
    % — Range —
    ax4 = subplot(2,3,5);  hold(ax4,'on');
    fill(ax4,[t; flipud(t)],[guid_dtbc; zeros(size(guid_dtbc))],GREEN, ...
        'FaceAlpha',0.07,'EdgeColor','none','HandleVisibility','off');
    plot(ax4, t,guid_dtbc,'Color',GREEN,'LineWidth',2.5,'DisplayName','Slant Range');
    scatter(ax4, t_cpa,miss_dist,120,RED,'v','filled', ...
        'DisplayName',sprintf('CPA = %.2f m',miss_dist));
    xlabel(ax4,'Time [s]','FontSize',9); ylabel(ax4,'Range [m]','FontSize',9);
    title(ax4,'Distance to Target','FontWeight','bold','FontSize',11);
    grid(ax4,'on'); legend(ax4,'Location','best','FontSize',8);
    ax4.Color = [0.97 0.97 0.97];
    
    % — G-force —
    ax5 = subplot(2,3,6);  hold(ax5,'on');
    fill(ax5,[t; flipud(t)],[G_total; zeros(size(G_total))],ORANGE, ...
        'FaceAlpha',0.07,'EdgeColor','none','HandleVisibility','off');
    plot(ax5, t,anx,    'Color',BLUE,  'LineWidth',2.0,'DisplayName','nz  normal');
    plot(ax5, t,ayx,    'Color',GREEN, 'LineWidth',1.8,'DisplayName','ny  lateral');
    plot(ax5, t,G_total,'Color',ORANGE,'LineWidth',2.5,'DisplayName','|G| total');
    % yline yerine plot — tüm MATLAB versiyonlarında çalışır, legend'a girmiyor
    plot(ax5,[t(1) t(end)],[ ALIMIT  ALIMIT],'--','Color',RED,'LineWidth',1.5, ...
        'DisplayName',sprintf('+/-%dg limit',ALIMIT));
    plot(ax5,[t(1) t(end)],[-ALIMIT -ALIMIT],'--','Color',RED,'LineWidth',1.5, ...
        'HandleVisibility','off');
    xlabel(ax5,'Time [s]','FontSize',9); ylabel(ax5,'Load Factor [g]','FontSize',9);
    title(ax5,'Missile G-Force','FontWeight','bold','FontSize',11);
    grid(ax5,'on'); legend(ax5,'Location','best','FontSize',8);
    ax5.Color = [0.97 0.97 0.97];
    
    %% ════════════════════════════════════════════════════════════════════════════
    %  FIGURE 2 — STATE VARIABLES  (4 × 4)
    %% ════════════════════════════════════════════════════════════════════════════
    figure('Name',sprintf('S%d – State Variables',SCENARIO), ...
        'Color','white','Position',[70 70 1700 960]);
    sgtitle([title_main '  —  State Variables'], ...
        'FontSize',12,'FontWeight','bold','Color',[0.1 0.1 0.1]);
    
    sp = @(r,c,ys,lbls,ttl,ylbl,cols) plot_panel(subplot(4,4,(r-1)*4+c), ...
        t, ys, lbls, ttl, ylbl, cols);
    
    sp(1,1,{kin_dvbe},         {'Speed'},            'Speed',             'm/s',   {BLUE});
    sp(1,2,{env_mach},         {'Mach'},             'Mach Number',       '–',     {BLUE});
    sp(1,3,{env_pdynmc/1e3},   {'q'},                'Dynamic Pressure',  'kPa',   {TEAL});
    sp(1,4,{mis_h},            {'Altitude'},         'Altitude',          'm',     {BLUE});
    sp(2,1,{kin_alpha,kin_beta},{'α AoA','β slip'},  'Incidence Angles',  'deg',   {BLUE,RED});
    sp(2,2,{kin_alpp},         {'αt total'},         'Total AoA',         'deg',   {ORANGE});
    sp(2,3,{kin_psibl,kin_thtbl,kin_phibl},{'ψ','θ','φ'},'Euler Angles','deg',    {BLUE,RED,GREEN});
    sp(2,4,{kin_psivl,kin_thtvl},{'ψv','γ'},        'Flight Path Angles', 'deg',   {BLUE,GREEN});
    sp(3,1,{kin_ppx,kin_qqx,kin_rrx},{'p','q','r'}, 'Angular Rates',      'deg/s', {BLUE,RED,GREEN});
    sp(3,2,{ctrl_dpcx,ctrl_dqcx,ctrl_drcx},{'δp','δq','δr'},'Fin Commands','deg', {BLUE,RED,GREEN});
    sp(3,3,{anx,ayx,G_total},  {'nz','ny','|G|'},    'Load Factors',      'g',     {BLUE,GREEN,ORANGE});
    sp(3,4,{guid_ancomx,guid_alcomx},{'an','al'},    'Guidance Commands',  'g',     {BLUE,RED});
    sp(4,1,{guid_dtbc},        {'Range'},            'Range to Target',   'm',     {GREEN});
    sp(4,2,{guid_tgoc},        {'TGO'},              'Time to Go',        's',     {ORANGE});
    sp(4,3,{prop_thrust},      {'Thrust'},           'Thrust',            'N',     {RED});
    sp(4,4,{prop_mass},        {'Mass'},             'Vehicle Mass',      'kg',    {PURPLE});
    
    %% ════════════════════════════════════════════════════════════════════════════
    %  FIGURE 3 — ACTUATOR
    %% ════════════════════════════════════════════════════════════════════════════
    figure('Name',sprintf('S%d – Actuator',SCENARIO), ...
        'Color','white','Position',[100 100 1100 950]);
    sgtitle([title_main '  —  Actuator: Command vs Actual'], ...
        'FontSize',12,'FontWeight','bold','Color',[0.1 0.1 0.1]);
    
    delcx1 = -ctrl_dpcx + ctrl_dqcx - ctrl_drcx;
    delcx2 = -ctrl_dpcx + ctrl_dqcx + ctrl_drcx;
    delcx3 = +ctrl_dpcx + ctrl_dqcx - ctrl_drcx;
    delcx4 = +ctrl_dpcx + ctrl_dqcx + ctrl_drcx;
    dels_cmd = {delcx1,delcx2,delcx3,delcx4};
    dels_act = {act_del1,act_del2,act_del3,act_del4};
    
    for i = 1:4
        ax_l = subplot(4,2,2*i-1);  hold(ax_l,'on');
        fill(ax_l,[t;flipud(t)],[dels_act{i};zeros(size(dels_act{i}))],BLUE, ...
            'FaceAlpha',0.06,'EdgeColor','none','HandleVisibility','off');
        plot(ax_l,t,dels_cmd{i},'Color',ORANGE,'LineWidth',1.8,'LineStyle','--', ...
            'DisplayName',sprintf('Fin %d  Cmd',i));
        plot(ax_l,t,dels_act{i},'Color',BLUE,  'LineWidth',2.0, ...
            'DisplayName',sprintf('Fin %d  Actual',i));
        yline(ax_l, FIN_LIM,':','Color',RED,'LineWidth',1.5,'Alpha',0.7, ...
            'HandleVisibility','off');
        yline(ax_l,-FIN_LIM,':','Color',RED,'LineWidth',1.5,'Alpha',0.7, ...
            'HandleVisibility','off');
        grid(ax_l,'on');
        title(ax_l,sprintf('Fin %d  —  Command vs Actual',i), ...
            'FontWeight','bold','FontSize',10);
        ylabel(ax_l,'Deflection [deg]','FontSize',9);
        legend(ax_l,'FontSize',8,'Location','best');
        ax_l.Color = [0.97 0.97 0.97];
    
        ax_r = subplot(4,2,2*i);  hold(ax_r,'on');
        err = dels_cmd{i} - dels_act{i};
        fill(ax_r,[t;flipud(t)],[err;zeros(size(err))],RED, ...
            'FaceAlpha',0.07,'EdgeColor','none','HandleVisibility','off');
        plot(ax_r,t,err,'Color',RED,'LineWidth',2.0, ...
            'DisplayName',sprintf('Fin %d  Error',i));
        yline(ax_r,0,'k-','LineWidth',0.8,'Alpha',0.5,'HandleVisibility','off');
        grid(ax_r,'on');
        title(ax_r,sprintf('Fin %d  —  Tracking Error',i), ...
            'FontWeight','bold','FontSize',10);
        ylabel(ax_r,'Error [deg]','FontSize',9);
        legend(ax_r,'FontSize',8,'Location','best');
        ax_r.Color = [0.97 0.97 0.97];
    end
    subplot(4,2,7); xlabel('Time [s]','FontSize',9);
    subplot(4,2,8); xlabel('Time [s]','FontSize',9);
end
%% ── Summary ──────────────────────────────────────────────────────────────────
fprintf('\n  ══════════════════════════════════════════\n');
fprintf('  SRAAM6  Scenario %d\n', SCENARIO);
fprintf('  Miss distance : %.2f m\n',  miss_dist);
fprintf('  CPA time      : %.3f s\n',  t_cpa);
fprintf('  Max |G|       : %.1f g\n',   max(G_total));
fprintf('  Max speed     : %.1f m/s\n', max(kin_dvbe));
fprintf('  Max Mach      : %.3f\n',     max(env_mach));
fprintf('  Max alpha     : %.1f deg\n', max(abs(kin_alpha)));
fprintf('  Max beta      : %.1f deg\n', max(abs(kin_beta)));
fprintf('  Max alpp      : %.1f deg\n', max(kin_alpp));
fprintf('  ══════════════════════════════════════════\n');

end % main function


%% ── Local helper ─────────────────────────────────────────────────────────────
function plot_panel(ax, t, ys, lbls, ttl, ylbl, cols)
hold(ax,'on');
for j = 1:numel(ys)
    c = cols{mod(j-1,numel(cols))+1};
    fill(ax,[t;flipud(t)],[ys{j};zeros(size(ys{j}))],c, ...
        'FaceAlpha',0.05,'EdgeColor','none','HandleVisibility','off');
    plot(ax,t,ys{j},'Color',c,'LineWidth',1.8,'DisplayName',lbls{j});
end
grid(ax,'on');
title(ax,ttl,'FontWeight','bold','FontSize',9);
ylabel(ax,ylbl,'FontSize',8);
xlabel(ax,'Time [s]','FontSize',8);
if numel(ys) > 1
    legend(ax,'FontSize',7,'Location','best');
end
ax.Color = [0.97 0.97 0.97];
ax.FontSize = 8;
end

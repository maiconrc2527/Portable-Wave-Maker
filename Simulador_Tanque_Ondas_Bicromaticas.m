% Simulador_Tanque_Ondas_Bicromaticas_fix_with_probes_mp4_final.m
% Versão final: LaTeX da fórmula, subplots espaçados, WP_1/WP_2,
% linhas tracejadas no subplot inferior, legenda reposicionada, T_total = 20s.
clear; close all; clc;

try
    % ----------------------- CONFIGURAÇÃO / PARÂMETROS -----------------------
    QUICK_DEBUG = false; % false -> configurações completas

    % =============================================================================
    % PARÂMETROS FÍSICOS E GEOMÉTRICOS
    % =============================================================================
    g = 9.81;            % m/s^2
    h = 0.15;            % profundidade (m)
    h_tank = 0.2;        % altura total tanque (m)
    L = 2.2+10;             % comprimento do tanque (m)

    % Parâmetros da cunha/paddle (valores originais)
    A_paddle = 0.030;
    T_wave = 0.80;
    omega = 2*pi / T_wave;

    % =============================================================================
    % INPUT (igual ao seu código)
    % =============================================================================
    A1 = input('Amplitude A1 do paddle (m) [default 0.010]: ');
    if isempty(A1), A1 = 0.010; end
    T1 = input('Período T1 (s) [default 0.8]: ');
    if isempty(T1), T1 = 0.8; end
    A2 = input('Amplitude A2 do paddle (m) [default 0.005]: ');
    if isempty(A2), A2 = 0.005; end
    T2 = input('Período T2 (s) [default 0.6]: ');
    if isempty(T2), T2 = 0.6; end

    % =============================================================================
    % PARÂMETROS NUMÉRICOS
    % =============================================================================
    dx = 0.005;
    x = 0:dx:L;
    nx = numel(x);

    if QUICK_DEBUG
        dt_user = 0.01;
        T_total = 5.0;
        M = 200;
        snap_step = 10;
    else
        dt_user = 0.004;
        T_total = 10.0;    % <-- ajustado para 20s conforme solicitado
        M = 400;
        snap_step = 10;
    end

    c = sqrt(g * h);
    dt_stable = 0.9 * dx / c;
    if dt_user > dt_stable
        fprintf('Aviso: dt pedido (%.5f) > dt_estavel (%.5f). Ajustando dt para estabilidade.\n', dt_user, dt_stable);
        dt = dt_stable;
    else
        dt = dt_user;
    end
    nt = max(10, floor(T_total / dt));

    % absorvedor (praia)
    x_beach_start = 1.6+8;
    x_beach_end = 2.079+10;
    sigma_max = 15.0;
    sigma = zeros(1,nx);
    x_beach_min = min(x_beach_start, x_beach_end);
    x_beach_max = max(x_beach_start, x_beach_end);
    for i = 1:nx
        xi = x(i);
        if xi >= x_beach_min && xi <= x_beach_max
            sigma(i) = sigma_max * ((xi - x_beach_min) / (x_beach_max - x_beach_min))^2;
        elseif xi > x_beach_max
            sigma(i) = sigma_max;
        end
    end

    % =============================================================================
    % CÁLCULO DINÂMICO DE a/S (método modal)
    % =============================================================================
    N = 16;
    d = 0.05;
    d_prime = h - d;
    theta_val = atan(1);
    theta_func = @(theta,y,dprime) (y <= dprime).*0 + (y > dprime).*theta;

    % encontra k0 (progressivo)
    k0 = find_k0_matlab(omega, g, h);

    % modos estacionários
    k_n = nan(N-1,1);
    epsv = 1e-6;
    for nn = 1:(N-1)
        lower = (((2*nn - 1) * pi/2) / h) + epsv;
        upper = (((2*nn) * pi/2) / h) - epsv;
        try
            f_lower = dispersion_kn(lower, omega, g, h);
            f_upper = dispersion_kn(upper, omega, g, h);
            if f_lower * f_upper > 0
                kn = NaN;
            else
                kn = fzero(@(k) dispersion_kn(k, omega, g, h), [lower, upper]);
            end
        catch
            kn = NaN;
        end
        k_n(nn) = kn;
    end

    % Monta B e D
    y_local = linspace(0,h,M).';
    B = complex(zeros(M,N));
    D_vec = complex(zeros(M,1));
    for m = 1:M
        y = y_local(m);
        current_theta = theta_func(theta_val, y, d_prime);
        D_vec(m) = tan(current_theta);
        B(m,1) = k0 * h * (1i * cosh(k0 * y) - tan(current_theta) * sinh(k0 * y)) * ...
                 exp(1i * k0 * (y - d_prime) * tan(current_theta));
        for n = 2:N
            kn = k_n(n-1);
            if isnan(kn)
                B(m,n) = 0;
            else
                B(m,n) = -kn * h * (cos(kn * y) - tan(current_theta) * sin(kn * y)) * ...
                         exp(-kn * (y - d_prime) * tan(current_theta));
            end
        end
    end

    M_mat = (B') * B;
    b_vec = (B') * D_vec;
    if rcond(M_mat) < 1e-12
        A_sol = pinv(M_mat) * b_vec;
    else
        A_sol = M_mat \ b_vec;
    end
    A_prime_1 = A_sol(1);
    a_over_S = abs(-1i * A_prime_1 * k0 * h * sinh(k0 * h));
    A1_scaled = real(a_over_S * A1);
    A2_scaled = real(a_over_S * A2);

    fprintf('a_over_S = %.5f  => A1_scaled = %.5f m, A2_scaled = %.5f m\n', a_over_S, A1_scaled, A2_scaled);

    % =============================================================================
    % SIMULAÇÃO 1D (pré-alocado)
    % =============================================================================
    u = zeros(1,nx);
    u_prev = zeros(1,nx);
    u_next = zeros(1,nx);

    nSnapshots = ceil(nt / snap_step);
    snapshots = zeros(nSnapshots, nx);
    snap_idx = 0;

    [~, wedge_index] = min(abs(x - 0.1));
    paddle_disp_user = @(t) A1 * sin(2*pi / T1 * t) + A2 * sin(2*pi / T2 * t);
    paddle_disp_scaled = @(t) A1_scaled * sin(2*pi / T1 * t) + A2_scaled * sin(2*pi / T2 * t);

    fprintf('Iniciando loop temporal: nt=%d, dt=%.5f (estavel<=%.5f). Snap_step=%d\n', nt, dt, dt_stable, snap_step);
    tstart = tic;
    for n = 1:nt
        t = (n-1) * dt;
        u(wedge_index) = paddle_disp_scaled(t);
        u(1) = u(wedge_index);
        u(end) = u(end-1);

        for i = 2:(nx-1)
            u_xx = (u(i+1) - 2*u(i) + u(i-1)) / dx^2;
            damping = sigma(i) * (u(i) - u_prev(i)) / dt;
            u_next(i) = 2*u(i) - u_prev(i) + dt^2 * (c^2 * u_xx - damping);
        end

        u_next(wedge_index) = paddle_disp_scaled(t + dt);
        u_next(end) = u_next(end-1);

        u_prev = u;
        u = u_next;

        if mod(n, snap_step) == 0
            snap_idx = snap_idx + 1;
            if snap_idx <= nSnapshots
                snapshots(snap_idx,:) = u;
            end
        end
    end
    elapsed = toc(tstart);
    fprintf('Loop temporal finalizado em %.2f s. Snapshots gravados: %d\n', elapsed, snap_idx);

    if snap_idx < nSnapshots
        snapshots = snapshots(1:snap_idx, :);
    end
    snapshot_times = (0:(snap_idx-1)) * snap_step * dt;

    % =============================================================================
    % PLOT EXTRA (sinal do paddle e WP1)
    % =============================================================================
    MM = 1000.0;
    wp1 = 1.00; [~, wp1_index] = min(abs(x - wp1));
    t_fine = 0:dt:(T_total - dt);
    paddle_signal_user = A1 * sin(2*pi / T1 * t_fine) + A2 * sin(2*pi / T2 * t_fine);
    paddle_signal_user_mm = paddle_signal_user * MM;

    if ~isempty(snapshots)
        wp_series = snapshots(:, wp1_index);
        wp_series_mm = wp_series * MM;
    else
        wp_series = zeros(size(snapshot_times));
        wp_series_mm = wp_series * MM;
    end

    color_paddle = [0.8941 0.4667 0.1294];
    color_wp_marker1 = [0.75 0.0 0.75];  % verde (marker no eixo superior)
    color_wp_marker2 = [0.0 0.39 0.0];% azul (marker no eixo superior)
    rest_color = [0 0 0];

    figure('Name','Sinal e Série Temporal','NumberTitle','off','Color','w');
    subplot(2,1,1);
    plot(t_fine, paddle_signal_user_mm,'LineWidth',1.0,'Color',color_paddle);
    title('Sinal bicromático');
    ylabel('\zeta_{cunha} (mm)');
    grid on;
    legend('\zeta_{cunha}','Location','northeast');

    subplot(2,1,2);
    plot(snapshot_times, wp_series_mm,'LineWidth',1.0,'Color',color_wp_marker1);
    title('Série temporal da superfície');
    xlabel('Tempo (s)');
    ylabel('\zeta (mm)');
    grid on;
    legend(sprintf('WP_{1} - %.2f m', wp1),'Location','northeast');

    tightfig();
    % -------------------- Ajuste nomes de arquivo com vírgula decimal ------
    % formata T1 e T2 com vírgula como separador decimal (ex.: 0,80)
    t1str = strrep(sprintf('%.3f', T1), '.', ',');   % "0,80"
    t2str = strrep(sprintf('%.3f', T2), '.', ',');   % "0,60"
    
    % nome do primeiro plot no padrão solicitado
    fig1_fname = sprintf('Plot_Sinal_Serie_T1-%ss_T2-%ss.png', t1str, t2str);
    
    set(gcf,'PaperPositionMode','auto');    % mantém o tamanho da figura como na tela
    try
        print(gcf, fig1_fname, '-dpng', '-r300');   % salva em PNG 300 DPI
        fprintf('Primeiro plot salvo como %s\n', fig1_fname);
    catch MEsave
        warning('Falha ao salvar o primeiro plot: %s', MEsave.message);
    end
    % ---------------------------------------------------------------------

    % =============================================================================
    % ANIMAÇÃO (2 subplots) e grava MP4
    % =============================================================================
    fig = figure('Name','Animacao Simulacao de Ondas (2 subplots)','NumberTitle','off','Color','w');
    set(fig,'Color','w');

    % cores visuais
    water_color = [0 191 255] / 255;
    wedge_color = [253 50 50] / 255;
    beach_color = [68 68 68] / 255;
    probe1_color = color_wp_marker1;  % marcadores topo (WP_1)
    probe2_color = color_wp_marker2;  % marcadores topo (WP_2)

    % localizações dos WPs (1.0 m e 1.5 m)
    wp1 = 1.00; [~, idx_wp1] = min(abs(x - wp1));
    wp2 = 6.00; [~, idx_wp2] = min(abs(x - wp2));

    nFrames = size(snapshots,1);
    filenameMp4 = sprintf('Simulacao_Ondas_Bicromatica__T1-%ss_T2-%ss.mp4', t1str, t2str);

    % preparar limites y do subplot inferior (mm)
    if ~isempty(snapshots)
        max_amp_mm = max(abs(snapshots(:))) * 1000;
        ylim_probe = max(5, 1.2 * max_amp_mm); % evita zero e muito pequeno
    else
        ylim_probe = 5; % valor default
    end

    if nFrames == 0
        warning('Nenhum snapshot gerado (nFrames == 0). Animação ignorada.');
    else
        % prepara VideoWriter
        v = VideoWriter(filenameMp4, 'MPEG-4');
        v.FrameRate = 30;
        v.Quality = 100;
        open(v);

        % definir posições manuais para espaçar os subplots e reservar área central
        % Ajustadas para dar mais espaço entre os subplots e evitar sobreposição
        ax1_pos = [0.08, 0.60, 0.88, 0.33]; % [left bottom width height] -> sobe o subplot superior
        ax2_pos = [0.08, 0.10, 0.88, 0.30]; % desce o subplot inferior

        for frame = 1:nFrames
            if ~ishandle(fig) || ~strcmp(get(fig,'Type'),'figure')
                fprintf('Figura fechada — animação interrompida (frame %d de %d)\n', frame-1, nFrames);
                break;
            end

            % ---- subplot superior: visão global do tanque ----
            ax1 = axes('Position',ax1_pos); cla(ax1); hold(ax1,'on');
            % paredes do tanque
            plot(ax1,[0, L],[0,0],'k','LineWidth',2);
            plot(ax1,[0, L],[h_tank,h_tank],'k','LineWidth',2);
            plot(ax1,[L,L],[0,h_tank],'k','LineWidth',2);
            plot(ax1,[0,0],[0,h_tank],'k','LineWidth',2);

            % praia (triângulo)
            beach_height = 0.195;
            x_left = x_beach_min;
            x_right = x_beach_max;
            wedge_x = x(wedge_index);
            beach_center = 0.5 * (x_left + x_right);
            if wedge_x < beach_center
                top_x = x_right;
            else
                top_x = x_left;
            end
            h_beach = patch(ax1, [x_left, x_right, top_x], [0, 0, beach_height], beach_color, ...
                            'FaceAlpha',0.5, 'EdgeColor','k');

            % cunha (visual)
            t_frame = (frame-1) * snap_step * dt;
            disp = paddle_disp_user(t_frame);
            x0 = 0.1;
            A_pt = [x0, h - 0.05 + disp];
            B_pt = [x0, h + 0.05 + disp];
            C_pt = [x0 + 0.1, h + 0.05 + disp];
            h_wedge = patch(ax1, [A_pt(1),B_pt(1),C_pt(1)], [A_pt(2),B_pt(2),C_pt(2)], wedge_color, 'EdgeColor', wedge_color);

            % superfície da água (patch + linha)
            u_snapshot = snapshots(frame,:);
            surface = h + u_snapshot;
            Xfill = [x, fliplr(x)];
            Yfill = [zeros(size(x)), fliplr(surface)];
            h_water = patch(ax1, Xfill, Yfill, water_color, 'FaceAlpha',0.9,'EdgeColor','none');
            h_surf_line = plot(ax1, x, surface, 'Color',[0.033,0.510,0.878],'LineWidth',2);
            h_rest = plot(ax1, [0, L],[h, h],'--','Color',[0 0 0],'LineWidth',1);

            % ---- pontos móveis dos WPs (apenas marcadores) ----
            z_wp1 = surface(idx_wp1); % em metros
            z_wp2 = surface(idx_wp2);
            h_wp1 = plot(ax1, wp1, z_wp1, 'o', 'MarkerSize',8, 'MarkerFaceColor', probe1_color, 'MarkerEdgeColor','k');
            h_wp2 = plot(ax1, wp2, z_wp2, 'o', 'MarkerSize',8, 'MarkerFaceColor', probe2_color, 'MarkerEdgeColor','k');

            xlim(ax1, [-0.2, L + 0.1]);
            ylim(ax1, [-0.05, 0.3]);
            xlabel(ax1,'x (m)');
            ylabel(ax1,'z (m)');  % <-- Alterado para z
            title(ax1, sprintf('Simulação de ondas Tanque Reditus (Tempo = %.2f s)', t_frame));

            % criar legenda centralizada (entre os subplots)
            legendEntries = {'Água', 'Superfície da água', 'Nível de repouso', sprintf('WP_{1} = %.2fm', wp1), sprintf('WP_{2} = %.2fm', wp2)};
            hLeg = legend(ax1, [h_water, h_surf_line, h_rest, h_wp1, h_wp2], legendEntries, ...
                   'Orientation','horizontal', 'Location','northoutside');
            % ajustar posição da legenda para ficar bem acima do subplot inferior
            set(hLeg,'Units','normalized');
            set(hLeg,'Position',[0.32, 0.49, 0.4, 0.035]); % move legenda mais para cima e centra
            set(hLeg,'Box','on','Color','white','FontSize',9);

            % fórmula LaTeX centralizada (η com T1 e T2 numéricos; A1/A2 simbólicos)
            formulaStr = sprintf(['$$\\eta(t)=A_1\\,\\sin\\left(\\frac{2\\pi}{%.3f}\\,t\\right) + A_2\\,\\sin\\left(\\frac{2\\pi}{%.3f}\\,t\\right)$$'], T1, T2);
            % caixa de texto centrada logo abaixo da legenda (baixada um pouco para não sobrepor o título do eixo inferior)
            annotation('textbox',[0.15, 0.42, 0.76, 0.06], 'String', formulaStr, ...
                       'Interpreter','latex', 'EdgeColor','none', 'HorizontalAlignment','center', 'BackgroundColor','white');

            hold(ax1,'off');

            % ---- subplot inferior: séries temporais dos WPs (histórico até frame) ----
            ax2 = axes('Position',ax2_pos); cla(ax2); hold(ax2,'on');
            % extrair histórico até o frame atual
            times_hist = snapshot_times(1:frame);
            probe1_hist = snapshots(1:frame, idx_wp1) * 1000; % mm
            probe2_hist = snapshots(1:frame, idx_wp2) * 1000; % mm

            % cores diferentes das usadas nos marcadores do topo
            hist_color1 = [0.75 0.0 0.75]; % magenta-ish (diferente dos marcadores)
            hist_color2 = [0.0 0.39 0.0];   % laranja (diferente dos marcadores)

            h_p1 = plot(ax2, times_hist, probe1_hist, '-', 'LineWidth',1.0, 'Color', hist_color1);
            h_p2 = plot(ax2, times_hist, probe2_hist, '-', 'LineWidth',1.0, 'Color', hist_color2);

            % marcador do instante atual (linha vertical)
            ylims = [-ylim_probe ylim_probe];
            plot(ax2, [t_frame t_frame], ylims, ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1);

            xlim(ax2, [0, snapshot_times(end)]);
            ylim(ax2, ylims);
            xlabel(ax2, 'Tempo (s)');
            ylabel(ax2, '\zeta (mm)'); % eixo vertical dos históricos permanece zeta em mm
            % title(ax2, sprintf('Histórico: WP_1 (%.2fm) e WP_2 (%.2fm) — até t = %.2fs', wp1, wp2, t_frame));
            legend(ax2, [h_p1, h_p2], {sprintf('WP_{1} - %.2fm', wp1), sprintf('WP_{2} - %.2fm', wp2)}, 'Location','northeast');

            grid(ax2,'on');
            hold(ax2,'off');

            drawnow;

            % Captura e escreve frame para MP4
            try
                frameIm = getframe(fig);
                writeVideo(v, frameIm);
            catch MEget
                warning('Falha ao capturar/gravar frame %d: %s\n', frame, MEget.message);
                break;
            end

            % limpa axes (para próxima iteração)
            delete(ax1);
            delete(ax2);
        end

        % finaliza vídeo
        close(v);
        fprintf('Vídeo salvo como %s\n', filenameMp4);
    end

catch ME
    fprintf('Ocorreu um erro: %s\n', ME.message);
    for k = 1:min(20, numel(ME.stack))
        fprintf('  em %s (linha %d)\n', ME.stack(k).name, ME.stack(k).line);
    end
end

% -------------------- Funções auxiliares --------------------
function val = dispersion_kn(k, omega_local, g_local, h_local)
    val = omega_local.^2 + g_local .* k .* tan(k .* h_local);
end

function k0 = find_k0_matlab(omega_local, g_local, h_local)
    lower = 1e-6;
    k_guess = omega_local / sqrt(g_local * h_local);
    upper = max(k_guess * 10, 1.0);
    fun = @(k) sqrt(g_local .* k .* tanh(k .* h_local)) - omega_local;
    cnt = 0;
    % garante upper suficientemente grande
    while fun(upper) < 0
        upper = upper * 2;
        cnt = cnt + 1;
        if cnt > 200
            error('Falha em encontrar bound superior para k0.');
        end
    end
    k0 = fzero(fun, [lower, upper]);
end

function tightfig()
    set(gcf,'Units','normalized');
    drawnow;
end

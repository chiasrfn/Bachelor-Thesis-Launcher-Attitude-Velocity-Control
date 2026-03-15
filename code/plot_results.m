clc;
clear;
close all;

% --- Settings ---
k = 5/0.001;    % Simulation time window / samples
c = 5;          % Line width
f = 20;         % Axis font size

% Load simulation data
sim = load("Output_c_roll_2.mat");
t = sim.out.tout(1:k);

% --- Attitude (Euler Angles) ---
% 1) Roll, Pitch, Yaw plots
figure(1)

% Roll (chi)
subplot(3,1,1)
plot(t, rad2deg(sim.out.state.Attitude.Data(1:k, 1)), 'Color', '#A603D6','LineWidth',c); % Purple
grid on
title('$\chi$ (deg)','Interpreter','latex');
set(gca,'FontSize',f);
yline(0,'--','Color','black', 'LineWidth', c);
error_chi = round(rad2deg(sim.out.state.Attitude.Data(end, 1)), 3, "significant");
annotation('textbox', [0.569791666666664,0.85907127429804,0.332812499999999,0.036106987651126], ...
    'String', "Error: " + error_chi, 'Interpreter','latex', 'FontSize',f, 'HorizontalAlignment','right','EdgeColor','none');

% Pitch (theta)
subplot(3,1,2)
plot(t,  rad2deg(sim.out.state.Attitude.Data(1:k, 2)), 'Color', '#FF7E06','LineWidth',c); % Orange
title('$\theta$ (deg)','Interpreter','latex');
grid on
set(gca,'FontSize',f);
yline(0,'--','Color','black', 'LineWidth', c);
error_theta = round(rad2deg(sim.out.state.Attitude.Data(end, 2)), 3, "significant");
annotation('textbox', [0.569921874999999,0.56263498920085,0.333854166666665,0.036106987651126], ...
    'String', "Error: " + error_theta, 'Interpreter','latex', 'FontSize',f, 'HorizontalAlignment','right', 'EdgeColor','none');

% Yaw (psi)
subplot(3,1,3)
plot(t,  rad2deg(sim.out.state.Attitude.Data(1:k, 3)), 'Color', '#75C700','LineWidth',c); % Green
title('$\psi$ (deg)', 'Interpreter','latex');
grid on
set(gca,'FontSize',f);
yline(0,'--','Color','black', 'LineWidth', c);
error_psi = round(rad2deg(sim.out.state.Attitude.Data(end, 3)), 3, "significant");
annotation('textbox', [0.571093749999999,0.266738660907116,0.333854166666666,0.036106987651126], ...
    'String', "Error: " + error_psi, 'Interpreter','latex', 'FontSize',f, 'HorizontalAlignment','right','EdgeColor','none');

% --- Angular Velocity (Body Frame) ---
figure(2)

% p (Roll rate)
subplot(3,1,1)
plot(t, rad2deg(sim.out.state.Wb.Data(1:k, 1)),'Color', '#A603D6','LineWidth',c); % Purple
title('p (deg/s)','Interpreter','latex');
grid on
set(gca,'FontSize',f);
yline(0,'--','Color','black', 'LineWidth', c);

% q (Pitch rate)
subplot(3,1,2)
plot(t, rad2deg(sim.out.state.Wb.Data(1:k, 2)), 'Color', '#FF7E06','LineWidth',c); % Orange
title('q (deg/s)','Interpreter','latex');
grid on
set(gca,'FontSize',f);
yline(0,'--','Color','black', 'LineWidth', c);

% r (Yaw rate)
subplot(3,1,3)
plot(t, rad2deg(sim.out.state.Wb.Data(1:k, 3)), 'Color', '#75C700','LineWidth',c); % Green
title('r (deg/s)','Interpreter','latex');
grid on
set(gca,'FontSize',f);
yline(0,'--','Color','black', 'LineWidth', c);

% --- Body Velocity ---
figure(3)

% u (Longitudinal velocity)
subplot(3,1,1)
plot(t, sim.out.state.Vb.Data(1:k, 1),'Color', '#A603D6','LineWidth',c); % Purple
title('u (m/s)','Interpreter','latex');
hold on
set(gca,'FontSize',f);

% Comparison with nominal case
if (sim.massa_variabile)
    u_nominal = load('Output_c_nominale_5').out.state.Vb.Data(1:k, 1);
elseif(sim.ambiente_variabile)
    u_nominal = load('Output_c_nominale_4').out.state.Vb.Data(1:k, 1);
elseif(sim.spinta_variabile)
    u_nominal = load('Output_c_nominale_3').out.state.Vb.Data(1:k, 1);
else
    u_nominal = load('Output_c_nominale_2').out.state.Vb.Data(1:k, 1);
end
plot(t,u_nominal,'--','Color','black', 'LineWidth', c);
grid on
hold off

% v (Lateral velocity)
subplot(3,1,2)
plot(t, sim.out.state.Vb.Data(1:k, 2), 'Color', '#FF7E06','LineWidth',c); % Orange
xlim([0 0.02]);
title('v (m/s)','Interpreter','latex');
grid on
set(gca,'FontSize',f);
yline(0,'--','Color','black', 'LineWidth', c);

% w (Vertical velocity)
subplot(3,1,3)
plot(t, sim.out.state.Vb.Data(1:k, 3), 'Color', '#75C700','LineWidth',c); % Green
title('w (m/s)','Interpreter','latex');
grid on
set(gca,'FontSize',f);
yline(0,'--','Color','black', 'LineWidth', c);

% --- Control Effort Comparison (TVC) ---
figure(4)
plot(t, rad2deg(sim.out.TVC.Data(1:k, 1)),'LineWidth',c, 'Color','#17A4F6');
hold on;
plot(t, rad2deg(sim.out.TVC.Data(1:k, 2)),'LineWidth',c, 'Color', '#EB367C');
grid on;

% Plot ideal control effort for comparison
plot(t, rad2deg(sim.out.TVC_d.Data(1:k, 1)),'--','LineWidth',c, 'Color','#005C92');
plot(t, rad2deg(sim.out.TVC_d.Data(1:k, 2)),'--','LineWidth',c, 'Color', '#9B023D');

title('Control Effort (deg)','Interpreter','latex');
set(gca,'FontSize',f);
legend('$\zeta$ (Yaw)','$\eta$ (Pitch)', 'Interpreter', 'latex');
hold off;

% --- Reaction Control System (RCS) Effort ---
figure(5)
plot(t, sim.out.RCS.Data(1:k),'LineWidth',c, 'Color','#17A4F6');
title('Control Effort (Nm)','Interpreter','latex');
grid on
set(gca,'FontSize',f);
legend('RCS', 'Interpreter', 'latex');

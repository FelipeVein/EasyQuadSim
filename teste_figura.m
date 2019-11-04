%% objetivo: gerar as figuras deste experimento
clc; clear all; close all

%salvar = 1; não salvar 0
to_save = 0;

fontsize = 15;

%coletando os sinais dos 5 experimentos
load('tabela2-caso1-meio-metro21-Jun-2017')

s1.t = t_val;
s1.u = u;

s1.tu = best.t_u_vect;
s1.f_v = best.f_v_x_vect;
s1.f_delta = best.f_delta_vect;
s1.penalty = best.Psi_vect;
s1.theta = best.Theta_vect;

s1.fo = best.f_o_vect;

s1.f_chi = k1*s1.theta + k2*s1.fo + k3*s1.f_delta;

fig1 = figure('units','centimeters','outerposition',[0 0 22 10]);

plot(best.Psi_vect,'LineWidth',2)

hold on
plot(s1.tu,'LineWidth',2)
plot(s1.f_delta,'LineWidth',2)
plot(s1.f_v,'LineWidth',2)
plot(s1.f_chi,'LineWidth',2)
%plot(aux_chi,'LineWidth',2)
fig1.CurrentAxes.YAxis.Scale = 'log';

h = legend('$\boldmath{\Psi}(\cdot)$','$t_u$','$\boldmath{f_{\hat{\delta}}(\cdot)}$','$\boldmath{f_V(\cdot)}$','$\boldmath{f_\Xi(\cdot)}$','Location','eastoutside');

set(h,'Interpreter','latex');
set(gca,'TickLabelInterpreter', 'latex');
set(gca,'fontsize',fontsize);

grid on

axis([1 100 0 2*max(s1.f_chi)])
xlabel('Itera\c{c}\~{a}o','Interpreter','latex')

set(gca,'YTick',[1E-5 1E-2 1E1 1E4 1E7 ])
set(gca,'XTick',[1 10:10:100])

if(to_save)
saveas(fig1,'tab2_optimization_BRA','fig')
saveas(fig1,'tab2_optimization_BRA','epsc')
saveas(fig1,'tab2_optimization_BRA','png') 
end



%% sinal de excitação

fig2 = figure('units','centimeters','outerposition',[0 0 22 7]);

plot(s1.t,s1.u,'LineWidth',2)
hold on
plot([s1.t(1) s1.t(end)],[24 24],'--k','LineWidth',2)
plot([s1.t(1) s1.t(end)],[-24 -24],'--k','LineWidth',2)

xlabel('Tempo','Interpreter','latex')
ylabel('$V$','Interpreter','latex')

%set(gca,'xticklabel',{[]})
%set(gca,'xtick',[0:2:length(delta)])

grid on
%title('(a)')
h = legend('$E_r$','$E_l$','Location','eastoutside');
set(h,'Interpreter','latex');
set(gca,'TickLabelInterpreter', 'latex');
set(gca,'fontsize',fontsize)

axis([s1.t(1) s1.t(end) -27 27])

set(gca,'YTick',-24:24:24)

if(to_save)
saveas(fig2,'tab2_figs_two_inputs_BRA','fig')
saveas(fig2,'tab2_figs_two_inputs_BRA','epsc')
saveas(fig2,'tab2_figs_two_inputs_BRA','png') 
end




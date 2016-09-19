%Maximizing reward
clear all 
close all
fsize=30;
lwidth=3;
N=108;
Na=10;
keySet={'a0r','a1r','a2r','a3r','a0h','a1h','a2h','a3h','reset','repair'};
valueSet={1,2,3,4,5,6,7,8,9,10};
discount = 0.95;
P=parse_transition();
R=reward_assignment();
mdp_check(P,R)
[V, policy] = mdp_policy_iteration(P, R, discount);
figure
plot(policy,'rs','Markersize',10,'LineWidth',lwidth,'MarkerEdgeColor','r','MarkerFaceColor','r')
xlabel('States','fontsize',fsize)
ylabel('Actions','fontsize',fsize)
ax = gca;
ay.YTick=1:10;

set(ax,'YTickLabel',keySet)
% set(gca,'YTickLabel','');
% set(gca, 'YTickLabelMode','manual')
% hxt = get(gca, 'YTick');
% xpos = min(xlim) - diff(xlim)*0.05;
% text(hxt, [1]*10*xpos, keySet, 'HorizontalAlignment', 'center')
set(gca,'fontsize',fsize)
axis tight
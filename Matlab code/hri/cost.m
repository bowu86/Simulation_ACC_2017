y=[24.26, 16.1, 11.65, 10.65 ]; %robot only, human only, with order, without order
x=[1 2 3 4];
fsize=30;
figure
bar(x,y)
set(gca,'xtick',x)
set(gca,'xticklabel',{'Robot only','Human only','LTL in (5)','LTL in (4)'})
set(gca,'fontsize',fsize)
ylabel('Average cost per cycle from the initial state','FontSize',fsize)


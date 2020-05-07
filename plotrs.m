function plotrs()
global r actual_r
load field.mat

figure
hold on
grid on
axis equal
plot(r(:,1),r(:,2),'o-');
plot(actual_r(:,1),actual_r(:,2),'.-');
axis([-1.5,2.5,-3.37,1])

xlabel('i')
ylabel('j')
contour(X, Y, V_tot, 40, 'ShowText', 'off');
plot(0,0,'k*')
plot(BoB(1),BoB(2),'ko');
plot(Obstacles(:,1),Obstacles(:,2),'rs')

legend({'planned position (r)','actual position (actual r)','field contour','start','end','obstacle',},'Location','eastoutside')

end
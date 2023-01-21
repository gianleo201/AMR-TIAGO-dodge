figure(10); clf
whitebg([1.0 1.0 1.0])
set(gcf,'Color',[1 1 1])

% compute dierct kinematics
dk_points = dk_tiago(state_sim(end,1:3));
p1 = dk_points(1,:);
p2 = dk_points(2,:);
p3 = dk_points(3,:);

temp_var = dk_tiago(Xref(end,1:3));
x_r = temp_var(3,1);
y_r = temp_var(3,2);

subplot(3,1,[1 2]);

% plot the robot body
line([p1(1) p1(1)], [0 p1(2)], 'color', 'k', 'Linewidth',4); hold on;
plot(p1(1),p1(2),'ro','MarkerSize',15,'Linewidth',10);
line([p1(1) p2(1)], [p1(2) p2(2)], 'color', 'k', 'Linewidth',4);
plot(p2(1),p2(2),'ro','MarkerSize',15,'Linewidth',10);
line([p2(1) p3(1)], [p2(2) p3(2)], 'color', 'k', 'Linewidth',4);
plot(p3(1),p3(2),'ro','MarkerSize',15,'Linewidth',10);

% plot the target position of the end-effector
plot(x_r, y_r, 'gx', 'MarkerSize', 16, 'Linewidth', 2);

grid on;
xlim([-5 5]);
ylim([0 2]);

subplot(3,1,3);
semilogy(time(1:end-1),KKT_MPC,'linewidth',1.5,'color','k','linestyle','--','marker','.');hold on

grid on;
xlabel('t [s]','FontSize',13);    ylabel('MPC KKT','FontSize',13)
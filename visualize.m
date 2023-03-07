figure(10); clf
whitebg([1.0 1.0 1.0])
set(gcf,'Color',[1 1 1])

% compute dierct kinematics
dk_points = dk_tiago(state_sim(end,1:3));
p1 = dk_points(1,:);
p2 = dk_points(2,:);
p3 = dk_points(3,:);

x_r = Yref(1,1);
y_r = Yref(1,2);

subplot(4,1,[1 2]);

% plot the robot body
line([p1(1)-rt4 p1(1)-rt4], [0 p1(2)], 'color', 'k', 'Linewidth',4,"color","Black"); hold on;
line([p1(1)-rt4 p1(1)], [p1(2) p1(2)], 'color', 'k', 'Linewidth',4,"color","Black");
plot(p1(1),p1(2),'o','MarkerSize',5,'Linewidth',5);
line([p1(1) p2(1)], [p1(2) p2(2)], 'color', 'k', 'Linewidth',4,"color","Blue");
plot(p2(1),p2(2),'o','MarkerSize',5,'Linewidth',5);
line([p2(1) p3(1)], [p2(2) p3(2)], 'color', 'k', 'Linewidth',4,"color","Green");
plot(p3(1),p3(2),'o','MarkerSize',5,'Linewidth',5);


% plot the target position of the end-effector
plot(x_r, y_r, 'gx', 'MarkerSize', 16, 'Linewidth', 2,"Color","cyan");

% plot the obstacle
viscircles(obs(1:2),obs(3));

if VelocityDamper == 1
  %plot the security distance area
  viscircles(obs(1:2), obs(3)+ds, 'LineStyle', ':');

  %plot the influence distance area
  viscircles(obs(1:2), obs(3)+di, 'LineStyle', '--');
end

% plot trajectory
plot(h_ref(:,1),h_ref(:,2),"LineWidth",2.0,"LineStyle","--","Color","cyan");

grid on;
xlim([-3 3]);
ylim([0 2]);
pbaspect([3 1 1]);

subplot(4,1,3);
semilogy(time(1:end-1),KKT_MPC,'linewidth',1.5,'color','k','linestyle','--','marker','.');hold on

grid on;
xlabel('t [s]','FontSize',13);    ylabel('MPC KKT','FontSize',13)

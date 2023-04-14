clf(myFig);
% whitebg([1.0 1.0 1.0]);
set(gcf,'Color',[1 1 1]);


% compute dierct kinematics
dk_points = dk_tiago(state_sim(end,1:3));
p1 = dk_points(1,:);
p2 = dk_points(2,:);
p3 = dk_points(3,:);

%center of the links
c1=dk_points(4,:);
c2=dk_points(5,:);
c3=dk_points(6,:);

x_r = Yref(1,1);
y_r = Yref(1,2);

% subplot(4,1,[1 2]);

% plot the robot body
rectangle("Position",[p1(1)-rt4+rt1-(db/2) rw db hb], 'Linewidth',4); hold on; % base rectangle
viscircles([p1(1)-rt4+rt1,rw],rw, 'Linewidth',3,"color","Black");  % driving wheel
line([p1(1)-rt4+rt1-(db/2) p1(1)-rt4+rt1-(db/2)],[rw/2 rw], 'Linewidth',2,"color","Black"); % left caster
viscircles([p1(1)-rt4+rt1-(db/2),rw/2],rw/2, 'Linewidth',1,"color","Black");
line([p1(1)-rt4+rt1+(db/2) p1(1)-rt4+rt1+(db/2)],[rw/2 rw], 'Linewidth',2,"color","Black"); % rigth caster
viscircles([p1(1)-rt4+rt1+(db/2),rw/2],rw/2, 'Linewidth',1,"color","Black");
line([p1(1)-rt4 p1(1)-rt4], [rw+hb p1(2)], 'color', 'k', 'Linewidth',4,"color","Black");
line([p1(1)-rt4 p1(1)], [p1(2) p1(2)], 'color', 'k', 'Linewidth',4,"color","Black");
plot(p1(1),p1(2),'o','MarkerSize',5,'Linewidth',5);
line([p1(1) p2(1)], [p1(2) p2(2)], 'color', 'k', 'Linewidth',4,"color","Blue");
plot(p2(1),p2(2),'o','MarkerSize',5,'Linewidth',5);
line([p2(1) p3(1)], [p2(2) p3(2)], 'color', 'k', 'Linewidth',4,"color","Green");
plot(p3(1),p3(2),'o','MarkerSize',5,'Linewidth',5);


%plot the first link body
viscircles(c1,(hb+ht+rw)/2, 'Color', 'k');
%plot the second link body
viscircles(c2,l2/2, 'Color', 'b');
%plot the third link body
viscircles(c3,l3/2, 'Color', 'g');

% plot the target position of the end-effector
plot(x_r, y_r, 'gx', 'MarkerSize', 16, 'Linewidth', 2,"Color","cyan");

% plot the obstacle
viscircles(obs(iter+1,1:2),obs(iter+1,3));

%plot the security distance area
  viscircles(obs(iter+1,1:2), obs(iter+1,3)+ds, 'LineStyle', ':');

if CONSTRAINT_TYPE == 3
  %plot the influence distance area
    viscircles(obs(iter+1,1:2), obs(iter+1,3)+di, 'LineStyle', '--');
end

% plot trajectory
plot(h_ref(:,1),h_ref(:,2),"LineWidth",2.0,"LineStyle","--","Color","cyan");

% plot the ZMP position
plot(state_sim(end,1)+ZMP_VALUES(end), 0, 'gx', 'MarkerSize', 16, 'Linewidth', 2,"Color","green");

grid on;
xlim([-3 3]);
ylim([0 2.08]);
pbaspect([3 1 1]);

if RECORD_SIMULATION

% settings for video recording
set(gcf, 'Position',[0 0 1920 862]);
% write the frame
frame = getframe(gcf);    
writeVideo(writerObj, frame);

end
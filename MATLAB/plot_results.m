close all
clc
%% Height only
figure
plot(-result(:,3))
title('Height')

%% X v Y
figure
plot(result(:,1)+2,result(:,2)+1)
title('X vs Y')
xlabel('meters [m]')
ylabel('meters [m]')

%% Rotor Speeds
figure
subplot(2,2,1)
title('Rotor Speeds')
plot(result(:,13))
subplot(2,2,2)
title('Rotor Speeds')
plot(result(:,14))
subplot(2,2,3)
title('Rotor Speeds')
plot(result(:,15))
subplot(2,2,4)
title('Rotor Speeds')
plot(result(:,16))
title('Rotor Speeds')

%% X, Y & Z
% close all
% clc
figure
subplot(3,1,1)
plot(result(:,1))
title('X')
subplot(3,1,2)
plot(result(:,2))
title('Y')
subplot(3,1,3)
plot(-result(:,3))
title('Z')

% xlabel('Timesteps [discrete]')
% ylabel('meters [m]')

h = axes('Position',[0 0 1 1],'Visible','off'); %add an axes on the left side of your subplots
set(gcf,'CurrentAxes',h)
text(.06,.45,'meters [m]',...
'VerticalAlignment','bottom',...
'HorizontalAlignment','left', 'Rotation', 90, 'FontSize',12)
text(.5,.01,'Timesteps [discrete]',...
'VerticalAlignment','bottom',...
'HorizontalAlignment','center','FontSize',12)

%% 3D Plot
figure
plot3(result(:,1)+2,result(:,2)+1,-result(:,3)/5,'r')
title('Trajectory in 3D Space')
xlabel('X meters [m]')
ylabel('Y meters [m]')
zlabel('Z meters [m]')
view(45,30)
grid on
%%
XX = stepinfo(result(:,1));
X_Overshoot = XX.Overshoot
X_Settling = XX.SettlingTime./length(result)*15
YY = stepinfo(result(:,2));
Y_Overshoot = YY.Overshoot
Y_Settling = YY.SettlingTime./length(result)*15
ZZ = stepinfo(-result(:,3));
Z_Overshoot = ZZ.Overshoot
Z_Settling = ZZ.SettlingTime./length(result)*15


%% Torques
figure
subplot(3,1,1)
plot(result(:,16).^2-result(:,14).^2)
title('Roll Torque')

subplot(3,1,2)
plot(result(:,13).^2-result(:,15).^2)
title('Pitch Torque')

subplot(3,1,3)
plot(result(:,13).^2+result(:,15).^2-result(:,14).^2-result(:,16).^2)
title('Yaw Torque')

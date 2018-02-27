% Bryce Mack
% Bradley University
% Senior Project
% 10/14/17


ss = [3 10];
quad = quadrotor;

global a1s b1s
    
% not quite sure what this is about -- PIC
if numel(a1s) == [0];
    a1s = zeros(1, quad.nrotors);
    b1s = zeros(1, quad.nrotors);
end

% vehicle dimensons
d = quad.d; %Hub displacement from COG
r = quad.r; %Rotor radius

for i = 1:quad.nrotors
    theta = (i-1)/quad.nrotors*2*pi;
    %   Di      Rotor hub displacements (1x3)
    % first rotor is on the x-axis, clockwise order looking down from above
    D(:,i) = [ d*cos(theta); d*sin(theta); 0];
    scal = ss(1)/4;
    %Attitude center displacements
    C(:,i) = [ scal*cos(theta); scal*sin(theta); 0];
end

figure;

%CREATE VIDEO OBJECT
vidObj = VideoWriter('C:\Users\Bryce\Google Drive\Crazy-Flies Senior Project 2017-18\Videos\Quadrotor_Flight_Sq_Traj');
open(vidObj);

for index = 1:length(result)
    
    axis([-ss(1) ss(1) -ss(1) ss(1) 0 ss(2)])
    s = ss(1);
    hold on;

    % plot the ground boundaries and the big cross
    plot3([-s -s],[s -s],[0 0],'-b')
    plot3([-s s],[s s],[0 0],'-b')
    plot3([s -s],[-s -s],[0 0],'-b')
    plot3([s s],[s -s],[0 0],'-b')
    plot3([s -s],[-s s],[0 0],'-b')
    plot3([-s s],[-s s],[0 0],'-b')
    grid on

    %READ STATE
    z = [result(index,1);
         result(index,2);
         result(index,3)];
     
    n = [result(index,4);
         result(index,5);
         result(index,6)];

    %PREPROCESS ROTATION MATRIX
    phi = n(1);    %Euler angles
    the = n(2);
    psi = n(3);

    R = [cos(the)*cos(phi) sin(psi)*sin(the)*cos(phi)-cos(psi)*sin(phi) cos(psi)*sin(the)*cos(phi)+sin(psi)*sin(phi);   %BBF > Inertial rotation matrix
    cos(the)*sin(phi) sin(psi)*sin(the)*sin(phi)+cos(psi)*cos(phi) cos(psi)*sin(the)*sin(phi)-sin(psi)*cos(phi);
    -sin(the)         sin(psi)*cos(the)                            cos(psi)*cos(the)];

    %Manual Construction
    %Q3 = [cos(psi) -sin(psi) 0;sin(psi) cos(psi) 0;0 0 1];   %Rotation mappings
    %Q2 = [cos(the) 0 sin(the);0 1 0;-sin(the) 0 cos(the)];
    %Q1 = [1 0 0;0 cos(phi) -sin(phi);0 sin(phi) cos(phi)];
    %R = Q3*Q2*Q1;    %Rotation matrix

    %CALCULATE FLYER TIP POSITONS USING COORDINATE FRAME ROTATION
    F = [1   0   0;
         0  -1   0;
         0   0  -1];

    %Draw flyer rotors
    t = [0:pi/8:2*pi];
    for j = 1:length(t)
        circle(:,j) = [r*sin(t(j));r*cos(t(j));0];
    end

    for i = 1:quad.nrotors
        hub(:,i) = F*(z + R*D(:,i)); %points in the inertial frame

        q = 1; %Flapping angle scaling for output display - makes it easier to see what flapping is occurring
        Rr = [cos(q*a1s(i))  sin(q*b1s(i))*sin(q*a1s(i)) cos(q*b1s(i))*sin(q*a1s(i));   %Rotor > Plot frame
            0              cos(q*b1s(i))               -sin(q*b1s(i));
            -sin(q*a1s(i)) sin(q*b1s(i))*cos(q*a1s(i)) cos(q*b1s(i))*cos(q*a1s(i))];

        tippath(:,:,i) = F*R*Rr*circle;
        plot3([hub(1,i)+tippath(1,:,i)],[hub(2,i)+tippath(2,:,i)],[hub(3,i)+tippath(3,:,i)],'b-')
    end

    %Draw flyer
    hub0 = F*z;  % centre of vehicle
    for i = 1:quad.nrotors
        % line from hub to centre plot3([hub(1,N) hub(1,S)],[hub(2,N) hub(2,S)],[hub(3,N) hub(3,S)],'-b')
        plot3([hub(1,i) hub0(1)],[hub(2,i) hub0(2)],[hub(3,i) hub0(3)],'-b')

        % plot a circle at the hub itself
        plot3([hub(1,i)],[hub(2,i)],[hub(3,i)],'o')

    end

    % plot the vehicle's centroid on the ground plane
    plot3([z(1) 0],[-z(2) 0],[0 0],'--k')
    plot3([z(1)],[-z(2)],[0],'xk')
    plot3(-s,s,-z(3),'xk')
    
%     [xx, yy] = meshgrid(-s:0.1:s);
%     zz = -z(3)*ones(length(xx));
%     plane = surf(xx,yy,zz);
%     shading flat
%     colormap gray
%     alpha(plane,0.5)
   

    % plot the vehicle's height on the z axis
    % label the axes
    xlabel('x');
    ylabel('y');
    zlabel('z (height above ground)');

    %global anim
    %anim.add();

    %WRITE VIDEO
    currFrame = getframe(gcf);
    writeVideo(vidObj,currFrame);
    
    % clear figure
    clf;
    
    fprintf('%d %% done\n', index/length(result))
end
close(vidObj)

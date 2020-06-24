clear;


% Use a fixed random seed to ensure repeatable results

rng(0.5);

%Declare number of initialized humans
numHum = 1;

%Declar initial position of human

xh = (rand(numHum,1)-0.5)*4; %Generate random x values in a numHumx1 vector in intervals [-2, 2]
yh = rand(numHum,1) + 1.75; %Generate random y values in a numHumx1 vector in intervals [1.75,2.75]
zh = rand(numHum,1)./2 + 0.5; %Generate random z values in a numHumx1 vector in intervals [0.5,1]

%Declare human randomness
humanRandom = true;

%Current Pose of human(s) (Relative to base of coordinates)

for i = 1:numHum
    Ph(:,i) = [xh(i); yh(i); 0]; % Vector pointing to bottom of human
    Uh(:,i) = [xh(i); yh(i); zh(i)]; %Vector pointing to head of human
    Rh(i) = 0.1; % The radius of the human
end

% Create platforms as start and end goals
platform1 = collisionBox(0.25,0.25,0.25);
platform1.Pose = trvec2tform([-1.6 0.8 0.125]);

platform2 = collisionBox(0.25,0.25,0.25);
platform2.Pose = trvec2tform([1.6 0.8 0.125]);

platform3 = collisionBox(0.25,0.25,0.25);
platform3.Pose = trvec2tform([0 1.5 0.2]);

%Load Robot model
robot = loadrobot("fanucM16ib","DataFormat","column","Gravity",[0 0 -9.81]);

%Matrices for start and end configurations joint angles
endPose = trvec2tform([-1.6,0.8,0.375])*axang2tform([0 1 0 pi/2]);
pickPose = trvec2tform([0,1.5,0.375])*axang2tform([0 1 0 pi/2]);
startPose = trvec2tform([1.6,0.8,0.375])*axang2tform([0 1 0 pi/2]);


%Use Inverse Kinematics for Joint Angles
ik = inverseKinematics("RigidBodyTree",robot);
weights = ones(1,6);
startConfig = ik("tool0",startPose,weights,robot.homeConfiguration);
pickConfig = ik("tool0",pickPose,weights,robot.homeConfiguration);
endConfig = ik("tool0",endPose,weights,robot.homeConfiguration);

% Show initial and final positions
figure;
show(robot,startConfig);
figure;
show(robot,pickConfig);
figure;
show(robot,endConfig);

%Initialize Joint transforms (for 6 Revolute joints)
US2 = eye(4);
US3 = eye(4);
US4 = eye(4);
US5 = eye(4);
US6 = eye(4);
US7 = eye(4);

%Generate Joint angle trajectory for human moving in Yaxis
samples = 400;
finPosy = 1;
finPosx = -1;
Ymove = [yh:-(yh-finPosy)/(samples-1):finPosy];
Xmove = [xh:-(xh-finPosx)/(samples-1):finPosx];

%Define Trajectory with a maxVelocity
maxVel = 0.8;
endTime = 1; %Defines how long each configuration will last for (i.e from startConfig -> pickConfig (1sec), pickConfig-> EndConfig -> (1 sec) 

%Create trapezium-like velocity -> Quadratic displacement (smooth movement)
[q,qd,qdd,t] = trapveltraj([homeConfiguration(robot),startConfig,pickConfig,endConfig],samples, "EndTime", endTime);
%Makes sure generated Trajectory stays within speed limit
while checkSpeed(qd,maxVel) == false
    endTime = endTime + 2;
    [q,qd,qdd,t] = trapveltraj([homeConfiguration(robot),startConfig,pickConfig,endConfig],samples, "EndTime", endTime);
end

%Define vectors 
alldist = NaN(5,length(q)); %Vector for all distances
xis = zeros(1,length(q)); %Vector for x positions of human
yis = zeros(1,length(q)); %Vector for y positions of human
lastSpot = length(q); %Number for when the robot will stop

stop_flag = 0; %Flag for robot stop condition

%Check to see if robot violated S distance
for i = 1:length(q)
    %Define all homogenous transformation matrices of robot joints
    US2 = robot.getTransform(q(:,i),'link_1');
    US3 = robot.getTransform(q(:,i),'link_2');
    US4 = robot.getTransform(q(:,i),'link_3');
    US5 = robot.getTransform(q(:,i),'link_4');
    US6 = robot.getTransform(q(:,i),'link_5');
    US7 = robot.getTransform(q(:,i),'link_6');
    %Update human(s) locations
    for j = 1:numHum
        if humanRandom == false 
            Ph(:,j) = [Xmove(1,i);Ymove(1,i);0];
            Uh(:,j) = [Xmove(1,i);Ymove(1,i);zh];
        else 
            xis(i) = (rand)*1.5/samples;
            yis(i) = (rand)*1.5/samples;
            Ph(:,j) = [Ph(1,j)+xis(i);Ph(2,j)-yis(i);0];
            Uh(:,j) = [Uh(1,j)+xis(i);Uh(2,j)-yis(i);zh(j)];
        end
    end
    clear stop_flag;
    [stop_flag,distance] = stop_flag(US2, US3, US4,US5,US6,US7, Ph, Uh, Rh, maxVel); %Update Stop_flag
    for k = 1:numHum  %record of all minimum distance across trajectory for each human
        alldist(k,i) = distance(k);
    end
    if stop_flag == 1  %Make sure human not within distance to disturb robot
        lastSpot = i 
        break;
    end    
end
get_S(maxVel)  %Lines to check values of Sp and minimum distance
min(alldist,[],2,'omitnan')


%plot the trajectory movements
show(platform1);
ax2 = gca;
hold all;
axis equal;
show(platform2);
show(platform3);

[x,y,z] = cylinder(0.1);

for i = 1:numHum
    humanY(i,:) = reshape(y+yh(i),1,[]);
    humanX(i,:) = reshape(x+xh(i),1,[]);
    humanZ(i,:) = reshape(z*Uh(3,i)+Ph(3,i),1,[]);
    s(i)=surf(reshape(humanX(i,:),[2,21]),reshape(humanY(i,:),[2,21]),reshape(humanZ(i,:),[2,21]));

end
% Loop through the other positions

for i = 1:lastSpot
    for j = 1:numHum
    %Plot human movement
        if humanRandom == false
            humanY(j,:) = reshape(y - Ymove(1,i),1,[]);
            humanX(j,:) = reshape(x - Xmove(1,i),1,[]);
        else
            humanY(j,:) = reshape(reshape(humanY(j,:),[2,21]) - yis(1,i),1,[]);
            humanX(j,:) = reshape(reshape(humanX(j,:),[2,21]) - xis(1,i),1,[]);
        end
    s(j).YData = reshape(humanY(j,:),[2,21]);
    s(j).XData = reshape(humanX(j,:),[2,21]);
    end
    show(robot,q(:,i),"Parent",ax2,"PreservePlot",false,"Visuals","on");
    axis([-3,3,-0.5,3,0,2]);
    xticks(-3:0.5:3);
    yticks(-0.5:0.5:3);
    % Update the figure    
    drawnow
   % pause(0.05)
end


%Plot Joint Position

figure
plot(t,q)
xlabel("Time")
ylabel("Joint Position")


%Plot Joint Velocity
figure
plot(t,qd)
xlabel("Time")
ylabel("Joint Velocity")

%Plot Joint Acceleration

figure
plot(t,qdd)
xlabel("Time")
ylabel("Joint Acceleration")
%}

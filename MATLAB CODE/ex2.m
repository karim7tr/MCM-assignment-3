%% Modelling and Control of Manipulator assignment 3 - Exercise 2 and 3: Inverse Kinematic Control
addpath('include')
model = load("panda.mat"); % don't worry about the warnings
% Simulation Parameters
ts = 0.5;
t_start = 0.0;
t_end = 30.0;
t = t_start:ts:t_end;

% Initial Joints configuration
q_init = [0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]';
% Joint limits
qmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
qmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];
% Initial transformation from <base> to <e-e>
bTe = getTransform(model.franka,[q_init',0,0],'panda_link7'); %useful for save initial end-effector orientation w.r.t robot base
%% END-EFFECTOR Goal definition 
bOge = [0.6; 0.4; 0.4];
edv=-pi/4;
Rotz = [cos(edv) -sin(edv) 0;sin(edv)  cos(edv) 0;0 0 1 ];% Rotation around the z axis with angle edv
 eRge = Rotz;% rotation matrix from end effector to goal effector
bRe(:,:)= bTe(1:3,1:3);% rotation matrix from base to end effector
bTge = [bRe*eRge bOge;0 0 0 1];% transformation matrix from base to goal effector
disp("bTge= ");disp(bTge);

%% TOOL Goal definition
bOgt = [0.6; 0.4; 0.4];
bdg=[0,0,0.2]';
eTt = [eye(3) bdg; 0 0 0 1];

% Transformation from te base to the tool
bTt = bTe*eTt; 
disp("bTt= ");disp(bTt);
tRg = bTt(1:3,1:3)*Rotz;% rotation matrix from tool to goal
disp("tRg= ");disp(tRg);
bTgt = [tRg bOgt; 0 0 0 1];% transformation matrix from base to goal tool
disp("bTgt= ");disp(bTgt);
% Control Proportional Gain 
angular_gain = 0.2;
linear_gain = 0.2;
% Preallocation variables
x_dot = zeros(6,1);
error_linear = zeros(3,1);
error_angular = zeros(3,1); 
% Start the inverse kinematic control 
tool = false; % change to true for using the tool 
q = q_init;

for i = t
    %% Compute the cartesian error to reach the goal
    if tool == true %compute the error between the tool frame and goal frame
        
        % Computing transformation matrix from base to end effector 
        bTe = getTransform(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        tmp = geometricJacobian(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        bJe = tmp(1:6,1:7); %DO NOT EDIT
       

        % computing tRgt rotation matrix from tool to goal tool
        tRgt = transpose(bTt(1:3,1:3)) * bTgt(1:3,1:3);

        % angular and linear error computation
        error_angular = (edv)*bTgt(1:3,3)-(edv)*bTt(1:3,3);
        error_linear = bTgt(1:3,4)-bTt(1:3,4);
        % linear velocity is the difference between vector from base to
        % goal tool and base to tool
        
    else % compute the error between the e-e frame and goal frame
        % Computing transformation matrix from base to end effector 
        bTe = getTransform(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        % Computing end effector jacobian w.r.t. base
        tmp = geometricJacobian(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        bJe = tmp(1:6,1:7); %DO NOT EDIT

        % computing the rotation eRg from "ee" to goal"
        eRge = transpose(bTe(1:3,1:3))*bTge(1:3,1:3);

        % angular and linear error computation
        error_angular = (edv)*bTge(1:3,3)-(edv)*bTe(1:3,3);
        error_linear = bTge(1:3,4)-bTe(1:3,4);
        % linear velocity is the difference between vector from base to
        % goal and base to end-effector
    end
           
    %% Compute the reference velocities

    % angular velocity
    angular_velocity = angular_gain*error_angular;

    % linear velocity
    linear_velocity = linear_gain*error_linear;

    x_dot = [angular_velocity; linear_velocity];
   
    %% Compute desired joint velocities 
    if tool == true
        % transformation from "ee" to "tool"
        rot = bTe(1:3,1:3) * eTt(1:3,4);
        skew =[0 -rot(3) rot(2); rot(3) 0 -rot(1); -rot(2) rot(1) 0];
        ps = [eye(3) zeros(3);-skew eye(3)];
        bJt = ps * bJe;% The jacobian from base to tool
        % joint velocity
        q_dot = pinv(bJt)*x_dot;
    else
        q_dot = pinv(bJe)*x_dot;
    end

    %% Simulate the robot - implement the function KinematicSimulation()
    q = KinematicSimulation(q(1:7), q_dot,ts, qmin, qmax);
    
    % DO NOT EDIT - plot the robot moving
    show(model.franka,[q',0,0],'visuals','on');%switch visuals to off for seeing only the frames
    hold on
    if tool == true
        plot3(bTt(1,4),bTt(2,4),bTt(3,4),'go','LineWidth',15);
        plot3(bOgt(1),bOgt(2),bOgt(3),'ro','LineWidth',5);
    else
        plot3(bTe(1,4),bTe(2,4),bTe(3,4),'go','LineWidth',15);
        plot3(bOge(1),bOge(2),bOge(3),'ro','LineWidth',5);
    end
    drawnow
    if(norm(x_dot) < 0.01)
        disp('REACHED THE REQUESTED GOAL POSITION !!!!!!!')
        break
    end
    if i < t_end
        % function that clean the window.
        cla();
    end
end
hold off

%% Exercice 3
% Q3.6

% New tool goal
bTgt = [0.9986 -0.0412 -0.0335 0.6;

            0.0329 -0.0163 0.9993 0.4; 

           -0.0417 -0.9990 -0.0149 0.4; 

           0 0 0 1];

bOgt = bTgt(1:3,4);
q = q_init;
for i = t
    %% Compute the cartesian error to reach the goal
        
    % Computing transformation matrix from base to end effector 
    bTe = getTransform(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
    tmp = geometricJacobian(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
    bJe = tmp(1:6,1:7); %DO NOT EDIT
    % ... 
    bTt = bTe * eTt;
    
    % computing tRgt rotation matrix from tool to goal tool
    tTgt = transpose(bTt)*bTg*eTt;
    tRgt = tTgt(1:3,1:3);

    % angular and linear error computation
    error_angular = (edv)*bTgt(1:3,3)-(edv)*bTt(1:3,3);
    error_linear = bTgt(1:3,4)-bTt(1:3,4);
    % linear velocity is the difference between vector from base to
    % goal tool and base to tool
    
    % Compute the reference velocities

    % angular velocity
    angular_velocity = angular_gain*error_angular;

    % linear velocity
    linear_velocity = linear_gain*error_linear;

    x_dot = [angular_velocity; linear_velocity];
   
    % Compute desired joint velocities 
    % transformation from "ee" to "tool"
    rotz = bTe(1:3,1:3) * eTt(1:3,4);
    skew =[0 -rotz(3) r_et(2); rotz(3) 0 -rotz(1); -rotz(2) rotz(1) 0];
    PS = [eye(3) zeros(3);-skew eye(3)];

    % The jacobian from base to tool is
    bJt = PS * bJe;

    % joint velocity
    q_dot = pinv(bJt)*x_dot;

    % Simulate the robot - implement the function KinematicSimulation()
    q = KinematicSimulation(q(1:7), q_dot,ts, qmin, qmax);
    
    % DO NOT EDIT - plot the robot moving
    show(model.franka,[q',0,0],'visuals','on');%switch visuals to off for seeing only the frames
    hold on

    plot3(bTt(1,4),bTt(2,4),bTt(3,4),'go','LineWidth',15);
    plot3(bOgt(1),bOgt(2),bOgt(3),'ro','LineWidth',5);

    drawnow
    if(norm(x_dot) < 0.03)
        disp('REACHED THE REQUESTED GOAL POSITION')
        break
    end
    if i < t_end
        % function that clean the window.
        cla();
    end
end
hold off
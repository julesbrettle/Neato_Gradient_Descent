% QEA1b Final Project: Gauntlet Challenge
% K. Hinh, J. Brettle
% Spring 2020

%% _____________________configure program______________________________
rosshutdown(); rosinit('localhost',11311, 'NodeHost','host.docker.internal')
clc
clear all
global lambda stepNum grad r_current theta r actual_r position

%% _____________________load and place Neato___________________________
close all
pubvel = rospublisher('raw_vel');

% Stop the robot if it's going right now
message = rosmessage(pubvel);
message.Data = [0, 0];
send(pubvel, message);

% Place the neato at the (0,0) with heading of postive i_hat
placeNeato(0, 0, 1, 0);
pause(2);

% Getting current position
sub_states = rossubscriber('/gazebo/model_states', 'gazebo_msgs/ModelStates');
odom_sub = rossubscriber('odom');

% update position
updatePosition(receive(odom_sub));

%% ______________________set constants________________________________
d = 0.235;
wheel_velocity = .05;
r = [0, 0]; % r0
actual_r = [];
lambda = 0.1;
sigma = 1.1;

stepNum = 0;
r_current = [0 0];
%% ____________________create the map and gradient____________________
syms x y % declare variables

% makeField(1,1) % uncomment to re-calculate field vectors
load field.mat % from makeField(wantGraphs,wantSave)

v_grad = gradient(v_tot);

% Substitute v_grad's X and Y with r_x and r_y
grad = -double(subs(v_grad, {x,y}, {r(1,1), r(1,2)}));
theta = position(3);

%% ____________________main control loop_______________________________

statusReport('starting main control loop')

while (norm(grad) > .5)
    % Update stuff
    r_prev = r(end, :);
    grad = -double(subs(v_grad, {x,y}, {r_prev(1,1), r_prev(1,2)}));
    [actual_x, actual_y] = getNeatoPosition(receive(sub_states));
    actual_r = [actual_r; actual_x, actual_y];
    statusReport('grad updated')
    
% ** CALCULATE NEXT STEP **

    % Calculate new r_current
    r_current = r_prev + lambda.*grad';
    r = [r;r_current]; % Adds the current r to the growing array of r's.
    statusReport('r_current updated')

    % Calculate new rotation angle
    vector = r_current - r_prev;
    [theta,rho] = cart2pol(vector(1),vector(2));
    theta = wrapTo2Pi(theta);
    rotate = double(theta - position(3))

% ** MOVE NEATO TO NEW R **
    
% *** rotate ***
    omega = (wheel_velocity - (-1 * wheel_velocity)) / d;
    time = abs(rotate / omega)

    message.Data = [-sign(rotate)*wheel_velocity, sign(rotate)*wheel_velocity];
    send(pubvel, message);

    start = rostime('now');
    while (1)
        rotate = double(theta - position(3));
        message.Data = [-sign(rotate)*wheel_velocity, sign(rotate)*wheel_velocity];
        send(pubvel, message);

        elapsed = rostime('now') - start;
        updatePosition(receive(odom_sub));

        if (elapsed.seconds > time)
            message.Data = [0, 0];
            send(pubvel, message);
            statusReport('reached time limit')
            break;
        end
        if abs(rotate) < 0.05
            message.Data = [0, 0];
            send(pubvel, message);
            statusReport('reached theta')
            break;
        end
        statusReport('rotating')
    end
    message.Data = [0,0];
    statusReport('done rotating')
    
% *** drive ***
    distance = norm(r_current - r_prev);
    time = abs(distance / wheel_velocity);
    message.Data = [wheel_velocity, wheel_velocity];
    send(pubvel, message);
    
    statusReport('starting drive')
    start = rostime('now');
    while (1)
        current = rostime('now');
        elapsed = current - start;
%         statusReport('driving')
        if (elapsed.seconds > time)
            message.Data = [0, 0];
            send(pubvel, message);
            break;
        end
    end
    statusReport('done driving')

    % Update lambda
    lambda = lambda * sigma;
    stepNum = stepNum +1;
    statusReport('lambda and stepNum updated, starting next loop')
end

% record last step
[actual_x, actual_y] = getNeatoPosition(receive(sub_states));
actual_r = [actual_r; actual_x, actual_y];

statusReport('reached target. done with main control loop')

message.Data = [0, 0];
send(pubvel, message);
statusReport('robot stopped')

save('r_run1','r','actual_r'); disp('path data saved');
%Phong Thanh Trinh

clear; close; clc;

% Set parameters
f1 = @(x1,x2) x1 - x2; f2 = @(x1,x2) x1.*(x1 - 1);
x1_range = [-1,2]; x2_range = [-1,2]; step = 0.2;

% Plot vector field
x1 = x1_range(1):step:x1_range(2);
x2 = x2_range(1):step:x2_range(2);
[X1,X2] = meshgrid(x1,x2);
quiver(X1,X2,f1(X1,X2),f2(X1,X2),'b'); 
hold on;
axis equal; 
xlim(x1_range);
ylim(x2_range);

% Plot nullclines using fimplicit
fimplicit(f1, x1_range,'--')
fimplicit(f2, x2_range)

% Find critical points
response = input('Find a critical point? (y/n): ','s');
while response == 'y'
    x0 = input('Enter an initial guess [x1,x2]: ');
    % Compute critical point
    F = @(x) [f1(x(1), x(2)); f2(x(1),x(2))];
    options = optimoptions('fsolve', 'Display', 'none');
    critical = fsolve(F, x0,options);
    disp(['Found a critical point at x1 = ', num2str(critical(1)), ' and x2 = ', num2str(critical(2)), '.'])
    plot(critical(1), critical(2), 'r.', 'Markersize', 20);
    response = input('Find a critical point? (y/n): ','s');
end

% Define RHS of system of equations
odefun = @(t,x) [f1(x(1),x(2)); f2(x(1),x(2))];

% Set options for ode45
options = odeset('Events', @(t,x) stop(t,x,odefun,x1_range,x2_range),'RelTol',1e-8,'AbsTol',1e-10);

% Plot sample solutions given some initial conditions
response = input('Plot a solution? (y/n): ','s');
while response == 'y'
    x0 = input('Enter initial condition [x1(0),x2(0)]: ');
    % Plot solution on [0,Inf] and [0,-Inf]
    hold on;
    tspan = [0,inf];
    [T,Y] = ode45(odefun, tspan, x0, options);
    plot(Y(:,1), Y(:,2), "b");
    tspan = [0, -inf];
    [T,Y] = ode45(odefun, tspan, x0, options);
    plot(Y(:,1), Y(:,2), "b");
    response = input('Plot a solution? (y/n): ','s');
end

% Set stopping criteria for ode45 when computing solutions
function [value,isterminal,direction] = stop(t,x,odefun,x1_range,x2_range)
    value = [1,1,1];
    x1_mid = mean(x1_range);
    x2_mid = mean(x2_range);
    position = x - [x1_mid; x2_mid];
    % Stop ode45 if the solution goes beyond the figure window
    if norm(position) > norm([diff(x1_range),diff(x2_range)])/2
        value(1) = 0;
    end
    % Stop ode45 if the derivatives are very small
    if norm(odefun(t,x)) < 1e-4
        value(2) = 0;
    end
    % Stop ode45 if t gets too large
    if abs(t) > 100
        value(3) = 0;
    end
    isterminal = [1,1,1];
    direction = [0,0,0];
end
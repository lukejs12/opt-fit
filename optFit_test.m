% optFit_test

% Input function
u = @(t) 1*sin(t*2*pi);

% Starting point
x0 = [0 0 0 0]';

% Time vector
ttraj = 0:0.01:4;

% Create anonymous function wrapper for ode
m1 = 0.24463;
b1 = 2.045;
c2 = 0.13248;
l2 = 0.3;
m2 = 0.09465;
b2 = 0.0035;
I2 = 0.00353067843;
g = 9.81;
ode = @(t, x, p_fit) [x(3:4); SinglePendulumCart_auto(I2,p_fit(1),p_fit(2),c2,g,m1,m2,x(3),x(2),x(4),u(t))];
% SinglePendulumCart_auto(I2,b1,b2,c2,g,m1,m2,q1_dot,q2,q2_dot,u)


% Solve ODE to create 'nominal' trajectory
sol = ode45(@(t, x) ode(t, x, [b1 b2]), [ttraj(1) ttraj(end)], x0);
xtraj = deval(sol, ttraj);
% Add a bit of noise to the trajectory 
% xtraj = xtraj*(1+ .1*rand());

% Now try and find damping automatically
optFit(ode, [0 0], ttraj, xtraj);
% 
% % Try fitting damping values and interia
% ode = @(t, x, p_fit) [x(3:4); SinglePendulumCart_auto(p_fit(1),p_fit(2),p_fit(3),c2,g,m1,m2,x(3),x(2),x(4),u(t))];
% optFit(ode, [0.01 0 0], ttraj, xtraj);
% 
% % Try fitting all parameters (except gravity)
% ode = @(t, x, p_fit) [x(3:4); SinglePendulumCart_auto(p_fit(1),p_fit(2),p_fit(3),p_fit(4),g,p_fit(5),p_fit(6),x(3),x(2),x(4),u(t))];
% optFit(ode, [0.01 0 0 0.2 0.5 0.5], ttraj, xtraj);
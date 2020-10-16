% Automatically finds system parameters of an ODE, e.g. mass, damping,
% moment of inertia, by using optimization to minimize error between
% nominal trajectory from real system, and ODE model of system.
%
% Inputs: 
% ode   -   point to function of form ode(t, x, params)
%           where t - times, x - state vector, params - system parameters
%           we're finding
% guess -   vector of guesses for system parameter values
% ttraj -   time vector
% xtraj -   m x n nominal system trajectory, where
%           m = dof * 2, n = number of frames of trajectory
%           [[q1_0 ... qm_0, q1_dot_0 ...  qm_dot_0]', ... [q1_n ... qm_n, q1_dot_n ...  qm_dot_n]']
% 
% par_names -   A cell array of variable names, in the same order as the
%           guess parameter. Added to plots to aid comprehensibility.
% Outputs
% opt_param     -   vector of fitted parameters

function opt_param = optFit(ode, guess, ttraj, xtraj, par_names)
    % Initialise cost function
    costfun(0, ode, ttraj, xtraj);
    % Do optimisation
%     [opt_param, fval] = fminsearch(@(x) costfun(x), guess);
    options = optimoptions(@fmincon, ...
        'TolFun', 1e-9, ...         % 1e-12
        'TolX', 1e-9, ...           % 1e-12
        'MaxFunEvals', 10000000, ...
        'Display', 'iter-detailed', ...
        'MaxIter', 1000000 ...
    );
    num_params = length(guess);
    problem.options = options;
    problem.solver = 'fmincon';
    problem.objective = @(x) costfun(x);
    problem.x0 = guess;
    problem.lb = zeros(1, num_params);
    problem.ub = Inf*ones(1, num_params);
    
    [opt_param, fval] = fmincon(problem);
    disp(['Cost: ' num2str(fval)]);
    disp(['Fitted parameters: ' num2str(opt_param)]);
    
    % Get trajectory using fitted parameters
    sol = ode45(@(t, x) ode(t, x, opt_param), [ttraj(1) ttraj(end)], xtraj(:,1));
    xtraj_fit = deval(sol, ttraj);
    % And nominal vs fitted trajectory
    figure; 
    [m, ~] = size(xtraj);
    subplot(2, 1, 1);
    hold on; grid on; xlim([ttraj(1), ttraj(end)]);
    xlabel('Time');
    ylabel('Positions');
    title('Nominal vs fitted parameter system response - Positions');
    for i = 1:m/2
        plot(ttraj, xtraj(i,:), '--', 'LineWidth', 1, 'Color', [0.5 0.5 0.5]);
        plot(ttraj, xtraj_fit(i, :), 'm');
    end
%     legend('Nominal', 'Fitted');
    subplot(2, 1, 2);
    hold on; grid on; xlim([ttraj(1), ttraj(end)]);
    xlabel('Time');
    ylabel('Velocities');
    title('Nominal vs fitted parameter system response - Velocities');
    for i = m/2+1:m
        plot(ttraj, xtraj(i,:), '--', 'LineWidth', 1, 'Color', [0.5 0.5 0.5]);
        plot(ttraj, xtraj_fit(i, :), 'm');
    end
    legend('Nominal', 'Fitted');
    str = 'Parameters: ';
    if(nargin > 4)
        for i = 1:length(guess)
            str = [str par_names{i} ' = ' num2str(opt_param(i))];
            if i ~= length(guess)
                str = [str ', '];
            end
        end
    else
        str = [str num2str(opt_param)];
    end
    annotation('textbox', [.15 .8 1 .1], 'String', {str, ['Cost: ' num2str(fval)]}, 'FitBoxToText', 'on');
end

% To initialise: costfun(~, fun, t, traj_nom)
% To call as cost function: costfun(param)
function cost = costfun(param, fun, t, traj_nom)
    persistent ttraj xtraj odefun
    
    if nargin > 2
        odefun = fun;
        ttraj = t;
        xtraj = traj_nom;
        return;
    end
    
    sol = ode45(@(t, x) odefun(t, x, param), [ttraj(1) ttraj(end)], xtraj(:,1));
    x_try = deval(sol, ttraj);
    cost = (xtraj - x_try);
    cost = sum(sum(cost.*cost));
end


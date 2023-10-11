function [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt)
%% BEFORE RUNNING THE CODE CHANGE NAME TO pred_step
    %% Parameter Definition
    % uPrev - is the mean of the prev state
    %covarPrev - covar of the prev state
    %angVel - angular velocity input at the time step
    %acc - acceleration at the timestep
    %dt - difference in time 

    
    % Constants for unscented transform
    n = length(uPrev); % state vector dimension
    alpha = 1e-3;
    kappa = 1;
    beta = 2; % for Gaussian distributions
    lambda = alpha^2 * (n + kappa) - n;

    % Unscented transform
    [sigmaPoints, wm, wc] = unscentedTransform(uPrev, covarPrev, lambda, alpha, beta);
    
    % Propagate sigma points through process model
    sigmaPointsNext = zeros(size(sigmaPoints));
    for i = 1:(2*n+1)
        sigmaPointsNext(:, i) = f(sigmaPoints(:, i), angVel, acc, dt);
    end

    % Compute predicted state mean
    uEst = zeros(n, 1);
    for i = 1:(2*n+1)
        uEst = uEst + wm(i) * sigmaPointsNext(:, i);
    end

    % Compute predicted state covariance
    Q = 0.01*diag([0.1, 0.1, 0.1, 0.01, 0.01, 0.01, 0.1, 0.1, 0.1, 0.0001, 0.0001, 0.0001, 0.001, 0.001, 0.001]);

    covarEst = Q;
    for i = 1:(2*n+1)
        delta = sigmaPointsNext(:, i) - uEst;
        covarEst = covarEst + wc(i) * (delta * delta');
    end
    

end

function [sigmaPoints, wm, wc] = unscentedTransform(u, covar, lambda, alpha, beta)
    n = length(u); % state vector dimension
    U = chol((n + lambda) * covar)'; % Cholesky factorization
    disp(size(U));
    % Sigma points
    sigmaPoints = zeros(n, 2*n+1);
    sigmaPoints(:, 1) = u;
    for i = 1:n
        sigmaPoints(:, i+1) = u + U(:, i);
        sigmaPoints(:, i+n+1) = u - U(:, i);
    end

    % Weights for mean and covariance
    wm = zeros(2*n+1, 1);
    wc = zeros(2*n+1, 1);
    wm(1) = lambda / (n + lambda);
    wc(1) = wm(1) + (1 - alpha^2 + beta);
    for i = 2:(2*n+1)
        wm(i) = 1 / (2 * (n + lambda));
        wc(i) = wm(i);
    end
end

function [x_next] = f(x, angVel, acc, dt)
    % x - state vector
    % angVel - angular velocity input
    % acc - acceleration input
    % dt - time step
    % x_next - next state

    X = [x(1), x(2), x(3)]; % Position
    orientation = [x(4), x(5), x(6)]; % Orientation: phi, theta, psi
    V = [x(7), x(8), x(9)]; % Velocity
    phi = orientation(1);
    theta = orientation(2);
    psi = orientation(3);

    G = [cos(theta)*cos(psi), -sin(psi), 0;
         cos(theta)*sin(psi), cos(psi), 0;
         -sin(theta), 0, 1];

    R = [cos(psi)*cos(theta), cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi), cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi);
         sin(psi)*cos(theta), sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi), sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi);
         -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)];
    g = [0, 0, -9.81]';
    x3 = V';

    gbx = x(10);
    gby = x(11);
    gbz = x(12);
    abx = x(13);
    aby = x(14);
    abz = x(15);
    
    n_bg = 0.1*[0.02 0.013 0.015]';
    n_ba = 0.01*[0.075 0.009 0.012]';
    x_dot = [x3;
             (G^-1)*(angVel - [gbx; gby; gbz]);
             g + R * (acc - [abx; aby; abz]);
             n_bg;n_ba]; % Assuming no process noise for bias states

    x_next = x + x_dot * dt;
end




function [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt)
%% BEFORE RUNNING THE CODE CHANGE NAME TO pred_step
    %% Parameter Definition
    % uPrev - is the mean of the prev state
    %covarPrev - covar of the prev state
    %angVel - angular velocity input at the time step
    %acc - acceleration at the timestep
    %dt - difference in time 

    % Constants for unscented transform
    n = length(uPrev) + 12; % state vector dimension
    alpha = 1e-3;
    kappa = 1;
    beta = 2; % for Gaussian distributions
    lambda = alpha^2 * (n + kappa) - n;

    % Unscented transform
    [sigmaPoints, wm, wc] = unscentedTransform(uPrev, covarPrev, lambda, alpha, beta,dt);
    
    % Propagate sigma points through process model
    sigmaPointsNext = zeros(length(uPrev), 2*n+1 );
    for i = 1:(2*n+1)
        sigmaPointsNext(:, i) = process_model(sigmaPoints(:, i), angVel, acc, dt);
    end

    % Compute predicted state mean

    uEst = zeros(15, 1);
    for i = 1:(2*n+1)
        uEst = uEst + wm(i) * sigmaPointsNext(:, i);
    end

    % Compute predicted state covariance
    
    Q = 0.0001*diag([0.1, 0.1, 0.1, 0.01, 0.01, 0.01, 0.1, 0.1, 0.1, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]);
    covarEst = Q;
    for i = 1:(2*n+1)
        delta = sigmaPointsNext(:, i) - uEst;
        covarEst = covarEst + wc(i) * (delta * delta');
    end
    

end

function [sigmaPoints, wm, wc] = unscentedTransform(u, covar, lambda, alpha, beta,dt)
    n = length(u)+12; % state vector dimension
    u_aug = [u;zeros(12,1)];

    Q = 0.15*eye(12);
    covar_aug = [covar,zeros(15,12);zeros(12,15),dt*Q];

    U = chol((n+lambda)*covar_aug,'lower'); % Cholesky factorization

    % Sigma points
    
    sigmaPoints = zeros(n, 2*n+1);
    sigmaPoints(:, 1) = u_aug;
    for i = 1:n
        sigmaPoints(:, i+1) = u_aug + U(:, i);
        sigmaPoints(:, i+n+1) = u_aug - U(:, i);
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

function [x_next] = process_model(x, angVel, acc, dt)
    % x - state vector
    % angVel - angular velocity input
    % acc - acceleration input
    % dt - time step
    % x_next - next state

    orientation = [x(4), x(5), x(6)]; % Orientation: phi, theta, psi
    V = [x(7), x(8), x(9)]; % Velocity
    phi = orientation(1);
    theta = orientation(2);
    psi = orientation(3);

    G = [-sin(theta), 0, 1; cos(theta)*sin(phi), cos(phi), 0; cos(theta)*cos(phi), -sin(phi), 0];

    R = [cos(psi)*cos(theta), cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi), cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi);
         sin(psi)*cos(theta), sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi), sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi);
         -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)];
    g = [0, 0, -9.81]';
    x3 = V';
    x4 = x(10:12);
    x5 = x(13:15);
    
    n_ad = x(16:18);
    n_bd = x(19:21);
    n_bad = x(25:27);
    n_bgd = x(22:24);
    gbx = x(10);
    gby = x(11);
    gbz = x(12);
    abx = x(13);
    aby = x(14);
    abz = x(15);

    x_dot = [x3;
             (G^-1)*(angVel - [gbx; gby; gbz]-n_bd);
             g + R * (acc - [abx; aby; abz]-n_ad);
             (x4 -n_bgd);(x5 - n_bad) ]; 

    x_next = x(1:15) + dt*x_dot ;
end




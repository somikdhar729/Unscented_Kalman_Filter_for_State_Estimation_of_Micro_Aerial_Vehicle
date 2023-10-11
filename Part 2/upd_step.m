function [uCurr, covar_curr] = upd_step(z_t, covarEst, uEst)
    % z_t is the measurement
    % covarEst and uEst are the predicted covariance and mean respectively
    % uCurr and covar_curr are the updated mean and covariance respectively

   % Constants for unscented transform
    n = length(uEst)+6; % state vector dimension
    alpha = 1e-3;
    kappa = 1;
    beta = 2; % for Gaussian distributions
    lambda = alpha^2 * (n + kappa) - n;
    
    [sigmaPoints, wm, wc] = unscentedTransform(uEst, covarEst, lambda, alpha, beta);
    
    % Propagate sigma points through process model
    sigmaPointsNext = zeros(6,2*n+1);
    for i = 1:(2*n+1)
        sigmaPointsNext(:, i) = measurement_model(sigmaPoints(:, i),z_t);
    end
    % Compute predicted state mean
    uEst_t = zeros(6,1);
    for i = 1:(2*n+1)
        uEst_t = uEst_t + wm(i) * sigmaPointsNext(:,i);
    end
    

    R1 = 0.15*diag([0.01, 0.01, 0.01, 0.001, 0.001, 0.001]);

    S = R1;
    covar_cross = zeros(length(uEst), 6);
    for i = 1:length(wc)
        z_diff = sigmaPointsNext(:,i) - uEst_t;

        S = S + wc(i) * (z_diff * z_diff');
        
        covar_cross = covar_cross + wc(i) * ((sigmaPoints(1:15, i) - uEst) * z_diff');
    end
    K_t = covar_cross*S^(-1);
    
    uCurr = uEst + K_t*(z_t-uEst_t);
    covar_curr = covarEst - K_t*S*K_t';


end

function [sigmaPoints, wm, wc] = unscentedTransform(u, covar, lambda, alpha, beta)
    n = length(u)+6; % state vector dimension
    u_aug = [u;zeros(6,1)];
    Q = 0.0001*eye(6);
    covar_aug = [covar,zeros(15,6);zeros(6,15),Q];
    U = chol((n + lambda) * covar_aug)'; % Cholesky factorization

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
function z = measurement_model(x,z_t)
    T_CB = [cos(pi/4), -sin(pi/4), 0, -0.04; -sin(pi/4), -cos(pi/4), 0, 0; 0, 0, -1, -0.03; 0, 0, 0, 1];
    R_CB = T_CB(1:3,1:3);

    ori = x(4:6);
    R = eul2rotm(ori','ZYX');
    vel = x(7:9);

    pos1 = [0.0283,-0.0283,-0.03]';
    
    R1 = R*R_CB;

    s = skew(pos1);
    angVel = z_t(4:6);

    z = [R1*vel - R_CB*s*R_CB'*angVel;R*angVel];
    
    
end
function S = skew(v)
    % Create a skew-symmetric matrix from a 3x1 vector
    S = [0 -v(3) v(2);
         v(3) 0 -v(1);
         -v(2) v(1) 0];
end
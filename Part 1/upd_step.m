    function [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst)
%% BEFORE RUNNING THE CODE CHANGE NAME TO upd_step
    %% Parameter Definition
    %z_t - is the sensor data at the time step
    %covarEst - estimated covar of the  state
    %uEst - estimated mean of the state
    
%     Ct = 1*[eye(6),zeros(6,9)];
%     R = 0.1*diag([0.0001,0.0001,0.0001,0.003,0.005,0.005]);
%     Kt = covarEst*Ct'*(Ct*covarEst*Ct'+R)^-1;
%     covar_curr = covarEst - Kt*Ct*covarEst;
%     uCurr = uEst + Kt*(z_t - uEst(1:6,:));
 % Define the measurement noise covariance matrix R
    R = 0.01*diag([0.1, 0.1, 0.1, 0.01, 0.01, 0.01]); % Adjust the values according to your sensor noise characteristics

    % Measurement matrix (H) definition
    H = [eye(6), zeros(6, 9)]; % Assuming you measure the first 6 states directly

    % Kalman Gain
    K = covarEst * H' * inv(H * covarEst * H' + R);

    % Update state estimate
    uCurr = uEst + K * (z_t - H * uEst);

    % Update covariance estimate
    covar_curr = (eye(size(covarEst)) - K * H) * covarEst;
end


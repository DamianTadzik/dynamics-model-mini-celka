function out = find_all_LQ(A, B, x0, u0)

    doSave = (nargin == 0);
    verbose = doSave;
    if nargin == 0
        data = load("tmp_linearized_A_B_x0_u0_4dof.mat", ...
            "A", "B", "u0", "x0");
        A  = data.A;
        B  = data.B;
        x0 = data.x0;
        u0 = data.u0;
    elseif nargin ~= 4
        error("Provide either no arguments or A, B, x0, u0.");
    end
    %% Load A B and reduce to desired control structure even further
    % load("tmp_linearized_A_B_x0_u0_4dof.mat", "A", "B", "u0", "x0") 
    % % remember no yaw and yaw rate and thrust in these A B u0 x0 alerady!
    % 
    % Select states that controller should control
    ix = [
            % 1  xW        surge position (world)
            % 2  xWdot     surge velocity
        3;  % 3  zW        heave position (world, NED, +down)
        4;  % 4  zWdot     heave velocity
        5;  % 5  phi       roll angle
        6;  % 6  theta     pitch angle
        7;  % 7  p         roll rate (body)
        8;  % 8  q         pitch rate (body)
        9; 10; 11;  % 9 10 11   FL FR R   augumented acutators dynamics
    ];    
    % Select what are we controlling and with what order
    iu = [
        1; % 1 alpha_FL
        2; % 2 alpha_FR
        3; % 3 alpha_R
    ];
    
    % Reduce matrices to only few states that should be inputs to the LQ
    Ar = A(ix,ix);
    Br = B(ix,iu);
    x0r = x0(ix);
    
    % check controlability (for theoretical u = kx controller)
    Co_r = ctrb(Ar,Br);
    if verbose
        fprintf('Controlled-subsystem ctrb rank = %d of %d\n', ...
                rank(Co_r), size(Ar,1));
    end
    s_r = svd(Co_r);
    if verbose
        fprintf('svd: ');
        disp(s_r.')
    end

    % NOTE check this matrix if it is correct for our system with xW xWdot
    % z zdot phi theta p q FL FR R
    Cr = [
        % 0 0 0 0 0 0 0 0 0 0;   % xWdot (GPS)
        1 0 0 0 0 0 0 0 0;   % z (ToF)
        0 0 1 0 0 0 0 0 0;   % phi (assuming we know phi and theta well)
        0 0 0 1 0 0 0 0 0;   % theta (assuming we know phi and theta well)
        0 0 0 0 1 0 0 0 0;   % p (gyro)
        0 0 0 0 0 1 0 0 0;   % q (gyro)
    
        % 1 0 0 0 0 0;   % z (ToF)
        % 0 0 1 0 0 0;   % phi (assuming we know phi and theta well)
        % 0 0 0 1 0 0;   % theta (assuming we know phi and theta well)
        % 0 0 0 0 1 0;   % p (gyro)
        % 0 0 0 0 0 1;   % q (gyro)
    ];
    Ob_r = obsv(Ar, Cr);
    if verbose
        fprintf('Observability rank = %d of %d\n', rank(Ob_r), size(Ar,1));
    end

    %% Stabilizability check using PBH test
    lambda = eig(Ar);
    n = size(Ar,1);
    
    if verbose
        fprintf('\nPBH stabilizability check:\n');
    end

    for i = 1:length(lambda)
        pbh_rank = rank([lambda(i)*eye(n) - Ar, Br]);
        
        if verbose
            fprintf('lambda = %+9.4f %+9.4fj, PBH rank = %d/%d', ...
                real(lambda(i)), imag(lambda(i)), pbh_rank, n);
        
            if real(lambda(i)) >= 0
                if pbh_rank == n
                    fprintf('  -> unstable/marginal mode controllable\n');
                else
                    fprintf('  -> WARNING: uncontrollable unstable/marginal mode\n');
                end
            else
                fprintf('\n');
            end
        end
    end
    
    %% Set the weights and get the K for continouus LQR
    % Penalization for state error
    Q = diag([ ...
        12000, ...   3   z       heave
        10, ...      4   zdot    heave velocity
        2000, ...    5   phi     roll angle
        10, ...      6   theta   pitch angle
        2000, ...    8   p       roll rate (body)
        800, ...     9   q       pitch rate (body)
        1000, ...    11  FL
        1000, ...    12  FR
        1000 ...     13  R
        ]); 
    % Penalization for actuation
    R = diag([ ...
        .1, ...      FL
        .1, ...      FR
        10 ...      R
        ] .*  100);
    
    K = lqr(Ar,Br,Q,R);
    
    % Display the LQR Gain Matrix K:
    if verbose
        disp('LQR Gain Matrix K:'); disp(K);
        disp('Q:'); disp(Q);
        disp('R:'); disp(R);
    end
    eig_cl = eig(Ar- Br*K);
    if verbose
        disp('Closed-loop max real eig:'); disp(max(real(eig_cl)))
        disp('Closed-loop eig:'); disp(eig_cl)
    end
    
    % Save values for continuous controller simulation in simulink
    if doSave
        save("tmp_continuous_controller.mat", "K", "u0", "x0", "ix", "iu");
    end
    
    %% Discrete LQR
    Ts_ctrl = 0.01;   % 100 Hz
    
    sysc = ss(Ar, Br, eye(size(Ar,1)), zeros(size(Ar,1), size(Br,2)));
    sysd = c2d(sysc, Ts_ctrl, 'zoh');   % ZOH is what digital controller does
    Ad = sysd.A;
    Bd = sysd.B;
    
    Kd = dlqr(Ad, Bd, Q, R);
    
    eig_cl_d = eig(Ad - Bd*Kd);
    if verbose
        fprintf('Discrete closed-loop max |eig| = %.4f (must be in the unit circle)\n', max(abs(eig_cl_d)));
        disp('Discrete closed-loop eig:'); disp(eig_cl_d.');
        disp('Discrete closed-loop abs(eig):'); disp(abs(eig_cl_d.'));
    end 
    % Save values for discrete controller simulation in simulink
    if doSave
        save("tmp_discrete_controller.mat", "Kd", "Ts_ctrl", "u0", "x0", "ix", "iu");
    end
    
    %% Augumented discrete LQR
    
    Ld = 4; % number of delay states per actuator
    nu = 3; % number of actuators
    
    % ONE DELAY STATE:
    % 9x9 gets augumented into (9+3*4)x(9+3*4)
    Aaug = [ ...
        Ad,                   Bd;
        zeros(nu,size(Ad,2)), zeros(nu,nu);
    ];
    % 9x3 gets augumented to (9+3*4)x3
    Baug = [...
        zeros(size(Bd,1),nu);
        eye(nu);
    ];
    % REST OF THE DELAY STATES
    for d = 2:Ld
        Aaug = [ ...
            Aaug,                   Baug;
            zeros(nu,size(Aaug,2)), zeros(nu,nu);
        ];
        Baug = [...
            zeros(size(Baug,1),nu);
            eye(nu);
        ];
    end
    
    % x0 also should be augumented.
    x0aug = x0r;
    for d = 1:Ld
        x0aug = [x0aug; x0aug(end-2); x0aug(end-1); x0aug(end-0)];
    end
    if verbose
        fprintf('Augmented system size: %d states, %d inputs\n', size(Aaug,1), size(Baug,2));
    end
    % No penalty for actuator delay states?
    Qaug = blkdiag(Q, zeros(nu*Ld));
    
    Kaug = dlqr(Aaug, Baug, Qaug, R);

    %% Augumented discrete LQI
    Ci = zeros(3, size(Aaug,1));
    Ci(1,1) = 1;   % z
    Ci(2,3) = 1;   % phi
    Ci(3,4) = 1;   % theta

    A_aug_lqi = [
        Aaug,       zeros(size(Aaug,1),3);
        Ts_ctrl*Ci, eye(3)
    ];
    
    B_aug_lqi = [
        Baug;
        zeros(3,size(Baug,2))
    ];

    Qi = diag([
        5e5, ...   % integral of z heave error
        5e4, ...   % integral of phi roll error
        5e2  ...   % integral of theta pitch error
    ]);
    
    Q_aug_lqi = blkdiag(Qaug, Qi);
    K_aug_lqi = dlqr(A_aug_lqi, B_aug_lqi, Q_aug_lqi, R);

    % Tests
    Acl_lqi = A_aug_lqi - B_aug_lqi*K_aug_lqi;
    eig_cl_lqi = eig(Acl_lqi);
    if verbose
        fprintf('LQI max |eig(Acl)| = %.6f\n', max(abs(eig_cl_lqi)));
        disp('LQI closed-loop eig:');
        disp(eig_cl_lqi.');
    end
    n_lqi = size(A_aug_lqi,1);
    lambda_lqi = eig(A_aug_lqi);
    if verbose
        fprintf('\nLQI PBH stabilizability check:\n');
    end
    for i = 1:length(lambda_lqi)
        r = rank([lambda_lqi(i)*eye(n_lqi) - A_aug_lqi, B_aug_lqi]);
        if verbose && abs(lambda_lqi(i)) >= 1
            fprintf('lambda = %+9.5f %+9.5fj, rank = %d/%d\n', ...
                real(lambda_lqi(i)), imag(lambda_lqi(i)), r, n_lqi);
        end
    end

    if doSave
        save("tmp_discrete_augumented_LQI_controller.mat", ...
            "K_aug_lqi", ...
            "x0aug", ...
            "u0", ...
            "ix", ...
            "iu", ...
            "Qi");
    end

    %%  TESTS
    % Closed-loop stability
    Acl = Aaug - Baug*Kaug;
    eig_cl = eig(Acl);
    if verbose
        fprintf('Max |eig(Acl)| = %.6f\n', max(abs(eig_cl)));
    end
    % Stabilizability of augmented plant
    n = size(Aaug,1);
    lambda = eig(Aaug);
    if verbose
        fprintf('\nPBH stabilizability check:\n');
    end
    for i = 1:length(lambda)
        r = rank([lambda(i)*eye(n) - Aaug,Baug]);
        if verbose
            if abs(lambda(i)) >= 1
                fprintf('lambda = %+9.5f %+9.5fj, rank = %d/%d\n', ...
                    real(lambda(i)), imag(lambda(i)), r, n);
            end
        end
    end
    % Controllability conditioning
    Co_aug = ctrb(Aaug,Baug);
    s_aug = svd(Co_aug);
    if verbose
        fprintf('\nControllability singular values:\n');
        disp(s_aug.');
    end
    % Closed-loop damping / slowest modes
    [~,idx] = sort(abs(eig_cl),'descend');
    if verbose
        disp('Dominant closed-loop poles:');
        disp(eig_cl(idx(1:min(10,end))).');
    end
    % Save 
    if doSave
        save("tmp_discrete_augumented_controller.mat", "Kaug", "x0aug", "u0", "ix", "iu");
    end

    %% Output
    out.K      = K;
    out.Kd     = Kd;
    out.Kaug   = Kaug;
    
    out.A      = A;
    out.B      = B;
    out.Ar     = Ar;
    out.Br     = Br;
    out.Ad     = Ad;
    out.Bd     = Bd;
    out.Aaug   = Aaug;
    out.Baug   = Baug;
    
    out.x0     = x0;
    out.u0     = u0;
    out.x0r    = x0r;
    out.x0aug  = x0aug;
    
    out.Q      = Q;
    out.R      = R;
    out.Qaug   = Qaug;
    
    out.ix     = ix;
    out.iu     = iu;
    
    out.Ts_ctrl = Ts_ctrl;
    out.Ld      = Ld;

    out.K_aug_lqi = K_aug_lqi;
    out.A_aug_lqi = A_aug_lqi;
    out.B_aug_lqi = B_aug_lqi;
    out.Qi        = Qi;
    out.Q_aug_lqi = Q_aug_lqi;
end
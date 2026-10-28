
function out = find_all_MPC(A, B, x0, u0)
    %% Reduce model
    ix = (3:11)';
    iu = (1:3)';

    Ar = A(ix, ix);
    Br = B(ix, iu);

    x0r = x0(ix);
    u0r = u0(iu);

    %% Discretization
    Ts = 0.01;

    sysc = ss(Ar, Br, eye(9), zeros(9,3));
    sysd = c2d(sysc, Ts, 'zoh');

    Ad = sysd.A;
    Bd = sysd.B;

    %% Augment actuator transport delays
    Ld = 4;
    nu = 3;

    Aaug = [Ad, Bd;
            zeros(nu,9), zeros(nu,nu)];

    Baug = [zeros(9,nu);
            eye(nu)];

    for d = 2:Ld
        Aaug = [Aaug, Baug;
                zeros(nu,size(Aaug,2)), zeros(nu,nu)];

        Baug = [zeros(size(Baug,1),nu);
                eye(nu)];
    end

    %% Controlled outputs: z, phi, theta
    C = zeros(3, size(Aaug,1));

    C(1,1) = 1;
    C(2,3) = 1;
    C(3,4) = 1;

    D = zeros(3,nu);

    plant = ss(Aaug, Baug, C, D, Ts);

    %% MPC controller
    Np = 30;
    Nc = 5;

    mpcobj = mpc(plant, Ts, Np, Nc);

    %% Initial tuning
    % Output weights (not the LQR Q matrix)
    mpcobj.Weights.OutputVariables = [10 10 10];

    % mpcobj.Weights.ManipulatedVariables = [0 0 0];
    % mpcobj.Weights.ManipulatedVariablesRate = [0.1 0.1 0.1];

    mpcobj.Weights.ManipulatedVariables = [0.01 0.01 0.02]; % FL FR R
    mpcobj.Weights.ManipulatedVariablesRate = [0.2 0.2 0.2];

    %% Input constraints
    % Assuming commanded foil angles use degrees
    for i = 1:nu
        mpcobj.MV(i).Min = -6 - u0r(i);
        mpcobj.MV(i).Max = 12 - u0r(i);
    end

    % Disable built-in disturbance estimation
    setoutdist(mpcobj, 'model', tf(zeros(3,1)));
    
    % Use external state estimation
    setEstimator(mpcobj, 'custom');

    %% Outputs
    out.mpcobj = mpcobj;

    out.Ad = Ad;
    out.Bd = Bd;
    out.Aaug = Aaug;
    out.Baug = Baug;

    out.C = C;
    out.D = D;

    out.x0 = x0r;
    out.u0 = u0r;

    out.ix = ix;
    out.iu = iu;

    out.Ts = Ts;
    out.Ld = Ld;
end

function [A,B,f0] = linearize_fd(f, x0, u0, dx, du, method)
%LINEARIZE_FD Numerical linearization of xdot = f(x,u) at (x0,u0).
%
% Inputs:
%   f       : function handle, f(x,u) -> xdot (nx1)
%   x0,u0   : trim point
%   dx,du   : step sizes (nx1, nu1) OR scalars
%   method  : "central" (default) or "forward"
%
% Outputs:
%   A,B     : Jacobians at (x0,u0)
%   f0      : f(x0,u0)

    if nargin < 6 || isempty(method), method = "central"; end
    
    x0 = x0(:); u0 = u0(:);
    f0 = f(x0,u0);
    nx = numel(x0);
    nu = numel(u0);
    
    % steps
    if isscalar(dx), dx = dx * ones(nx,1); else, dx = dx(:); end
    if isscalar(du), du = du * ones(nu,1); else, du = du(:); end
    
    A = zeros(nx,nx);
    B = zeros(nx,nu);
    
    switch lower(string(method))
    case "central"
        % A
        for i = 1:nx
            h = dx(i);
            xp = x0; xm = x0;
            xp(i) = xp(i) + h;
            xm(i) = xm(i) - h;
            fp = f(xp,u0);
            fm = f(xm,u0);
            A(:,i) = (fp - fm) / (2*h);
        end
        % B
        for j = 1:nu
            h = du(j);
            up = u0; um = u0;
            up(j) = up(j) + h;
            um(j) = um(j) - h;
            fp = f(x0,up);
            fm = f(x0,um);
            B(:,j) = (fp - fm) / (2*h);
        end
    
    case "forward"
        % A
        for i = 1:nx
            h = dx(i);
            xp = x0; xp(i) = xp(i) + h;
            fp = f(xp,u0);
            A(:,i) = (fp - f0) / h;
        end
        % B
        for j = 1:nu
            h = du(j);
            up = u0; up(j) = up(j) + h;
            fp = f(x0,up);
            B(:,j) = (fp - f0) / h;
        end
    
    otherwise
        error("method must be 'central' or 'forward'");
    end
end

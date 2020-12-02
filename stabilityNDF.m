A = @(alpha) [0, 1; -1, 2*cos(alpha)];





%% Functions

function fun = stability_fcn(F, h, alpha)

% PROTOTYPE:
%  fun = stability_fcn(F, h, alpha)
% 
% DESCRIPTION:
%  Function to get a function handle expressing the problem abs(eig(F)) ==
%  1, where F is an operator associated to linear dynamics (x_dot = Ax) and
%  to a particular algorithm to solve ODEs. The operator is such that
%  x_{k+1} = F(h,alpha) x_k, where h is the step size and alpha is the
%  angle of the complex eigenvalues lambda (lambda = eig(A)). The max
%  operator is used to get the maximum eigenvalue, in order to provide the
%  most restrictive condition on h.
%
% INPUT ARGUMENTS:
%  F      [-]    ODE operator (function handle)
%  h      [-]    step size (variable)
%  alpha  [-]    angle from the positive real axis (variable)
%
% OUTPUT ARGUMENTS:
%  fun    [-]    output function handle
%
% CALLED FUNCTIONS:
%
%
% CONTRIBUTORS AND LOG
%  Lorenzo Porcelli 13-10-20, v. 1.0

fun = eig(F(h, alpha));
fun = max(abs(fun));

end

function plot_stability(F, h0, h0_pos)

% PROTOTYPE:
%  plot_stability(F, h0, h0_pos)
% 
% DESCRIPTION:
%  Function to sketch the regions of stability of a numerical ODE solver in
%  the complex plane h_lambda. The plot is obtained through the solution of
%  a zero-finding problem defined in the function stability_fcn (involving 
%  F(h, alpha)).
%
% INPUT ARGUMENTS:
%  F       [-]       ODE operator (function handle)
%  h0      [1x1]     guess for the first fsolve iteration
%  h0_pos  [char]    string that identifies the approximated position for
%                    the initial guess: h0_loc = 'zero' means that h0 is
%                    guess for alpha close to 0, while h0_loc = 'pi' is a
%                    guess for alpha close to pi. The first case is
%                    suitable for methods which have the region of marginal
%                    stability in the LHP, while the second works for
%                    methods with the region of marginal stability in the
%                    RHP.
%
% OUTPUT ARGUMENTS:
%
% CALLED FUNCTIONS:
%  stability_fcn
%
% CONTRIBUTORS AND LOG
%  Lorenzo Porcelli 13-10-20, v. 1.0

n_points = 200;

%The vectors for alpha angles are defined accordingly to the location of
%the initial guess in order to get a smooth plot
switch h0_pos
    case 'zero'
        angles = linspace(0, pi, n_points);
    case 'pi'
        angles = linspace(pi, 0, n_points);
    otherwise
        error('Not a valid input for h0_pos.')
end

hh = zeros(1, n_points);

%Eigenvalues of matrix A = [0, 1; -1, 2*cos(alpha)]
lambda = cos(angles) + 1j*sin(angles);

options = optimoptions('fsolve', 'Display', 'off');

%Loop to find h values associated to alphas (from 0 to pi)
for k = 1:length(angles)
    hh(k) = fsolve(@(h) 1 - stability_fcn(F, h, angles(k)), h0, options);
    h0 = hh(k);
end

%Defining the trace of the region
%The conj operation is to get the plot of the region for alpha [pi,2*pi]
%The fliplr operation is done to get a continous line
h_lambda = [hh.*lambda, fliplr(hh.*conj(lambda))];
plot(h_lambda, 'LineWidth', 1.5);

xlabel('Re \{$h\lambda$\}')
ylabel('Im \{$h\lambda$\}')
end
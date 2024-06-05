function [y_ref, Psi_ref, Psi_dot_ref]= reference_function()

% Define the parameters
B = 2.5; % Example value
x1 = 30 ; % Example value
y1 = 0.92; % Example value
y_tol = 0.01; % Example value
C1 = log((B/y_tol)-1);
C2 = 5; % Example value
Vx = 90/3.6;

% Calculate k1, k2, k3
k1 = (B * x1)^2 / 16 - (B * C2)^2 / 16;
k2 = -B^2 * x1 * C1 / 8 - B * y1 * x1 + B^2 * x1 / 4;
k3 = (B * C1)^2 / 16 + y1^2 + B^2 / 4 + B * y1 * C1 - B * y1 - B^2 * C1 / 4 - C2^2;

% Calculate a and c
a = (-k2 + sqrt(k2^2 - 4 * k1 * k3)) / (2 * k1);
c = C1 / a;

% Define the range of x
x = linspace(-5, 35, 1000); % Example range
y_ref = B ./ (1 + exp(-a * (x - c)));

% Compute the first and second derivatives of y_ref with respect to x
dydx = diff(y_ref) ./ diff(x);
d2ydx2 = diff(dydx) ./ diff(x(1:end-1));

% Define the expression for kappa_1
kappa_1 = d2ydx2 ./ (1 + dydx(1:end-1).^2).^(3/2);

% Compute Psi_ref
Psi_ref = atan(dydx);

% Compute Psi_dot_ref
Psi_dot_ref = kappa_1 * Vx;




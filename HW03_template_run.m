% Dr. Barys Shyrokau
% Template for homework assignment #3
% RO47017 Vehicle Dynamics & Control
% Use and distribution of this material outside the RO47017 course 
% only with the permission of the course coordinator

clc; clear all; close all; clear mex;

% controller settings
Ts = 0.01;	% controller frequency

% vehicle parameters (bicycle model)
veh_parameters;

%% ACADO set up 
DifferentialState vx vy r yaw Xp Yp delta; % definition of controller states
Control d_delta; % definition of controller input
% controller model of the plant
beta = atan(par.l_r * tan (delta) / par.L);

f_ctrl = [
    dot(vx)  == vy*r;...
    dot(vy) == -(par.Caf + par.Car) / (par.mass * vx) * vy + ...
    ((par.l_r * par.Car - par.l_f * par.Caf) / (par.mass*vx) - vx)*r + ...
    par.Caf / par.mass * delta;...
    dot(r) == ((par.l_r * par.Car - par.l_f * par.Caf)/(par.Izz * vx)) * vy -...
    ((par.l_r^2 * par.Car + par.l_f^2 * par.Caf)/(par.Izz * vx))*r + ...
    par.l_f * par.Caf / par.Izz * delta;...
    dot(yaw) == r;...
    dot(Xp)  == vx * cos(yaw) - vy*sin(yaw);...
    dot(Yp)  == vx * sin(yaw) + vy*cos(yaw);...
    dot(delta) == d_delta];

%% ACADO: controller formulation
acadoSet('problemname', 'PF_problem');
Np = 40;                                  % prediction horizon
ocp  = acado.OCP( 0.0, Np*Ts, Np);        % ACADO ocp

% Residual function definition based on ACADO
h = [diffStates ; controls];
hN = [diffStates];                       % terminal

% Initialization weights
W = acado.BMatrix(eye(length(h)));
WN = acado.BMatrix(eye(length(hN)));     % terminal

% Cost definition
ocp.minimizeLSQ(W,h);
ocp.minimizeLSQEndTerm(WN,hN);           % terminal

% Constraints definition
beta_thd       = 10 / 180*pi;            % absolute sideslip 
delta_thd   = 2.76*360/180*pi / par.i_steer;	 % absolute steering position
dotdelta_thd = 800*pi/(180 * par.i_steer);
sideslip = 5*pi/180;
sidesliprate = 25*pi/180;
lat_acc = 0.85*par.g*0.9;

% constraints in ACADO
ocp.subjectTo(0 <= vx <= 170/3.6);
ocp.subjectTo(-sideslip <= vy/vx <= sideslip);
ocp.subjectTo(-sidesliprate <= dot(vy)/vx <= sidesliprate);
ocp.subjectTo(-lat_acc <= dot(vy) + vx*r <= lat_acc);
ocp.subjectTo( -delta_thd   <= delta    <= delta_thd);
% ocp.subjectTo( -beta_thd   <= beta    <= beta_thd);
ocp.subjectTo( -dotdelta_thd <= d_delta <= dotdelta_thd);

% define ACADO prediction model
ocp.setModel(f_ctrl);
    
% ACADO settings [Don't change these settings in your HMA]
mpc = acado.OCPexport( ocp );
mpc.set('HESSIAN_APPROXIMATION', 'GAUSS_NEWTON');       % solving algorithm
mpc.set('DISCRETIZATION_TYPE', 'MULTIPLE_SHOOTING');    % discretization algorithm
mpc.set('INTEGRATOR_TYPE', 'INT_IRK_GL2');              % intergation algorithm
mpc.set('NUM_INTEGRATOR_STEPS', 3*Np);                  % number of integration steps
mpc.set('LEVENBERG_MARQUARDT', 1e-4);                   % value for Levenberg-Marquardt regularization -> affects performance
mpc.set('SPARSE_QP_SOLUTION', 'FULL_CONDENSING_N2');
mpc.set('QP_SOLVER', 'QP_QPOASES3');
mpc.set('MAX_NUM_QP_ITERATIONS', 20) ;
mpc.set('HOTSTART_QP','YES');
mpc.set('GENERATE_SIMULINK_INTERFACE', 'YES');

%% Export and Compile flags
EXPORT  = 1;
COMPILE = 1;

% export code to the defined folder
if EXPORT
    mpc.exportCode('export_MPC');
end

% compilation of the S-function using autogenerated make script
if COMPILE
    global ACADO_;
    copyfile([ACADO_.pwd '/../../external_packages/qpoases3'], 'export_MPC/qpoases3')
    cd export_MPC
    make_acado_solver_sfunction
    copyfile('acado_solver_sfun.mex*', '../')
    cd ..
end

%% initial MPC settings
disp('Initialization')
X0       = [V_ref 0 0 0 0 0 0];             % initial state conditions
[y_ref, Psi_ref, Psi_dot_ref]= reference_function();
% initialize controller bus
input.x  = repmat(X0, Np + 1, 1).';      % size Np + 1
input.od = zeros(Np + 1, 1);            % size Np + 1
Uref     = zeros(Np, 1);
input.u  = Uref.';
input.y  = [repmat(X0, Np, 1) Uref].';   % reference trajectory, size Np + 1
input.yN = X0.';                        % terminal reference, size Np + 1
% redefined in Simulink
input.W  = diag([0 0 0 5e-3 1e-1 0 0 0]);     % weight tuning !! Tune them in the Simulink model !!
input.WN = diag([0 0 0 0 0 0 0]);             % terminal weight tuning
input.x0 = X0.';
% controller bus initialization
init.x   = input.x(:).';                  % state trajectory
init.u   = input.u(:).';                  % control trajectory
init.y   = input.y(:).';                  % reference trajectory (up to Np - 1)
init.yN  = input.yN(:).';                % terminal reference value (only for Np)
init.W   = input.W(:).';                  % stage cost matrix (up to Np - 1)
init.WN  = input.WN(:).';                % terminal cost matrix (only for Np)
init.x0  = input.x0(:).';                % initial state value
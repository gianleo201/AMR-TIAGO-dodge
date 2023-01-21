clear;

BEGIN_ACADO; % Always start with "BEGIN_ACADO".

acadoSet( "problemname" ,  "TIAGO_goto_setpoint");   % Call the problem TIAGO_goto_setpoint

% define the time sample
Ts = 0.1;

% define differential states
DifferentialState x1 x2 x3 x4 x5 x6;
N_XD = length(diffStates);

% define controls
Control u1 u2 u3; 
N_U = length(controls);

% define time variable
TIME t;


% Differential Equation (built from matlab function)
f = acado.DifferentialEquation(); % Set the differential equation object
f.linkMatlabODE('TIAGO'); % Link to a Matlab ODE

% output function (up to know simply the state)
h = [x1;x2;x3];

% weight matrix for LSQ function
W = eye(N_XD/2);

% reference reference for the output function
r = zeros(N_XD/2,1);


%%%% HERE COMES THE OPTIMIZATION PROBLEM %%%%


N = 10; % number of iterations

% Optimal control problem
ocp = acado.OCP( 0.0 , N*Ts , N );

ocp.minimizeLSQ( W , h , r );   %% minimize the loss function w.r.t the reference output

% constraints on the optimization problem

ocp.setModel(f);   % differential equations

ocp.subjectTo( -2*pi <= [x1;x2;x3] <= 2*pi);  % Joint limit constraint

ocp.subjectTo(  -100 <= [x4;x5;x6] <= 100 )  % Joint velocity limit

ocp.subjectTo( 'AT_START', diffStates == [0;0;-pi/4;0;0;0]);  % set initial condition for states


% Optimization algorithm ( solve just optimal control problem NO NMPC )
algo = acado.OptimizationAlgorithm(ocp);  % Set up the optimization algorithm
algo.set('KKT_TOLERANCE', 1e-9);   % Set a KKT tolerance

    

END_ACADO; % End with "END ACADO" to compile.


out = TIAGO_goto_setpoint_RUN(); % Run the test.

%% plot result

for i=1:(N/2)
    plot(out.STATES(:,1),out.STATES(:,i+1),LineWidth=2.0); grid on; hold on;
end

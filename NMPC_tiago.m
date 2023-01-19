clear;

BEGIN_ACADO; % Always start with "BEGIN_ACADO".




acadoSet( "problemname" ,  "TIAGO_goto_setpoint");   % Call the problem active damping


DifferentialState x1 x2 x3 x4 x5 x6; % Differential States


Control u1 u2 u3; % Controls


TIME t;


% Differential Equation
f = acado.DifferentialEquation(); % Set the differential equation object
f.linkMatlabODE("TIAGO"); % Link to a Matlab ODE

% output function (up to know simply the state)
h = diffStates;
% weight matrix
W_mat = eye(6);
W = acado.BMatrix(W_mat);


%%%% HERE COMES THE OPTIMIZATION PROBLEM %%%%

% Optimal control problem
ocp = acado.OCP(0.0,15.0,100);

ocp.minimizeLSQ(W,h);   %% minimize the loss function w.r.t the reference output

ocp.setModel(f);   % Optimize w.r.t. differential equations
ocp.subjectTo( -2*pi <= [x1;x2;x3] <= 2*pi);

% Optimization algorithm ( solve just optimal control problem NO NMPC )
algo = acado.OptimizationAlgorithm(ocp);  % Set up the optimization algorithm
algo.set('KKT_TOLERANCE', 1e-10);   % Set a KKT tolerance 

    


END_ACADO; % End with "END ACADO" to compile.
out = TIAGO_goto_setpoint_RUN(); % Run the test.

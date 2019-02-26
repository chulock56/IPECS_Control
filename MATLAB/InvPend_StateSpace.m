%% IPECS: Inverted Pendulum to Educate Controls Students
%{

Drexel MEM Senior Design Team #027: 
    Chulock, A., Pruitt, W., Givens, G., Maher, C., Insalaco, D.

This code written by: Chulock A., Insalaco, D. on 01/08/2019

! FUNCTION "rscale.m" MUST BE IN THE SAME FOLDER AS THIS CODE FOR THIS CODE
TO WORK !

This code receives the model parameter values from "InvPend_Params.m",
creates the linear state space model, and simulates the state feedback
controller.

The final section of code outputs lines of C-code meant to be copy/pasted
into the Arduino IDE state-feedback control program.

Desired Specs:

-Settling time, theta & alpha: < 2s
-Rise time, theta:             < 0.5s
-SS Error, theta & alpha:      < 2%
-Pendulum angle, alpha:        < 0.1745rad (10deg) from vertical, ALWAYS
-Motor Voltage, Vm:            < 10V, ALWAYS

%}

%% Linear State Space Model
% States are defined as: x = [ theta, alpha, theta_dot, alpha_dot ]'
% Input: u = Vm

% A Matrix
A = 1/JT*[ 0 0                          JT                    0                ; 
           0 0                          0                     JT               ;
           0 0.25*mp^2*Lp^2*Lr*gty      -(Jp+0.25*mp*Lp^2)*Br -0.5*mp*Lp*Lr*Bp ;
           0 0.5*mp*Lp*gty*(Jr+mp*Lr^2) -0.5*mp*Lp*Lr*Br      -(Jr+mp*Lr^2)*Bp ] ;

% B Matrix
B = 1/JT*[ 0 0 Jp+0.25*mp*Lp^2 0.5*mp*Lp*Lr ]' ;

% C Matrix
C = [ 1 0 0 0 ;
      0 1 0 0 ] ;

% D Matrix  
D = zeros(2,1) ;

% Convert input from torque to motor voltage
A(3,3) = A(3,3) - Kg^2*kt*km*etag*etam/Rm*B(3) ;
A(4,3) = A(4,3) - Kg^2*kt*km*etag*etam/Rm*B(4) ;
B = Kg*kt*etag*etam/Rm * B ;

%% Open-Loop Poles (Eigen Values)

states = {'theta' 'alpha' 'theta_dot' 'alpha_dot'} ;
inputs = {'Vm'} ;
outputs = {'theta'; 'alpha'} ;

% generate open-loop state space model
pend_ol = ss( A, B, C, D, 'statename', states, 'inputname', inputs, 'outputname', outputs ) ;

poles = eig( A ) % calc eigen values (poles) of open-loop system

%% Controllability & Observability

% check that rank of controllability matrix is equal to no. of states (4)
coMat = ctrb( pend_ol ) ;
coMatRank = rank( coMat )

% check that rank of observability matrix is equal to no. of states (4)
obMat = obsv( pend_ol );
obMatRank = rank( obMat )

%% State-Feedback Design (w/ Known Desired Damping and Nat Freq)

zetaD  = 0.7 % set desired damping ratio
omegaN = 4   % set desired natural frequency (rad/s)

% poles = -sigma +- i*omegaD
sigma  = zetaD*omegaN ; % calc sigma (real part of desired pole)
omegaD = omegaN*sqrt( 1 - zetaD^2 ) ; % calc damped frequency (imag. part of desired pole)

PoleDes = [ -sigma+1i*omegaD, -sigma-1i*omegaD, -30, -40 ]' % define desired poles (based on sys. specs)
K = place( A, B, PoleDes ) % calculate required gain matrix

Ad = A-B*K ; % define compensated system matrices
Bd = B ;
Cd = C ;
Dd = D ;

% calculate precompensator gain
Cn = [1 0 0 0] ; % define C matrix to represent reference as a command on theta only
sys_ss = ss( A, B, Cn, 0 ) ;
Nbar = rscale( sys_ss, K ) 

states = {'theta' 'alpha' 'theta_dot' 'alpha_dot'} ;
inputs = {'r'} ;
outputs = {'theta'; 'alpha'} ;

% define closed-loop state space model
pend_des = ss( Ad, Bd*Nbar, Cd, Dd, 'statename', states, 'inputname', inputs, 'outputname', outputs ) ;

t = 0:0.01:5 ; % time
r = 0.35*ones( size(t) ) ; % theta reference input (step)

[ y, t, x ] = lsim( pend_des, r, t ) ; % get step response plot data

figure(1)
plot( t, rad2deg(y(:,1)), t, rad2deg(y(:,2)), t, rad2deg(r), '--b' )
title( 'Step Response with State-Feedback Control (w/ Known Nat. Freq. & Damping' )
xlabel('Time, s')
ylabel('Angle, deg')
legend('Rotary Arm Angle', 'Pendulum Angle', 'Rotary Arm Setpoint')
grid on

figure(2)
plot( t, r.*Nbar-K*x')
title( 'Step Response with State-Feedback Control' )
xlabel( 'Time (s)' )
ylabel( 'Motor Input Voltage (V)' )
grid on

%% State-Feedback Design Using LQR

% Q and R are weighting parameters that determine the balance between
% control effort and error
Q = C'*C ; % Q(1,1): theta weighting; Q(2,2): alpha weighting
Q(1,1) = 500 ;  % 30, 1000, 1000, 500
Q(2,2) = 800 ;  % 70, 5000, 5000, 800
Q(3,3) = 1 ;    % 1,  0,    0,    1
R      = 4 ;    % 1,  1,    5,    4
K = lqr( A, B, Q, R ) % calculate gain matrix using lqr() fcn

Ac = A-B*K ; % define LQR-compensated system matrices
Bc = B ;
Cc = C ;
Dc = D ;

% calculate precompensator gain
Cn = [1 0 0 0] ; % define C matrix to represent reference as a command on theta only
sys_ss = ss( A, B, Cn, 0 ) ;
Nbar = rscale( sys_ss, K ) ;

states = {'theta' 'alpha' 'theta_dot' 'alpha_dot'} ;
inputs = {'r'} ;
outputs = {'theta'; 'alpha'} ;

% define closed-loop state space model
pend_lqr = ss( Ac, Bc*Nbar, Cc, Dc, 'statename', states, 'inputname', inputs, 'outputname', outputs ) ;

t = 0:0.01:10 ; % time
% r = 0.35*ones( size(t) ) ; % theta reference input (step)
r = 0.35*square(0.1*2*pi*t) ; % square wave (Hz)

[ y, t, x ] = lsim( pend_lqr, r, t ) ; % get step response plot data

figure(3)
plot( t, rad2deg(y(:,1)), t, rad2deg(y(:,2)), t, rad2deg(r), '--b' )
title( 'Step Response with LQR Control' )
xlabel('Time, s')
ylabel('Angle, deg')
legend('Rotary Arm Angle', 'Pendulum Angle', 'Rotary Arm Setpoint')
grid on
grid minor

figure(4)
plot( t, r.*Nbar-K*x')
title( 'Step Response with LQR Control' )
xlabel( 'Time (s)' )
ylabel( 'Motor Input Voltage (V)' )
grid on

%% LQR-Based Design w/ Observer

poles = eig( Ac ) % find poles of closed-loop sys
P = [ -40 -41 -42 -43 ] ; % choose estimator poles to be ~10 times faster than the slowest controller pole
L = place( A', C', P )' % generate the estimator gain matrix using A'-C'L' (A-BK)

% define closed-loop state-space model w/ observer based control
Ace = [ (A-B*K)     (B*K);
        zeros(size(A)) (A-L*C) ] ;
   
Bce = [ B*Nbar;     % includes precompensation
        zeros(size(B)) ] ;
   
Cce = [ Cc zeros(size(Cc)) ] ;

Dce = [ 0; 0 ] ;

states = {'theta' 'alpha' 'theta_dot' 'alpha_dot' 'e1' 'e2' 'e3' 'e4'} ;
inputs = {'r'} ;
outputs = {'theta'; 'alpha'} ;

% define closed-loop state space model w/ observer
pend_est_lqr = ss( Ace, Bce, Cce, Dce, 'statename', states, 'inputname', inputs, 'outputname', outputs ) ;

t = 0:0.01:10 ;
% r = 0.35*ones( size(t) ) ; % theta reference input (step)
r = 0.35*square(0.1*2*pi*t) ; % square wave (Hz)

[ y, t, x ] = lsim( pend_est_lqr, r, t ) ; % get step response plot data

figure(5)
plot( t, rad2deg(y(:,1)), t, rad2deg(y(:,2)), t, rad2deg(r), '--b' )
title( 'Step Response with Observer-Based LQR Control' )
xlabel('Time, s')
ylabel('Angle, deg')
legend('Rotary Arm Angle', 'Pendulum Angle', 'Rotary Arm Setpoint')
grid on

figure(6)
plot( t, r.*Nbar-K*x(:,1:4)')
title( 'Step Response with Observer-Based LQR Control' )
xlabel( 'Time (s)' )
ylabel( 'Motor Input Voltage (V)' )
grid on

xDeg = rad2deg(x);

% figure(7)
% subplot(1,2,1), plot( t, xDeg(:,1:2) )
% title( 'Step Response with Observer-Based LQR Control' )
% xlabel('Time, s')
% ylabel('Angular Position, deg')
% legend('Rotary Arm Ang. Pos.', 'Pendulum Ang. Pos.')
% grid on
% subplot(1,2,2), plot( t, xDeg(:,3:4) )
% xlabel('Time, s')
% ylabel('Angular Velocity, deg/s')
% legend('Rotary Arm Ang. Vel.', 'Pendulum Ang. Vel.')
% grid on

figure(7)
plot( t, xDeg )
title( 'Step Response with Observer-Based LQR Control' )
xlabel('Time, s')
ylabel('Angle (deg) / Angular Velocity (deg/s)')
legend('Rotary Arm Ang. Pos.', 'Pendulum Ang. Pos.', 'Rotary Arm Ang. Vel.', 'Pendulum Ang. Vel.')
grid on

%% C-Code Generation

% Display system model for use in C-Code
vecAB = [A(3,2) A(3,3) A(3,4) A(4,2) A(4,3) A(4,4) B(3,1) B(4,1)] ;
fprintf('Model declaration in C-Code should be (just copy/paste into code):\nInvPendModel model(%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f);\n\n', vecAB)

% Display control gain matrix for use in C-Code
fprintf('Controller gain declaration in C-Code should be (just copy/paste into code):\nMatrix<1,4> K = {%.3f, %.3f, %.3f, %.3f};\n\n', K)

% Display estimator gain matrix for use in C-Code
vecL = [L(1,1) L(1,2) L(2,1) L(2,2) L(3,1) L(3,2) L(4,1) L(4,2)] ;
fprintf('Estimator gain declaration in C-Code should be (just copy/paste into code):\nMatrix<4,2> L = {%.3f, %.3f,\n%.3f, %.3f,\n%.3f, %.3f,\n%.3f, %.3f};\n\n', vecL)
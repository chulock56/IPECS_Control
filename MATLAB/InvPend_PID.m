%% IPECS: Inverted Pendulum to Educate Controls Students
%{

Drexel MEM Senior Design Team #027: 
    Chulock, A., Pruitt, W., Givens, G., Maher, C., Insalaco, D.

This code written by: Chulock A., Insalaco, D. on 2/7/2019

This code receives the model parameter values from "InvPend_Params.m",
forms the transfer functions of the inverted pendulum system, 
and simulates the PID controller.

Desired Specs:

-Settling time, theta & alpha: < 2s
-Rise time, theta:             < 0.5s
-SS Error, theta & alpha:      < 2%
-Pendulum angle, alpha:        < 0.1745rad (10deg) from vertical, ALWAYS
-Motor Voltage, Vm:            < 10V, ALWAYS

%}

%% Define Substitutive Variables

% VToAlpha:
a = ( Jp + 0.25*mp*Lp^2 )*Br ;
b = 0.5*mp*Lp*Lr*Bp ;
c = 0.25*mp^2*Lp^2*Lr*gty ;
d = ( Jp + 0.25*mp*Lp^2 ) * ( etag*etam*kt/Rm ) ;

e = 0.5*mp*Lp*Lr*Br ;
f = ( Jr + mp*Lr^2 )*Bp ;
g = 0.5*mp*Lp*gty*( Jr + mp*Lr^2 ) ;
h = ( 0.5*mp*Lp*Lr ) * ( etag*etam*kt/Rm ) ;

q = a + d*Kg*km ;
r = e + h*Kg*km ;

% VToTheta:
a2 = -( Jp + 0.25*mp*Lp^2 )*Br / JT ;
b2 = -0.5*mp*Lp*Lr*Bp / JT ;
c2 = 0.25*mp^2*Lp^2*Lr*gty / JT ;
d2 = ( Jp + 0.25*mp*Lp^2 )*( etag*etam*kt )/( JT*Rm ) ;
e2 = -( Jp + 0.25*mp*Lp^2 )*( etag*etam*kt )*( Kg*km )/( JT*Rm ) ;

f2 = -0.5*mp*Lp*Lr*Br / JT ;
g2 = -( Jr + mp*Lr^2 )*Bp / JT ;
h2 = 0.5*mp*Lp*gty*( Jr + mp*Lr^2 ) / JT ;
i2 = ( 0.5*mp*Lp*Lr )*( etag*etam*kt )/( JT*Rm ) ;
j2 = -( 0.5*mp*Lp*Lr )*( etag*etam*kt )*( Kg*km )/( JT*Rm ) ;

q2 = g2 + a2 + e2 ;
r2 = -h2 + g2*( a2 + e2 ) - b2*( f2 + j2 ) ;
t2 = h2*( a2 + e2 ) - c2*( f2 + j2 ) ;

%% Form Transfer Functions

s = tf('s') ;
VToTheta = ( d2*s^2 + (b2*i2 - d2*g2)*s + c2*i2 - d2*h2 )/( s^4 - q2*s^3 + r2*s^2 + t2*s )

VToAlpha = ( JT*h*s + h*q - d*r )/( JT^2*s^3 + JT*(q + f)*s^2 + (f*q - b*r - JT*g)*s + c*r - g*q ) 

ThetaToAlpha = ( s^2 - (a2 + e2 - d2/i2*(f2 + j2))*s )/( d2/i2*s^2 + (b2 - d2*g2/i2)*s + c2 - d2*h2/i2 )

%% PID Design

rlocus( VToAlpha )
title( 'Root Locus of Open-Loop Plant (Motor Voltage to Alpha)' )

C = 1/s ;
rlocus( C*VToAlpha )
title( 'Root Locus of Plant w/ Integral Control (Motor Voltage to Alpha)' )
IntZeros = zero( C*VToAlpha )
IntPoles = sort( pole( C*VToAlpha ) )

%% Design Compensator Zeros Based on Phase-Angle Method

PO = 20 ;    % Define desired percent overshoot
Ts = 2 ;     % Define desired settling time

zetaD  = sqrt( log(PO/100)^2 / (pi^2 + log(PO/100)^2 ) )    % calculate desired damping ratio
omegaN = 4/(Ts*zetaD)   % set desired natural frequency (rad/s)

% poles = -sigma +- j*omegaD
sigma  = zetaD*omegaN ; % calc sigma (real part of desired pole)
omegaD = omegaN*sqrt( 1 - zetaD^2 ) ; % calc damped frequency (imag. part of desired pole)

zc1 = 3 ;  % choose first pole based on open-loop root locus (input is positive, but zero is in LHP)
zc2 = sigma + omegaD / tand( -180 - atand( omegaD/(zc1-sigma)) ... 
    + ( 180 - atand( omegaD/(sigma+IntPoles(4))) ) ...
    + ( 180 - atand( omegaD/(sigma-abs(IntPoles(2)))) ) ...
    + atand( omegaD/(abs(IntPoles(1))-sigma) ) ) ; % zc2 returned as positive, but actually in LHP

z = [ -zc1 -zc2 ] 
p = 0 ;
k = 1 ;
C = zpk( z, p, k ) ;

rlocus(  C*VToAlpha )
title( 'Root Locus of Plant w/ PID Controller (Motor Voltage to Alpha)' )

% rltool( C*VToAlpha ) % use to determine requires gain, K for the compensator

%% Select Gain and Simulate Response

K = 0.465 ; % select based on root locus above
T = feedback( VToAlpha, K*C ) ; % generates output of alpha/Vm
t = 0:0.01:5 ;

T2 = feedback( 1, VToAlpha*K*C ) * VToTheta ; % generates output of theta/Vm (block diagram manipulation)
% T2 = minreal(T2) ; % pole-zero cancellation stabilizes rotary arm, but
% not physically accurate

figure(1)
subplot(2,1,1), impulse( T, t )
grid on
title('Impulse Disturbance Response of Pendulum Angle w/ PID Control (MATLAB Generated)');
ylabel( 'Alpha (rad)' )
xlabel( 'Time' )
axis( [ 0 t(end) -0.2 0.2 ] )

subplot(2,1,2), impulse( T2, t )
grid on
title('Impulse Disturbance Response of Rotary Arm Angle w/ PID Control (MATLAB Generated)');
ylabel( 'Theta (rad)' )
xlabel( 'Time' )
axis( [ 0 t(end) -pi/4 pi/4 ] )

% Display PID gains
Kp = (zc1 + zc2)*K
Ki = zc1*zc2*K
Kd = K

%% Run and Plot Simulink Simulation
sim( 'PID_ControlSim' )

figure(2)
subplot(3,1,1), plot(timeOut, VmApp)
grid on
title( 'Simulink - Applied Voltage' )
ylabel( 'Voltage (V)' )
xlabel( 'Time (seconds)' )

subplot(3,1,2), plot(timeOut, AlphaOut)
grid on
title( 'Simulink - Observed Pendulum Angle, alpha' )
ylabel( 'Alpha (deg)' )
xlabel( 'Time (seconds)' )

subplot(3,1,3), plot(timeOut, ThetaOut)
grid on
title( 'Simulink - Observed Rotary Arm Angle, theta' )
ylabel( 'Theta (deg)' )
xlabel( 'Time (seconds)' )
axis( [0 10 -90 90] )

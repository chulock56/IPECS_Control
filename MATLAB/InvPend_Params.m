%% IPECS: Inverted Pendulum to Educate Controls Students
%{

Drexel MEM Senior Design Team #027: 
    Chulock, A., Pruitt, W., Givens, G., Maher, C., Insalaco, D.

This code written by: Chulock A., Insalaco, D. on 2/7/2019

This code defines the IPECS inverted pendulum model parameters and
constants for use with the control design code.

%}

clear
clc

%% Define Model Constants

% General
    gty = 9.81 ; % acceleration due to gravity [m/s^2]

% Motor
% Quanser Values
    % etag = 0.90 ; % gearbox efficiency
    % etam = 0.69 ; % motor efficiency
    % kt   = 7.683e-3 ; % motor current-torque constant [N-m/A]
    % Kg   = 70 ; % total gear ratio (motor to rotary arm)
    % km   = 7.678e-3 ; % motor back-EMF constant [V/(rad/s)]
    % Rm   = 2.60 ; % motor terminal resistance [Ohms]
% SD Values - Allied Motion CL40 (7W), 12V DC
    etag = 1.00 ;
    etam = 0.68 ;
    kt   = 30e-3 ;
    Kg   = 1 ;
    km   = 29.70e-3 ;
    Rm   = 4.60 ;

% Pendulum
% Quanser Values
    % Jp = 1.199e-3 ; % pendulum moment of inertia about its CoM [kg-m^2]
    % mp = 0.127 ; % pendulum mass [kg]
    % Lp = 0.3366 ; % pendulum length [m]
    % Bp = 2.40e-3 ; % pendulum viscous damping coeff. [N-s/m]
% SD Estimate Values
    % Jp = 4.4285e-7 ;
    % mp = 0.0391 ;
    % Lp = 0.2032 ;
    Bp = 2.40e-3 ;
% Exact (measured values)
    dp = 9.64e-3 ;
    d  = 0.1522 ;  % = Lr
  % Short Pendulum
    % mp = ;
    % Lp = ;
  % Medium Pendulum
    mp = 40.9e-3 ;
    Lp = 20.219e-2 ;
  % Long Pendulum
    % mp = 85.9e-3 ;
    % Lp = 0.4325 ;

  Jp = 1/8*mp*dp^2 + mp*d^2 ;

% Rotary Arm
% Quanser Values
    % Jr = 9.983e-4 ; % rotary arm moment of inertia about its CoM [kg-m^2]
    % Lr = 0.2159 ; % rotary arm length [m]
    % Br = 2.40e-3 ; % rotary arm viscous damping coeff. [N-s/m]
% SD Estimate Values
    % Jr = 1.2412e-4 ;
    % Lr = 0.1524 ;
    Br = 2.40e-3 ;
% Exact (measured values)
    mr = 107.8e-3 ;
    Lr = 0.1522 ;
    a  = Lr ;
    c  = 31.80e-3 ;
    Jr = 1/12*mr*(a^2 + c^2) + 1/4*mr*a^2 ;

% Pendulum & Rotary Arm
    JT = Jp*mp*Lr^2 + Jr*Jp + 0.25*Jr*mp*Lp^2 ; % substitutive variable [kg-m^2]
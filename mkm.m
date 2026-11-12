clear; clc; close all;

%%\ PREAMBLE
n = 5;                           % number of DOF (joints)

x = repmat([1;0;0], 1, n);       % x(:,i): x axis of body frame {i}, in {0}
y = repmat([0;1;0], 1, n);       % y(:,i): y axis of body frame {i}, in {0}
z = repmat([0;0;1], 1, n);       % z(:,i): z axis of body frame {i}, in {0}

dtheta = [0 0 0 0 0]*pi/180;     % joint rotations: pre-initial -> initial [rad]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":46.9}
%---

clear; clc; close all;

n = 5;

% link vectors: components in own body frame {i}   (columns: l_12, l_23, l_34, l_45, l_5t)
Lb = [0.2  0    0    0.3  0.2;
      0    0    0    0    0  ;
      0    0.5  0.4  0    0  ];

% joint axes: components in own body frame {i}     (columns: h_1, ..., h_5)
hb = [0  0  0  0  1;
      0  1  0  1  0;
      1  0  1  0  0];

x = repmat([1;0;0], 1, n);       % x(:,i): x axis of body frame {i}, in {0}
y = repmat([0;1;0], 1, n);       % y(:,i): y axis of body frame {i}, in {0}
z = repmat([0;0;1], 1, n);       % z(:,i): z axis of body frame {i}, in {0}

dtheta = zeros(1,n)*pi/180;      % joint rotations: pre-initial -> initial [rad], one per joint

skew = @(a) [ 0     -a(3)   a(2);
              a(3)   0     -a(1);
              -a(2)   a(1)   0  ];      % skew(a)*b = a x b

for i = 1:n
    h = x.*hb(1,:) + y.*hb(2,:) + z.*hb(3,:);   % joint axes in {0}, from the current frames
    w = [ 0        -h(3,i)   h(2,i);
        h(3,i)    0       -h(1,i);
        -h(2,i)    h(1,i)   0      ];           % skew matrix of h_i (w*a = h_i x a)

    R = eye(3) + w*sin(dtheta(i)) + w^2*(1 - cos(dtheta(i)));   % Rodrigues' formula

    x(:,i:n) = R*x(:,i:n);
    y(:,i:n) = R*y(:,i:n);
    z(:,i:n) = R*z(:,i:n);
end

L = x.*Lb(1,:) + y.*Lb(2,:) + z.*Lb(3,:);   % link vectors in {0}: l_12, l_23, l_34, l_45, l_5t
phit = zeros(6, 6*n);
phit(:, 6*n-5:6*n) = [  eye(3)         zeros(3);
    -skew(L(:,n))   eye(3)   ];     % phi_tn

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":46.9}
%---

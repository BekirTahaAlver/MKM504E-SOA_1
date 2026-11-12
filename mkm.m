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
%dtheta = [30 45 20 -60 15]*pi/180

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

H = zeros(6*n, n);
for i = 1:n
    H(6*i-5:6*i-3, i) = h(:,i);      % H_i = [h_i; 0] in block row i, column i
end

phi = eye(6*n);
for i = 1:n-1
    phi = phi + diag(ones(1, 6*(n-i)), -6*i);
end
for i = 1:n-1
    r = 6*(i+1)-2 : 6*(i+1);       % rows: lower half of block row i+1
    c = 6*i-5     : 6*i-3;         % columns: left half of block column i
    phi(r, c) = -skew(L(:,i));     % phi_{i+1,i}: -l_{i,i+1} x
end

for i = 3:n
    for j = 1:i-2
        r  = 6*i-2     : 6*i;          % rows: lower half of block row i
        rp = 6*(i-1)-2 : 6*(i-1);      % rows: lower half of block row i-1
        c  = 6*j-5     : 6*j-3;        % columns: left half of block column j
        cd = 6*(i-1)-5 : 6*(i-1)-3;    % columns: left half of block column i-1
        phi(r, c) = phi(rp, c) + phi(r, cd);
    end
end

J = phit*phi*H;                  % [w_t; v_t] = J * thetadot

p = zeros(3, n+1);               % p(:,i): joint point i, p(:,n+1): tip t
for i = 1:n
    p(:,i+1) = p(:,i) + L(:,i);
end

Jc = zeros(6, n);
for i = 1:n
    Jc(:,i) = [h(:,i); cross(h(:,i), p(:,n+1) - p(:,i))];
end

max(abs(J(:) - Jc(:)))           % should be ~1e-16 %[output:1de73a47]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":46.9}
%---
%[output:1de73a47]
%   data: {"dataType":"textualVariable","outputData":{"name":"ans","value":"5.5511e-17"}}
%---

% Define system transfer funtion
num = [25];
den = [1 2 4];
G = tf(num, den);
rg = order(G);

% Find controller canonical state-space representation
[Ac, Bc, Cc, Dc] = tf2ss(num, den)

% Convert to phase variable form
%V = [0 1; 1 0]
V = fliplr(eye(rg))
Ap = inv(V)*Ac*V
Bp = Cc' 
Cp = Bc'
Dp = Dc

% Use these equations to make Bp = [0 1]' and Cp = [25 0]
%Bp = inv(V)*Bc 
%Cp = Cc*V

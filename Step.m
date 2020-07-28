% Reference:
% Daniël Karssen and Martijn Wisse
% Delft Biorobotics Lab, TUDelft, March 2008

function [s_end, t_end, data] = Step(s0, t0, par)

abs_tol = 1e-10;
max_time = 5;
time_step = 0.001;

data.event = 'none';
options = odeset('Events',@EventDetection,'AbsTol',abs_tol);
tspan = t0:time_step:t0+max_time;

[t_new,s_new,~,~,events] = ode45(@EOM,tspan,s0,options,par);

data.t = t_new;
data.s = s_new;
t_end = t_new(end);
s_end = s_new(end,:)';
data.posfoot = zeros(size(data.t));

if t_end==(t0+max_time)
    data.event = 'timeup';
    s_end = NaN*ones(4,1);
    return
elseif events(end)~=1
    data.event = 'fall';
    s_end = NaN*ones(4,1);
    return
end

[t_new,s_new] = Impact(t_end,s_end,par);

data.t = [data.t; t_new];
data.s = [data.s; s_new'];
t_end = t_new;
s_end = s_new;

phi_sw = s_end(2);
posfoot = -2*par.R*phi_sw-2*(par.L-par.R)*sin(phi_sw);
data.posfoot(end+1) = posfoot;





%% EOM
function s_dot = EOM(~,s,par)

phi_st = s(1);
phi_sw = s(2);
phi_st_d = s(3);
phi_sw_d = s(4); 

T = [ -par.R-(par.L-par.R)*cos(phi_st)-sin(phi_st)*par.B+cos(phi_st)*par.C, 0;
-(par.L-par.R)*sin(phi_st)+cos(phi_st)*par.B+sin(phi_st)*par.C, 0;
1, 0;
-par.R-(par.L-par.R)*cos(phi_st),-sin(phi_sw)*par.B+cos(phi_sw)*par.C;
-(par.L-par.R)*sin(phi_st), cos(phi_sw)*par.B+sin(phi_sw)*par.C;
0, 1];

D = [ ((par.L-par.R)*sin(phi_st)-cos(phi_st)*par.B-sin(phi_st)*par.C)*phi_st_d^2;
(-(par.L-par.R)*cos(phi_st)-sin(phi_st)*par.B+cos(phi_st)*par.C)*phi_st_d^2;
0;
(par.L-par.R)*sin(phi_st)*phi_st_d^2+(-cos(phi_sw)*par.B-sin(phi_sw)*par.C)*phi_sw_d^2;
-(par.L-par.R)*cos(phi_st)*phi_st_d^2+(-sin(phi_sw)*par.B+cos(phi_sw)*par.C)*phi_sw_d^2;
0];

M = diag([par.m, par.m, par.I, par.m, par.m, par.I]);

Mr = T.'*M*T;

f = M*[sin(par.gamma);-cos(par.gamma);0;sin(par.gamma);-cos(par.gamma);0]*par.g;

par.rf = 0;
par.hf = 0;
hip_d = phi_st_d-phi_sw_d;
fric =   [-par.rf*phi_st_d; 0]...
       + [-par.hf*hip_d;par.hf*hip_d];

rhs = T.'*f  - T.'*M*D + fric;

udot = Mr\rhs;

s_dot = [phi_st_d; phi_sw_d; udot];





%% Detect Event
function [value, isterminal, direction] = EventDetection(~,s,~)

phi_st = s(1);
phi_sw = s(2);

value(1) = phi_st + phi_sw;

if phi_st < -0.001
  isterminal(1) = 1;
else
  isterminal(1) = 0;
end
direction(1) = -1;

value(2) = phi_st+pi/2;
direction(2) = -1;
isterminal(2) = 1;

value(3) = phi_st-pi/2;
direction(3) =  1;
isterminal(3) = 1;


%% Impact
function [t,s_plus] = Impact(t,s_minus,par)

phi_st = s_minus(1);
phi_sw = s_minus(2);
phi_st_d = s_minus(3);
phi_sw_d = s_minus(4);

T_st = [-par.R-(par.L-par.R)*cos(phi_st)-sin(phi_st)*par.B+cos(phi_st)*par.C, 0;
-(par.L-par.R)*sin(phi_st)+cos(phi_st)*par.B+sin(phi_st)*par.C, 0;
1, 0;
-par.R-(par.L-par.R)*cos(phi_st), -sin(phi_sw)*par.B+cos(phi_sw)*par.C;
-(par.L-par.R)*sin(phi_st), cos(phi_sw)*par.B+sin(phi_sw)*par.C;
0, 1];

T_sw = [ -sin(phi_st)*par.B+cos(phi_st)*par.C, -par.R-(par.L-par.R)*cos(phi_sw);
 cos(phi_st)*par.B+sin(phi_st)*par.C, -(par.L-par.R)*sin(phi_sw);
1, 0;
0, -par.R-(par.L-par.R)*cos(phi_sw)-sin(phi_sw)*par.B+cos(phi_sw)*par.C;
0, -(par.L-par.R)*sin(phi_sw)+cos(phi_sw)*par.B+sin(phi_sw)*par.C;
0, 1];

M = diag([par.m, par.m, par.I, par.m, par.m, par.I]);

Mr = T_sw.'*M*T_sw;

x_d = T_st*[phi_st_d; phi_sw_d];

rhs = T_sw.'*M*x_d;

solution = Mr\rhs;

s_plus = [phi_sw; 
          phi_st; 
          solution(2);
          solution(1)];


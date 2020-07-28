% Reference:
% Daniël Karssen and Martijn Wisse
% Delft Biorobotics Lab, TUDelft, March 2008

function [s_end, t_end, data] = Walk(s0, t0, par, nr_steps)

s = s0;
t = t0;
data.s = [];
data.t = [];
data.posfoot = [];
prev_pos_foot = 0;

for n=1:nr_steps
    
    [s,t,datanew] = Step(s,t,par);
    
    data.s = [data.s; datanew.s];
    data.t = [data.t; datanew.t];
    data.posfoot = [data.posfoot; datanew.posfoot+prev_pos_foot];
    prev_pos_foot = data.posfoot(end);

    if strcmp(datanew.event,'fall') || strcmp(datanew.event,'timeup')
        break;
    end
    
end

s_end = data.s(end,:);
t_end = data.t(end);
data.event = datanew.event;
% Reference:
% Daniël Karssen and Martijn Wisse
% Delft Biorobotics Lab, TUDelft, March 2008

function Walk_and_Step_Animate(data,par,speed)

%% Settings
leg1color = [1 0 0];
leg2color = [0 0 1];
leg_width = 0.015;
mass_width = 0.02;

%% Create leg shape
if leg_width<par.R
    alpha = acos(leg_width/par.R);
else
    alpha = 0;
end

arc = [];
for n=linspace(pi/2-alpha,-pi/2+alpha,20)
    arc = [arc, par.R*[sin(n);cos(n)]];
end
nom_leg_pos = [-leg_width, leg_width,  arc(1,:)
                       0,          0,  -par.L+par.R-arc(2,:)];

%% Calculate hip position
data.hippos = zeros(length(data.t),2);
for n=1:length(data.t)
    data.hippos(n,:) = [-par.R*data.s(n,1) - (par.L-par.R)*sin(data.s(n,1))
                        par.R  + (par.L-par.R)*cos(data.s(n,1))];
    data.hippos(n,:) = data.hippos(n,:) + [data.posfoot(n), 0];
end

%% Objects
animationfig = figure(2); set(animationfig,'Position',[0 0 500 400])
AxesHandle = axes('Parent',animationfig,  'Position',[0 0 1 1]);
Leg1Handle = patch('Parent',AxesHandle, 'FaceColor',leg1color);
if isfield(par, 'mm') && par.mm~=0
    Mass1Handle = rectangle('Parent',AxesHandle,'FaceColor',[0 1 0], 'Curvature',[1 1]);
end
Leg2Handle = patch('Parent',AxesHandle, 'FaceColor',leg2color);
FloorHandle = line('Parent',AxesHandle, 'Color',[0 0 0], 'LineWidth',2);
AxleHandle = rectangle('Parent',AxesHandle, 'FaceColor',[0 0 0], 'Curvature',[1 1]);
if isfield(par, 'mm') && par.mm~=0
    Mass2Handle = rectangle('Parent',AxesHandle,'FaceColor',[0 1 0], 'Curvature',[1 1]);
end
TextHandle = text('Parent',AxesHandle,'Position',[0,-0.1],'BusyAction','queue','Color',[0 0 0]);

%% Size window
min_x = min(data.hippos(:,1))-par.L;
max_x = max(data.hippos(:,1))+par.L;
min_y = -par.L/2;
max_y = max(data.hippos(:,2))+par.L/2;

size_xy = 1.44;
if (max_x-min_x)/size_xy > (max_y-min_y)
    min_y = min_y-((max_x-min_x)/size_xy - (max_y-min_y))/2;
    max_y = max_y+((max_x-min_x)/size_xy - (max_y-min_y))/2;
else
    min_x = min_x-((max_y-min_y)*size_xy - (max_x-min_x))/2;
    max_x = max_x+((max_y-min_y)*size_xy - (max_x-min_x))/2;
end
set(AxesHandle,...
   'Ylim',[min_y max_y],...
   'Xlim',[min_x max_x]);
axis off
set(TextHandle,...
    'Position',[min_x,max_y],...
    'HorizontalAlignment','left',...
    'VerticalAlignment','top');

%% Speed control
if nargin<3; speed = 1; end     % default speed
dt = data.t(2)-data.t(1);
nom_cycle_time = 0.003;
nn = ceil(nom_cycle_time/dt*speed);

%% Animation
tic % start timer

for n=1:nn:length(data.t) 
    % state vector
    phi_st = data.s(n,1);
    phi_sw = data.s(n,2);
    
    % leg 1
    leg1pos = rotationmatrix(phi_st)*nom_leg_pos;     % rotate leg to correct angle
    leg1pos(1,:) = leg1pos(1,:) + data.hippos(n,1);   % add hip offset (x-direction)
    leg1pos(2,:) = leg1pos(2,:) + data.hippos(n,2);   % add hip offset (y-direction)
    leg1pos = rotationmatrix(-par.gamma)*leg1pos;     % rotate coordinate system with sloop
    set(Leg1Handle,'Xdata',leg1pos(1,:),'Ydata',leg1pos(2,:));
    
    % leg 2
    leg2pos = rotationmatrix(phi_sw)*nom_leg_pos;     % rotate leg to correct angle
    leg2pos(1,:) = leg2pos(1,:) + data.hippos(n,1);   % add hip offset (x-direction)
    leg2pos(2,:) = leg2pos(2,:) + data.hippos(n,2);   % add hip offset (y-direction) 
    leg2pos = rotationmatrix(-par.gamma)*leg2pos;     % rotate coordinate system with sloop    
    set(Leg2Handle,'Xdata',leg2pos(1,:),'Ydata',leg2pos(2,:));
    
    % floor
    floor_pos = [min_x, max_x; -0.001 -0.001];
    floor_pos = rotationmatrix(-par.gamma)*floor_pos; % rotate coordinate system with sloop 
    set(FloorHandle,'Xdata',floor_pos(1,:),'Ydata',floor_pos(2,:));
    
    % axle
    axle_pos = data.hippos(n,:)';
    axle_pos = rotationmatrix(-par.gamma)*axle_pos;   % rotate coordinate system with sloop
    axle_pos = [axle_pos(1)-mass_width/4, axle_pos(2)-mass_width/4, mass_width/2, mass_width/2];
    set(AxleHandle,'Position',axle_pos);
    
    if isfield(par, 'mm') && par.mm~=0
        % mass 1
        mass1_pos = rotationmatrix(phi_st)*[par.mB; -par.mC];
        mass1_pos = mass1_pos + data.hippos(n,:)';         % add hip offset
        mass1_pos = rotationmatrix(-par.gamma)*mass1_pos;  % rotate coordinate system with sloop
        mass1_pos = [mass1_pos(1)-mass_width/2, mass1_pos(2)-mass_width/2, mass_width, mass_width];
        set(Mass1Handle,'Position',mass1_pos);

        % mass 2
        mass2_pos = rotationmatrix(phi_sw)*[par.mB; -par.mC];
        mass2_pos = mass2_pos + data.hippos(n,:)';         % add hip offset
        mass2_pos = rotationmatrix(-par.gamma)*mass2_pos;  % rotate coordinate system with sloop
        mass2_pos = [mass2_pos(1)-mass_width/2, mass2_pos(2)-mass_width/2, mass_width, mass_width];
        set(Mass2Handle,'Position',mass2_pos);
    end
    
    % text
    set(TextHandle,'String',sprintf(' t = %0.2fs',data.t(n)));
    
    drawnow
    
    % speed control
    while toc<(data.t(n)/speed)
        1+1;        % waste time
    end 
end

pause(1)
close(animationfig)


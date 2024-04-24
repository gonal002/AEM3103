	global CL CD S m g rho	
	S		=	0.017;			% Reference Area, m^2
	AR		=	0.86;			% Wing Aspect Ratio
	e		=	0.9;			% Oswald Efficiency Factor;
	m		=	0.003;			% Mass, kg
	g		=	9.8;			% Gravitational acceleration, m/s^2
	rho		=	1.225;			% Air density at Sea Level, kg/m^3	
	CLa		=	3.141592 * AR/(1 + sqrt(1 + (AR / 2)^2));
							% Lift-Coefficient Slope, per rad
	CDo		=	0.02;			% Zero-Lift Drag Coefficient
	epsilon	=	1 / (3.141592 * e * AR);% Induced Drag Factor	
	CL		=	sqrt(CDo / epsilon);	% CL for Maximum Lift/Drag Ratio
	CD		=	CDo + epsilon * CL^2;	% Corresponding CD
	LDmax	=	CL / CD;			% Maximum Lift/Drag Ratio
	Gam		=	-atan(1 / LDmax);	% Corresponding Flight Path Angle, rad
	V		=	sqrt(2 * m * g /(rho * S * (CL * cos(Gam) - CD * sin(Gam))));
							% Corresponding Velocity, m/s
	Alpha	=	CL / CLa;			% Corresponding Angle of Attack, rad
	
%%	a) Equilibrium Glide at Maximum Lift/Drag Ratio -- Question 2
	H		=	2;			% h_0 [m]
	R		=	0;			% R_0 [m]
	to		=	0;			% t_0 [sec]
	tf		=	6;			% t_f [sec]
	tspan	=	[to tf];
    
    % Same Flight path angle, varying velocity
    Gam = -0.18;
    Vn = 3.55; % Nominal
	xoa		=	[Vn;Gam;H;R];
	[ta,xa]	=	ode23('EqMotion',tspan,xoa);

    Vl = 2; % Lower
    xob		=	[Vl;Gam;H;R];
	[tb,xb]	=	ode23('EqMotion',tspan,xob);

    Vh = 7.5; % Higher
    xoc		=	[Vh;Gam;H;R];
	[tc,xc]	=	ode23('EqMotion',tspan,xoc);

    % Same Velocity, varying Flight Path Angle
    Gam_n = -0.18; % Nominal
	xod		=	[Vn;Gam_n;H;R];
	[td,xd]	=	ode23('EqMotion',tspan,xod);

    Gam_l = -0.5; % Lower
    xoe		=	[Vn;Gam_l;H;R];
	[te,xe]	=	ode23('EqMotion',tspan,xoe);

    Gam_h = 0.4; % Higher
    xog		=	[Vn;Gam_h;H;R];
	[tg,xg]	=	ode23('EqMotion',tspan,xog);
    
    % Plot 2
    figure
	subplot(2,1,1)
    plot(xa(:,4),xa(:,3),xb(:,4),xb(:,3),xc(:,4),xc(:,3))
    xlabel('Range, m'), ylabel('Height, m'), grid
    title('Varying Velocity', 'FontSize', 12);
    %% Change colors of different velocities

    subplot(2,1,2)
    plot(xd(:,4),xd(:,3),xe(:,4),xe(:,3),xg(:,4),xg(:,3))
    xlabel('Range, m'), ylabel('Height, m'), grid
    title('Varying Flight Path Angle', 'FontSize', 12);
    %% Change colors of different velocities

    %% Question 3
    


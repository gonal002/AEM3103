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
	tspan	=	to:0.01:tf;
    
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
    hold on
    plot(xa(:,4),xa(:,3), 'k-')
    plot(xb(:,4),xb(:,3), 'r-')
    plot(xc(:,4),xc(:,3), 'g-')
    xlabel('Range, m'), ylabel('Height, m'), grid
    title('Varying Velocity', 'FontSize', 12);
    hold off

    subplot(2,1,2)
    hold on
    plot(xd(:,4),xd(:,3), 'k-')
    plot(xe(:,4),xe(:,3), 'r-')
    plot(xg(:,4),xg(:,3), 'g-')
    xlabel('Range, m'), ylabel('Height, m'), grid
    title('Varying Flight Path Angle', 'FontSize', 12);
    hold off

    %% Question 3
    figure
    hold on
    altitudes = 0; 
    ranges = 0;
    times = 0;

    for i = 1:100
        Vi = Vl + (Vh - Vl)*rand(1);
        Gam_i = Gam_l + (Gam_h - Gam_l)*rand(1);
        xoi	= [Vi;Gam_i;H;R];
	    [ti,xi]	= ode23('EqMotion',tspan,xoi);
        plot(xi(:,4),xi(:,3), 'k--')
        altitudes = cat(1, altitudes, xi(:,3));
        ranges = cat(1, ranges, xi(:,4));
        times = cat(1, times, ti);
    end
    xlabel('Range, m'), ylabel('Height, m'), grid
    title('Varying Velocity & Flight Path Angle 100x', 'FontSize', 12);

    %% Question 4
    x = times;
    a = altitudes;
    ph = polyfit(x, a, 5);
    h_y_fit = polyval(ph, tspan);

    figure
    title('Fitting Simulation Data to 5th order Polynomial')
    subplot(2,1,1)
    plot(tspan, h_y_fit, 'g-')
    xlabel('Time [s]')
    ylabel('Height [m]')

    r = ranges;
    pr = polyfit(x, r, 5);
    r_y_fit = polyval(pr, tspan);

    subplot(2,1,2)
    plot(tspan, r_y_fit, 'r-')
    xlabel('Time [s]')
    ylabel('Range [m]')
   
    %% Question 5
    d_h = polyder(ph);
    d_r = polyder(pr);
    der_h = polyval(d_h, tspan);
    der_r = polyval(d_r, tspan);

    figure
    subplot(2,1,1)
    plot(tspan, der_h)
    xlabel('Time [s]')
    ylabel('Change in Height [m/s]')

    subplot(2,1,2)
    plot(tspan, der_r)
    xlabel('Time [s]')
    ylabel('Ground Speed [m/s]')

    % sum_alt = 0;
    % sum_ran = 0;
    % avg_altitudes = zeros(1, length(tspan));
    % avg_ranges = zeros(1, length(tspan));
    % for i = 1:length(tspan)
    %     for j = 1:100
    %         sum_alt = sum_alt + altitudes(i, j);
    %         sum_ran = sum_ran + ranges(i, j);
    %     end
    %     avg_altitudes(i) = sum_alt/100;
    %     avg_ranges(i) = sum_ran/100;
    %     sum_alt = 0;
    %     sum_ran = 0;
    % end

    % t = tspan.';
    % avgs = zeros(length(tspan), 3);
    % avgs(:, 1) = tspan;
    % avgs(:, 2) = avg_altitudes;
    % avgs(:, 3) = avg_ranges;
    
    % plot(avg_ranges, avg_altitudes, 'm-', 'LineWidth', 2)
    % hold off
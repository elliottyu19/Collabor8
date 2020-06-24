function S = get_S(Vmax)
    V_rmax = Vmax; %Maximum robot velocity is 0.4 m/s
    V_hmax = 1.6; %Maximum human velocity is 1.6 m/s
    t_reac = 0.1; %The amount of time the robot needs to react to min dist braeach
    t_stop = 0.25; %The maximum amount of time the robot needs to stop
    C = 0.;
    Zd = 0.05;
    Zr = 0.05;

    Sr = V_rmax * t_reac; %The contribution to seperation distance from robot's reaction time.
    Ss = V_rmax * t_stop; %The contribution to seperation distance from robot's stopping time.
    Sh = V_hmax*(t_reac + t_stop); %The contribution from the user's change in location.

    S = Sh + Sr + Ss + C + Zd + Zr;
end
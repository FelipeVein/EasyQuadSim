function Draw_Trajectory(varargin)


global quad;

quad.Tmax = quad.waypoints.T(end);
quad.plot.t = 0:quad.dt:quad.Tmax;

quad.rdes_plot= [];


[quad.rdes_plot(:,1),~, ~, ~, ~]=TrajectoryPlanner(quad.waypoints.X,quad.dt,quad.waypoints.T);
[quad.rdes_plot(:,2),~, ~, ~, ~]=TrajectoryPlanner(quad.waypoints.Y,quad.dt,quad.waypoints.T);
[quad.rdes_plot(:,3),~, ~, ~, ~]=TrajectoryPlanner(quad.waypoints.Z,quad.dt,quad.waypoints.T);


quad.rdes_plot = quad.rdes_plot';



end
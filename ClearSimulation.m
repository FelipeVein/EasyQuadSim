function ClearSimulation(varargin)

global quad;

choice = questdlg('Erase waypoints?', ...
	'End simulation', ...
	'Yes','No','Cancel','No');
% Handle response
switch choice
    case 'Yes'
        quad.ApagarWaypoints = 1;
    case 'No'
        quad.ApagarWaypoints = 0;
    case 'Cancel'
        return
    case ''
        return
end
quad.StartSimulation =3;

set(quad.StartButton,'Visible','on');
set(quad.ControllerList,'Visible','on');
end
function MarkWaypoint(varargin)

    global quad;
    x=str2double(get(quad.handler.X,'String'));
    y=str2double(get(quad.handler.Y,'String'));
    z=str2double(get(quad.handler.Z,'String'));
    yaw=str2double(get(quad.handler.YAW,'String'));
    t=str2double(get(quad.handler.T,'String'));
    
    if(isnan(x)||isnan(y)||isnan(z)||isnan(yaw)||isnan(t))
        errordlg('ERROR - INVALID CHARACTER.');
        return
    end
    
    
    if(t > quad.waypoints.T(end))
        quad.waypoints.X = [quad.waypoints.X x];
        quad.waypoints.Y = [quad.waypoints.Y y];
        quad.waypoints.Z = [quad.waypoints.Z z];
        quad.waypoints.YAW = [quad.waypoints.YAW yaw];
        quad.waypoints.T = [quad.waypoints.T t];
    else
        errordlg(['ERROR - T SHOULD BE GREATER THAN ' mat2str(quad.waypoints.T(end)) '.']);
        return 
    end
    
    Draw_Trajectory();

end
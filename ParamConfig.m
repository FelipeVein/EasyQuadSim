function ParamConfig(varargin)

global quad;
figure('units','normalized','position',[0.4 .25 .3 .65],'name','Parâmetros do Quadrotor - UFJF','color','w','numbertitle','off');

quad.handler.rotacaoMax = uicontrol('units','normalized','position',[.45 .29 .35 .075],'style','edit','fontsize',10,'string',quad.rotacaoMax,'backgroundcolor','w');
quad.handler.m = uicontrol('units','normalized','position',[.45 .85 .35 .075],'style','edit','fontsize',10,'string',quad.m,'backgroundcolor','w');
quad.handler.l = uicontrol('units','normalized','position',[.45 .77 .35 .075],'style','edit','fontsize',10,'string',quad.l,'backgroundcolor','w');
quad.handler.b = uicontrol('units','normalized','position',[.45 .69 .35 .075],'style','edit','fontsize',10,'string',quad.b,'backgroundcolor','w');
quad.handler.k = uicontrol('units','normalized','position',[.45 .61 .35 .075],'style','edit','fontsize',10,'string',quad.k,'backgroundcolor','w');
quad.handler.Ixx = uicontrol('units','normalized','position',[.45 .53 .35 .075],'style','edit','fontsize',10,'string',quad.Ixx,'backgroundcolor','w');
quad.handler.Iyy = uicontrol('units','normalized','position',[.45 .45 .35 .075],'style','edit','fontsize',10,'string',quad.Iyy,'backgroundcolor','w');
quad.handler.Izz = uicontrol('units','normalized','position',[.45 .37 .35 .075],'style','edit','fontsize',10,'string',quad.Izz,'backgroundcolor','w');% quad.handler.xaxis.min = uicontrol('units','normalized','position',[.45 .29 .15 .075],'style','edit','fontsize',10,'string',quad.xaxis.min,'backgroundcolor','w');
quad.handler.rotacaoMax = uicontrol('units','normalized','position',[.45 .29 .35 .075],'style','edit','fontsize',10,'string',quad.rotacaoMax,'backgroundcolor','w');
quad.handler.rotacaoMin = uicontrol('units','normalized','position',[.45 .21 .35 .075],'style','edit','fontsize',10,'string',quad.rotacaoMin,'backgroundcolor','w');

uicontrol('units','normalized','position',[.2 .85 .2 .075],'style','text','fontsize',10,'string','m','backgroundcolor','w');
uicontrol('units','normalized','position',[.2 .77 .2 .075],'style','text','fontsize',10,'string','l','backgroundcolor','w');
uicontrol('units','normalized','position',[.2 .69 .2 .075],'style','text','fontsize',10,'string','b','backgroundcolor','w');
uicontrol('units','normalized','position',[.2 .61 .2 .075],'style','text','fontsize',10,'string','k','backgroundcolor','w');
uicontrol('units','normalized','position',[.2 .53 .2 .075],'style','text','fontsize',10,'string','Ixx','backgroundcolor','w');
uicontrol('units','normalized','position',[.2 .45 .2 .075],'style','text','fontsize',10,'string','Iyy','backgroundcolor','w');
uicontrol('units','normalized','position',[.2 .37 .2 .075],'style','text','fontsize',10,'string','Izz','backgroundcolor','w');
uicontrol('units','normalized','position',[.2 .29 .2 .075],'style','text','fontsize',10,'string','Rotor Max Speed [rpm]','backgroundcolor','w');
uicontrol('units','normalized','position',[.2 .21 .2 .075],'style','text','fontsize',10,'string','Rotor Min Speed [rpm]','backgroundcolor','w');



uicontrol('units','normalized','position',[.4 .05 .2 .05],'style','pushbutton','fontsize',10,'string','Accept','callback',@AcceptConfig);

end



function AcceptConfig(varargin)

global quad;
quad.m = str2double(get(quad.handler.m,'String'));
quad.l = str2double(get(quad.handler.l,'String'));
quad.b = str2double(get(quad.handler.b,'String'));
quad.k = str2double(get(quad.handler.k,'String'));
quad.Ixx = str2double(get(quad.handler.Ixx,'String'));
quad.Iyy = str2double(get(quad.handler.Iyy,'String'));
quad.Izz = str2double(get(quad.handler.Izz,'String'));

quad.xaxis.min = str2double(get(quad.handler.xaxis.min,'String'));
quad.xaxis.max = str2double(get(quad.handler.xaxis.max,'String'));
quad.yaxis.min = str2double(get(quad.handler.yaxis.min,'String'));
quad.yaxis.max = str2double(get(quad.handler.yaxis.max,'String'));
quad.zaxis.min = str2double(get(quad.handler.zaxis.min,'String'));
quad.zaxis.max = str2double(get(quad.handler.zaxis.max,'String'));

quad.b1 = [-quad.l;0;0];
quad.b2 = [0;-quad.l;0];
quad.b3 = [quad.l;0;0];
quad.b4 = [0;quad.l;0];


quad.I = [quad.Ixx 0 0;0 quad.Iyy 0; 0 0 quad.Izz];

close(gcf);

end
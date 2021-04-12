
function Make_Animation(TimeTable,IK_Table,Task_params,Cube,Handler_IK_Solution,Handler_IK_Model)

tf = TimeTable(end);
dt = 0.1;
% 
% Cube_origin = [0.3 -0.2 -0.3138];
% Cube_size = [-0.45 0.5 -0.2];

Handler_Simulation = SRD_get_handler__Simulation(...
    'TimeLog',TimeTable );
DrawRobot_function = SRD_DrawRobot_get_function('DrawRobot_Type', 'STL', ... %'Default' or 'STL' or 'Custom'
    'DrawRobot_Custom_handle', [], ...
    'Function_Type', 'DrawGivenPosition', ... %'DrawGivenPosition' or 'DrawInitialPosition'  or 'DrawCurrentPosition'
    'Chain', [], ...
    'FileName_visuals_config', 'datafile_visuals_config.mat'); %use default visuals
SRD__animate__position_m('Handler_Simulation', Handler_Simulation, ...
    'Handler_Logger_position', Handler_IK_Solution, ...
    'AnimationTimeLog', 0:10*dt:(tf-dt), ...
    'DrawRobot_function', DrawRobot_function, ...
    'Handler_IK_Model', Handler_IK_Model,...
    'Task_params',Task_params,...
    'IK_Table', IK_Table,...
    'Cube',Cube,...
    'NewFigure', true, ...
    'FigureName', 'Animation', ...
    'FileName_visuals_config', []);
end





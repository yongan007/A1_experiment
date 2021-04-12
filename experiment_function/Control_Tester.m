

function [Max_Error_x,Max_cost_u,cost_qr]=Control_Tester(time_table,x_table, K_table, solution_tape,Q,R) 

n = size(x_table,1)/2;
Error_x = solution_tape-x_table';
Max_Error_x = max(abs(Error_x(:,1:n)));

cost_u = zeros(n,length(time_table));
cost_qr = 0;
for i=1:length(time_table)
cost_u(:,i)= K_table(:,:,i)*Error_x(i,:)';
cost_qr =+ Error_x(i,:)*Q*Error_x(i,:)'+cost_u(:,i)'*R*cost_u(:,i);
end
Max_cost_u = max(abs(cost_u'));

disp("Max Error of each state:")
disp(Max_Error_x)
disp("Max Cost U:")
disp(Max_cost_u)
disp("Max Cost QR: "+cost_qr)

figure('Color', 'w')
% subplot(2, 1, 1)
SRDgraphic_PlotGeneric(time_table', x_table(1:n,:)', ...
    'NewFigure', false, ...
    'Title', 'Joint position', ...
    'LableVariable', 'q');
hold on
plot(time_table, solution_tape(:,1:n), 'LineWidth', 3)
legend('${q_{des1}}$','$q_{des2}$','$q_{des3}$',...
    '$q_{sol1}$','$q_{sol2}$','$q_{sol3}$','interpreter','latex')
hold off

% subplot(2, 1, 2)
% SRDgraphic_PlotGeneric(time_table', x_table(:,1:n)', ...
%     'NewFigure', false, ...
%     'Title', 'Joint Velocity', ...
%     'LableVariable', '\dot{q}');
% hold on
% plot(time_table, solution_tape(:,n:2*n), 'LineWidth', 3)
% legend('$\dot{q}_{des1}$','$\dot{q}_{des2}$','$\dot{q}_{des3}$',...
%     '$\dot{q}_{sol1}$','$\dot{q}_{sol2}$','$\dot{q}_{sol3}$','interpreter','latex')
% hold off
drawnow;


figure('Color', 'w')
% subplot(2, 1, 1)
SRDgraphic_PlotGeneric(time_table', Error_x(:,1:n), ...
    'NewFigure', false, ...
    'Title', 'Joints position Erorr', ...
    'LableVariable', 'e');
% subplot(2, 1, 2)
% SRDgraphic_PlotGeneric(time_table', Error_x(:,:), ...
%     'NewFigure', false, ...
%     'Title', 'Joints velocity Erorr', ...
%     'LableVariable', '\dot{e}');
drawnow;

SRDgraphic_PlotGeneric(time_table', cost_u(:,:)', ...
    'NewFigure', true, ...
    'Title', 'Controler cost |u-u*|', ...
    'LableVariable', '{e_u}');
drawnow;
end 
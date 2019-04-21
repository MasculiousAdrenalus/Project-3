%Author: Ian Bartlett, z3419581
%Program: Utility function to initalize plot handles for Part 3

function [handles] = initialize_plots(times)

handles.master_fig = figure(1);
clf; hold on;
handles.att_subplot = subplot(121);
handles.att_trace = plot(0,0,'b-');
handles.att_title = title('Vehicle Attitude vs Time');
xlabel('Time (s)');
ylabel('Attitude (deg.)');
grid on;
xlim([0, range(times)]);
ylim([0,500]);

handles.xy_subplot = subplot(122);
handles.xy_trace = plot(0,0,'b-');
handles.xy_title = title('Vehicle XY Position (Dead Reckoning)');
xlabel('X Position (m)');
ylabel('Y Position (m)');
grid on;

handles.object_figure = figure(2);
clf; hold on;
handles.object_title = title('OOI Plot');
handles.object_global_centers = plot(0,0,'g *');
handles.object_global_labels = text(zeros(1,10),zeros(1,10),'','Color','green');
handles.object_local_labels = text(zeros(1,10),zeros(1,10),'','Color','magenta');
handles.object_local_centers = plot(0,0,'m *');
handles.object_data_points = plot(0,0,'k. ');
handles.object_bright_points = plot(0,0,'rx ');
handles.vehicle_trace = plot(0,0,'b- ');
handles.kalman_trace = plot(0,0,'r- ');
xlabel('X Position (m)');
ylabel('Y Position (m)');
axis equal;
axis manual;
xlim([-10,10]);
ylim([-10,10]);
grid on;

handles.bias_figure = figure(3);
clf; hold on;
handles.bias_title = title('Gyroscope bias estimate');
handles.bias_initial = plot(0,0,'b-')
handles.bias_kalman = plot(0,0,'r-')
xlabel('Time (s)');
ylabel('Bias estimate (rad s^-1)');
end
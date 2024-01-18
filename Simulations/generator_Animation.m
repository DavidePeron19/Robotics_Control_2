%% With this sscript we will generate thee aanimations for the switcching controllers developed
simulations_names = ["SW_01", "SW_02" "SW_03", "SW_04", "SW_05", "SW_06", "SW_07", "SW_08"];
figures_names =["Switching: Position Regulation and Output Feedback with Change of Variables",
                "Switching: Position Regulation and State Feedback with Exact Linearization",
                "Switching: Position Regulation and Output Feedback with Further Derivatives",
                "Switching: Position Regulation and State Feedback with Approximate Linearization",
                "Switching: Posture Regulation and Output Feedback with Change of Variables",
                "Switching: Posture Regulation and State Feedback with Exact Linearization",
                "Switching: Posture Regulation and Output Feedback with Further Derivatives",
                "Switching: Posture Regulation and State Feedback with Approximate Linearization"];

for j=1:length(simulations_names)
    load("Test_Results/"+simulations_names(j)+".mat");
    simul = eval(simulations_names(j));
    generateAnimation(figures_names(j), simulations_names(j), simul, true, true, 2);
    disp("DONE Controller: "+num2str(j));
end

function generateAnimation(figure_name, save_name, simulation, regulation, tracking)
    % define room dimensions
    room.width=7;
    room.height=1;
    
    % define plot properties
    fig = figure('WindowState','maximized');
    ax = axes();
    title(figure_name, 'FontSize', 24)
    xlabel 'x [m]'
    ylabel 'y [m]'
    zlabel 'z [m]'
    grid on
    hold on
    rotate3d on
    view([25 60]);
    xlim  ([-0.5 room.width+0.5]);
    ylim ([-0.5 room.width+0.5]);
    zlim ([0, room.height]);
    xticks(-1:1:room.width+1)
    yticks(-1:1:room.width+1)
    box on
    
    %marking the edges of the room
    room.walls = rectangle('Position',[0 0 room.width room.width], 'LineWidth',1.5,'EdgeColor','red');
    
    
    % define general time
    t = simulation.theta_rad1.time;
    t = decimate(t,10);
    
    % define trajectories for robot 1
    x_1 = simulation.x1.signals.values;
    y_1 = simulation.y1.signals.values;
    theta_1 = simulation.theta_rad1.signals.values;
    x_1 = decimate(x_1,10);
    y_1 = decimate(y_1,10);
    theta_1 = decimate(theta_1,10);
    
    % define trajectories for robot 2
    x_2 = simulation.x2.signals.values;
    y_2 = simulation.y2.signals.values;
    theta_2 = simulation.theta_rad2.signals.values;
    x_2 = decimate(x_2,10);
    y_2 = decimate(y_2,10);
    theta_2 = decimate(theta_2,10);
    
    % define trajectories for robot 3
    x_3 = simulation.x3.signals.values;
    y_3 = simulation.y3.signals.values;
    theta_3 = simulation.theta_rad3.signals.values;
    x_3 = decimate(x_3,10);
    y_3 = decimate(y_3,10);
    theta_3 = decimate(theta_3,10);
    
    % Define Rendez-vous Point only if we have regulation
    if(regulation)
        plot(simulation.x_rv, simulation.y_rv, 'color', [0.4940 0.1840 0.5560] ,'linestyle','none','marker','x','markersize',10,'DisplayName','Rendez-Vous Point', LineWidth=3)
        %vertical line to make the rendez vous point visible
        plot3(simulation.x_rv*ones(10), simulation.y_rv*ones(10), linspace(0, 0.3, 10),'--','color', [0.4940 0.1840 0.5560], 'HandleVisibility', 'off', LineWidth=1);
    end
    
    % Define Desired Trajectory only if tracking
    if (tracking)
        x_des = simulation.x_d1.signals.values;
        y_des = simulation.y_d1.signals.values;
        x_des = decimate(x_des,10);
        y_des = decimate(y_des,10);
        plot(x_des, y_des, 'color', [0.4940 0.1840 0.5560], 'DisplayName','Desired Trajectory', LineWidth=2)        
    end
    
    % plotting robots trajectory
    plot(x_1, y_1, 'color', [0.8500 0.3250 0.0980], 'DisplayName','Robot 1 Trajectory');
    plot(x_2, y_2, 'color', [0 0.4470 0.7410], 'DisplayName','Robot 2 Trajectory' );
    plot(x_3, y_3, 'color', [0.4660 0.6740 0.1880], 'DisplayName','Robot 3 Trajectory' );
    
    % Creating Legend
    lgd = legend( 'Location','southeast');
    fontsize(lgd,16,'points')
    
    % creating robots
    robot_1 = create_robot([0.8500 0.3250 0.0980], ax);
    robot_2 = create_robot([0 0.4470 0.7410], ax);
    robot_3 = create_robot([0.4660 0.6740 0.1880], ax);

    % creating video 
    obj = VideoWriter(save_name, 'MPEG-4');
    obj.Quality = 100;
    obj.FrameRate = 100;
    open(obj);
    
    for i = 1:(length(t))
        
        set(robot_1, 'Matrix', makehgtform('translate',[x_1(i) y_1(i) 0], 'zrotate', theta_1(i)));
        set(robot_2, 'Matrix', makehgtform('translate',[x_2(i) y_2(i) 0], 'zrotate', theta_2(i)));
        set(robot_3, 'Matrix', makehgtform('translate',[x_3(i) y_3(i) 0], 'zrotate', theta_3(i)));
        str = ['t = ' num2str(round(t(i),2)) ' s'];
        subtitle(str)
        drawnow
        frame = getframe(gcf);
        writeVideo(obj, frame);
    end
    
    obj.close();
    close(fig);
end

% this function creates the robots models
function tf=create_robot(color, ax)

    % Define the vertices of the triangle
    x_body = [-0.25+0.25/3, -0.25+0.25/3, 0.25+0.25/3];
    y_body = [-0.1, 0.1, 0];
    z_body = [0, 0, 0];
    % Create the transform object
    tf = hgtransform('Parent', ax);
    
    % Create the base of the prism
    base = fill3(x_body, y_body, z_body, color, 'Parent', tf);
    
    % Create the top of the prism at z=0.
    top = fill3(x_body, y_body, z_body+0.1, color, 'Parent', tf);
        
    % Create the sides of the prism
    for i = 1:3
        X = [x_body(i), x_body(i), x_body(mod(i,3)+1), x_body(mod(i,3)+1)];
        Y = [y_body(i), y_body(i), y_body(mod(i,3)+1), y_body(mod(i,3)+1)];
        Z = [0, 0.1, 0.1, 0];
        sides(i) = fill3(X, Y, Z, color, 'Parent', tf);
    end

    % adding markers to better see orientation of the robot
    x_marker1 = [0.25/3, 0.25/3, 0.25+0.25/3];
    y_marker1 = [-0.05, 0.05, 0];
    marker1 = fill3(x_marker1, y_marker1, z_body+0.1, [0 0 0], 'Parent', tf);
    x_marker2=[0.25/3, 0.25/3, 0.25+0.25/3, 0.25+0.25/3];
    y_marker2=[-0.05, -0.05, 0, 0];
    z_marker2=[0.1, 0, 0, 0.1];
    marker2 = fill3(x_marker2, y_marker2, z_marker2, [0 0 0], 'Parent', tf);
    x_marker3=[0.25/3, 0.25/3, 0.25+0.25/3, 0.25+0.25/3];
    y_marker3=[0.05, 0.05, 0, 0];
    z_marker3=[0.1, 0, 0, 0.1];
    marker3 = fill3(x_marker3, y_marker3, z_marker3, [0 0 0], 'Parent', tf);

end
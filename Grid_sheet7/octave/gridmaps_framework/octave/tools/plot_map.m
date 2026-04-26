function plot_map(map, mapBox, robPoseMapFrame, poses, laserEndPntsMapFrame, gridSize, offset, t)

    % Open a figure with a specific ID (1)
    figure(1);

    % Set up the axis and hold on for plotting
    axis(mapBox);
    hold on;

    % Plot the map
    map = map';
    imshow(ones(size(map)) - log_odds_to_prob(map))

    % Plot robot trajectory
    traj = [poses(1:t,1)';poses(1:t,2)'];
    traj = world_to_map_coordinates(traj, gridSize, offset);
    plot(traj(1,:),traj(2,:),'g')

    % Plot robot pose
    plot(robPoseMapFrame(1),robPoseMapFrame(2),'bo','markersize',5,'linewidth',4)

    % Plot laser endpoints
    plot(laserEndPntsMapFrame(1,:),laserEndPntsMapFrame(2,:),'ro','markersize',2)

    % Set figure position
    s = size(map)(1:2);
    set(gcf, "position", [50 50 s*5])
    set(gca, "position", [.05 .05 .9 .9])

    % Pause for a short duration (adjust as needed)
    pause(0.005);

    % If it's the last iteration, save the figure
    %if t == size(poses, 1)
    if t == 400
        filename = sprintf('../plots/gridmap_t_400_R0_1.png');
        print(filename, '-dpng');
    end

end


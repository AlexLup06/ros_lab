function plotCoordinateSystems(H,plotRef)
    % Input:
    % H - 4x4 homogeneous transformation matrix
    
    % Original coordinate system at origin
    origin = [0; 0; 0];
    X0 = [1; 0; 0];
    Y0 = [0; 1; 0];
    Z0 = [0; 0; 1];

    % Transformed coordinate system
    R = H(1:3,1:3);
    t = H(1:3,4);

    X1 = R(:,1);
    Y1 = R(:,2);
    Z1 = R(:,3);
    
    % Plot original frame
    % figure; hold on; 
    grid on; axis equal;
    if plotRef
        quiver3(origin(1), origin(2), origin(3), X0(1), X0(2), X0(3), 0.5, 'r', 'LineWidth', 2);
        quiver3(origin(1), origin(2), origin(3), Y0(1), Y0(2), Y0(3), 0.5, 'g', 'LineWidth', 2);
        quiver3(origin(1), origin(2), origin(3), Z0(1), Z0(2), Z0(3), 0.5, 'b', 'LineWidth', 2);
    end
    % Plot transformed frame
    quiver3(t(1), t(2), t(3), X1(1), X1(2), X1(3), 0.5, 'r--', 'LineWidth', 2);
    quiver3(t(1), t(2), t(3), Y1(1), Y1(2), Y1(3), 0.5, 'g--', 'LineWidth', 2);
    quiver3(t(1), t(2), t(3), Z1(1), Z1(2), Z1(3), 0.5, 'b--', 'LineWidth', 2);

    % Labels
    % legend({'X0','Y0','Z0','X1','Y1','Z1'}, 'Location','best');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Original and Transformed Coordinate Systems');
    view(3);
end
function PlotRobot(app, states, params)
    %{
    Plots the mobile-manipulator robot given its states and parameters.
    Input:
        states = States of the mobile manipulator robot
                 [posX,posY,rotZ,q1,q2]
                 posX = Positional X-coordinate of mobile robot
                 posY = Positional Y-coordinate of mobile robot
                 rotZ = Rotational Z-coordinate of mobile robot
                 q1 = Joint 1 angle of manipulator robot
                 q2 = Joint 2 angle of manipulator robot
        params = Parameters of the mobile manipulator robot
                 [L,W,r,w,L1,L2]
            L = Mobile robot footprint (rectangle) length
            W = Mobile robot footprint (rectangle) width
            r = Wheel radius of mobile robot
            w = Wheel width of mobile robot
            L1 = Link 1 length of manipulator robot
            L2 = Link 2 length of manipulator robot
    %}

    % Mobile robot base
    m_x = [states(1)-(params(1)/2), states(1)-(params(1)/2), states(1)+(params(1)/2), states(1)+(params(1)/2)]; % Base vertices (X-coordinates)
    m_y = [states(2)+(params(2)/2), states(2)-(params(2)/2), states(2)-(params(2)/2), states(2)+(params(2)/2)]; % Base vertices (Y-coordinates)
    R = [cos(states(3)) -sin(states(3));sin(states(3)) cos(states(3))]; % Rotation matrix
    m_v = R*[m_x(:)-mean(m_x) m_y(:)-mean(m_y)]'; % Center and rotate
    m_x = m_v(1,:)+mean(m_x); % Restore original position
    m_y = m_v(2,:)+mean(m_y); % Restore original position
    
    % Mobile robot right wheel
    w_r_c_x = states(1)+(params(2)/2)*sin(states(3)); % Wheel center X-coordinate
    w_r_c_y = states(2)-(params(2)/2)*cos(states(3)); % Wheel center Y-coordinate
    w_r_x = [w_r_c_x-params(3), w_r_c_x-params(3), w_r_c_x+params(3), w_r_c_x+params(3)]; % Wheel vertices (X-coordinates)
    w_r_y = [w_r_c_y+(params(4)/2), w_r_c_y-(params(4)/2), w_r_c_y-(params(4)/2), w_r_c_y+(params(4)/2)]; % Wheel vertices (Y-coordinates)
    R = [cos(states(3)) -sin(states(3));sin(states(3)) cos(states(3))]; % Rotation matrix
    w_r_v = R*[w_r_x(:)-mean(w_r_x) w_r_y(:)-mean(w_r_y)]'; % Center and rotate
    w_r_x = w_r_v(1,:)+mean(w_r_x); % Restore original position
    w_r_y = w_r_v(2,:)+mean(w_r_y); % Restore original position
    
    % Mobile robot left wheel
    w_l_c_x = states(1)-(params(2)/2)*sin(states(3)); % Wheel center X-coordinate
    w_l_c_y = states(2)+(params(2)/2)*cos(states(3)); % Wheel center Y-coordinate
    w_l_x = [w_l_c_x-params(3), w_l_c_x-params(3), w_l_c_x+params(3), w_l_c_x+params(3)]; % Wheel vertices (X-coordinates)
    w_l_y = [w_l_c_y+(params(4)/2), w_l_c_y-(params(4)/2), w_l_c_y-(params(4)/2), w_l_c_y+(params(4)/2)]; % Wheel vertices (Y-coordinates)
    R = [cos(states(3)) -sin(states(3));sin(states(3)) cos(states(3))]; % Rotation matrix
    w_l_v = R*[w_l_x(:)-mean(w_l_x) w_l_y(:)-mean(w_l_y)]'; % Center and rotate
    w_l_x = w_l_v(1,:)+mean(w_l_x); % Restore original position
    w_l_y = w_l_v(2,:)+mean(w_l_y); % Restore original position
    
    % Manipulator robot link 1
    l1_x = [mean(m_x), mean(m_x)+params(5)*cos(states(3)+states(4))];
    l1_y = [mean(m_y), mean(m_y)+params(5)*sin(states(3)+states(4))];
    % Manipulator robot link 2
    l2_x = [l1_x(2), l1_x(2)+params(6)*cos(states(3)+states(4)+states(5))];
    l2_y = [l1_y(2), l1_y(2)+params(6)*sin(states(3)+states(4)+states(5))];
    
    % Plot mobile robot base
    fill(app.RobotViz,m_x,m_y,'black','FaceAlpha',0.3);
    hold(app.RobotViz,'on')
    fill(app.RobotViz,w_r_x,w_r_y,'black');
    fill(app.RobotViz,w_l_x,w_l_y,'black');
    plot(app.RobotViz,l1_x,l1_y,'.-black','LineWidth',2.5,'MarkerSize',25);
    plot(app.RobotViz,l2_x,l2_y,'black','LineWidth',2.5);
    title(app.RobotViz,"Robot Visualization")
    xlabel(app.RobotViz,"X (m)")
    ylabel(app.RobotViz,"Y (m)")
    xlim(app.RobotViz,[-5,5])
    ylim(app.RobotViz,[-5,5])
    pbaspect(app.RobotViz,[1,1,1])
    hold(app.RobotViz,'off')
end
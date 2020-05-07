function makeField(wantGraphs,wantSave)
    syms x y
    v_tot = 0;

    BoB = [0.75, -2.5];
    Obstacles = [-0.25, -1; 1, -0.7; 1.41, -2]; % [x1,y1;x2,y2;...]

    % Define equation v_tot
    v_tot = addBumps(v_tot,BoB,-8);
    v_tot = addBumps(v_tot,Obstacles,[1 1 1]);
    % Substitute to get matrix V_tot
    [X,Y] = meshgrid(linspace(-1.5,2.5,50),linspace(-3.37,1,50));
    V_tot = double(subs(v_tot, {x,y}, {X,Y}));

    % Define the gradient symbolically to get dvdx and dvdy
    dvdx = diff(v_tot,x)
    dvdy = diff(v_tot,y)
    % Substitute to get matrix V_gradx and V_grady
    V_gradx = double(subs(dvdx, {x,y}, {X,Y}));
    V_grady = double(subs(dvdy, {x,y}, {X,Y}));

    if wantGraphs 
        % plot topographical
        figure
        hold on
        title("Gauntlet Map");
        xlabel('i')
        ylabel('j')
        contour(X, Y, V_tot, 40, 'ShowText', 'on');
        plot(0,0,'k*')
        plot(BoB(1),BoB(2),'ko');
        plot(Obstacles(:,1),Obstacles(:,2),'rs')
        axis equal
        grid on

        % plot 3d surface
        figure
        surf(X,Y,V_tot)
        xlabel('i')
        ylabel('j')
        zlabel('k')

        % Plot Gradient
        figure
        quiver(X,Y,V_gradx,V_grady);
        hold on
        title('\Delta V_{tot} gradient vector field')
        xlabel('i');
        ylabel('j');
        axis equal
        axis([-1.5,2.5,-3.37,1])
        plot(0,0,'k*')
        plot(BoB(1),BoB(2),'ko');
        plot(Obstacles(:,1),Obstacles(:,2),'rs')
        grid on
    end

    if wantSave
        save('field.mat','v_tot','V_tot','dvdx','dvdy','V_gradx','V_grady','X','Y','BoB','Obstacles');
    end

end
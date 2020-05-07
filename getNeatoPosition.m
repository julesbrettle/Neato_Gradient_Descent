function [posX, posY] = getNeatoPosition(msg)
    for i = 1 : length(msg.Name)
        if strcmp(msg.Name{i}, 'neato_standalone')
            posX = msg.Pose(i).Position.X;
            posY = msg.Pose(i).Position.Y;
            return
        end
    end
end
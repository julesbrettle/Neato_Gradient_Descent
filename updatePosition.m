function new_position = updatePosition(odom)
global position
pose = odom.Pose.Pose;
q = pose.Orientation;
eul = quat2eul([q.W q.X q.Y q.Z]);
currentPos = pose.Position;
position = [currentPos.X, currentPos.Y, eul(1)]; % [x, y, theta]
position(3) = wrapTo2Pi(position(3));

new_position = position;
end
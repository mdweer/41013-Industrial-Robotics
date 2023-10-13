function wayPtTr = createWaypoint(armController,desiredTr,currentScale)
    eeTr = self.robot.model.getPos();
    wayPtDist = getClockSpeed() * maxVelocity;
    approachVect = (currentScale-1)^2 * armController.velocityMax;
    estZdistance = 

    % Getting x & y Transform
    xyDist = norm(desiredTr-eeTr);
    
    wayPtTr = desiredTr * transl(0,0,distWaypoint-distance);
    % shifting z axis towards EE
    % (distance will be equal to maximum movement
    % allowed for current clock cycle)
end
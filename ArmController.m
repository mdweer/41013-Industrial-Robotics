%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef ArmController < handle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    properties (Constant)
        DEFAULT_STEPS_PER_METRE = 25;
        DEFAULT_IK_ERROR_MAX = 0.05;
        DEFAULT_VELOCITY_MAX = 0.1; % m/s
        DEFAULT_MEASURE_OF_MANIPULABILITY_MIN = 0.1;
        DEFAULT_DAMPED_LEAST_SQUARES = 0.01;
    end

    properties(Access = private)
        robot
        jointSub
        jointPub
        jointMsg


        gripperController
        gripTarget_h
        gripTargetVerts
        gripTargetTr


        stepsPerMetre
        ikErrorMax
        velocityMax
        dampedLS
        manipMin


        previousState
        previousTr
        
        currentState
        currentTr
        nextTr


        errorCode
        EStopButton
    end

    properties(Access = public)
        jointCount
        qCurrent
        gripperCloseFlag
    end

    properties(Dependent)
        baseTr
        eeTr
        jointPose
        nextState
        EStopFlag
    end

    methods
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%//Constructors///////////////////////////////////////////////////////////%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function self = ArmController( ...
                        robot,stepsPerMetre,gripperCtrl,jointStateSub, ...
                        jointStatePub,jointStateMsg,ikErrorMax,velocityMax)
            %Instantiate class object
            %Setting member attributes to default values
            self.errorCode = 0;
            self.stepsPerMetre = self.DEFAULT_STEPS_PER_METRE;
            self.ikErrorMax = self.DEFAULT_IK_ERROR_MAX;
            self.velocityMax = self.DEFAULT_VELOCITY_MAX;
            self.dampedLS = self.DEFAULT_DAMPED_LEAST_SQUARES;
            self.manipMin = self.DEFAULT_MEASURE_OF_MANIPULABILITY_MIN;
            self.gripperCloseFlag = false;
            % ArmState is initialised until constructor is complete
            self.currentState = armState.Init;
        
            if nargin > 0
                % Just robot arm passed
                self.robot = robot;
                if nargin > 1
                    % Arm and steps per metre
                    self.stepsPerMetre = stepsPerMetre;
                end
                if nargin > 2
                    self.gripperController = gripperCtrl;
                    self.gripperController.baseTr = self.eeTr;
                end
                if nargin > 5
                    % Arm, steps per metre and ros subscriber, publisher
                    % and message strings
                    self.jointSub = jointStateSub;
                    self.jointPub = jointStatePub;
                    self.jointMsg = jointStateMsg;
                    jointSub = rossubscriber(jointStateSub, ...
                                             'sensor_msgs/JointState');
                    pause(0.5);
                    
                    armJointConfig = (jointSub.LatestMessage.Position)';
                    armJointConfig = [0,armJointConfig];
                    self.robot.model.animate(deg2rad(armJointConfig));
                end
                if nargin > 6
                    % Arm, steps and inv kinematic error max
                    self.ikErrorMax = ikErrorMax;
                end
                if nargin > 7
                    % All elements passed
                    self.velocityMax = velocityMax;
                end
            else % Nothing passed!
                self.errorCode = 1;
                error('No robot passed to controller');
            end

            % Constructor is complete, arm is waiting to be assigned a pose
            self.nextState = armState.WaitingForNextTr;
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%//Getters////////////////////////////////////////////////////////////////%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function eeTr = get.eeTr(self)
            eeTr = self.robot.model.fkine(self.robot.model.getpos).T;
        end
%-------------------------------------------------------------------------%
        function qCurrent = get.qCurrent(self)
            qCurrent = self.robot.model.getpos;
        end
%-------------------------------------------------------------------------%
        function errorCode = get.errorCode(self)
            errorCode = self.errorCode;
        end
%-------------------------------------------------------------------------%
        function previousState = get.previousState(self)
            previousState = self.previousState;
        end
%-------------------------------------------------------------------------%
        function currentState = get.currentState(self)
            currentState = self.currentState;
        end
%-------------------------------------------------------------------------%
        function stepsPerMetre = get.stepsPerMetre(self)
            stepsPerMetre = self.stepsPerMetre;
        end
%-------------------------------------------------------------------------%
        function ikErrorMax = get.ikErrorMax(self)
            ikErrorMax = self.ikErrorMax;
        end
%-------------------------------------------------------------------------%
        function velMax = get.velocityMax(self)
            velMax = self.velocityMax;
        end
%-------------------------------------------------------------------------%
        function previousTr = get.previousTr(self)
            previousTr = self.previousTr;
        end
%-------------------------------------------------------------------------%
        function currentTr = get.currentTr(self)
            currentTr = self.currentTr;
        end
%-------------------------------------------------------------------------%
        function nJoints = get.jointCount(self)
            nJoints = self.robot.model.n;
        end
%-------------------------------------------------------------------------%
        function gripObj_h = get.gripTarget_h(self)
            gripObj_h = self.gripTarget_h;
        end
%-------------------------------------------------------------------------%
        function gripObjVerts = get.gripTargetVerts(self)
            gripObjVerts = self.gripTargetVerts;
        end
%-------------------------------------------------------------------------%
        function gripObjTr = get.gripTargetTr(self)
            gripObjTr = self.gripTargetTr;
        end
%-------------------------------------------------------------------------%
        function gripperClose = get.gripperCloseFlag(self)
            gripperClose = self.gripperCloseFlag;
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%//Setters////////////////////////////////////////////////////////////////%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function self = set.baseTr(self,baseTr)
                self.robot.model.base = baseTr;
                self.robot.model.animate(self.robot.model.getpos);
        end
%-------------------------------------------------------------------------%
        function self = set.previousState(self,state)
            try isenum(state)
                self.previousState = state;
            catch ME
                error("previousState must be an item from armState")
            end
        end
%-------------------------------------------------------------------------%
        function self = set.currentState(self,state)
            try isenum(state)
                self.currentState = state;
            catch ME
                error("currentState must be an item from armState")
            end
        end
%-------------------------------------------------------------------------%
        function self = set.nextState(self,state)
                self.previousState = self.currentState;
                self.currentState = state;
        end
%-------------------------------------------------------------------------%
        function self = EStop(self, stopBool)
            if stopBool
                self.previousState = self.currentState;
                self.currentState = armState.EStop;
            else
                self.currentState = self.previousState;
            end
        end
%-------------------------------------------------------------------------%
        function self = set.previousTr(self,previousTr)
            self.previousTr{end+1} = previousTr;
        end
%-------------------------------------------------------------------------%
        function self = set.currentTr(self,currentTr)
            self.currentTr = currentTr;
        end
%-------------------------------------------------------------------------%
        function self = set.nextTr(self,nextTr)
            % Adds current robot arm pose (translation and rotation) to the
            % previousTr array (doesn't save nextTr here as that
            % wouldn't account for end-effector error)
            self.previousTr = self.currentTr;
            % Assigns nextTr
            self.nextTr = nextTr;
        end
%-------------------------------------------------------------------------%
        function self = set.gripTarget_h(self,gripObj_h)
            self.gripTarget_h = gripObj_h;
        end
%-------------------------------------------------------------------------%
        function self = set.gripTargetVerts(self,gripObjVerts)
            self.gripTargetVerts = gripObjVerts;
        end
%-------------------------------------------------------------------------%
        function self = set.gripTargetTr(self,gripObjTr)
            self.gripTargetTr = gripObjTr;
        end
%-------------------------------------------------------------------------%
        function self = set.gripperCloseFlag(self,gripperClose)
            self.gripperCloseFlag = gripperClose;
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%//Functions//////////////////////////////////////////////////////////////%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function self = SetGrabTarget(self,gripObjTr,gripObj_h)

            self.gripTarget_h = gripObj_h;
            self.gripTargetVerts = [get(self.gripTarget_h,'Vertices'),...
                ones(size(get(self.gripTarget_h,'Vertices'),1),1)];
            self.gripTargetTr = gripObjTr;
        end
%-------------------------------------------------------------------------%
        function self = grabTarget(self)
            disp('self.gripObjTr');
            disp(self.gripTargetTr);
            disp('self.gripTarget_h');
            disp(self.gripTarget_h);
            disp('self.gripTargetVerts');
            disp(self.gripTargetVerts);
            gripObjTr = self.currentTr * inv(self.gripTargetTr);
            
            nextVertices = (self.gripTargetVerts(:,1:3) * ...
                            gripObjTr(1:3,1:3)') + gripObjTr(1:3,4)';
    
            set(self.gripTarget_h,'Vertices',nextVertices)
            drawnow();
        end
%-------------------------------------------------------------------------%
        function SetQ(self,q)
            self.robot.model.animate(q);
        end
%-------------------------------------------------------------------------%
    function moveJoint(self, jointNumber, jointAngle)
        self.qCurrent = self.robot.model.getpos();
        self.qCurrent(jointNumber) = jointAngle;
        self.SetQ(self.qCurrent);
    end
%-------------------------------------------------------------------------%
        function [jointPose,self] = GetJointPose(self,jointNumber)
            %Get the pose transform for the specified joint
            % (returns empty if outside of joint bounds)
            if 0 > jointNumber || jointNumber > self.robot.model.n
                %OUT OF BOUNDS
                self.errorCode = 2;
                error('Joint out of bounds');
            else %SELECTION WITHIN JOINT NUMBER
                    jointTr = self.robot.model.base.T;
                    currentq = self.qCurrent;
                    jointArray = zeros(3,jointNumber);
                    isPris = self.robot.model.isprismatic;

                    for i = 1:jointNumber
                        if isPris(i)
                            % disp('prismatic joint at:')
                            % disp(i)
                            theta = self.robot.model.links(i).offset + pi;
                            d = self.robot.model.links(i).d + currentq(i);
                            a = self.robot.model.links(i).a;
                            alpha = self.robot.model.links(i).alpha;
                        else
                            theta = self.robot.model.links(i).offset + ...
                                    currentq(i);
                            d = self.robot.model.links(i).d;
                            a = self.robot.model.links(i).a;
                            alpha = self.robot.model.links(i).alpha;
    
                            if isempty(theta)
                                theta = 0;
                            end
                            if isempty(alpha)
                                alpha = 0;
                            end
                        end
    
                        Tr = eye(4) * ...
                            trotz(rad2deg(theta), 'deg') * ...
                            transl(a,0,d) * ...
                            trotx(rad2deg(alpha), 'deg');

                        jointTr = jointTr * Tr;
                        jointArray(1,i) = jointTr(1,4);
                        jointArray(2,i) = jointTr(2,4);
                        jointArray(3,i) = jointTr(3,4);
                    end
                jointPose = jointTr;

                plot3(jointArray(1,:), ...
                      jointArray(2,:), ...
                      jointArray(3,:),'*-w');
            end
        end
%-------------------------------------------------------------------------%
function [goalReached, err] = moveToNextPoint(self, desiredTr, qPath)
    delay = GetClockSpeed() / length(qPath);
    goalReached = true;
    err = 0;
    stepCurrent = 1;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if ~isempty(self.jointSub) && ...
        ~isempty(self.jointPub) && ...
        ~isempty(self.jointMsg)
        %Set target joint state
        jointTarget = qPath(end,2:5);
        
        [targetJointTrajPub,targetJointTrajMsg] = ...
        rospublisher(self.jointPub);
        trajectoryPoint = rosmessage(self.jointMsg);
        trajectoryPoint.Positions = jointTarget;
        targetJointTrajMsg.Points = trajectoryPoint;
        
        send(targetJointTrajPub,targetJointTrajMsg);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    while length(qPath(:,1)) > stepCurrent
        if self.currentState == armState.EStop
            disp('OPERATIONS CANNOT CONTINUE UNTIL ESTOP RELEASED');
            pause(1); % pause for a moment before checking again
        else
            self.currentTr = self.eeTr;
            self.robot.model.animate(qPath(stepCurrent,:));

            % If ArmController contains a gripper controller handle
            % Move baseTr of gripper based on End Effector position
            if ~isempty(self.gripperController)
                self.gripperController.baseTr = self.currentTr;
            end

            if self.gripperCloseFlag
                grabTarget();
            end
            drawnow();
            
            % Plot End-Effector position during steps
            plot3(self.currentTr(1,4), self.currentTr(2,4), ...
                self.currentTr(3,4), '.-b');
            pause(delay);
            stepCurrent = stepCurrent + 1;
        end
    end

    if self.currentState ~= armState.EStop
        PdesTr = atan(desiredTr(3,2) / desiredTr(3,3));
        RdesTr = atan(-desiredTr(3,1) / sqrt(desiredTr(3,2)^2 + ...
            desiredTr(3,3)^2));
        YdesTr = atan(desiredTr(2,1) / desiredTr(1,1));

        eeTr = self.currentTr;
        PeeTr = atan(eeTr(3,2) / eeTr(3,3));
        ReeTr = atan(-eeTr(3,1) / sqrt(eeTr(3,2)^2 + eeTr(3,3)^2));
        YeeTr = atan(eeTr(2,1) / eeTr(1,1));

        err(1) = norm(transl(desiredTr) - transl(self.currentTr));
        err(2) = norm(PdesTr - PeeTr);
        err(3) = norm(RdesTr - ReeTr);
        err(4) = norm(YdesTr - YeeTr);

        for i = 1:length(err)
            if err(i) > self.ikErrorMax
                plot3(self.currentTr(1,4), ...
                      self.currentTr(2,4), ...
                      self.currentTr(3,4), 'x:r');
                goalReached = false;
            end
        end

        if goalReached
            plot3(self.currentTr(1,4), ...
                  self.currentTr(2,4), ...
                  self.currentTr(3,4), 'o:g');
        end
    end
end

%-------------------------------------------------------------------------%
        function qPath = genPath(self,desiredTr,profile,coordinateFrame)
            %Calculating distance and required steps based on current
            %Clock Rate
            eeTr = self.robot.model.fkine(self.robot.model.getpos);
            displ = norm(transl(desiredTr) - transl(eeTr));
            steps = round(displ * self.stepsPerMetre);

            q0 = self.robot.model.getpos;
            q1 = self.robot.model.ikcon(desiredTr,q0);

            switch profile
                case 'Quin'
                    qPath = jtraj(q0,q1,steps);
                case 'Trap'
                    s = lspb(0,1,steps); %interp scalar
                    qPath = nan(steps,self.jointCount);
                    for i = 1:steps
                        qPath(i,:) = (1-s(i))*q0 + s(i)*q1;
                    end
                case 'RMRC'
                    qPath(1,:) = q0;
                    dT = self.velocityMax * GetClockSpeed();
                    eeTr = self.robot.model.fkine( ...
                           self.robot.model.getpos).T;
                    % disp(desiredTr)
        
                    manip = zeros(1,steps);
                    err = nan(self.jointCount,steps);

                    PdesTr = atan(desiredTr(3,2)/desiredTr(3,3));
                    RdesTr = atan(-desiredTr(3,1)/ ...
                                sqrt(desiredTr(3,2)^2 + desiredTr(3,3)^2));
                    YdesTr = atan(desiredTr(2,1)/desiredTr(1,1));

                    PeeTr = atan(eeTr(3,2)/eeTr(3,3));
                    ReeTr = atan(-eeTr(3,1)/ ...
                                 sqrt(eeTr(3,2)^2 + eeTr(3,3)^2));
                    YeeTr = atan(eeTr(2,1)/eeTr(1,1));

                    if (desiredTr(3,2) == 0) && (desiredTr(3,3) == 0)
                        PdesTr = 0;
                    end
                    if (desiredTr(2,1) == 0) && (desiredTr(1,1) == 0)
                        YdesTr = 0;
                    end
                    if (eeTr(3,2) == 0) && (eeTr(3,3) == 0)
                        PeeTr = 0;
                    end
                    if (eeTr(2,1) == 0) && (eeTr(1,1) == 0)
                        YeeTr = 0;
                    end

                    x = linspace(eeTr(1,4),desiredTr(1,4),steps);
                    y = linspace(eeTr(2,4),desiredTr(2,4),steps);
                    z = linspace(eeTr(3,4),desiredTr(3,4),steps);
                    P = linspace(PeeTr,PdesTr,steps);
                    R = linspace(ReeTr,RdesTr,steps);
                    Y = linspace(YeeTr,YdesTr,steps);
        
                    X = [x',y',z',P',R',Y']';
                    X = X(1:self.jointCount,:);
        
                    for i = 1:steps-1
                        xd = (X(:,i+1) - X(:,i))/dT;
                        switch coordinateFrame
                            case 'Global'
                            J = self.robot.model.jacob0(qPath(i,:));
                            case 'Local'
                            J = self.robot.model.jacobe(qPath(i,:));
                        end
                        J = J(1:self.jointCount,:);
                        manip(:,i)= sqrt(det(J*J'));
        
                        % Checking if close to singularity
                        if manip(:,i) < self.manipMin
                            qd = inv(J'*J + self.dampedLS* ...
                                eye(length(J(:,1))))*J' * xd;
                            %qd = pinv(J) * xd;
                        else
                            qd = inv(J) * xd;
                        end
                        
                        errVal = xd - J*qd;
                        err(:,i) = errVal;
                        qPath(i+1,:) = qPath(i,:) + dT * qd';
                    end
            end

            if self.currentState ~= armState.EStop
                self.currentState = armState.AssignedNextTr;
            end
        end
%-------------------------------------------------------------------------%
        function calcWorkEnvelope(self)
            qlim = self.robot.model.qlim;
            stepRads = deg2rad(20);
            pointCloudeSize = prod(floor((qlim(1:6,2) - ...
                                   qlim(1:6,1))/stepRads + 1));
            pointCloud = zeros(pointCloudeSize,3);
            counter = 1;
            q = zeros(1,self.jointCount);

            for q1 = qlim(1,1):stepRads:qlim(1,2)
                for q2 = qlim(2,1):stepRads:qlim(2,2)
                    for q3 = qlim(3,1):stepRads:qlim(3,2)
                        q(1) = q1;
                        q(2) = q2;
                        q(3) = q3;
                        tr = self.robot.model.fkine(q).T;
                        pointCloud(counter,:) = tr(1:3,4)';
                        counter = counter + 1;
                    end
                end
            end
            centre = mean(pointCloud);
            distances = sqrt(sum((pointCloud - centre).^2, 2));

            distx = max(sqrt(sum((pointCloud(:,1) - centre(:,1)).^2, 2)));
            disty = max(sqrt(sum((pointCloud(:,2) - centre(:,2)).^2, 2)));
            distz = max(sqrt(sum((pointCloud(:,3) - centre(:,3)).^2, 2)));
            
            plotRadius = max(distances);
            plotVolume = (4/3) * pi * plotRadius^3;
        end
%-------------------------------------------------------------------------%
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
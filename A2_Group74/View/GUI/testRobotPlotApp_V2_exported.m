classdef testRobotPlotApp_V2_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        figure1                   matlab.ui.Figure
        Joint1SpinnerLabel        matlab.ui.control.Label
        Joint1Spinner             matlab.ui.control.Spinner
        Joint2SpinnerLabel        matlab.ui.control.Label
        Joint2Spinner             matlab.ui.control.Spinner
        Joint3SpinnerLabel        matlab.ui.control.Label
        Joint3Spinner             matlab.ui.control.Spinner
        Joint4SpinnerLabel        matlab.ui.control.Label
        Joint4Spinner             matlab.ui.control.Spinner
        Joint5SpinnerLabel        matlab.ui.control.Label
        Joint5Spinner             matlab.ui.control.Spinner
        Joint6SpinnerLabel        matlab.ui.control.Label
        Joint6Spinner             matlab.ui.control.Spinner
        EstopButton               matlab.ui.control.StateButton
        ResumeButton              matlab.ui.control.Button
    end

    properties (Access = private)
        armControllerObj   % Instance of ArmController class
        armControllerObj1
        armState armState = armState.Stopped; % Default state
    end

    methods (Access = public)
        function setArmControllerObj(app, armController)
            app.armControllerObj = armController;
        end

        function setArmControllerObj1(app, armController)
            app.armControllerObj1 = armController;
        end
    end

    % Callbacks that handle component events
    methods (Access = private)
        function init(app, varargin)
            disp('App Initialized');
        end

        function moveTM5Joint(app, jointNumber, jointAngle)
            if app.armState == armState.Operating || app.armState == armState.ReadyToResume
                app.armControllerObj1.moveJoint(jointNumber, jointAngle);
            end
        end

        function moveDobotJoint(app, jointNumber, jointAngle)
            if app.armState == armState.Operating || app.armState == armState.ReadyToResume
                app.armControllerObj.moveJoint(jointNumber, jointAngle);
            end
        end



        function EstopButtonValueChanged(app, event)
            if app.armState == armState.EStopped
                app.armState = armState.ReadyToResume;
                app.EstopButton.Text = 'Ready to Resume';
            else
                app.armState = armState.EStopped;
                app.armControllerObj.EStop(true);
                app.armControllerObj1.EStop(true);
                app.EstopButton.Text = 'EStopped';
            end
        end
        
        function ResumeButtonValueChanged(app, event)
            if app.armState == armState.ReadyToResume
                app.armState = armState.Operating;
                app.armControllerObj.EStop(false);
                app.armControllerObj1.EStop(false);
                app.EstopButton.Text = 'EStop';
            end
        end
    end

    % Component initialization
    methods (Access = private)
        function createComponents(app)
            app.figure1 = uifigure('Visible', 'off');
            app.figure1.Position = [100 100 400 300];
            app.figure1.Name = 'TestRobotPlot_V2';

            % Create EstopButton
            app.EstopButton = uibutton(app.figure1, 'state');
            app.EstopButton.Text = 'Estop';
            app.EstopButton.BackgroundColor = [0.9804 0.2824 0.2824];
            app.EstopButton.FontWeight = 'bold';
            app.EstopButton.Position = [10 70 40 22];
            app.EstopButton.ValueChangedFcn = createCallbackFcn(app, @EstopButtonValueChanged, true);

            % Create ResumeButton
            app.ResumeButton = uibutton(app.figure1, 'push');
            app.ResumeButton.Text = 'Resume';
            app.ResumeButton.Position = [60 70 60 22];
            app.ResumeButton.BackgroundColor = [0.3922 0.8314 0.0745];  % Green color
            app.ResumeButton.ButtonPushedFcn = createCallbackFcn(app, @ResumeButtonValueChanged, true);
            app.ResumeButton.Visible = true;  % Make the Resume button visible at the beginning



            app.createSpinner('Joint 1', 1, [-pi, pi], [10, 250, 40, 22], [65, 250, 100, 22]);
            app.createSpinner('Joint 2', 2, [-pi, pi], [10, 220, 40, 22], [65, 220, 100, 22]);
            app.createSpinner('Joint 3', 3, [-pi, pi], [10, 190, 40, 22], [65, 190, 100, 22]);
            app.createSpinner('Joint 4', 4, [-pi, pi], [10, 160, 40, 22], [65, 160, 100, 22]);
            app.createSpinner('Joint 5', 5, [-pi, pi], [10, 130, 40, 22], [65, 130, 100, 22]);
            app.createSpinner('Joint 6', 6, [-pi, pi], [10, 100, 40, 22], [65, 100, 100, 22]);
          
            app.createDobotSpinner('Dobot Joint 1', 1, [-pi, 1], [400, 250, 100, 22], [510, 250, 60, 22]);
            app.createDobotSpinner('Dobot Joint 2', 2, [-pi, pi], [400, 220, 100, 22], [510, 220, 60, 22]);
            app.createDobotSpinner('Dobot Joint 3', 3, [-pi, pi], [400, 190, 100, 22], [510, 190, 60, 22]);
            app.createDobotSpinner('Dobot Joint 4', 4, [-pi, pi], [400, 160, 100, 22], [510, 160, 60, 22]);
            app.createDobotSpinner('Dobot Joint 5', 5, [-pi, pi], [400, 130, 100, 22], [510, 130, 60, 22]);

            app.figure1.Visible = 'on';
        end
          function createSpinner(app, label, jointNumber, limits, labelPosition, spinnerPosition)
            % Create Label
            jointLabel = uilabel(app.figure1);
            jointLabel.Position = labelPosition;
            jointLabel.Text = label;

            % Create Spinner
            jointSpinner = uispinner(app.figure1);
            jointSpinner.Position = spinnerPosition;
            jointSpinner.Limits = limits;
            jointSpinner.ValueChangedFcn = @(s, e) moveTM5Joint(app, jointNumber, jointSpinner.Value);
          end
               function createDobotSpinner(app, label, jointNumber, limits, labelPosition, spinnerPosition)
            % Create Label
            jointLabel = uilabel(app.figure1);
            jointLabel.Position = labelPosition;
            jointLabel.Text = label;

            % Create Spinner
            jointSpinner = uispinner(app.figure1);
            jointSpinner.Position = spinnerPosition;
            jointSpinner.Limits = limits;
            jointSpinner.ValueChangedFcn = @(s, e) moveDobotJoint(app, jointNumber, jointSpinner.Value);
        end
    end

    % App creation and deletion
    methods (Access = public)
        function app = testRobotPlotApp_V2_exported(varargin)
            runningApp = getRunningApp(app);
            if isempty(runningApp)
                createComponents(app)
                registerApp(app, app.figure1)
                % app.armControllerObj = ArmController(LinearDM);
                % app.armControllerObj1 = ArmController(TM5);
                runStartupFcn(app, @(app)init(app, varargin{:}))
            else
                figure(runningApp.figure1)
                app = runningApp;
            end
            if nargout == 0
                clear app
            end
        end

        function delete(app)
            delete(app.figure1)
        end
    end
end



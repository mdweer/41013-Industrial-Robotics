classdef testRobotPlotApp_V2_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        figure1                   matlab.ui.Figure
        MakeLoadedDobotMoveButton matlab.ui.control.Button
        Joint1Spinner             matlab.ui.control.Spinner
        Joint2Spinner             matlab.ui.control.Spinner
        Joint1SpinnerLabel        matlab.ui.control.Label
        Joint2SpinnerLabel        matlab.ui.control.Label
        Joint3Spinner             matlab.ui.control.Spinner
        Joint3SpinnerLabel        matlab.ui.control.Label
        Joint4Spinner             matlab.ui.control.Spinner
        Joint4SpinnerLabel        matlab.ui.control.Label
        Joint5Spinner             matlab.ui.control.Spinner
        Joint5SpinnerLabel        matlab.ui.control.Label
        TM5Joint1Spinner             matlab.ui.control.Spinner
        TM5Joint2Spinner             matlab.ui.control.Spinner
        TM5Joint3Spinner             matlab.ui.control.Spinner
        TM5Joint4Spinner             matlab.ui.control.Spinner
        TM5Joint5Spinner             matlab.ui.control.Spinner
        TM5Joint6Spinner             matlab.ui.control.Spinner
        TM5Joint1SpinnerLabel        matlab.ui.control.Label
        TM5Joint2SpinnerLabel        matlab.ui.control.Label
        TM5Joint3SpinnerLabel        matlab.ui.control.Label
        TM5Joint4SpinnerLabel        matlab.ui.control.Label
        TM5Joint5SpinnerLabel        matlab.ui.control.Label
        TM5Joint6SpinnerLabel        matlab.ui.control.Label
        MakeLoadedTM5MoveButton      matlab.ui.control.Button
        
        EstopButton               matlab.ui.control.StateButton
        Switch                    matlab.ui.control.Switch
        plot                      matlab.ui.control.UIAxes
    end

    properties (Access = private)
        modelDobotMagician % Where I will put the loaded Dobot Magician
        qDobotHome = [0, pi/4, pi/4, 0, 0]; % Home position for Dobot Magician
        modelTM5            % TM5 Model
        qTM5Home = zeros(1,6); % Home position for TM5

    
    
    end

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function init(app, varargin)
            % Code to run once GUI is loaded successfully
            ax = gca;
            axis([-2 2, -2 2, 0 2]);
            app.modelDobotMagician = DobotMagician();
            app.modelDobotMagician.model.base = transl([0, -0.5, 0]);
            app.modelDobotMagician.model.animate(app.qDobotHome);
            drawnow();

            app.modelTM5 = TM5();
            app.modelTM5.model.base = transl([0, 0.5, 0]); % Adjust the base position as required
            app.modelTM5.model.animate(app.qTM5Home);
            drawnow();
            for i = size(ax.Children,1):-1:1
                if strcmp(ax.Children(i).Type,'surface')
                    delete(ax.Children(i));
                end
            end

            % Set up Joint1Spinner
            app.Joint1Spinner.Limits = app.modelDobotMagician.model.qlim(1,:);
            app.Joint1Spinner.Value = app.qDobotHome(1);
            app.Joint1Spinner.ValueChangedFcn = createCallbackFcn(app, @updateRobot, true);
            
            % Set up Joint2Spinner
            app.Joint2Spinner.Limits = app.modelDobotMagician.model.qlim(2,:);
            app.Joint2Spinner.Value = app.qDobotHome(2);
            app.Joint2Spinner.ValueChangedFcn = createCallbackFcn(app, @updateRobot, true);

  
            % Set up Joint2Spinner
            app.Joint3Spinner.Limits = app.modelDobotMagician.model.qlim(3,:);
            app.Joint3Spinner.Value = app.qDobotHome(3);
            app.Joint3Spinner.ValueChangedFcn = createCallbackFcn(app, @updateRobot, true);

            % Set up Joint2Spinner
            app.Joint4Spinner.Limits = app.modelDobotMagician.model.qlim(4,:);
            app.Joint4Spinner.Value = app.qDobotHome(4);
            app.Joint4Spinner.ValueChangedFcn = createCallbackFcn(app, @updateRobot, true);
         
            % Set up Joint2Spinner
            app.Joint5Spinner.Limits = app.modelDobotMagician.model.qlim(5,:);
            app.Joint5Spinner.Value = app.qDobotHome(5);
            app.Joint5Spinner.ValueChangedFcn = createCallbackFcn(app, @updateRobot, true);
 

            app.TM5Joint1Spinner.Limits = app.modelTM5.model.qlim(1,:);
            app.TM5Joint1Spinner.Value = app.qTM5Home(1);   
            app.TM5Joint1Spinner.ValueChangedFcn = createCallbackFcn(app, @updateRobot, true);
            
            app.TM5Joint2Spinner.Limits = app.modelTM5.model.qlim(2,:);
            app.TM5Joint2Spinner.Value = app.qTM5Home(2);
            app.TM5Joint2Spinner.ValueChangedFcn = createCallbackFcn(app, @updateRobot, true);

            app.TM5Joint3Spinner.Limits = app.modelTM5.model.qlim(3,:);
            app.TM5Joint3Spinner.Value = app.qTM5Home(3);
            app.TM5Joint3Spinner.ValueChangedFcn = createCallbackFcn(app, @updateRobot, true);
            
            app.TM5Joint4Spinner.Limits = app.modelTM5.model.qlim(4,:);
            app.TM5Joint4Spinner.Value = app.qTM5Home(4);
            app.TM5Joint4Spinner.ValueChangedFcn = createCallbackFcn(app, @updateRobot, true);

            app.TM5Joint5Spinner.Limits = app.modelTM5.model.qlim(5,:);
            app.TM5Joint5Spinner.Value = app.qTM5Home(5);      
            app.TM5Joint5Spinner.ValueChangedFcn = createCallbackFcn(app, @updateRobot, true);

            app.TM5Joint6Spinner.Limits = app.modelTM5.model.qlim(6,:);
            app.TM5Joint6Spinner.Value = app.qTM5Home(6);
            app.TM5Joint6Spinner.ValueChangedFcn = createCallbackFcn(app, @updateRobot, true);

            app.MakeLoadedTM5MoveButton.Enable = true;
            app.MakeLoadedDobotMoveButton.Enable = true;
        end

        function MakeLoadedDobotMoveButtonPushed(app, event)
            qMatrix = jtraj(app.qDobotHome, app.modelDobotMagician.model.qlim(:,1)', 20);
            for robotStepIndex = 1:size(qMatrix, 1)
                app.modelDobotMagician.model.animate(qMatrix(robotStepIndex, :));
                drawnow;
            end
        end



        function MakeLoadedTM5MoveButtonPushed(app, event)
            qMatrix = jtraj(app.qTM5Home, app.modelTM5.model.qlim(:,1)', 20);
            for robotStepIndex = 1:size(qMatrix, 1)
                app.modelTM5.model.animate(qMatrix(robotStepIndex, :));
                drawnow;
            end
        end

       
        function updateRobot(app, ~)
            % Get current joint angles
            qCurrent = app.modelDobotMagician.model.getpos();
            qCurrents = app.modelTM5.model.getpos();

            % Update joint angles based on spinners
            qCurrent(1) = app.Joint1Spinner.Value;
            qCurrent(2) = app.Joint2Spinner.Value;
            qCurrent(3) = app.Joint3Spinner.Value;
            qCurrent(4) = app.Joint4Spinner.Value;
            qCurrent(5) = app.Joint5Spinner.Value;
            for i = 1:6
                qCurrents(i) = app.(['TM5Joint', num2str(i), 'Spinner']).Value;
            end
            % Animate robot
            app.modelDobotMagician.model.animate(qCurrent);
            app.modelTM5.model.animate(qCurrents);

        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create figure1 and hide until all components are created
            app.figure1 = uifigure('Visible', 'off');
            app.figure1.Position = [100 100 812 560];
            app.figure1.Name = 'TestRobotPlot_V2';
            app.figure1.HandleVisibility = 'callback';
            app.figure1.Tag = 'figure1';

            app.MakeLoadedDobotMoveButton = uibutton(app.figure1, 'push');
            app.MakeLoadedDobotMoveButton.ButtonPushedFcn = createCallbackFcn(app, @MakeLoadedDobotMoveButtonPushed, true);
            app.MakeLoadedDobotMoveButton.Position = [16 475 155 25];
            app.MakeLoadedDobotMoveButton.Text = 'Dobot';
            % Create Joint1SpinnerLabel
            app.Joint1SpinnerLabel = uilabel(app.figure1);
            app.Joint1SpinnerLabel.HorizontalAlignment = 'right';
            app.Joint1SpinnerLabel.Position = [10 427 40 22];
            app.Joint1SpinnerLabel.Text = 'Joint 1';
        
            % Create Joint1Spinner
            app.Joint1Spinner = uispinner(app.figure1);
            app.Joint1Spinner.Position = [65 428 100 22];
        
            % Create Joint2SpinnerLabel
            app.Joint2SpinnerLabel = uilabel(app.figure1);
            app.Joint2SpinnerLabel.HorizontalAlignment = 'right';
            app.Joint2SpinnerLabel.Position = [10 388 40 22];
            app.Joint2SpinnerLabel.Text = 'Joint 2';
        
            % Create Joint2Spinner
            app.Joint2Spinner = uispinner(app.figure1);
            app.Joint2Spinner.Position = [65 388 100 22];
        
            % Create Joint3SpinnerLabel
            app.Joint3SpinnerLabel = uilabel(app.figure1);
            app.Joint3SpinnerLabel.HorizontalAlignment = 'right';
            app.Joint3SpinnerLabel.Position = [10 343 40 22];
            app.Joint3SpinnerLabel.Text = 'Joint 3';
        
            % Create Joint3Spinner
            app.Joint3Spinner = uispinner(app.figure1);
            app.Joint3Spinner.Position = [65 343 100 22];
        
            % Create Joint4SpinnerLabel
            app.Joint4SpinnerLabel = uilabel(app.figure1);
            app.Joint4SpinnerLabel.HorizontalAlignment = 'right';
            app.Joint4SpinnerLabel.Position = [10 295 40 22];
            app.Joint4SpinnerLabel.Text = 'Joint 4';
        
            % Create Joint4Spinner
            app.Joint4Spinner = uispinner(app.figure1);
            app.Joint4Spinner.Position = [65 295 100 22];
        
            % Create Joint5SpinnerLabel
            app.Joint5SpinnerLabel = uilabel(app.figure1);
            app.Joint5SpinnerLabel.HorizontalAlignment = 'right';
            app.Joint5SpinnerLabel.Position = [10 248 40 22];
            app.Joint5SpinnerLabel.Text = 'Joint 5';

            % Create Joint5Spinner
            app.Joint5Spinner = uispinner(app.figure1);
            app.Joint5Spinner.Position = [65 248 100 22];



            % Create Joint1Spinner_6Label
            app.TM5Joint1SpinnerLabel = uilabel(app.figure1);
            app.TM5Joint1SpinnerLabel.HorizontalAlignment = 'right';
            app.TM5Joint1SpinnerLabel.Position = [171 428 40 22];
            app.TM5Joint1SpinnerLabel.Text = 'Joint 1';

            % Create Joint1Spinner_6
            app.TM5Joint1Spinner = uispinner(app.figure1);
            app.TM5Joint1Spinner.Position = [226 428 100 22];

            % Create Joint2Spinner_2Label
            app.TM5Joint2SpinnerLabel = uilabel(app.figure1);
            app.TM5Joint2SpinnerLabel.HorizontalAlignment = 'right';
            app.TM5Joint2SpinnerLabel.Position = [171 388 40 22];
            app.TM5Joint2SpinnerLabel.Text = 'Joint 2';

            % Create Joint2Spinner_2
            app.TM5Joint2Spinner = uispinner(app.figure1);
            app.TM5Joint2Spinner.Position = [226 388 100 22];

            % Create Joint3Spinner_2Label
            app.TM5Joint3SpinnerLabel = uilabel(app.figure1);
            app.TM5Joint3SpinnerLabel.HorizontalAlignment = 'right';
            app.TM5Joint3SpinnerLabel.Position = [169 343 40 22];
            app.TM5Joint3SpinnerLabel.Text = 'Joint 3';

            % Create Joint3Spinner_2
            app.TM5Joint3Spinner = uispinner(app.figure1);
            app.TM5Joint3Spinner.Position = [224 343 100 22];

            % Create Joint4Spinner_2Label
            app.TM5Joint4SpinnerLabel = uilabel(app.figure1);
            app.TM5Joint4SpinnerLabel.HorizontalAlignment = 'right';
            app.TM5Joint4SpinnerLabel.Position = [169 295 40 22];
            app.TM5Joint4SpinnerLabel.Text = 'Joint 4';

            % Create Joint4Spinner_2
            app.TM5Joint4Spinner = uispinner(app.figure1);
            app.TM5Joint4Spinner.Position = [224 295 100 22];

            % Create Joint5Spinner_2Label
            app.TM5Joint5SpinnerLabel = uilabel(app.figure1);
            app.TM5Joint5SpinnerLabel.HorizontalAlignment = 'right';
            app.TM5Joint5SpinnerLabel.Position = [171 248 40 22];
            app.TM5Joint5SpinnerLabel.Text = 'Joint 5';

            % Create Joint5Spinner_2
            app.TM5Joint5Spinner = uispinner(app.figure1);
            app.TM5Joint5Spinner.Position = [226 248 100 22];

            % Create Joint6SpinnerLabel
            app.TM5Joint6SpinnerLabel = uilabel(app.figure1);
            app.TM5Joint6SpinnerLabel.HorizontalAlignment = 'right';
            app.TM5Joint6SpinnerLabel.Position = [171 200 40 22];
            app.TM5Joint6SpinnerLabel.Text = 'Joint 6';

            % Create Joint6Spinner
            app.TM5Joint6Spinner = uispinner(app.figure1);
            app.TM5Joint6Spinner.Position = [226 200 100 22];

            app.MakeLoadedTM5MoveButton = uibutton(app.figure1, 'push');
            app.MakeLoadedTM5MoveButton.ButtonPushedFcn = createCallbackFcn(app, @MakeLoadedTM5MoveButtonPushed, true);
            app.MakeLoadedTM5MoveButton.Position = [192 476 148 23];
            app.MakeLoadedTM5MoveButton.Text = 'TM5';

            % Create Switch
            app.Switch = uiswitch(app.figure1, 'slider');
            app.Switch.Items = {'Start', 'Pause'};
            app.Switch.FontColor = [0.149 0.149 0.149];
            app.Switch.Position = [38 154 45 20];
            app.Switch.Value = 'Start';

            % Create EstopButton
            app.EstopButton = uibutton(app.figure1, 'state');
            app.EstopButton.Text = 'Estop';
            app.EstopButton.BackgroundColor = [0.9804 0.2824 0.2824];
            app.EstopButton.FontWeight = 'bold';
            app.EstopButton.Position = [16 101 100 23];


            
            % Create plot
            app.plot = uiaxes(app.figure1);
            xlabel(app.plot, 'X');
            ylabel(app.plot, 'Y');
            app.plot.FontSize = 12;
            app.plot.NextPlot = 'replace';
            app.plot.Tag = 'axes1';
            app.plot.Position = [297 101 445 360];
            


            % Show the figure after all components are created
            app.figure1.Visible = 'on';
        end
    end
    
    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = testRobotPlotApp_V2_exported(varargin)

            runningApp = getRunningApp(app);

            % Check for running singleton app
            if isempty(runningApp)

                % Create UIFigure and components
                createComponents(app)

                % Register the app with App Designer
                registerApp(app, app.figure1)

                % Execute the startup function
                runStartupFcn(app, @(app)init(app, varargin{:}))
            else

                % Focus the running singleton app
                figure(runningApp.figure1)

                app = runningApp;
            end

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.figure1)
        end
    end
end




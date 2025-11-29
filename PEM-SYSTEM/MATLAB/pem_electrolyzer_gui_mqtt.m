function pem_electrolyzer_gui_mqtt()
% PEM ELECTROLYZER CONTROL SYSTEM - Industrial Dark Theme
% DUAL POWER SOURCE: GRID + PV + MQTT 3-WAY COMMUNICATION

    % Industrial color scheme
    colors.industrial_dark = [0.08 0.11 0.15];
    colors.industrial_panel = [0.12 0.16 0.22];
    colors.industrial_border = [0.20 0.25 0.35];
    colors.accent_blue = [0 0.7 1];
    colors.accent_green = [0.2 0.8 0.4];
    colors.accent_orange = [1 0.6 0];
    colors.accent_red = [1 0.3 0.3];
    colors.text_light = [0.9 0.95 1];
    colors.text_dim = [0.6 0.7 0.8];
    
    % Create industrial dark theme figure
    screen_size = get(0, 'ScreenSize');
    fig = uifigure('Name', '🏭 PEM Electrolyzer - Industrial Control', ...
                  'Position', [100 100 1400 800], ...
                  'Color', colors.industrial_dark, ...
                  'Resize', 'on');
    
    % === INDUSTRIAL HEADER ===
    headerPanel = uipanel(fig, 'Position', [20 750 1360 40], ...
                         'BackgroundColor', [0.05 0.08 0.12], ...
                         'BorderColor', colors.accent_blue, ...
                         'BorderWidth', 1);
    
    uilabel(headerPanel, 'Position', [20 5 400 30], ...
           'Text', '🏭 PEM ELECTROLYZER CONTROL SYSTEM', ...
           'FontSize', 18, 'FontWeight', 'bold', ...
           'FontColor', colors.text_light);
    
    % Connection status
    uilabel(headerPanel, 'Position', [450 5 300 30], ...
           'Text', '🌐 MQTT Broker: broker.hivemq.com:1883', ...
           'FontSize', 11, 'FontColor', colors.text_dim);
    
    statusPanel = uipanel(headerPanel, 'Position', [800 5 550 30], ...
                         'BackgroundColor', [0.1 0.15 0.2], ...
                         'BorderColor', colors.industrial_border);
    
    mqttStatus = uilamp(statusPanel, 'Position', [10 5 20 20]);
    uilabel(statusPanel, 'Position', [35 5 80 20], 'Text', 'MQTT', 'FontColor', colors.text_light);
    
    matlabStatus = uilamp(statusPanel, 'Position', [130 5 20 20]);
    uilabel(statusPanel, 'Position', [155 5 80 20], 'Text', 'MATLAB', 'FontColor', colors.text_light);
    
    arduinoStatus = uilamp(statusPanel, 'Position', [250 5 20 20]);
    uilabel(statusPanel, 'Position', [275 5 80 20], 'Text', 'ARDUINO', 'FontColor', colors.text_light);
    
    webStatus = uilamp(statusPanel, 'Position', [370 5 20 20]);
    uilabel(statusPanel, 'Position', [395 5 60 20], 'Text', 'WEB', 'FontColor', colors.text_light);
    
    % === MAIN CONTROL PANEL ===
    controlPanel = uipanel(fig, 'Position', [20 500 400 240], ...
                          'Title', '🎛️ CONTROL PANEL', ...
                          'BackgroundColor', colors.industrial_panel, ...
                          'ForegroundColor', colors.text_light, ...
                          'BorderColor', colors.industrial_border, ...
                          'FontWeight', 'bold');
    
    % Control buttons
    startBtn = uibutton(controlPanel, 'push', ...
        'Position', [30 180 100 35], ...
        'Text', '▶️ START', ...
        'FontColor', [1 1 1], ...
        'FontWeight', 'bold', ...
        'BackgroundColor', colors.accent_green, ...
        'ButtonPushedFcn', @(btn,event) startElectrolyzer());
    
    stopBtn = uibutton(controlPanel, 'push', ...
        'Position', [140 180 100 35], ...
        'Text', '⏹️ STOP', ...
        'FontColor', [1 1 1], ...
        'FontWeight', 'bold', ...
        'BackgroundColor', colors.accent_red, ...
        'ButtonPushedFcn', @(btn,event) stopElectrolyzer());
    stopBtn.Enable = 'off';
    
    mqttBtn = uibutton(controlPanel, 'push', ...
        'Position', [250 180 120 35], ...
        'Text', '📡 CONNECT MQTT', ...
        'FontColor', [1 1 1], ...
        'FontWeight', 'bold', ...
        'BackgroundColor', colors.accent_blue, ...
        'ButtonPushedFcn', @(btn,event) connectMQTT());
    
    % Production control - FIXED: Remove BackgroundColor property
    uilabel(controlPanel, 'Position', [30 140 120 20], ...
           'Text', 'PRODUCTION RATE:', ...
           'FontColor', colors.text_light, 'FontWeight', 'bold');
    
    currentSlider = uislider(controlPanel, ...
        'Position', [30 120 300 3], ...
        'Limits', [50 150], ...
        'Value', 80, ...
        'ValueChangingFcn', @(sld,event) updateProductionRate(event.Value)); % REMOVED BackgroundColor
    
    currentLabel = uilabel(controlPanel, 'Position', [340 115 50 25], ...
                          'Text', '80 A', ...
                          'FontSize', 14, 'FontWeight', 'bold', ...
                          'FontColor', colors.accent_orange, ...
                          'BackgroundColor', [0.15 0.2 0.3], ...
                          'HorizontalAlignment', 'center');
    
    % System status
    uilabel(controlPanel, 'Position', [30 90 100 20], ...
           'Text', 'STATUS:', 'FontColor', colors.text_light, 'FontWeight', 'bold');
    statusLabel = uilabel(controlPanel, 'Position', [100 90 150 20], ...
                         'Text', 'READY', ...
                         'FontWeight', 'bold', 'FontColor', colors.accent_green);
    
    uilabel(controlPanel, 'Position', [30 60 100 20], ...
           'Text', 'VOLTAGE:', 'FontColor', colors.text_light, 'FontWeight', 'bold');
    voltageLabel = uilabel(controlPanel, 'Position', [100 60 80 20], ...
                          'Text', '1.8 V', ...
                          'FontWeight', 'bold', 'FontColor', colors.accent_orange);
    
    uilabel(controlPanel, 'Position', [200 60 100 20], ...
           'Text', 'POWER:', 'FontColor', colors.text_light, 'FontWeight', 'bold');
    powerLabel = uilabel(controlPanel, 'Position', [260 60 80 20], ...
                        'Text', '144 W', ...
                        'FontWeight', 'bold', 'FontColor', colors.accent_orange);
    
    % === DUAL POWER SOURCE CONTROL ===
    powerPanel = uipanel(fig, 'Position', [20 250 400 240], ...
                        'Title', '⚡ DUAL POWER CONTROL', ...
                        'BackgroundColor', colors.industrial_panel, ...
                        'ForegroundColor', colors.text_light, ...
                        'BorderColor', colors.industrial_border, ...
                        'FontWeight', 'bold');
    
    % Power source selection
    uilabel(powerPanel, 'Position', [30 190 120 20], ...
           'Text', 'POWER SOURCE:', 'FontColor', colors.text_light, 'FontWeight', 'bold');
    
    powerSourceDropdown = uidropdown(powerPanel, ...
        'Position', [150 190 200 25], ...
        'Items', {'🔄 AUTO MPC', '🏭 GRID ONLY', '☀️ PV ONLY', '🔀 MIXED'}, ...
        'Value', '🔄 AUTO MPC', ...
        'FontColor', colors.text_light, ...
        'BackgroundColor', [0.15 0.2 0.3], ...
        'ValueChangedFcn', @(dd,event) updatePowerSource());
    
    % Power blending - FIXED: Remove BackgroundColor property
    uilabel(powerPanel, 'Position', [30 150 100 20], ...
           'Text', 'GRID/PV BLEND:', 'FontColor', colors.text_light, 'FontWeight', 'bold');
    
    gridPowerSlider = uislider(powerPanel, ...
        'Position', [30 130 300 3], ...
        'Limits', [0 100], ...
        'Value', 50, ...
        'ValueChangingFcn', @(sld,event) updatePowerBlend(event.Value)); % REMOVED BackgroundColor
    
    gridPowerLabel = uilabel(powerPanel, 'Position', [30 100 120 20], ...
                            'Text', 'GRID: 50%', ...
                            'FontWeight', 'bold', 'FontColor', [1 0.8 0.2]);
    
    pvPowerLabel = uilabel(powerPanel, 'Position', [160 100 120 20], ...
                          'Text', 'PV: 50%', ...
                          'FontWeight', 'bold', 'FontColor', [0.2 0.8 1]);
    
    % Power metrics
    uilabel(powerPanel, 'Position', [30 70 80 20], ...
           'Text', 'GRID POWER:', 'FontColor', [1 0.8 0.2]);
    gridPowerValue = uilabel(powerPanel, 'Position', [120 70 80 20], ...
                            'Text', '72 W', 'FontWeight', 'bold', 'FontColor', colors.text_light);
    
    uilabel(powerPanel, 'Position', [210 70 60 20], ...
           'Text', 'PV POWER:', 'FontColor', [0.2 0.8 1]);
    pvPowerValue = uilabel(powerPanel, 'Position', [280 70 80 20], ...
                          'Text', '72 W', 'FontWeight', 'bold', 'FontColor', colors.text_light);
    
    % MPC Status
    uilabel(powerPanel, 'Position', [30 40 120 20], ...
           'Text', 'MPC STATUS:', 'FontColor', colors.text_light, 'FontWeight', 'bold');
    mpcStatusLabel = uilabel(powerPanel, 'Position', [150 40 200 20], ...
                            'Text', 'OPTIMIZING...', ...
                            'FontWeight', 'bold', 'FontColor', colors.accent_green);
    
    % === ELECTROLYZER VISUALIZATION ===
    visPanel = uipanel(fig, 'Position', [440 500 500 240], ...
                      'Title', '🏭 ELECTROLYZER UNIT', ...
                      'BackgroundColor', colors.industrial_panel, ...
                      'ForegroundColor', colors.text_light, ...
                      'BorderColor', colors.industrial_border, ...
                      'FontWeight', 'bold');
    
    electrolyzerAxes = uiaxes(visPanel, 'Position', [50 30 400 180]);
    set(electrolyzerAxes, 'Color', [0.08 0.11 0.15], ...
                         'XColor', colors.text_dim, ...
                         'YColor', colors.text_dim, ...
                         'GridColor', [0.3 0.3 0.3], ...
                         'GridAlpha', 0.3);
    axis(electrolyzerAxes, 'equal');
    xlim(electrolyzerAxes, [0 10]);
    ylim(electrolyzerAxes, [0 15]);
    electrolyzerAxes.Box = 'on';
    
    % === GAS STORAGE TANKS ===
    tankPanel = uipanel(fig, 'Position', [960 500 420 240], ...
                       'Title', '💨 GAS STORAGE SYSTEM', ...
                       'BackgroundColor', colors.industrial_panel, ...
                       'ForegroundColor', colors.text_light, ...
                       'BorderColor', colors.industrial_border, ...
                       'FontWeight', 'bold');
    
    o2TankAxes = uiaxes(tankPanel, 'Position', [50 30 150 180]);
    set(o2TankAxes, 'Color', [0.08 0.11 0.15], ...
                   'XColor', colors.text_dim, ...
                   'YColor', colors.text_dim);
    title(o2TankAxes, 'O₂ TANK', 'Color', colors.text_light, 'FontWeight', 'bold');
    axis(o2TankAxes, 'equal');
    xlim(o2TankAxes, [0 5]);
    ylim(o2TankAxes, [0 12]);
    
    h2TankAxes = uiaxes(tankPanel, 'Position', [220 30 150 180]);
    set(h2TankAxes, 'Color', [0.08 0.11 0.15], ...
                   'XColor', colors.text_dim, ...
                   'YColor', colors.text_dim);
    title(h2TankAxes, 'H₂ TANK', 'Color', colors.text_light, 'FontWeight', 'bold');
    axis(h2TankAxes, 'equal');
    xlim(h2TankAxes, [0 5]);
    ylim(h2TankAxes, [0 12]);
    
    % === SYSTEM MONITORING ===
    monitorPanel = uipanel(fig, 'Position', [440 250 500 240], ...
                          'Title', '📊 SYSTEM MONITORING', ...
                          'BackgroundColor', colors.industrial_panel, ...
                          'ForegroundColor', colors.text_light, ...
                          'BorderColor', colors.industrial_border, ...
                          'FontWeight', 'bold');
    
    % Production gauges
    uilabel(monitorPanel, 'Position', [50 180 120 20], ...
           'Text', 'O₂ PRODUCTION RATE', 'FontColor', colors.text_dim, 'FontSize', 10);
    o2RateGauge = uigauge(monitorPanel, 'linear', 'Position', [50 140 120 40]);
    o2RateGauge.Limits = [0 0.05];
    o2RateGauge.FontColor = colors.text_light;
    
    uilabel(monitorPanel, 'Position', [200 180 120 20], ...
           'Text', 'H₂ PRODUCTION RATE', 'FontColor', colors.text_dim, 'FontSize', 10);
    h2RateGauge = uigauge(monitorPanel, 'linear', 'Position', [200 140 120 40]);
    h2RateGauge.Limits = [0 0.1];
    h2RateGauge.FontColor = colors.text_light;
    
    % Tank level gauges
    uilabel(monitorPanel, 'Position', [350 180 80 20], ...
           'Text', 'O₂ TANK', 'FontColor', colors.text_dim, 'FontSize', 10);
    o2TankGauge = uigauge(monitorPanel, 'circular', 'Position', [340 120 100 60]);
    o2TankGauge.Limits = [0 100];
    o2TankGauge.FontColor = colors.text_light;
    
    uilabel(monitorPanel, 'Position', [470 180 80 20], ...
           'Text', 'H₂ TANK', 'FontColor', colors.text_dim, 'FontSize', 10);
    h2TankGauge = uigauge(monitorPanel, 'circular', 'Position', [460 120 100 60]);
    h2TankGauge.Limits = [0 100];
    h2TankGauge.FontColor = colors.text_light;
    
    % System metrics
    metricsPanel = uipanel(monitorPanel, 'Position', [50 30 400 80], ...
                          'BackgroundColor', [0.1 0.15 0.2], ...
                          'BorderColor', colors.industrial_border);
    
    uilabel(metricsPanel, 'Position', [20 50 80 20], ...
           'Text', 'EFFICIENCY:', 'FontColor', colors.text_dim);
    efficiencyLabel = uilabel(metricsPanel, 'Position', [100 50 60 20], ...
                             'Text', '85%', 'FontWeight', 'bold', 'FontColor', colors.accent_green);
    
    uilabel(metricsPanel, 'Position', [180 50 80 20], ...
           'Text', 'TEMPERATURE:', 'FontColor', colors.text_dim);
    tempLabel = uilabel(metricsPanel, 'Position', [260 50 60 20], ...
                       'Text', '65°C', 'FontWeight', 'bold', 'FontColor', colors.accent_orange);
    
    uilabel(metricsPanel, 'Position', [20 20 80 20], ...
           'Text', 'O₂ PURITY:', 'FontColor', colors.text_dim);
    purityLabel = uilabel(metricsPanel, 'Position', [100 20 60 20], ...
                         'Text', '99.5%', 'FontWeight', 'bold', 'FontColor', colors.accent_green);
    
    uilabel(metricsPanel, 'Position', [180 20 80 20], ...
           'Text', 'ENERGY COST:', 'FontColor', colors.text_dim);
    costLabel = uilabel(metricsPanel, 'Position', [260 20 80 20], ...
                       'Text', '0.12 $/kWh', 'FontWeight', 'bold', 'FontColor', colors.accent_orange);
    
    % === SYSTEM LOG ===
    logPanel = uipanel(fig, 'Position', [20 20 1360 220], ...
                      'Title', '📝 SYSTEM LOG & MPC DECISIONS', ...
                      'BackgroundColor', colors.industrial_panel, ...
                      'ForegroundColor', colors.text_light, ...
                      'BorderColor', colors.industrial_border, ...
                      'FontWeight', 'bold');
    
    logText = uitextarea(logPanel, 'Position', [20 20 1320 160], ...
                        'Value', {'🏭 SYSTEM INITIALIZED - READY FOR OPERATION'; ...
                                 '🌐 MQTT: Waiting for connection...'; ...
                                 '⚡ POWER: Dual-source system ready'; ...
                                 '🧠 MPC: Economic optimization available'}, ...
                        'BackgroundColor', [0.08 0.11 0.15], ...
                        'FontColor', colors.text_light, ...
                        'FontName', 'Consolas', ...
                        'FontSize', 10);
    
    % === SYSTEM VARIABLES ===
    isOperating = false;
    animationTimer = [];
    stopRequested = false;
    simulationTime = 100;
    currentAnimationData = [];
    
    % MQTT Communication
    mqttClient = [];
    isMQTTConnected = false;
    
    % Dual Power Source Configuration
    powerConfig.gridRatio = 0.5;
    powerConfig.pvRatio = 0.5;
    powerConfig.currentSource = '🔄 AUTO MPC';
    powerConfig.gridAvailable = true;
    powerConfig.pvAvailable = true;
    powerConfig.gridPrice = 0.15;
    powerConfig.pvPrice = 0.05;
    powerConfig.mpcActive = true;
    
    % Tank animation data
    tankBubbles = struct();
    tankBubbles.o2_positions = [];
    tankBubbles.h2_positions = [];
    
    % === DUAL POWER SOURCE FUNCTIONS ===
    function updatePowerSource()
        powerConfig.currentSource = powerSourceDropdown.Value;
        addToLog(sprintf('⚡ Power source changed to: %s', powerConfig.currentSource));
        
        switch powerConfig.currentSource
            case '🏭 GRID ONLY'
                powerConfig.gridRatio = 1.0;
                powerConfig.pvRatio = 0.0;
                mpcStatusLabel.Text = 'MANUAL - GRID';
                mpcStatusLabel.FontColor = [1 0.8 0.2];
            case '☀️ PV ONLY'
                powerConfig.gridRatio = 0.0;
                powerConfig.pvRatio = 1.0;
                mpcStatusLabel.Text = 'MANUAL - PV';
                mpcStatusLabel.FontColor = [0.2 0.8 1];
            case '🔀 MIXED'
                mpcStatusLabel.Text = 'MANUAL - MIXED';
                mpcStatusLabel.FontColor = [0.8 0.8 0.2];
            case '🔄 AUTO MPC'
                powerConfig.mpcActive = true;
                mpcStatusLabel.Text = 'MPC OPTIMIZING';
                mpcStatusLabel.FontColor = colors.accent_green;
        end
        
        updatePowerLabels();
        publishPowerStatus();
    end

    function updatePowerBlend(gridPercent)
        powerConfig.gridRatio = gridPercent / 100;
        powerConfig.pvRatio = 1 - powerConfig.gridRatio;
        updatePowerLabels();
        
        if ~strcmp(powerConfig.currentSource, '🔄 AUTO MPC')
            publishPowerStatus();
        end
    end

    function updatePowerLabels()
        gridPowerLabel.Text = sprintf('GRID: %d%%', round(powerConfig.gridRatio * 100));
        pvPowerLabel.Text = sprintf('PV: %d%%', round(powerConfig.pvRatio * 100));
        
        gridPowerSlider.Value = powerConfig.gridRatio * 100;
        
        % Calculate power values - FIXED: Access currentSlider correctly
        current_val = currentSlider.Value;
        total_power = current_val * 1.8;
        grid_power = total_power * powerConfig.gridRatio;
        pv_power = total_power * powerConfig.pvRatio;
        
        gridPowerValue.Text = sprintf('%d W', round(grid_power));
        pvPowerValue.Text = sprintf('%d W', round(pv_power));
        powerLabel.Text = sprintf('%d W', round(total_power));
        
        % Update cost
        total_cost = (grid_power/1000 * powerConfig.gridPrice) + (pv_power/1000 * powerConfig.pvPrice);
        costLabel.Text = sprintf('%.3f $/kWh', total_cost/(total_power/1000));
    end

    function publishPowerStatus()
        if isMQTTConnected
            try
                powerData = struct(...
                    'source', powerConfig.currentSource, ...
                    'grid_ratio', powerConfig.gridRatio, ...
                    'pv_ratio', powerConfig.pvRatio, ...
                    'grid_price', powerConfig.gridPrice, ...
                    'pv_price', powerConfig.pvPrice, ...
                    'total_power', currentSlider.Value * 1.8, ...
                    'timestamp', datestr(now, 'HH:MM:SS') ...
                );
                
                jsonStr = jsonencode(powerData);
                publish(mqttClient, 'power/status', jsonStr);
                
                publish(mqttClient, 'power/source/selection', powerConfig.currentSource);
                publish(mqttClient, 'power/blending/ratio', num2str(powerConfig.gridRatio, '%.2f'));
                
            catch ME
                addToLog(sprintf('❌ MQTT: Power status error - %s', ME.message));
            end
        end
    end

    function handlePowerCommand(topic, message)
        try
            addToLog(sprintf('⚡ Power Command: %s - %s', topic, message));
            
            if contains(topic, 'power/source')
                switch message
                    case 'grid'
                        powerSourceDropdown.Value = '🏭 GRID ONLY';
                    case 'pv'
                        powerSourceDropdown.Value = '☀️ PV ONLY';
                    case 'mixed'
                        powerSourceDropdown.Value = '🔀 MIXED';
                    case 'auto'
                        powerSourceDropdown.Value = '🔄 AUTO MPC';
                end
                updatePowerSource();
                
            elseif contains(topic, 'power/blending')
                newRatio = str2double(message);
                if ~isnan(newRatio) && newRatio >= 0 && newRatio <= 100
                    gridPowerSlider.Value = newRatio;
                    updatePowerBlend(newRatio);
                end
                
            elseif contains(topic, 'power/mpc/decision')
                try
                    decisionData = jsondecode(message);
                    if isfield(decisionData, 'grid_ratio')
                        powerConfig.gridRatio = decisionData.grid_ratio;
                        powerConfig.pvRatio = 1 - powerConfig.gridRatio;
                        updatePowerLabels();
                        mpcStatusLabel.Text = 'WEB MPC ACTIVE';
                        mpcStatusLabel.FontColor = colors.accent_green;
                        addToLog('🧠 WEB MPC: Economic optimization applied');
                    end
                catch
                    addToLog('❌ MPC: Error parsing economic decision');
                end
            end
            
        catch ME
            addToLog(sprintf('❌ Power command error: %s', ME.message));
        end
    end

    function simulateMPCDecision()
        if strcmp(powerConfig.currentSource, '🔄 AUTO MPC') && isOperating
            % Economic optimization logic
            if powerConfig.pvPrice < powerConfig.gridPrice && powerConfig.pvAvailable
                newGridRatio = max(0, powerConfig.gridRatio - 0.1);
            else
                newGridRatio = min(1, powerConfig.gridRatio + 0.1);
            end
            
            powerConfig.gridRatio = newGridRatio;
            powerConfig.pvRatio = 1 - newGridRatio;
            updatePowerLabels();
            
            % Simulate price fluctuations
            powerConfig.gridPrice = 0.12 + rand() * 0.08;
            powerConfig.pvPrice = 0.03 + rand() * 0.04;
        end
    end

    % === MAIN SYSTEM FUNCTIONS ===
    function startElectrolyzer()
        stopRequested = false;
        isOperating = true;
        startBtn.Text = '🔄 RUNNING';
        startBtn.BackgroundColor = [0.8 0.8 0];
        startBtn.Enable = 'off';
        stopBtn.Enable = 'on';
        statusLabel.Text = 'OPERATING';
        statusLabel.FontColor = colors.accent_green;
        
        current_val = currentSlider.Value; % FIXED: Now accessible
        addToLog(sprintf('▶️ System started at %.0f A production rate', current_val));
        
        % Send MQTT commands
        if isMQTTConnected
            try
                publish(mqttClient, 'pem/control', 'start');
                publish(mqttClient, 'pem/current', num2str(current_val, '%.1f'));
                addToLog('📡 MQTT: Start command sent to Arduino');
            catch
                addToLog('❌ MQTT: Failed to send start command');
            end
        end
        
        operateSystem();
    end
    
    function operateSystem()
        if ~isempty(animationTimer)
            stop(animationTimer);
            delete(animationTimer);
            animationTimer = [];
        end
        
        try
            simOut = sim('PEM_Electrolyzer_Complete', 'StopTime', num2str(simulationTime));
            
            if isfield(simOut, 'O2_Rate_Out')
                o2_rate = simOut.O2_Rate_Out.Data;
                h2_rate = simOut.H2_Rate_Out.Data;
                o2_tank = simOut.O2_Level_Out.Data;
                h2_tank = simOut.H2_Level_Out.Data;
                voltage_data = simOut.Voltage_Out.Data;
                time_data = simOut.O2_Rate_Out.Time;
                addToLog('📊 Real-time data acquisition active');
            else
                [o2_rate, h2_rate, o2_tank, h2_tank, voltage_data, time_data] = generateSystemData(currentSlider.Value);
                addToLog('📊 Using system data model');
            end
            
            currentAnimationData = struct(...
                'o2_rate', o2_rate, ...
                'h2_rate', h2_rate, ...
                'o2_tank', o2_tank, ...
                'h2_tank', h2_tank, ...
                'voltage', voltage_data, ...
                'time', time_data ...
            );
            
            initializeTankBubbles();
            startRealtimeMonitoring();
            
        catch ME
            addToLog(sprintf('❌ System error: %s', ME.message));
            [o2_rate, h2_rate, o2_tank, h2_tank, voltage_data, time_data] = generateSystemData(currentSlider.Value);
            currentAnimationData = struct(...
                'o2_rate', o2_rate, ...
                'h2_rate', h2_rate, ...
                'o2_tank', o2_tank, ...
                'h2_tank', h2_tank, ...
                'voltage', voltage_data, ...
                'time', time_data ...
            );
            initializeTankBubbles();
            startRealtimeMonitoring();
        end
    end
    
    function stopElectrolyzer()
        stopRequested = true;
        isOperating = false;
        
        if ~isempty(animationTimer)
            stop(animationTimer);
            delete(animationTimer);
            animationTimer = [];
        end
        
        startBtn.Text = '▶️ START';
        startBtn.BackgroundColor = colors.accent_green;
        startBtn.Enable = 'on';
        stopBtn.Enable = 'off';
        statusLabel.Text = 'STOPPED';
        statusLabel.FontColor = colors.accent_red;
        
        if isMQTTConnected
            try
                publish(mqttClient, 'pem/control', 'stop');
                addToLog('📡 MQTT: Stop command sent to Arduino');
            catch
                addToLog('❌ MQTT: Failed to send stop command');
            end
        end
        
        addToLog('⏹️ System stopped by operator');
        drawnow;
    end
    
    function connectMQTT()
        try
            if ~exist('mqttclient', 'file')
                addToLog('❌ MQTT: Support package not installed');
                return;
            end
            
            mqttClient = mqttclient('tcp://broker.hivemq.com:1883');
            
            % Subscribe to all topics
            subscribe(mqttClient, 'arduino/control', 'Callback', @handleArduinoCommand);
            subscribe(mqttClient, 'arduino/current', 'Callback', @handleArduinoCommand);
            subscribe(mqttClient, 'arduino/sensors', 'Callback', @handleArduinoCommand);
            subscribe(mqttClient, 'arduino/status', 'Callback', @handleArduinoCommand);
            
            subscribe(mqttClient, 'power/source/selection', 'Callback', @handlePowerCommand);
            subscribe(mqttClient, 'power/blending/ratio', 'Callback', @handlePowerCommand);
            subscribe(mqttClient, 'power/mpc/decision', 'Callback', @handlePowerCommand);
            
            subscribe(mqttClient, 'web/control', 'Callback', @handleWebCommand);
            subscribe(mqttClient, 'web/mpc/status', 'Callback', @handleWebCommand);
            
            isMQTTConnected = true;
            mqttStatus.Color = 'green';
            mqttBtn.Text = '📡 MQTT CONNECTED';
            mqttBtn.BackgroundColor = colors.accent_green;
            
            addToLog('✅ MQTT: Connected to HiveMQ broker');
            addToLog('✅ MQTT: Subscribed to Arduino/Web/Power topics');
            
        catch ME
            addToLog(sprintf('❌ MQTT: Connection failed - %s', ME.message));
            mqttStatus.Color = 'red';
            mqttBtn.Text = '📡 CONNECT MQTT';
            mqttBtn.BackgroundColor = colors.accent_blue;
            isMQTTConnected = false;
        end
    end
    
    function handleArduinoCommand(topic, message)
        try
            addToLog(sprintf('🔧 Arduino: %s - %s', topic, message));
            
            if startsWith(message, '{')
                jsonData = jsondecode(message);
                
                if isfield(jsonData, 'prodRateSet')
                    newRate = jsonData.prodRateSet;
                    currentSlider.Value = newRate;
                    currentLabel.Text = sprintf('%d A', round(newRate));
                    addToLog(sprintf('🎚️ Arduino slider: %.0f%%', newRate));
                end
                
                if isfield(jsonData, 'state')
                    arduinoState = jsonData.state;
                    switch arduinoState
                        case 2 % STATE_RUNNING
                            if ~isOperating
                                startElectrolyzer();
                            end
                        case 4 % STATE_STOPPED
                            if isOperating
                                stopElectrolyzer();
                            end
                    end
                end
                
            else
                if contains(topic, 'control')
                    if strcmp(message, 'start') && ~isOperating
                        startElectrolyzer();
                    elseif strcmp(message, 'stop') && isOperating
                        stopElectrolyzer();
                    end
                elseif contains(topic, 'current')
                    newCurrent = str2double(message);
                    if ~isnan(newCurrent) && newCurrent >= 50 && newCurrent <= 150
                        currentSlider.Value = newCurrent;
                        currentLabel.Text = sprintf('%d A', round(newCurrent));
                        updateProductionRate(newCurrent);
                    end
                end
            end
            
        catch ME
            addToLog(sprintf('❌ MQTT: Arduino message error - %s', ME.message));
        end
    end
    
    function handleWebCommand(topic, message)
        try
            addToLog(sprintf('🌐 Web: %s - %s', topic, message));
            
            if contains(topic, 'control')
                if strcmp(message, 'start') && ~isOperating
                    startElectrolyzer();
                elseif strcmp(message, 'stop') && isOperating
                    stopElectrolyzer();
                end
            elseif contains(topic, 'mpc/status')
                try
                    mpcData = jsondecode(message);
                    if isfield(mpcData, 'status')
                        mpcStatusLabel.Text = sprintf('WEB %s', mpcData.status);
                        mpcStatusLabel.FontColor = colors.accent_green;
                    end
                catch
                end
            end
            
        catch ME
            addToLog(sprintf('❌ Web command error: %s', ME.message));
        end
    end
    
    function publishSystemData(o2_rate, h2_rate, o2_tank, h2_tank, voltage)
        if isMQTTConnected
            try
                data = struct(...
                    'o2_rate', o2_rate, ...
                    'h2_rate', h2_rate, ...
                    'o2_tank', o2_tank, ...
                    'h2_tank', h2_tank, ...
                    'voltage', voltage, ...
                    'current', currentSlider.Value, ...
                    'power_source', powerConfig.currentSource, ...
                    'grid_ratio', powerConfig.gridRatio, ...
                    'timestamp', datestr(now, 'HH:MM:SS') ...
                );
                
                jsonStr = jsonencode(data);
                publish(mqttClient, 'matlab/simulation', jsonStr);
                
                publish(mqttClient, 'matlab/current', num2str(currentSlider.Value, '%.1f'));
                publishPowerStatus();
                
            catch
            end
        end
    end
    
    function initializeTankBubbles()
        tankBubbles.o2_positions = rand(8, 2) .* [2.5, 6] + [1.2, 3];
        tankBubbles.h2_positions = rand(12, 2) .* [2.5, 6] + [1.2, 3];
    end
    
    function startRealtimeMonitoring()
        if ~isempty(animationTimer)
            stop(animationTimer);
            delete(animationTimer);
            animationTimer = [];
        end
        
        if isempty(currentAnimationData)
            addToLog('❌ Error: No animation data available');
            return;
        end
        
        o2_rate = currentAnimationData.o2_rate;
        h2_rate = currentAnimationData.h2_rate;
        o2_tank = currentAnimationData.o2_tank;
        h2_tank = currentAnimationData.h2_tank;
        voltage_data = currentAnimationData.voltage;
        time_data = currentAnimationData.time;
        
        numPoints = length(time_data);
        currentPoint = 1;
        
        animationTimer = timer(...
            'ExecutionMode', 'fixedRate', ...
            'Period', 0.05, ...
            'TimerFcn', @updateSystemDisplay);
        
        start(animationTimer);
        
        function updateSystemDisplay(~,~)
            if stopRequested || currentPoint > numPoints
                if ~stopRequested && currentPoint > numPoints
                    addToLog('🔄 System operation cycle completed');
                    stopElectrolyzer();
                else
                    stop(animationTimer);
                    delete(animationTimer);
                    animationTimer = [];
                end
                return;
            end
            
            try
                o2RateGauge.Value = o2_rate(currentPoint);
                h2RateGauge.Value = h2_rate(currentPoint);
                o2TankGauge.Value = min(100, o2_tank(currentPoint));
                h2TankGauge.Value = min(100, h2_tank(currentPoint));
                voltageLabel.Text = sprintf('%.1f V', voltage_data(currentPoint));
                
                updateElectrolyzerDisplay(electrolyzerAxes, o2_rate(currentPoint), h2_rate(currentPoint), currentPoint);
                updateTankDisplay(o2TankAxes, o2_tank(currentPoint), [0.3 0.6 1.0], 'o2', currentPoint);
                updateTankDisplay(h2TankAxes, h2_tank(currentPoint), [0.2 0.8 0.3], 'h2', currentPoint);
                
                if mod(currentPoint, 20) == 0
                    simulateMPCDecision();
                end
                
                publishSystemData(o2_rate(currentPoint), h2_rate(currentPoint), o2_tank(currentPoint), h2_tank(currentPoint), voltage_data(currentPoint));
                
                currentPoint = currentPoint + 1;
                
            catch ME
                addToLog(sprintf('❌ Display error: %s', ME.message));
                stop(animationTimer);
            end
        end
    end
    
    function updateElectrolyzerDisplay(axes, o2_rate, h2_rate, frame)
        cla(axes);
        
        % Electrolyzer body
        rectangle(axes, 'Position', [2 2 6 10], 'FaceColor', [0.15 0.2 0.3], 'EdgeColor', [0.4 0.6 0.8], 'LineWidth', 2);
        text(axes, 5, 12.5, 'ELECTROLYZER', 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Color', colors.text_light, 'FontSize', 10);
        
        % Power source indicators
        if powerConfig.gridRatio > 0
            text(axes, 2, 1.2, sprintf('🏭 %d%%', round(powerConfig.gridRatio*100)), ...
                 'Color', [1 0.8 0.2], 'FontWeight', 'bold', 'FontSize', 8);
        end
        if powerConfig.pvRatio > 0
            text(axes, 6, 1.2, sprintf('☀️ %d%%', round(powerConfig.pvRatio*100)), ...
                 'Color', [0.2 0.8 1], 'FontWeight', 'bold', 'FontSize', 8);
        end
        
        current_val = currentSlider.Value;
        production_factor = current_val / 80;
        time = frame * 0.1 * production_factor;
        
        % O2 bubbles
        if o2_rate > 0
            num_o2 = max(3, min(10, round(o2_rate * 200 * production_factor)));
            o2_x = 2.2 + rand(1, num_o2) * 1.8;
            o2_y = 2.5 + rand(1, num_o2) * 8;
            o2_sizes = 15 + o2_rate * 800 * production_factor;
            
            wave = sin(time * 2 + (1:num_o2) * 0.5) * 0.4;
            o2_y = o2_y + wave;
            
            scatter(axes, o2_x, o2_y, o2_sizes, [0.3 0.6 1.0], 'filled', ...
                   'MarkerEdgeAlpha', 0.3, 'MarkerFaceAlpha', 0.7);
        end
        
        % H2 bubbles
        if h2_rate > 0
            num_h2 = max(5, min(15, round(h2_rate * 250 * production_factor)));
            h2_x = 5.8 + rand(1, num_h2) * 1.8;
            h2_y = 2.5 + rand(1, num_h2) * 8;
            h2_sizes = 30 + h2_rate * 1000 * production_factor;
            
            wave = cos(time * 1.8 + (1:num_h2) * 0.6) * 0.4;
            h2_y = h2_y + wave;
            
            scatter(axes, h2_x, h2_y, h2_sizes, [0.2 0.8 0.3], 'filled', ...
                   'MarkerEdgeAlpha', 0.3, 'MarkerFaceAlpha', 0.7);
        end
        
        drawnow;
    end
    
    function updateTankDisplay(tankAxes, level, color, gas_type, frame)
        cla(tankAxes);
        
        rectangle(tankAxes, 'Position', [0.8 0 3.4 11], 'FaceColor', [0.2 0.25 0.35], 'EdgeColor', [0.5 0.5 0.5], 'LineWidth', 3);
        rectangle(tankAxes, 'Position', [1 0 3 10], 'FaceColor', [0.25 0.3 0.4], 'EdgeColor', [0.6 0.6 0.6], 'LineWidth', 2);
        
        fill_level = min(9.5, (level / 100) * 9.5);
        
        if fill_level > 0
            rectangle(tankAxes, 'Position', [1 0 3 fill_level], 'FaceColor', color, 'EdgeColor', 'none');
            
            current_val = currentSlider.Value;
            production_factor = current_val / 80;
            time = frame * 0.1 * production_factor;
            
            if strcmp(gas_type, 'o2')
                bubbles = tankBubbles.o2_positions;
                bubble_color = [0.3 0.6 1.0];
            else
                bubbles = tankBubbles.h2_positions;
                bubble_color = [0.2 0.8 0.3];
            end
            
            bubble_speed = (0.05 + rand(size(bubbles,1), 1) * 0.1) * production_factor;
            bubbles(:,2) = mod(bubbles(:,2) + bubble_speed, 8) + 2;
            
            valid_bubbles = bubbles(:,2) < fill_level;
            if any(valid_bubbles)
                bubble_sizes = 5 + rand(sum(valid_bubbles), 1) * 12;
                scatter(tankAxes, bubbles(valid_bubbles,1), bubbles(valid_bubbles,2), ...
                       bubble_sizes, bubble_color, 'filled', ...
                       'MarkerEdgeAlpha', 0.4, 'MarkerFaceAlpha', 0.6);
            end
            
            if strcmp(gas_type, 'o2')
                tankBubbles.o2_positions = bubbles;
            else
                tankBubbles.h2_positions = bubbles;
            end
        end
        
        text(tankAxes, 2.5, 5, sprintf('%.0fL', level), ...
             'HorizontalAlignment', 'center', 'FontWeight', 'bold', ...
             'BackgroundColor', [0.15 0.2 0.3], 'Color', colors.text_light, 'Margin', 2, 'FontSize', 9);
    end
    
    function updateProductionRate(newCurrent)
        try
            set_param('PEM_Electrolyzer_Complete/Current_Setpoint', 'Value', num2str(newCurrent));
            currentLabel.Text = sprintf('%d A', round(newCurrent));
            
            estimated_voltage = 1.8 + (newCurrent - 80) * 0.01;
            voltageLabel.Text = sprintf('%.1f V', estimated_voltage);
            
            if isMQTTConnected
                try
                    publish(mqttClient, 'matlab/current', num2str(newCurrent, '%.1f'));
                    publish(mqttClient, 'pem/current', num2str(newCurrent, '%.1f'));
                catch
                end
            end
            
        catch
            addToLog('❌ System: Could not update production rate');
        end
    end
    
    function [o2_rate, h2_rate, o2_tank, h2_tank, voltage_data, time_data] = generateSystemData(current)
        time_data = (0:0.1:simulationTime)';
        base_o2 = current * 0.00021;
        base_h2 = current * 0.00042;
        base_voltage = 1.8 + (current - 80) * 0.01;
        
        o2_rate = base_o2 * (1 + 0.08 * sin(time_data * 0.3) .* exp(-time_data/50));
        h2_rate = base_h2 * (1 + 0.08 * sin(time_data * 0.4) .* exp(-time_data/50));
        voltage_data = base_voltage * ones(size(time_data));
        
        o2_tank = 10 + cumsum(o2_rate) * 0.1;
        h2_tank = 10 + cumsum(h2_rate) * 0.1;
    end
    
    function addToLog(message)
        current_log = logText.Value;
        if length(current_log) > 15
            current_log = current_log(end-14:end);
        end
        current_log{end+1} = ['[' datestr(now, 'HH:MM:SS') '] ' message];
        logText.Value = current_log;
    end
    
    % Initialize system
    initializeTankBubbles();
    updateTankDisplay(o2TankAxes, 10, [0.3 0.6 1.0], 'o2', 1);
    updateTankDisplay(h2TankAxes, 10, [0.2 0.8 0.3], 'h2', 1);
    updateElectrolyzerDisplay(electrolyzerAxes, 0, 0, 1);
    updatePowerSource();
    
    % Set initial status
    matlabStatus.Color = 'green';
    arduinoStatus.Color = 'yellow';
    webStatus.Color = 'yellow';
    
    % System cleanup
    fig.CloseRequestFcn = @(src,event) cleanup();
    function cleanup()
        if ~isempty(animationTimer)
            stop(animationTimer);
            delete(animationTimer);
        end
        if ~isempty(mqttClient)
            try
                clear mqttClient;
            catch
            end
        end
        delete(fig);
    end
    
    fprintf('🏭 INDUSTRIAL PEM CONTROL SYSTEM READY\n');
    fprintf('   ⚡ Dual Power Source: Grid + PV\n');
    fprintf('   🌐 3-Way MQTT Communication\n');
    fprintf('   🧠 Economic MPC Optimization\n');
    fprintf('   📊 Professional Industrial Interface\n');
end

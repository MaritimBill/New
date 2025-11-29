// Main Application Controller - 3-Way MQTT Communication
class PEMApplication {
    constructor() {
        this.mqttClient = null;
        this.isConnected = false;
        this.kenyaData = new KenyaRealData();
        this.mpcController = new MPCController();
        this.henmpc = new HENMPC();
        
        // System state
        this.systemState = {
            operating: false,
            current: 80,
            voltage: 1.8,
            power: 144,
            efficiency: 85,
            temperature: 65,
            purity: 99.5,
            cost: 0.12,
            gridRatio: 0.5,
            pvRatio: 0.5,
            powerSource: 'auto'
        };
        
        // MQTT configuration
        this.mqttConfig = {
            broker: 'wss://broker.hivemq.com:8884/mqtt',
            options: {
                clientId: 'web_pem_dashboard_' + Math.random().toString(16).substr(2, 8),
                clean: true,
                reconnectPeriod: 4000,
                connectTimeout: 4000
            }
        };
        
        // Charts
        this.charts = {};
        
        this.initializeApplication();
    }

    // Initialize the complete application
    async initializeApplication() {
        console.log('🏭 Initializing PEM Electrolyzer Application...');
        
        try {
            // Initialize UI components
            this.initializeUI();
            
            // Initialize charts
            this.initializeCharts();
            
            // Load initial Kenya data
            await this.loadInitialData();
            
            // Initialize MQTT connection
            this.initializeMQTT();
            
            // Start data updates
            this.startDataUpdates();
            
            console.log('✅ PEM Application initialized successfully');
            
        } catch (error) {
            console.error('❌ Application initialization failed:', error);
        }
    }

    // Initialize UI event listeners and components
    initializeUI() {
        // Control buttons
        document.getElementById('startBtn').addEventListener('click', () => this.startSystem());
        document.getElementById('stopBtn').addEventListener('click', () => this.stopSystem());
        document.getElementById('connectBtn').addEventListener('click', () => this.connectMQTT());
        
        // Production control
        const currentSlider = document.getElementById('currentSlider');
        currentSlider.addEventListener('input', (e) => this.updateProductionRate(e.target.value));
        
        // Power control
        const powerSourceDropdown = document.getElementById('powerSourceDropdown');
        powerSourceDropdown.addEventListener('change', (e) => this.updatePowerSource(e.target.value));
        
        const gridPowerSlider = document.getElementById('gridPowerSlider');
        gridPowerSlider.addEventListener('input', (e) => this.updatePowerBlend(e.target.value));
        
        // MPC strategy buttons
        document.querySelectorAll('.strategy-btn').forEach(btn => {
            btn.addEventListener('click', (e) => this.selectMPCStrategy(e.target.dataset.strategy));
        });
        
        // MPC parameter sliders
        document.getElementById('populationSlider').addEventListener('input', (e) => this.updateMPCParams('population', e.target.value));
        document.getElementById('mutationSlider').addEventListener('input', (e) => this.updateMPCParams('mutation', e.target.value));
        
        console.log('🎛️ UI components initialized');
    }

    // Initialize Chart.js charts
    initializeCharts() {
        // Production monitoring chart
        this.charts.production = new Chart(document.getElementById('productionChart'), {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    {
                        label: 'O₂ Production (L/h)',
                        data: [],
                        borderColor: '#00a8ff',
                        backgroundColor: 'rgba(0, 168, 255, 0.1)',
                        tension: 0.4,
                        fill: true
                    },
                    {
                        label: 'Hospital Demand (L/h)',
                        data: [],
                        borderColor: '#00d8a7',
                        backgroundColor: 'rgba(0, 216, 167, 0.1)',
                        tension: 0.4,
                        fill: true
                    }
                ]
            },
            options: this.getChartOptions('Production vs Demand')
        });
        
        // Economic optimization chart
        this.charts.economic = new Chart(document.getElementById('economicChart'), {
            type: 'bar',
            data: {
                labels: ['Revenue', 'Grid Cost', 'PV Cost', 'Operational', 'Profit'],
                datasets: [{
                    label: 'Economic Breakdown (KSh/h)',
                    data: [0, 0, 0, 0, 0],
                    backgroundColor: [
                        '#00d8a7',
                        '#ff9f43',
                        '#00a8ff',
                        '#8b9bb4',
                        '#2ecc71'
                    ]
                }]
            },
            options: this.getChartOptions('Economic Performance')
        });
        
        // Power distribution chart
        this.charts.power = new Chart(document.getElementById('powerChart'), {
            type: 'doughnut',
            data: {
                labels: ['Grid Power', 'PV Power'],
                datasets: [{
                    data: [50, 50],
                    backgroundColor: ['#ff9f43', '#00a8ff'],
                    borderWidth: 2,
                    borderColor: '#1a1f2e'
                }]
            },
            options: {
                responsive: true,
                plugins: {
                    legend: {
                        position: 'bottom',
                        labels: { color: '#e4e7eb' }
                    },
                    title: {
                        display: true,
                        text: 'Power Distribution',
                        color: '#e4e7eb',
                        font: { size: 14 }
                    }
                }
            }
        });
        
        console.log('📊 Charts initialized');
    }

    // Get common chart options
    getChartOptions(title) {
        return {
            responsive: true,
            maintainAspectRatio: false,
            plugins: {
                legend: {
                    labels: { color: '#e4e7eb' }
                },
                title: {
                    display: true,
                    text: title,
                    color: '#e4e7eb',
                    font: { size: 14 }
                }
            },
            scales: {
                x: {
                    ticks: { color: '#8b9bb4' },
                    grid: { color: 'rgba(139, 155, 180, 0.1)' }
                },
                y: {
                    ticks: { color: '#8b9bb4' },
                    grid: { color: 'rgba(139, 155, 180, 0.1)' }
                }
            }
        };
    }

    // Load initial Kenya data
    async loadInitialData() {
        try {
            const data = this.kenyaData.getCurrentData();
            
            // Update Kenya context display
            this.updateKenyaContext(data);
            
            // Update economic parameters
            this.updateEconomicDisplay(data);
            
            console.log('🇰🇪 Kenya data loaded');
            
        } catch (error) {
            console.error('❌ Failed to load Kenya data:', error);
        }
    }

    // Initialize MQTT connection
    initializeMQTT() {
        try {
            this.mqttClient = mqtt.connect(this.mqttConfig.broker, this.mqttConfig.options);
            
            this.mqttClient.on('connect', () => {
                console.log('✅ MQTT Connected to broker');
                this.isConnected = true;
                this.updateConnectionStatus('mqtt', true);
                this.subscribeToTopics();
            });
            
            this.mqttClient.on('message', (topic, message) => {
                this.handleMQTTMessage(topic, message.toString());
            });
            
            this.mqttClient.on('error', (error) => {
                console.error('❌ MQTT Error:', error);
                this.updateConnectionStatus('mqtt', false);
            });
            
            this.mqttClient.on('close', () => {
                console.log('🔌 MQTT Connection closed');
                this.updateConnectionStatus('mqtt', false);
            });
            
        } catch (error) {
            console.error('❌ MQTT initialization failed:', error);
        }
    }

    // Subscribe to MQTT topics
    subscribeToTopics() {
        const topics = [
            'arduino/sensors',
            'arduino/status',
            'arduino/current',
            'matlab/simulation',
            'matlab/current',
            'matlab/control',
            'pem/control',
            'pem/current',
            'power/status',
            'power/source/selection',
            'power/blending/ratio',
            'power/mpc/decision'
        ];
        
        topics.forEach(topic => {
            this.mqttClient.subscribe(topic, (err) => {
                if (err) {
                    console.error(`❌ Failed to subscribe to ${topic}:`, err);
                } else {
                    console.log(`✅ Subscribed to ${topic}`);
                }
            });
        });
    }

    // Handle incoming MQTT messages
    handleMQTTMessage(topic, message) {
        try {
            console.log(`📨 MQTT: ${topic} - ${message}`);
            
            // Update connection status based on message source
            if (topic.startsWith('arduino/')) {
                this.updateConnectionStatus('arduino', true);
            } else if (topic.startsWith('matlab/')) {
                this.updateConnectionStatus('matlab', true);
            }
            
            // Parse JSON messages
            let data;
            try {
                data = JSON.parse(message);
            } catch {
                data = message; // Use string directly if not JSON
            }
            
            // Route message to appropriate handler
            this.routeMQTTMessage(topic, data);
            
        } catch (error) {
            console.error('❌ Error handling MQTT message:', error);
        }
    }

    // Route MQTT messages to specific handlers
    routeMQTTMessage(topic, data) {
        const handlers = {
            'arduino/sensors': (data) => this.handleArduinoData(data),
            'matlab/simulation': (data) => this.handleMATLABData(data),
            'power/status': (data) => this.handlePowerData(data),
            'arduino/current': (data) => this.updateCurrentDisplay(data),
            'matlab/current': (data) => this.updateCurrentDisplay(data),
            'pem/control': (data) => this.handleControlCommand(data),
            'power/source/selection': (data) => this.updatePowerSource(data),
            'power/blending/ratio': (data) => this.updatePowerBlend(data)
        };
        
        const handler = handlers[topic];
        if (handler) {
            handler(data);
        }
    }

    // Handle Arduino sensor data
    handleArduinoData(data) {
        if (typeof data === 'object') {
            this.systemState.current = data.appliedCurrent || this.systemState.current;
            this.systemState.temperature = data.temperature || this.systemState.temperature;
            this.systemState.purity = data.o2Purity || this.systemState.purity;
            
            this.updateSystemDisplay();
            this.addToLog(`🔧 Arduino: Current=${data.appliedCurrent}A, Temp=${data.temperature}°C`);
        }
    }

    // Handle MATLAB simulation data
    handleMATLABData(data) {
        if (typeof data === 'object') {
            this.systemState.voltage = data.voltage || this.systemState.voltage;
            this.systemState.power = data.current * 1.8 || this.systemState.power;
            
            this.updateSystemDisplay();
            this.updateCharts(data);
        }
    }

    // Handle power control data
    handlePowerData(data) {
        if (typeof data === 'object') {
            this.systemState.gridRatio = data.grid_ratio || this.systemState.gridRatio;
            this.systemState.pvRatio = data.pv_ratio || this.systemState.pvRatio;
            this.systemState.powerSource = data.source || this.systemState.powerSource;
            
            this.updatePowerDisplay();
        }
    }

    // Handle control commands
    handleControlCommand(command) {
        if (command === 'start') {
            this.startSystem();
        } else if (command === 'stop') {
            this.stopSystem();
        }
    }

    // Update system display
    updateSystemDisplay() {
        // Update numeric displays
        document.getElementById('currentValue').textContent = `${Math.round(this.systemState.current)} A`;
        document.getElementById('voltageDisplay').textContent = `${this.systemState.voltage.toFixed(1)} V`;
        document.getElementById('powerDisplay').textContent = `${Math.round(this.systemState.current * 1.8)} W`;
        document.getElementById('efficiencyDisplay').textContent = `${this.systemState.efficiency}%`;
        
        // Update status displays
        document.getElementById('statusDisplay').textContent = this.systemState.operating ? 'OPERATING' : 'READY';
        document.getElementById('statusDisplay').style.color = this.systemState.operating ? '#00d8a7' : '#8b9bb4';
        
        // Update gauges
        this.updateGauges();
    }

    // Update power display
    updatePowerDisplay() {
        document.getElementById('gridPowerLabel').textContent = `GRID: ${Math.round(this.systemState.gridRatio * 100)}%`;
        document.getElementById('pvPowerLabel').textContent = `PV: ${Math.round(this.systemState.pvRatio * 100)}%`;
        
        document.getElementById('gridPowerValue').textContent = `${Math.round(this.systemState.current * 1.8 * this.systemState.gridRatio)} W`;
        document.getElementById('pvPowerValue').textContent = `${Math.round(this.systemState.current * 1.8 * this.systemState.pvRatio)} W`;
        
        // Update power source dropdown
        document.getElementById('powerSourceDropdown').value = this.systemState.powerSource;
        
        // Update power chart
        this.updatePowerChart();
    }

    // Update Kenya context display
    updateKenyaContext(data) {
        document.getElementById('hospitalDemand').textContent = `${data.hospital.demand} L/min`;
        document.getElementById('currentTariff').textContent = `${data.energy.tariff.rate} KSh/kWh`;
        document.getElementById('solarPower').textContent = `${data.energy.solar.toFixed(1)} kW`;
        document.getElementById('gridStatus').textContent = data.energy.grid.available ? 'AVAILABLE' : 'UNAVAILABLE';
    }

    // Update economic display
    updateEconomicDisplay(data) {
        const economicParams = this.kenyaData.getEconomicParameters();
        const production = this.systemState.current * 0.00021 * 3600;
        const revenue = Math.min(production, data.hospital.demand * 60) * economicParams.oxygenPrice;
        const cost = this.calculateProductionCost();
        
        this.charts.economic.data.datasets[0].data = [
            revenue,
            cost.grid,
            cost.pv,
            cost.operational,
            revenue - cost.total
        ];
        
        this.charts.economic.update();
    }

    // Calculate production costs
    calculateProductionCost() {
        const power = this.systemState.current * 1.8 / 1000; // kWh
        const tariff = this.kenyaData.getCurrentTariff().rate;
        
        const gridCost = power * this.systemState.gridRatio * tariff;
        const pvCost = power * this.systemState.pvRatio * 2.0;
        const operationalCost = power * 0.17;
        const totalCost = gridCost + pvCost + operationalCost;
        
        return { grid: gridCost, pv: pvCost, operational: operationalCost, total: totalCost };
    }

    // Update gauges
    updateGauges() {
        // This would update the gauge components
        // Implementation depends on your gauge library
        console.log('📊 Updating gauges with current state');
    }

    // Update charts with new data
    updateCharts(data) {
        const now = new Date();
        const timeLabel = now.toLocaleTimeString('en-US', { hour12: false });
        
        // Update production chart
        if (this.charts.production.data.labels.length > 20) {
            this.charts.production.data.labels.shift();
            this.charts.production.data.datasets[0].data.shift();
            this.charts.production.data.datasets[1].data.shift();
        }
        
        this.charts.production.data.labels.push(timeLabel);
        this.charts.production.data.datasets[0].data.push(data.o2_rate * 3600);
        this.charts.production.data.datasets[1].data.push(this.kenyaData.getCurrentHospitalDemand() * 60);
        this.charts.production.update();
        
        // Update power chart
        this.updatePowerChart();
    }

    // Update power distribution chart
    updatePowerChart() {
        this.charts.power.data.datasets[0].data = [
            this.systemState.gridRatio * 100,
            this.systemState.pvRatio * 100
        ];
        this.charts.power.update();
    }

    // Start data updates
    startDataUpdates() {
        // Update Kenya data every 30 seconds
        setInterval(() => {
            this.loadInitialData();
        }, 30000);
        
        // Update MPC performance comparison every 10 seconds
        setInterval(() => {
            if (this.isConnected) {
                this.mpcController.compareVariants();
            }
        }, 10000);
        
        // Automatic HENMPC optimization every 15 seconds
        setInterval(() => {
            if (this.systemState.operating) {
                this.runMPCOptimization();
            }
        }, 15000);
        
        console.log('🔄 Data updates started');
    }

    // Run MPC optimization
    runMPCOptimization() {
        const currentState = {
            current: this.systemState.current,
            gridRatio: this.systemState.gridRatio,
            hospitalDemand: this.kenyaData.getCurrentHospitalDemand(),
            tariff: this.kenyaData.getCurrentTariff().rate,
            solar: this.kenyaData.getCurrentSolarPower()
        };
        
        const decision = this.henmpc.generateControlDecision(currentState);
        
        // Apply optimization decision
        if (decision && decision.confidence > 0.7) {
            this.systemState.current = decision.optimalCurrent;
            this.systemState.gridRatio = decision.gridRatio;
            this.systemState.pvRatio = decision.pvRatio;
            
            this.updateSystemDisplay();
            this.updatePowerDisplay();
            
            // Send decision to MATLAB and Arduino via MQTT
            this.publishMPCDecision(decision);
            
            this.addToLog(`🧠 HENMPC: Optimization applied (Confidence: ${(decision.confidence * 100).toFixed(0)}%)`);
        }
    }

    // Publish MPC decision via MQTT
    publishMPCDecision(decision) {
        if (!this.isConnected) return;
        
        const decisionData = {
            optimal_current: decision.optimalCurrent,
            grid_ratio: decision.gridRatio,
            pv_ratio: decision.pvRatio,
            confidence: decision.confidence,
            timestamp: new Date().toISOString(),
            reasoning: decision.reasoning
        };
        
        this.mqttClient.publish('web/mpc/decision', JSON.stringify(decisionData));
        this.mqttClient.publish('power/mpc/decision', JSON.stringify(decisionData));
        
        console.log('📤 Published MPC decision:', decisionData);
    }

    // System control functions
    startSystem() {
        this.systemState.operating = true;
        this.updateSystemDisplay();
        
        // Enable stop button, disable start button
        document.getElementById('startBtn').disabled = true;
        document.getElementById('stopBtn').disabled = false;
        
        // Send start command via MQTT
        this.publishControlCommand('start');
        
        this.addToLog('▶️ System started');
    }

    stopSystem() {
        this.systemState.operating = false;
        this.updateSystemDisplay();
        
        // Enable start button, disable stop button
        document.getElementById('startBtn').disabled = false;
        document.getElementById('stopBtn').disabled = true;
        
        // Send stop command via MQTT
        this.publishControlCommand('stop');
        
        this.addToLog('⏹️ System stopped');
    }

    // Publish control command via MQTT
    publishControlCommand(command) {
        if (!this.isConnected) return;
        
        this.mqttClient.publish('web/control', command);
        this.mqttClient.publish('pem/control', command);
        
        console.log(`📤 Published control command: ${command}`);
    }

    // Update production rate
    updateProductionRate(newCurrent) {
        this.systemState.current = parseInt(newCurrent);
        this.updateSystemDisplay();
        
        // Send current update via MQTT
        if (this.isConnected) {
            this.mqttClient.publish('web/current', newCurrent.toString());
        }
    }

    // Update power source
    updatePowerSource(source) {
        this.systemState.powerSource = source;
        this.updatePowerDisplay();
        
        // Send power source update via MQTT
        if (this.isConnected) {
            this.mqttClient.publish('power/source/selection', source);
        }
    }

    // Update power blend
    updatePowerBlend(gridPercent) {
        this.systemState.gridRatio = gridPercent / 100;
        this.systemState.pvRatio = 1 - this.systemState.gridRatio;
        this.updatePowerDisplay();
        
        // Send power blend update via MQTT
        if (this.isConnected) {
            this.mqttClient.publish('power/blending/ratio', gridPercent.toString());
        }
    }

    // Select MPC strategy
    selectMPCStrategy(strategy) {
        this.mpcController.selectVariant('henmpc'); // Always use HENMPC for now
        this.henmpc.updateWeights(strategy);
        
        // Update strategy buttons
        document.querySelectorAll('.strategy-btn').forEach(btn => {
            btn.classList.remove('active');
        });
        event.target.classList.add('active');
        
        this.addToLog(`🎯 MPC strategy changed to: ${strategy.toUpperCase()}`);
    }

    // Update MPC parameters
    updateMPCParams(param, value) {
        if (param === 'population') {
            this.henmpc.populationSize = parseInt(value);
            document.getElementById('popSize').textContent = value;
        } else if (param === 'mutation') {
            // Update mutation rate display
            document.getElementById('mutationRate').textContent = value;
        }
    }

    // Connect to MQTT manually
    connectMQTT() {
        if (!this.isConnected) {
            this.initializeMQTT();
        }
    }

    // Update connection status
    updateConnectionStatus(component, connected) {
        const lamp = document.getElementById(component + 'Lamp');
        if (lamp) {
            lamp.className = `status-lamp ${connected ? 'connected' : ''}`;
        }
        
        // Update web status
        this.updateConnectionStatus('web', true);
    }

    // Add message to system log
    addToLog(message) {
        const logContainer = document.getElementById('systemLog');
        if (!logContainer) return;
        
        const logEntry = document.createElement('div');
        logEntry.className = 'log-entry info';
        logEntry.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
        
        logContainer.appendChild(logEntry);
        logContainer.scrollTop = logContainer.scrollHeight;
        
        // Keep only last 50 entries
        while (logContainer.children.length > 50) {
            logContainer.removeChild(logContainer.firstChild);
        }
    }

    // Get application status
    getStatus() {
        return {
            connected: this.isConnected,
            operating: this.systemState.operating,
            currentVariant: this.mpcController.currentVariant,
            kenyaData: this.kenyaData.getCurrentData(),
            performance: this.mpcController.getPerformanceStats()
        };
    }

    // Export application data
    exportData() {
        return {
            systemState: this.systemState,
            kenyaData: this.kenyaData.getCurrentData(),
            mpcPerformance: this.mpcController.exportPerformanceData(),
            timestamp: new Date()
        };
    }
}

// Initialize application when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
    window.pemApp = new PEMApplication();
});

// Export for testing
if (typeof module !== 'undefined' && module.exports) {
    module.exports = PEMApplication;
}

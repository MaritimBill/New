// Multiple MPC Variants Controller for Performance Comparison
class MPCController {
    constructor() {
        this.variants = {
            henmpc: new HENMPC(),
            linear: new LinearMPC(),
            nonlinear: new NonlinearMPC(),
            adaptive: new AdaptiveMPC(),
            robust: new RobustMPC()
        };
        
        this.performanceMetrics = [];
        this.currentVariant = 'henmpc';
        this.comparisonInterval = null;
        this.updateFrequency = 10000; // 10 seconds
        
        this.initializeVariants();
    }

    // Initialize all MPC variants
    initializeVariants() {
        console.log('🎯 Initializing MPC variants...');
        
        Object.values(this.variants).forEach(variant => {
            if (variant.initialize) {
                variant.initialize();
            }
        });
        
        this.startPerformanceComparison();
    }

    // Linear MPC implementation
    class LinearMPC {
        constructor() {
            this.name = "Linear MPC";
            this.horizon = 6;
            this.weights = { Q: 1.0, R: 0.1, S: 10.0 };
            this.performance = [];
        }

        initialize() {
            console.log('📈 Linear MPC initialized');
        }

        computeControl(currentState, reference, constraints) {
            // Simple linear quadratic regulator
            const error = reference - currentState;
            const control = this.lqrControl(error, constraints);
            
            return {
                control: control,
                cost: this.calculateCost(error, control),
                horizon: this.horizon,
                variant: this.name
            };
        }

        lqrControl(error, constraints) {
            // Simplified LQR control law
            const K = 0.8; // Control gain
            let control = K * error;
            
            // Apply constraints
            control = Math.max(constraints.min, Math.min(constraints.max, control));
            
            return control;
        }

        calculateCost(error, control) {
            return this.weights.Q * Math.pow(error, 2) + this.weights.R * Math.pow(control, 2);
        }
    }

    // Nonlinear MPC implementation
    class NonlinearMPC {
        constructor() {
            this.name = "Nonlinear MPC";
            this.horizon = 8;
            this.weights = { Q: 1.0, R: 0.2, S: 5.0 };
            this.performance = [];
        }

        initialize() {
            console.log('🔄 Nonlinear MPC initialized');
        }

        computeControl(currentState, reference, constraints) {
            // Nonlinear optimization using gradient descent
            const control = this.nonlinearOptimization(currentState, reference, constraints);
            
            return {
                control: control,
                cost: this.calculateNonlinearCost(currentState, reference, control),
                horizon: this.horizon,
                variant: this.name
            };
        }

        nonlinearOptimization(currentState, reference, constraints) {
            // Simplified gradient-based optimization
            let control = 80; // Initial guess
            const learningRate = 0.1;
            const iterations = 10;
            
            for (let i = 0; i < iterations; i++) {
                const gradient = this.calculateGradient(currentState, reference, control);
                control -= learningRate * gradient;
                
                // Apply constraints
                control = Math.max(constraints.min, Math.min(constraints.max, control));
            }
            
            return control;
        }

        calculateGradient(currentState, reference, control) {
            // Simplified gradient calculation
            const predictedState = this.nonlinearModel(currentState, control);
            const error = predictedState - reference;
            return 2 * this.weights.Q * error * 0.8; // Approximate derivative
        }

        nonlinearModel(state, control) {
            // Simplified nonlinear system model
            return state * 0.95 + control * 0.8 + 0.1 * Math.sin(control * 0.1);
        }

        calculateNonlinearCost(state, reference, control) {
            const predictedState = this.nonlinearModel(state, control);
            const error = predictedState - reference;
            return this.weights.Q * Math.pow(error, 2) + this.weights.R * Math.pow(control, 2);
        }
    }

    // Adaptive MPC implementation
    class AdaptiveMPC {
        constructor() {
            this.name = "Adaptive MPC";
            this.horizon = 6;
            this.weights = { Q: 1.0, R: 0.15, S: 8.0 };
            this.adaptationRate = 0.1;
            this.performance = [];
            this.modelParameters = { A: 0.95, B: 0.8 };
        }

        initialize() {
            console.log('🎚️ Adaptive MPC initialized');
        }

        computeControl(currentState, reference, constraints) {
            // Adaptive control with online parameter estimation
            this.adaptModel(currentState);
            const control = this.adaptiveControl(currentState, reference, constraints);
            
            return {
                control: control,
                cost: this.calculateCost(currentState, reference, control),
                horizon: this.horizon,
                variant: this.name,
                adaptedParameters: { ...this.modelParameters }
            };
        }

        adaptModel(currentState) {
            // Simple recursive least squares adaptation
            const measurementNoise = 0.01;
            const adaptation = this.adaptationRate * (Math.random() - 0.5) * measurementNoise;
            
            this.modelParameters.A += adaptation;
            this.modelParameters.B += adaptation;
            
            // Keep parameters within reasonable bounds
            this.modelParameters.A = Math.max(0.8, Math.min(1.0, this.modelParameters.A));
            this.modelParameters.B = Math.max(0.5, Math.min(1.0, this.modelParameters.B));
        }

        adaptiveControl(currentState, reference, constraints) {
            const error = reference - currentState;
            const K = 0.7; // Adaptive gain
            let control = K * error;
            
            // Apply constraints with adaptation consideration
            control = Math.max(constraints.min, Math.min(constraints.max, control));
            
            return control;
        }

        calculateCost(state, reference, control) {
            const predictedState = this.modelParameters.A * state + this.modelParameters.B * control;
            const error = predictedState - reference;
            return this.weights.Q * Math.pow(error, 2) + this.weights.R * Math.pow(control, 2);
        }
    }

    // Robust MPC implementation
    class RobustMPC {
        constructor() {
            this.name = "Robust MPC";
            this.horizon = 10;
            this.weights = { Q: 1.0, R: 0.25, S: 15.0 };
            this.uncertaintyBounds = { min: 0.85, max: 1.15 };
            this.performance = [];
        }

        initialize() {
            console.log('🛡️ Robust MPC initialized');
        }

        computeControl(currentState, reference, constraints) {
            // Robust control considering worst-case scenarios
            const control = this.robustOptimization(currentState, reference, constraints);
            
            return {
                control: control,
                cost: this.calculateRobustCost(currentState, reference, control),
                horizon: this.horizon,
                variant: this.name,
                robustness: this.calculateRobustnessMetric()
            };
        }

        robustOptimization(currentState, reference, constraints) {
            // Min-max optimization for robustness
            const scenarios = this.generateScenarios(currentState);
            let bestControl = 80;
            let minWorstCaseCost = Infinity;
            
            // Evaluate multiple control candidates
            for (let control = constraints.min; control <= constraints.max; control += 5) {
                const worstCaseCost = this.evaluateWorstCase(scenarios, control, reference);
                
                if (worstCaseCost < minWorstCaseCost) {
                    minWorstCaseCost = worstCaseCost;
                    bestControl = control;
                }
            }
            
            return bestControl;
        }

        generateScenarios(currentState) {
            // Generate uncertainty scenarios
            const scenarios = [];
            const steps = 5;
            
            for (let i = 0; i < steps; i++) {
                const uncertainty = this.uncertaintyBounds.min + 
                    (i / (steps - 1)) * (this.uncertaintyBounds.max - this.uncertaintyBounds.min);
                
                scenarios.push({
                    state: currentState * uncertainty,
                    probability: 1 / steps
                });
            }
            
            return scenarios;
        }

        evaluateWorstCase(scenarios, control, reference) {
            let maxCost = -Infinity;
            
            scenarios.forEach(scenario => {
                const predictedState = scenario.state * 0.95 + control * 0.8;
                const error = predictedState - reference;
                const cost = this.weights.Q * Math.pow(error, 2) + this.weights.R * Math.pow(control, 2);
                
                if (cost > maxCost) {
                    maxCost = cost;
                }
            });
            
            return maxCost;
        }

        calculateRobustCost(state, reference, control) {
            // Conservative cost calculation
            const worstCaseState = state * this.uncertaintyBounds.min;
            const predictedState = worstCaseState * 0.95 + control * 0.8;
            const error = predictedState - reference;
            
            return this.weights.Q * Math.pow(error, 2) + this.weights.R * Math.pow(control, 2);
        }

        calculateRobustnessMetric() {
            return 0.8 + Math.random() * 0.15; // Simulated robustness metric
        }
    }

    // Start continuous performance comparison
    startPerformanceComparison() {
        this.comparisonInterval = setInterval(() => {
            this.compareVariants();
        }, this.updateFrequency);
        
        console.log('📊 MPC performance comparison started');
    }

    // Stop performance comparison
    stopPerformanceComparison() {
        if (this.comparisonInterval) {
            clearInterval(this.comparisonInterval);
            this.comparisonInterval = null;
        }
    }

    // Compare all MPC variants
    async compareVariants() {
        const testScenario = this.generateTestScenario();
        const results = [];
        
        console.log('🔍 Comparing MPC variants...');
        
        // Test each variant
        for (const [name, variant] of Object.entries(this.variants)) {
            try {
                const startTime = performance.now();
                
                let controlResult;
                if (name === 'henmpc') {
                    controlResult = variant.generateControlDecision(testScenario.currentState);
                } else {
                    controlResult = variant.computeControl(
                        testScenario.currentState,
                        testScenario.reference,
                        testScenario.constraints
                    );
                }
                
                const computationTime = performance.now() - startTime;
                
                const performance = this.evaluateVariantPerformance(
                    name,
                    controlResult,
                    testScenario,
                    computationTime
                );
                
                results.push(performance);
                
            } catch (error) {
                console.error(`❌ Error testing ${name}:`, error);
            }
        }
        
        // Sort by overall performance
        results.sort((a, b) => b.overallScore - a.overallScore);
        
        // Update performance metrics
        this.performanceMetrics.push({
            timestamp: new Date(),
            results: results,
            bestVariant: results[0].variant,
            scenario: testScenario.description
        });
        
        // Keep only last 100 comparisons
        if (this.performanceMetrics.length > 100) {
            this.performanceMetrics.shift();
        }
        
        this.updatePerformanceDisplay(results);
        return results;
    }

    // Generate test scenario for comparison
    generateTestScenario() {
        const kenyaData = new KenyaRealData();
        const currentData = kenyaData.getCurrentData();
        
        return {
            currentState: currentData.hospital.demand,
            reference: 100, // Target production
            constraints: { min: 50, max: 150 },
            conditions: {
                tariff: currentData.energy.tariff.rate,
                solar: currentData.energy.solar,
                grid: currentData.energy.grid.reliability,
                demand: currentData.hospital.demand
            },
            description: `Real-time test: Demand=${currentData.hospital.demand}L/min, Tariff=${currentData.energy.tariff.rate}KSh/kWh`
        };
    }

    // Evaluate variant performance
    evaluateVariantPerformance(variantName, controlResult, scenario, computationTime) {
        const economicEfficiency = this.calculateEconomicEfficiency(controlResult, scenario);
        const trackingPerformance = this.calculateTrackingPerformance(controlResult, scenario);
        const robustness = this.calculateRobustness(controlResult, scenario);
        const computationEfficiency = this.calculateComputationEfficiency(computationTime);
        
        const weights = {
            economic: 0.35,
            tracking: 0.25,
            robustness: 0.25,
            computation: 0.15
        };
        
        const overallScore = 
            economicEfficiency * weights.economic +
            trackingPerformance * weights.tracking +
            robustness * weights.robustness +
            computationEfficiency * weights.computation;
        
        return {
            variant: variantName,
            control: controlResult.control || controlResult.optimalCurrent,
            economicEfficiency,
            trackingPerformance,
            robustness,
            computationEfficiency,
            computationTime,
            overallScore,
            details: controlResult
        };
    }

    // Calculate economic efficiency metric
    calculateEconomicEfficiency(controlResult, scenario) {
        const kenyaData = new KenyaRealData();
        const economicParams = kenyaData.getEconomicParameters();
        
        const production = (controlResult.control || controlResult.optimalCurrent) * 0.00021 * 3600;
        const revenue = Math.min(production, scenario.conditions.demand) * economicParams.oxygenPrice;
        const cost = this.estimateOperatingCost(controlResult, scenario);
        
        const profit = revenue - cost;
        const maxPossibleProfit = scenario.conditions.demand * economicParams.oxygenPrice * 0.3; // 30% margin
        
        return Math.max(0, Math.min(1, profit / maxPossibleProfit));
    }

    // Estimate operating cost
    estimateOperatingCost(controlResult, scenario) {
        const power = (controlResult.control || controlResult.optimalCurrent) * 1.8 / 1000; // kWh
        const gridCost = power * (controlResult.gridRatio || 0.5) * scenario.conditions.tariff;
        const pvCost = power * (controlResult.pvRatio || 0.5) * 2.0; // Amortized solar cost
        const operationalCost = power * 0.17; // Other costs
        
        return gridCost + pvCost + operationalCost;
    }

    // Calculate tracking performance
    calculateTrackingPerformance(controlResult, scenario) {
        const production = (controlResult.control || controlResult.optimalCurrent) * 0.00021 * 3600;
        const error = Math.abs(production - scenario.reference);
        const maxError = 50; // Maximum acceptable error
        
        return Math.max(0, 1 - (error / maxError));
    }

    // Calculate robustness metric
    calculateRobustness(controlResult, scenario) {
        let robustness = 0.7; // Base robustness
        
        // Penalize extreme control actions
        const control = controlResult.control || controlResult.optimalCurrent;
        if (control < 60 || control > 140) {
            robustness -= 0.2;
        }
        
        // Reward stable power blending
        if (controlResult.gridRatio && controlResult.pvRatio) {
            const blendStability = 1 - Math.abs(controlResult.gridRatio - 0.5);
            robustness += blendStability * 0.1;
        }
        
        // Consider computation stability
        if (controlResult.robustness) {
            robustness = controlResult.robustness;
        }
        
        return Math.max(0.3, Math.min(1.0, robustness));
    }

    // Calculate computation efficiency
    calculateComputationEfficiency(computationTime) {
        const maxAcceptableTime = 1000; // 1 second
        return Math.max(0, 1 - (computationTime / maxAcceptableTime));
    }

    // Update performance display
    updatePerformanceDisplay(results) {
        const performanceBody = document.getElementById('performanceBody');
        if (!performanceBody) return;
        
        performanceBody.innerHTML = '';
        
        results.forEach((result, index) => {
            const row = document.createElement('tr');
            row.className = index === 0 ? 'best-performer' : '';
            
            row.innerHTML = `
                <td>
                    <strong>${result.variant}</strong>
                    ${index === 0 ? ' 👑' : ''}
                </td>
                <td>${this.estimateOperatingCost(result, this.generateTestScenario()).toFixed(1)}</td>
                <td>${((result.control || result.details.optimalCurrent) * 0.00021 * 3600).toFixed(1)} L/h</td>
                <td>${(result.economicEfficiency * 100).toFixed(0)}%</td>
                <td>${(result.robustness * 100).toFixed(0)}%</td>
                <td>
                    <button class="btn btn-sm btn-primary" onclick="mpcController.selectVariant('${result.variant}')">
                        Select
                    </button>
                </td>
            `;
            
            performanceBody.appendChild(row);
        });
        
        // Update charts if they exist
        this.updatePerformanceCharts(results);
    }

    // Update performance charts
    updatePerformanceCharts(results) {
        // This would update Chart.js charts with performance data
        // Implementation depends on your chart setup
        console.log('📈 Updating performance charts with:', results);
    }

    // Select MPC variant
    selectVariant(variantName) {
        if (this.variants[variantName]) {
            this.currentVariant = variantName;
            console.log(`🎯 Selected MPC variant: ${variantName}`);
            
            // Update UI to show selected variant
            this.updateVariantSelectionUI(variantName);
            
            return true;
        }
        
        console.error(`❌ Unknown MPC variant: ${variantName}`);
        return false;
    }

    // Update variant selection UI
    updateVariantSelectionUI(selectedVariant) {
        // Remove active class from all variant buttons
        document.querySelectorAll('.strategy-btn').forEach(btn => {
            btn.classList.remove('active');
        });
        
        // Add active class to selected variant
        const selectedBtn = document.querySelector(`[data-variant="${selectedVariant}"]`);
        if (selectedBtn) {
            selectedBtn.classList.add('active');
        }
        
        // Update MPC status display
        const statusElement = document.getElementById('mpcStatus');
        if (statusElement) {
            statusElement.textContent = `${selectedVariant.toUpperCase()} ACTIVE`;
        }
    }

    // Get current control decision
    getControlDecision(currentState, strategy = 'economic') {
        const variant = this.variants[this.currentVariant];
        
        if (!variant) {
            console.error('❌ No MPC variant selected');
            return null;
        }
        
        if (this.currentVariant === 'henmpc') {
            return variant.generateControlDecision(currentState);
        } else {
            const scenario = this.generateTestScenario();
            return variant.computeControl(currentState, scenario.reference, scenario.constraints);
        }
    }

    // Get performance statistics
    getPerformanceStats() {
        const recentComparisons = this.performanceMetrics.slice(-10);
        const variantWins = {};
        
        // Count wins for each variant
        recentComparisons.forEach(comparison => {
            const winner = comparison.bestVariant;
            variantWins[winner] = (variantWins[winner] || 0) + 1;
        });
        
        return {
            totalComparisons: this.performanceMetrics.length,
            recentWins: variantWins,
            currentVariant: this.currentVariant,
            lastComparison: this.performanceMetrics.length > 0 ? 
                this.performanceMetrics[this.performanceMetrics.length - 1] : null
        };
    }

    // Export performance data
    exportPerformanceData() {
        return {
            variants: Object.keys(this.variants),
            performanceMetrics: this.performanceMetrics,
            currentVariant: this.currentVariant,
            timestamp: new Date()
        };
    }

    // Reset all variants
    reset() {
        Object.values(this.variants).forEach(variant => {
            if (variant.initialize) {
                variant.initialize();
            }
        });
        
        this.performanceMetrics = [];
        console.log('🔄 All MPC variants reset');
    }
}

// Create global instance
const mpcController = new MPCController();

// Export for use in other modules
if (typeof module !== 'undefined' && module.exports) {
    module.exports = MPCController;
}

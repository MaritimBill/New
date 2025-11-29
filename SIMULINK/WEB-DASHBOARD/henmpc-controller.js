// Hybrid Evolutionary Neural Model Predictive Controller
class HENMPC {
    constructor() {
        this.populationSize = 50;
        this.generation = 0;
        this.population = [];
        this.bestSolution = null;
        this.performanceHistory = [];
        this.neuralNetwork = this.createNeuralNetwork();
        this.kenyaData = new KenyaRealData();
        
        // Economic optimization weights
        this.weights = {
            economic: 0.35,      // Profit maximization
            reliability: 0.25,   // System stability
            efficiency: 0.20,    // Energy efficiency
            sustainability: 0.15, // Environmental impact
            safety: 0.05         // Operational safety
        };
        
        // MPC parameters
        this.horizon = 8;        // Prediction horizon (hours)
        this.controlInterval = 15; // Control interval (minutes)
        
        this.initializePopulation();
    }

    // Create neural network for demand prediction
    createNeuralNetwork() {
        return {
            // Simple neural network architecture
            layers: [
                { type: 'input', units: 8 },    // [hour, demand, tariff, solar, temp, humidity, grid, previous]
                { type: 'dense', units: 32, activation: 'relu' },
                { type: 'dropout', rate: 0.2 },
                { type: 'dense', units: 16, activation: 'relu' },
                { type: 'dense', units: 8, activation: 'linear' }
            ],
            
            // Simplified prediction (in real implementation, use TensorFlow.js)
            predict: function(inputs) {
                return this.simulatePrediction(inputs);
            },
            
            simulatePrediction: function(inputs) {
                const [hour, currentDemand, tariff, solar, temperature, humidity, gridStatus] = inputs;
                
                // Simulate complex neural network predictions
                const timeFactor = 0.5 * Math.sin((hour - 6) * Math.PI / 12) + 0.5;
                const weatherFactor = 1.0 - (humidity / 200); // Higher humidity = lower solar
                const economicFactor = 1.2 - (tariff / 50); // Higher tariffs = lower demand
                
                const predictedDemand = currentDemand * (1 + 0.1 * timeFactor * economicFactor);
                const predictedSolar = solar * weatherFactor;
                const predictedTariff = tariff * (1 + 0.1 * Math.cos(hour * Math.PI / 6));
                
                return {
                    demand: Math.max(20, predictedDemand),
                    solar: Math.max(0, predictedSolar),
                    tariff: predictedTariff,
                    confidence: 0.85 + Math.random() * 0.1
                };
            },
            
            train: function(trainingData) {
                // Simulate training process
                console.log('🧠 Neural Network: Training with', trainingData.length, 'samples');
                return true;
            }
        };
    }

    // Initialize evolutionary population
    initializePopulation() {
        this.population = [];
        
        for (let i = 0; i < this.populationSize; i++) {
            const solution = {
                id: i,
                // Control parameters
                currentSetpoint: 80 + Math.random() * 70, // 80-150A
                gridRatio: Math.random(), // 0-1
                pvRatio: 1 - Math.random(), // 0-1
                productionRate: 0,
                
                // Strategy parameters
                responseAggressiveness: 0.5 + Math.random() * 0.5, // 0.5-1.0
                riskTolerance: 0.3 + Math.random() * 0.4, // 0.3-0.7
                optimizationFocus: Math.floor(Math.random() * 3), // 0:economic, 1:reliability, 2:efficiency
                
                // Performance metrics
                fitness: 0,
                constraintsViolated: 0,
                generation: 0
            };
            
            this.population.push(solution);
        }
        
        console.log('🧬 HENMPC: Population initialized with', this.populationSize, 'solutions');
    }

    // Main optimization function
    optimize(currentState, strategy = 'economic') {
        this.generation++;
        const startTime = Date.now();
        
        // Update weights based on strategy
        this.updateWeights(strategy);
        
        // Get current Kenya data
        const kenyaData = this.kenyaData.getCurrentData();
        
        // Evaluate population fitness
        this.evaluateFitness(currentState, kenyaData);
        
        // Evolutionary operations
        this.selection();
        this.crossover();
        this.mutation();
        
        // Update best solution
        this.updateBestSolution();
        
        // Record performance
        this.performanceHistory.push({
            generation: this.generation,
            timestamp: new Date(),
            bestFitness: this.bestSolution.fitness,
            averageFitness: this.getAverageFitness(),
            computationTime: Date.now() - startTime,
            strategy: strategy
        });
        
        console.log(`🧬 HENMPC Generation ${this.generation}: Best fitness = ${this.bestSolution.fitness.toFixed(2)}`);
        
        return this.bestSolution;
    }

    // Evaluate fitness of all solutions
    evaluateFitness(currentState, kenyaData) {
        const hospitalDemand = kenyaData.hospital.demand;
        const tariff = kenyaData.energy.tariff.rate;
        const solarPower = kenyaData.energy.solar;
        const gridStatus = kenyaData.energy.grid;
        
        this.population.forEach(solution => {
            let fitness = 0;
            let constraints = 0;
            
            // Calculate production metrics
            solution.productionRate = solution.currentSetpoint * 0.00021 * 3600; // L/h
            
            // Economic fitness
            const economicFitness = this.calculateEconomicFitness(solution, hospitalDemand, tariff, solarPower);
            fitness += economicFitness * this.weights.economic;
            
            // Reliability fitness
            const reliabilityFitness = this.calculateReliabilityFitness(solution, gridStatus, hospitalDemand);
            fitness += reliabilityFitness * this.weights.reliability;
            
            // Efficiency fitness
            const efficiencyFitness = this.calculateEfficiencyFitness(solution, solarPower);
            fitness += efficiencyFitness * this.weights.efficiency;
            
            // Sustainability fitness
            const sustainabilityFitness = this.calculateSustainabilityFitness(solution, solarPower);
            fitness += sustainabilityFitness * this.weights.sustainability;
            
            // Safety fitness
            const safetyFitness = this.calculateSafetyFitness(solution);
            fitness += safetyFitness * this.weights.safety;
            
            // Penalize constraint violations
            constraints = this.checkConstraints(solution, hospitalDemand);
            fitness -= constraints * 10;
            
            solution.fitness = fitness;
            solution.constraintsViolated = constraints;
        });
    }

    // Calculate economic fitness component
    calculateEconomicFitness(solution, demand, tariff, solar) {
        const production = solution.productionRate;
        const delivered = Math.min(production, demand);
        
        // Revenue from oxygen sales
        const oxygenRevenue = delivered * 150; // KSh/h
        
        // Revenue from hydrogen byproduct
        const hydrogenRevenue = (solution.currentSetpoint * 0.00042 * 3600) * 40; // KSh/h
        
        // Production costs
        const productionCost = this.calculateProductionCost(solution, tariff, solar);
        
        // Net profit
        const netProfit = oxygenRevenue + hydrogenRevenue - productionCost;
        
        return netProfit / 1000; // Normalize
    }

    // Calculate production costs
    calculateProductionCost(solution, tariff, solar) {
        const powerRequired = solution.currentSetpoint * 1.8; // Watts
        const gridPower = powerRequired * solution.gridRatio;
        const pvPower = powerRequired * solution.pvRatio;
        
        // Electricity costs
        const gridCost = (gridPower / 1000) * tariff; // KSh/h
        const pvCost = (pvPower / 1000) * 2.0; // Amortized solar cost
        
        // Operational costs
        const operationalCost = solution.productionRate * 0.17; // Water + maintenance + membrane
        
        return gridCost + pvCost + operationalCost;
    }

    // Calculate reliability fitness
    calculateReliabilityFitness(solution, gridStatus, demand) {
        let reliability = 1.0;
        
        // Penalize over-reliance on grid during low reliability
        if (solution.gridRatio > 0.8 && gridStatus.reliability < 0.9) {
            reliability *= 0.7;
        }
        
        // Reward meeting hospital demand
        const demandSatisfaction = Math.min(1.0, solution.productionRate / demand);
        reliability *= demandSatisfaction;
        
        // Penalize rapid changes in setpoint
        if (this.bestSolution && Math.abs(solution.currentSetpoint - this.bestSolution.currentSetpoint) > 20) {
            reliability *= 0.8;
        }
        
        return reliability;
    }

    // Calculate efficiency fitness
    calculateEfficiencyFitness(solution, solarPower) {
        let efficiency = 1.0;
        
        // Reward efficient solar utilization
        const solarUtilization = Math.min(1.0, (solution.pvRatio * solution.currentSetpoint * 1.8) / (solarPower * 1000));
        efficiency *= solarUtilization;
        
        // Reward operation in efficient current range (80-120A)
        const currentEfficiency = 1.0 - Math.abs(solution.currentSetpoint - 100) / 100;
        efficiency *= currentEfficiency;
        
        return efficiency;
    }

    // Calculate sustainability fitness
    calculateSustainabilityFitness(solution, solarPower) {
        // Carbon emissions reduction
        const gridEmissionFactor = 0.5; // kg CO2/kWh (Kenya grid)
        const solarEmissionFactor = 0.05; // kg CO2/kWh (solar)
        
        const gridPower = solution.currentSetpoint * 1.8 * solution.gridRatio / 1000;
        const pvPower = solution.currentSetpoint * 1.8 * solution.pvRatio / 1000;
        
        const emissions = (gridPower * gridEmissionFactor) + (pvPower * solarEmissionFactor);
        const maxEmissions = solution.currentSetpoint * 1.8 * 0.5 / 1000; // All grid scenario
        
        return 1.0 - (emissions / maxEmissions);
    }

    // Calculate safety fitness
    calculateSafetyFitness(solution) {
        let safety = 1.0;
        
        // Penalize operation near limits
        if (solution.currentSetpoint > 140) safety *= 0.8;
        if (solution.currentSetpoint < 60) safety *= 0.9;
        
        // Penalize extreme power blends
        if (solution.gridRatio > 0.9 || solution.pvRatio > 0.9) safety *= 0.85;
        
        return safety;
    }

    // Check operational constraints
    checkConstraints(solution, demand) {
        let violations = 0;
        
        // Current constraints
        if (solution.currentSetpoint < 50 || solution.currentSetpoint > 150) violations++;
        
        // Power blend constraints
        if (solution.gridRatio < 0 || solution.gridRatio > 1) violations++;
        if (solution.pvRatio < 0 || solution.pvRatio > 1) violations++;
        if (Math.abs(solution.gridRatio + solution.pvRatio - 1) > 0.01) violations++;
        
        // Production constraints
        if (solution.productionRate < 0) violations++;
        
        return violations;
    }

    // Evolutionary selection (tournament selection)
    selection() {
        const newPopulation = [];
        const tournamentSize = 5;
        
        // Keep best solution (elitism)
        newPopulation.push({...this.bestSolution});
        
        // Tournament selection for remaining population
        while (newPopulation.length < this.populationSize) {
            let bestInTournament = null;
            
            for (let i = 0; i < tournamentSize; i++) {
                const candidate = this.population[Math.floor(Math.random() * this.populationSize)];
                
                if (!bestInTournament || candidate.fitness > bestInTournament.fitness) {
                    bestInTournament = candidate;
                }
            }
            
            newPopulation.push({...bestInTournament});
        }
        
        this.population = newPopulation;
    }

    // Crossover operation
    crossover() {
        for (let i = 1; i < this.population.length; i += 2) {
            if (Math.random() < 0.8) { // 80% crossover probability
                const parent1 = this.population[i];
                const parent2 = this.population[i + 1] || this.population[0];
                
                // Single-point crossover
                const crossoverPoint = Math.floor(Math.random() * 3);
                
                switch (crossoverPoint) {
                    case 0:
                        // Swap current setpoints
                        [parent1.currentSetpoint, parent2.currentSetpoint] = 
                        [parent2.currentSetpoint, parent1.currentSetpoint];
                        break;
                    case 1:
                        // Swap power ratios
                        [parent1.gridRatio, parent2.gridRatio] = 
                        [parent2.gridRatio, parent1.gridRatio];
                        parent1.pvRatio = 1 - parent1.gridRatio;
                        parent2.pvRatio = 1 - parent2.gridRatio;
                        break;
                    case 2:
                        // Swap strategy parameters
                        [parent1.responseAggressiveness, parent2.responseAggressiveness] = 
                        [parent2.responseAggressiveness, parent1.responseAggressiveness];
                        break;
                }
            }
        }
    }

    // Mutation operation
    mutation() {
        const mutationRate = 0.1;
        
        this.population.forEach(solution => {
            if (Math.random() < mutationRate) {
                // Mutate current setpoint
                solution.currentSetpoint += (Math.random() - 0.5) * 20;
                solution.currentSetpoint = Math.max(50, Math.min(150, solution.currentSetpoint));
            }
            
            if (Math.random() < mutationRate) {
                // Mutate power ratios
                solution.gridRatio += (Math.random() - 0.5) * 0.2;
                solution.gridRatio = Math.max(0, Math.min(1, solution.gridRatio));
                solution.pvRatio = 1 - solution.gridRatio;
            }
            
            if (Math.random() < mutationRate) {
                // Mutate strategy parameters
                solution.responseAggressiveness += (Math.random() - 0.5) * 0.1;
                solution.responseAggressiveness = Math.max(0.5, Math.min(1.0, solution.responseAggressiveness));
            }
        });
    }

    // Update best solution
    updateBestSolution() {
        let best = this.population[0];
        
        this.population.forEach(solution => {
            if (solution.fitness > best.fitness) {
                best = solution;
            }
        });
        
        this.bestSolution = {...best};
    }

    // Get average fitness
    getAverageFitness() {
        const total = this.population.reduce((sum, solution) => sum + solution.fitness, 0);
        return total / this.population.length;
    }

    // Update optimization weights based on strategy
    updateWeights(strategy) {
        const strategyWeights = {
            economic: { economic: 0.5, reliability: 0.2, efficiency: 0.15, sustainability: 0.1, safety: 0.05 },
            reliability: { economic: 0.2, reliability: 0.5, efficiency: 0.15, sustainability: 0.1, safety: 0.05 },
            efficiency: { economic: 0.25, reliability: 0.15, efficiency: 0.4, sustainability: 0.15, safety: 0.05 }
        };
        
        if (strategyWeights[strategy]) {
            this.weights = strategyWeights[strategy];
        }
    }

    // Generate control decision for system
    generateControlDecision(currentState) {
        const optimizedSolution = this.optimize(currentState, 'economic');
        const kenyaData = this.kenyaData.getCurrentData();
        
        const decision = {
            timestamp: new Date(),
            optimalCurrent: Math.round(optimizedSolution.currentSetpoint),
            gridRatio: optimizedSolution.gridRatio,
            pvRatio: optimizedSolution.pvRatio,
            expectedProduction: optimizedSolution.productionRate,
            fitness: optimizedSolution.fitness,
            generation: this.generation,
            reasoning: this.explainDecision(optimizedSolution, kenyaData),
            confidence: this.calculateConfidence(optimizedSolution)
        };
        
        console.log('🎯 HENMPC Decision:', decision);
        return decision;
    }

    // Explain the decision for transparency
    explainDecision(solution, kenyaData) {
        const reasons = [];
        const demand = kenyaData.hospital.demand;
        const tariff = kenyaData.energy.tariff.rate;
        const solar = kenyaData.energy.solar;
        
        // Economic reasoning
        if (tariff > 25 && solution.pvRatio > 0.6) {
            reasons.push("High electricity tariffs - maximizing solar utilization");
        }
        
        if (demand > 120 && solution.currentSetpoint > 130) {
            reasons.push("High hospital demand - increased production");
        }
        
        // Reliability reasoning
        if (solution.gridRatio < 0.3 && kenyaData.energy.grid.reliability < 0.9) {
            reasons.push("Grid instability - minimizing grid dependency");
        }
        
        // Efficiency reasoning
        if (solar > 2 && solution.pvRatio > 0.7) {
            reasons.push("Good solar conditions - prioritizing renewable energy");
        }
        
        return reasons.length > 0 ? reasons : ["Optimal economic operation maintained"];
    }

    // Calculate decision confidence
    calculateConfidence(solution) {
        let confidence = 0.7; // Base confidence
        
        // Increase confidence for stable solutions
        if (this.performanceHistory.length > 5) {
            const recentImprovement = this.performanceHistory.slice(-5);
            const avgImprovement = recentImprovement.reduce((sum, perf) => sum + perf.bestFitness, 0) / 5;
            
            if (avgImprovement > solution.fitness * 0.95) {
                confidence += 0.2;
            }
        }
        
        // Decrease confidence for constraint violations
        if (solution.constraintsViolated > 0) {
            confidence -= 0.1 * solution.constraintsViolated;
        }
        
        return Math.max(0.3, Math.min(0.95, confidence));
    }

    // Get performance statistics
    getPerformanceStats() {
        return {
            totalGenerations: this.generation,
            currentBestFitness: this.bestSolution ? this.bestSolution.fitness : 0,
            averageFitness: this.getAverageFitness(),
            convergence: this.calculateConvergence(),
            computationTime: this.performanceHistory.length > 0 ? 
                this.performanceHistory[this.performanceHistory.length - 1].computationTime : 0
        };
    }

    // Calculate convergence metric
    calculateConvergence() {
        if (this.performanceHistory.length < 10) return 0;
        
        const recent = this.performanceHistory.slice(-10);
        const improvements = recent.filter((perf, i) => i > 0 && perf.bestFitness > recent[i-1].bestFitness);
        
        return improvements.length / 9; // 9 possible improvements in 10 generations
    }
}

// Create global instance
const henmpc = new HENMPC();

// Export for use in other modules
if (typeof module !== 'undefined' && module.exports) {
    module.exports = HENMPC;
}

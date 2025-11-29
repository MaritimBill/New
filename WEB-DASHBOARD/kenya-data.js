// Kenya Real Data Integration for PEM Electrolyzer System
class KenyaRealData {
    constructor() {
        this.hospitalData = this.initializeHospitalData();
        this.solarData = this.initializeSolarData();
        this.tariffData = this.initializeTariffData();
        this.weatherData = this.initializeWeatherData();
        this.gridData = this.initializeGridData();
        this.cache = new Map();
        this.updateInterval = 30000; // 30 seconds
        this.lastUpdate = 0;
    }

    // Initialize KNH hospital data patterns
    initializeHospitalData() {
        return {
            // Based on KNH 1800-bed capacity and WHO guidelines
            baseConsumption: {
                icu: 8,        // L/min per ICU patient
                ward: 2,       // L/min per ward patient  
                surgery: 15,   // L/min per surgery
                emergency: 25  // L/min per emergency case
            },
            
            // Patient distribution (estimated)
            patientDistribution: {
                icu: 100,
                ward: 1500,
                surgery: 20,    // per hour
                emergency: 10   // per hour
            },
            
            // 24-hour demand pattern (KNH typical)
            hourlyPattern: [
                0.4, 0.3, 0.3, 0.3, 0.3, 0.4,  // 00:00-05:00
                0.6, 0.8, 1.0, 1.1, 1.2, 1.1,  // 06:00-11:00
                1.0, 0.9, 0.8, 0.9, 1.0, 1.2,  // 12:00-17:00
                1.1, 0.9, 0.7, 0.6, 0.5, 0.4   // 18:00-23:00
            ],
            
            // Seasonal variations in Kenya
            seasonalMultipliers: {
                'Jan': 1.0, 'Feb': 1.0, 'Mar': 1.1,
                'Apr': 1.2, 'May': 1.3, 'Jun': 1.1,
                'Jul': 1.0, 'Aug': 1.0, 'Sep': 1.0,
                'Oct': 0.9, 'Nov': 0.9, 'Dec': 0.95
            },
            
            // Day of week patterns
            dayOfWeekMultipliers: {
                'Monday': 1.15, 'Tuesday': 1.10, 'Wednesday': 1.05,
                'Thursday': 1.05, 'Friday': 1.00, 'Saturday': 0.85, 'Sunday': 0.75
            }
        };
    }

    // Initialize Nairobi solar data
    initializeSolarData() {
        return {
            // NASA POWER data for Nairobi (kWh/m²/day)
            monthlyIrradiance: {
                'Jan': 6.2, 'Feb': 6.5, 'Mar': 6.1,
                'Apr': 5.2, 'May': 4.8, 'Jun': 4.9,
                'Jul': 4.7, 'Aug': 5.1, 'Sep': 5.8,
                'Oct': 5.9, 'Nov': 5.7, 'Dec': 6.0
            },
            
            // Typical daily profile (clear day)
            dailyProfile: [
                0.00, 0.00, 0.00, 0.00, 0.00, 0.00,  // 00:00-05:00
                0.05, 0.15, 0.35, 0.60, 0.80, 0.95,  // 06:00-11:00
                1.00, 0.95, 0.85, 0.70, 0.50, 0.30,  // 12:00-17:00
                0.10, 0.00, 0.00, 0.00, 0.00, 0.00   // 18:00-23:00
            ],
            
            // Weather impact factors
            weatherImpact: {
                'clear': 1.0,
                'partly_cloudy': 0.6,
                'cloudy': 0.3,
                'rainy': 0.1,
                'stormy': 0.05
            },
            
            // System configuration (20kW commercial array)
            systemConfig: {
                capacity: 20,      // kW
                efficiency: 0.18,  // 18%
                degradation: 0.005 // 0.5% per year
            }
        };
    }

    // Initialize KPLC tariff data (2024 Commercial rates)
    initializeTariffData() {
        return {
            commercial: {
                off_peak: {
                    rate: 15.80,
                    hours: [0, 1, 2, 3, 4, 5, 22, 23], // 22:00-06:00
                    description: "Night Rate"
                },
                standard: {
                    rate: 25.20,
                    hours: [6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17], // 06:00-18:00
                    description: "Day Rate"
                },
                peak: {
                    rate: 31.60,
                    hours: [18, 19, 20, 21], // 18:00-22:00
                    description: "Evening Peak"
                }
            },
            
            // Additional charges
            fixedCharges: {
                demandCharge: 450,    // KSh/kVA/month
                serviceCharge: 1200,  // KSh/month
                fuelCostAdjustment: 2.50 // KSh/kWh (typical)
            },
            
            // Time blocks for easy reference
            timeBlocks: {
                off_peak: [0, 1, 2, 3, 4, 5, 22, 23],
                standard: [6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17],
                peak: [18, 19, 20, 21]
            }
        };
    }

    // Initialize weather patterns for Nairobi
    initializeWeatherData() {
        return {
            nairobiClimate: {
                // Average temperatures (°C)
                temperatures: {
                    'Jan': 25, 'Feb': 26, 'Mar': 25,
                    'Apr': 24, 'May': 23, 'Jun': 22,
                    'Jul': 21, 'Aug': 21, 'Sep': 23,
                    'Oct': 24, 'Nov': 23, 'Dec': 23
                },
                
                // Rainfall patterns (mm/month) - Kenya Met Department
                rainfall: {
                    'Jan': 64,  'Feb': 56,  'Mar': 92,
                    'Apr': 219, 'May': 176, 'Jun': 35,
                    'Jul': 18,  'Aug': 24,  'Sep': 28,
                    'Oct': 55,  'Nov': 154, 'Dec': 107
                },
                
                // Typical weather conditions by month
                weatherPatterns: {
                    'Jan': 'clear', 'Feb': 'clear', 'Mar': 'partly_cloudy',
                    'Apr': 'rainy', 'May': 'rainy', 'Jun': 'partly_cloudy',
                    'Jul': 'clear', 'Aug': 'clear', 'Sep': 'partly_cloudy',
                    'Oct': 'partly_cloudy', 'Nov': 'rainy', 'Dec': 'rainy'
                }
            },
            
            // Current weather simulation
            currentWeather: this.simulateCurrentWeather()
        };
    }

    // Initialize Kenya grid reliability data
    initializeGridData() {
        return {
            // Kenya Power reliability statistics
            reliabilityByRegion: {
                'nairobi': 0.92,
                'central': 0.85,
                'coast': 0.78,
                'western': 0.82,
                'rural': 0.65
            },
            
            // Typical outage patterns
            outagePatterns: {
                'load_shedding': {
                    probability: 0.08,
                    hours: [18, 19, 20, 21], // Evening peak
                    duration: 2 // hours
                },
                'maintenance': {
                    probability: 0.03,
                    hours: [10, 11, 12, 13], // Mid-day
                    duration: 4
                },
                'random': {
                    probability: 0.02,
                    hours: [14, 15, 16, 22, 23], // Various
                    duration: 1
                }
            },
            
            // Grid frequency stability
            frequencyStability: {
                normalRange: [49.8, 50.2],
                typicalVariation: 0.3,
                emergencyThreshold: 49.5
            }
        };
    }

    // Get current hospital oxygen demand (L/min)
    getCurrentHospitalDemand() {
        const now = new Date();
        const hour = now.getHours();
        const day = now.toLocaleString('en-us', { weekday: 'long' });
        const month = now.toLocaleString('en-us', { month: 'short' });
        
        // Calculate base demand from patient distribution
        let baseDemand = (this.hospitalData.patientDistribution.icu * this.hospitalData.baseConsumption.icu) +
                        (this.hospitalData.patientDistribution.ward * this.hospitalData.baseConsumption.ward) +
                        (this.hospitalData.patientDistribution.surgery * this.hospitalData.baseConsumption.surgery) +
                        (this.hospitalData.patientDistribution.emergency * this.hospitalData.baseConsumption.emergency);
        
        // Apply hourly pattern
        const hourlyFactor = this.hospitalData.hourlyPattern[hour];
        baseDemand *= hourlyFactor;
        
        // Apply day of week adjustment
        const dayFactor = this.hospitalData.dayOfWeekMultipliers[day] || 1.0;
        baseDemand *= dayFactor;
        
        // Apply seasonal adjustment
        const seasonalFactor = this.hospitalData.seasonalMultipliers[month] || 1.0;
        baseDemand *= seasonalFactor;
        
        // Add random variation (±20%)
        const randomVariation = 0.8 + (Math.random() * 0.4);
        baseDemand *= randomVariation;
        
        // Apply emergency scenarios
        baseDemand = this.applyEmergencyScenarios(baseDemand);
        
        return Math.max(20, Math.round(baseDemand));
    }

    // Apply emergency scenarios to demand
    applyEmergencyScenarios(baseDemand) {
        const scenarios = [
            { 
                name: 'normal', 
                probability: 0.80, 
                multiplier: 1.0,
                description: "Normal operation"
            },
            { 
                name: 'busy_day', 
                probability: 0.15, 
                multiplier: 1.4,
                description: "Increased admissions"
            },
            { 
                name: 'emergency', 
                probability: 0.04, 
                multiplier: 2.0,
                description: "Mass casualty event"
            },
            { 
                name: 'crisis', 
                probability: 0.01, 
                multiplier: 3.0,
                description: "Pandemic surge"
            }
        ];
        
        const rand = Math.random();
        let cumulativeProb = 0;
        
        for (const scenario of scenarios) {
            cumulativeProb += scenario.probability;
            if (rand <= cumulativeProb) {
                console.log(`🏥 Hospital scenario: ${scenario.description}`);
                return baseDemand * scenario.multiplier;
            }
        }
        
        return baseDemand;
    }

    // Get current electricity tariff (KSh/kWh)
    getCurrentTariff() {
        const now = new Date();
        const hour = now.getHours();
        
        const tariffs = this.tariffData.commercial;
        
        if (this.tariffData.timeBlocks.off_peak.includes(hour)) {
            return {
                rate: tariffs.off_peak.rate,
                period: 'off_peak',
                description: tariffs.off_peak.description
            };
        } else if (this.tariffData.timeBlocks.peak.includes(hour)) {
            return {
                rate: tariffs.peak.rate,
                period: 'peak',
                description: tariffs.peak.description
            };
        } else {
            return {
                rate: tariffs.standard.rate,
                period: 'standard',
                description: tariffs.standard.description
            };
        }
    }

    // Get current solar power generation (kW)
    getCurrentSolarPower() {
        const now = new Date();
        const hour = now.getHours();
        const month = now.toLocaleString('en-us', { month: 'short' });
        
        // Get base solar profile
        const dailyProfile = this.solarData.dailyProfile[hour] || 0;
        
        // Apply monthly irradiance factor
        const monthlyIrradiance = this.solarData.monthlyIrradiance[month] || 5.5;
        const monthlyFactor = monthlyIrradiance / 6.0; // Normalize to average
        
        // Apply weather impact
        const weatherImpact = this.getWeatherImpact();
        
        // Calculate actual power generation
        const systemCapacity = this.solarData.systemConfig.capacity;
        const efficiency = this.solarData.systemConfig.efficiency;
        
        let solarPower = dailyProfile * monthlyFactor * weatherImpact * systemCapacity * efficiency;
        
        // Ensure non-negative value
        return Math.max(0, solarPower);
    }

    // Get current weather impact on solar
    getWeatherImpact() {
        const month = new Date().toLocaleString('en-us', { month: 'short' });
        const weatherType = this.weatherData.nairobiClimate.weatherPatterns[month] || 'partly_cloudy';
        
        return this.solarData.weatherImpact[weatherType] || 0.7;
    }

    // Simulate current weather conditions
    simulateCurrentWeather() {
        const conditions = ['clear', 'partly_cloudy', 'cloudy', 'rainy', 'stormy'];
        const weights = [0.4, 0.3, 0.15, 0.1, 0.05]; // Probability weights
        
        let random = Math.random();
        let cumulativeWeight = 0;
        
        for (let i = 0; i < conditions.length; i++) {
            cumulativeWeight += weights[i];
            if (random <= cumulativeWeight) {
                return conditions[i];
            }
        }
        
        return 'partly_cloudy';
    }

    // Get grid status and reliability
    getGridStatus() {
        const now = new Date();
        const hour = now.getHours();
        
        // Base reliability for Nairobi
        let reliability = this.gridData.reliabilityByRegion.nairobi;
        
        // Time-based reliability adjustments
        if (this.gridData.outagePatterns.load_shedding.hours.includes(hour)) {
            reliability *= (1 - this.gridData.outagePatterns.load_shedding.probability);
        }
        
        if (this.gridData.outagePatterns.maintenance.hours.includes(hour)) {
            reliability *= (1 - this.gridData.outagePatterns.maintenance.probability);
        }
        
        // Random outage probability
        const outageProbability = (1 - reliability);
        const isGridAvailable = Math.random() > outageProbability;
        
        return {
            available: isGridAvailable,
            reliability: reliability,
            expectedOutages: this.calculateExpectedOutages(hour),
            frequency: this.simulateGridFrequency()
        };
    }

    // Calculate expected outages for planning
    calculateExpectedOutages(hour) {
        let totalProbability = 0;
        
        for (const [type, pattern] of Object.entries(this.gridData.outagePatterns)) {
            if (pattern.hours.includes(hour)) {
                totalProbability += pattern.probability;
            }
        }
        
        return totalProbability;
    }

    // Simulate grid frequency variations
    simulateGridFrequency() {
        const baseFrequency = 50.0; // Hz
        const variation = (Math.random() - 0.5) * 2 * this.gridData.frequencyStability.typicalVariation;
        return baseFrequency + variation;
    }

    // Get economic parameters for MPC
    getEconomicParameters() {
        return {
            // Revenue streams (KSh)
            oxygenPrice: 150,    // Medical grade O₂ per liter
            hydrogenPrice: 40,   // H₂ byproduct per liter
            carbonCredits: 5,    // KSh per kg CO2 avoided
            
            // Operational costs (KSh)
            waterCost: 0.02,     // Per liter of water
            maintenanceCost: 0.05, // Per liter O₂ produced
            membraneCost: 0.10,  // Per liter O₂ (amortized)
            laborCost: 0.15,     // Per liter O₂
            
            // System parameters
            electricalEfficiency: 4.5,    // kWh per Nm³ O₂
            waterEfficiency: 0.8,         // Liters water per liter O₂
            systemLifetime: 10,           // Years
            degradationRate: 0.001        // Per operating hour
        };
    }

    // Generate forecast data for MPC (next 24 hours)
    generateForecast(hours = 24) {
        const forecast = [];
        const now = new Date();
        
        for (let i = 0; i < hours; i++) {
            const forecastTime = new Date(now.getTime() + i * 60 * 60 * 1000);
            const hour = forecastTime.getHours();
            const month = forecastTime.toLocaleString('en-us', { month: 'short' });
            
            forecast.push({
                timestamp: forecastTime,
                hour: hour,
                hospitalDemand: this.forecastHospitalDemand(hour, month),
                solarPower: this.forecastSolarPower(hour, month),
                electricityTariff: this.forecastTariff(hour),
                gridReliability: this.forecastGridReliability(hour),
                weather: this.forecastWeather(month)
            });
        }
        
        return forecast;
    }

    // Forecast hospital demand
    forecastHospitalDemand(hour, month) {
        const base = 75; // L/min average
        const hourlyFactor = this.hospitalData.hourlyPattern[hour];
        const seasonalFactor = this.hospitalData.seasonalMultipliers[month] || 1.0;
        
        return base * hourlyFactor * seasonalFactor * (0.9 + Math.random() * 0.2);
    }

    // Forecast solar power
    forecastSolarPower(hour, month) {
        const profile = this.solarData.dailyProfile[hour] || 0;
        const monthlyIrradiance = this.solarData.monthlyIrradiance[month] || 5.5;
        const monthlyFactor = monthlyIrradiance / 6.0;
        
        return profile * monthlyFactor * this.solarData.systemConfig.capacity * this.solarData.systemConfig.efficiency;
    }

    // Forecast electricity tariff
    forecastTariff(hour) {
        const tariff = this.getCurrentTariff();
        return tariff.rate; // Simplified - same structure throughout day
    }

    // Forecast grid reliability
    forecastGridReliability(hour) {
        let reliability = this.gridData.reliabilityByRegion.nairobi;
        
        // Reduce reliability during peak hours
        if (this.gridData.outagePatterns.load_shedding.hours.includes(hour)) {
            reliability *= 0.9;
        }
        
        return reliability;
    }

    // Forecast weather
    forecastWeather(month) {
        return this.weatherData.nairobiClimate.weatherPatterns[month] || 'partly_cloudy';
    }

    // Get all current data for dashboard
    getCurrentData() {
        const now = Date.now();
        
        // Cache data for 30 seconds to avoid rapid recalculations
        if (this.cache.has('currentData') && (now - this.lastUpdate) < this.updateInterval) {
            return this.cache.get('currentData');
        }
        
        const currentData = {
            timestamp: new Date(),
            hospital: {
                demand: this.getCurrentHospitalDemand(),
                scenario: this.getCurrentScenario()
            },
            energy: {
                solar: this.getCurrentSolarPower(),
                tariff: this.getCurrentTariff(),
                grid: this.getGridStatus()
            },
            economic: this.getEconomicParameters(),
            forecast: this.generateForecast(12) // 12-hour forecast
        };
        
        this.cache.set('currentData', currentData);
        this.lastUpdate = now;
        
        return currentData;
    }

    // Get current operational scenario
    getCurrentScenario() {
        const demand = this.getCurrentHospitalDemand();
        
        if (demand > 200) return 'crisis';
        if (demand > 150) return 'emergency';
        if (demand > 120) return 'busy';
        return 'normal';
    }

    // Calculate optimal production based on current conditions
    calculateOptimalProduction() {
        const data = this.getCurrentData();
        const tariff = data.energy.tariff.rate;
        const solar = data.energy.solar;
        const demand = data.hospital.demand;
        
        // Simple economic optimization
        const productionCost = this.calculateProductionCost(tariff, solar);
        const revenue = demand * this.getEconomicParameters().oxygenPrice;
        
        return {
            optimalCurrent: Math.min(150, Math.max(80, demand * 0.8)), // Simple scaling
            productionCost: productionCost,
            expectedRevenue: revenue,
            profitability: revenue - productionCost,
            recommendedAction: this.getRecommendedAction(productionCost, revenue)
        };
    }

    // Calculate production cost
    calculateProductionCost(tariff, solar) {
        const params = this.getEconomicParameters();
        const baseCost = params.waterCost + params.maintenanceCost + params.membraneCost + params.laborCost;
        
        // Adjust for electricity cost based on solar availability
        const gridPowerRatio = Math.max(0, 1 - (solar / 3.6)); // 3.6kW needed for max production
        const electricityCost = gridPowerRatio * tariff * params.electricalEfficiency / 1000;
        
        return baseCost + electricityCost;
    }

    // Get recommended operational action
    getRecommendedAction(cost, revenue) {
        const profitMargin = (revenue - cost) / revenue;
        
        if (profitMargin > 0.3) return 'maximize_production';
        if (profitMargin > 0.1) return 'maintain_production';
        if (profitMargin > 0) return 'reduce_production';
        return 'consider_shutdown';
    }
}

// Create global instance
const kenyaData = new KenyaRealData();

// Export for use in other modules
if (typeof module !== 'undefined' && module.exports) {
    module.exports = KenyaRealData;
}

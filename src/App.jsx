import React, { useState, useEffect, useRef } from 'react';
import { Battery, Wifi, Navigation, Package, Zap, Settings, Play, Square, Camera, Radio, PlayCircle, PauseCircle } from 'lucide-react';

export default function DroneControlUI() {
  const [armed, setArmed] = useState(false);
  const [flying, setFlying] = useState(false);
  const [shapeMode, setShapeMode] = useState('standard');
  const [graspMode, setGraspMode] = useState(false);
  const [battery, setBattery] = useState(100);
  const [altitude, setAltitude] = useState(0);
  const [speed, setSpeed] = useState(0);
  const [signalStrength, setSignalStrength] = useState(98);
  const [morphProgress, setMorphProgress] = useState(0);
  const [deliveryStatus, setDeliveryStatus] = useState('ready');
  
  // Advanced Flight Parameters (Real Drone Telemetry)
  const [throttle, setThrottle] = useState(0); // 0-100%
  const [pitch, setPitch] = useState(0); // -45 to +45 degrees
  const [roll, setRoll] = useState(0); // -45 to +45 degrees
  const [yaw, setYaw] = useState(0); // 0-360 degrees
  const [heading, setHeading] = useState(0); // Compass heading 0-360
  const [verticalSpeed, setVerticalSpeed] = useState(0); // m/s
  const [groundSpeed, setGroundSpeed] = useState(0); // m/s
  const [gpsCoordinates, setGpsCoordinates] = useState({ lat: 37.7749, lon: -122.4194 }); // San Francisco base
  const [gpsAccuracy, setGpsAccuracy] = useState(1.5); // meters
  const [gpsSatellites, setGpsSatellites] = useState(12);
  
  // Environmental Sensors
  const [windSpeed, setWindSpeed] = useState(2.3); // m/s
  const [windDirection, setWindDirection] = useState(135); // degrees
  const [temperature, setTemperature] = useState(22); // Celsius
  const [pressure, setPressure] = useState(1013.25); // hPa
  const [humidity, setHumidity] = useState(65); // %
  
  // Motor & Power Telemetry
  const [motorRPM, setMotorRPM] = useState([0, 0, 0, 0]); // RPM for each of 4 motors
  const [motorTemp, setMotorTemp] = useState([25, 25, 25, 25]); // Celsius
  const [voltage, setVoltage] = useState(16.8); // Volts (4S LiPo)
  const [current, setCurrent] = useState(0); // Amperes
  const [powerConsumption, setPowerConsumption] = useState(0); // Watts
  const [batteryCell, setBatteryCell] = useState([4.2, 4.2, 4.2, 4.2]); // Individual cell voltages
  
  // IMU Sensors (Inertial Measurement Unit)
  const [acceleration, setAcceleration] = useState({ x: 0, y: 0, z: 9.81 }); // m/s²
  const [gyroscope, setGyroscope] = useState({ x: 0, y: 0, z: 0 }); // rad/s
  const [magnetometer, setMagnetometer] = useState({ x: 0, y: 0, z: 0 }); // μT
  
  // Flight Controller Stats
  const [cpuLoad, setCpuLoad] = useState(15); // %
  const [loopTime, setLoopTime] = useState(250); // microseconds
  const [flightMode, setFlightMode] = useState('MANUAL'); // MANUAL, AUTO, RTH, STABILIZE
  const [failsafeStatus, setFailsafeStatus] = useState('OK');
  
  // Physics and movement
  const [dronePosition, setDronePosition] = useState({ x: 10, y: 90 });
  const [velocity, setVelocity] = useState({ x: 0, y: 0 });
  const [targetPosition, setTargetPosition] = useState({ x: 90, y: 20 });
  const [packagePosition] = useState({ x: 30, y: 40 });
  const [packageGrabbed, setPackageGrabbed] = useState(false);
  const [rotorRotation, setRotorRotation] = useState(0);
  const [totalFlightTime, setTotalFlightTime] = useState(0); // seconds
  const [maxAltitudeReached, setMaxAltitudeReached] = useState(0);
  
  // Simulation states
  const [autoSimulation, setAutoSimulation] = useState(false);
  const [simulationStep, setSimulationStep] = useState(0);
  const [missionLog, setMissionLog] = useState([]);
  
  const animationFrameRef = useRef(null);
  const [maxSpeed, setMaxSpeed] = useState(25);

  // Add log entry
  const addLog = (message) => {
    const timestamp = new Date().toLocaleTimeString();
    setMissionLog(prev => [...prev.slice(-5), `${timestamp}: ${message}`]);
  };

  // Shape mode effects on drone performance
  const getShapeMultipliers = () => {
    switch(shapeMode) {
      case 'wide-grasp': return { speed: 0.7, efficiency: 0.8, grasp: 2.0 };
      case 'precision': return { speed: 0.85, efficiency: 0.95, grasp: 1.5 };
      case 'compact': return { speed: 1.3, efficiency: 1.1, grasp: 0.5 };
      default: return { speed: 1.0, efficiency: 1.0, grasp: 1.0 };
    }
  };

  // Calculate distance between two points
  const calculateDistance = (p1, p2) => {
    return Math.sqrt(Math.pow(p2.x - p1.x, 2) + Math.pow(p2.y - p1.y, 2));
  };

  // Advanced Physics simulation - runs continuously when flying
  useEffect(() => {
    if (!flying) {
      setAltitude(0);
      setSpeed(0);
      setGroundSpeed(0);
      setVerticalSpeed(0);
      setVelocity({ x: 0, y: 0 });
      setThrottle(0);
      setPitch(0);
      setRoll(0);
      setMotorRPM([0, 0, 0, 0]);
      setCurrent(0);
      setPowerConsumption(0);
      setTotalFlightTime(0);
      return;
    }

    const physicsLoop = setInterval(() => {
      const multipliers = getShapeMultipliers();
      
      // Flight time tracking
      setTotalFlightTime(prev => prev + 0.1);
      
      // Rotor animation - speed based on throttle
      const rotorSpeed = 10 + (throttle / 100) * 40;
      setRotorRotation(prev => (prev + rotorSpeed) % 360);
      
      // Motor RPM calculation (realistic quadcopter range: 0-15000 RPM)
      const baseRPM = armed ? 3000 : 0;
      const flightRPM = flying ? baseRPM + (throttle / 100) * 12000 : baseRPM;
      const windEffect = windSpeed * 50;
      setMotorRPM([
        Math.round(flightRPM + Math.random() * 200 + windEffect),
        Math.round(flightRPM + Math.random() * 200 - windEffect * 0.5),
        Math.round(flightRPM + Math.random() * 200 + windEffect * 0.3),
        Math.round(flightRPM + Math.random() * 200 - windEffect * 0.2)
      ]);
      
      // Motor temperature (increases with use, affected by shape mode)
      setMotorTemp(prev => prev.map((temp, i) => {
        const targetTemp = 25 + (motorRPM[i] / 15000) * 45 + (1 / multipliers.efficiency) * 10;
        return Math.min(temp + (targetTemp - temp) * 0.05, 85); // Max 85°C
      }));
      
      // Throttle calculation based on altitude target
      const targetAlt = 50;
      const altError = targetAlt - altitude;
      const newThrottle = Math.max(0, Math.min(100, 50 + altError * 2));
      setThrottle(newThrottle);
      
      // Altitude physics with vertical speed
      const climbRate = (newThrottle - 50) / 10; // m/s
      setVerticalSpeed(climbRate);
      setAltitude(prev => {
        const newAlt = Math.max(0, Math.min(prev + climbRate * 0.1, 150));
        setMaxAltitudeReached(max => Math.max(max, newAlt));
        return newAlt;
      });
      
      // Calculate speeds
      const currentSpeed = Math.sqrt(velocity.x * velocity.x + velocity.y * velocity.y) * 10 * multipliers.speed;
      setSpeed(currentSpeed);
      setGroundSpeed(currentSpeed);
      
      // Pitch and Roll based on movement direction
      setPitch(-velocity.y * 5); // Forward/backward tilt
      setRoll(velocity.x * 5); // Left/right tilt
      
      // Yaw and Heading updates
      if (velocity.x !== 0 || velocity.y !== 0) {
        const targetHeading = (Math.atan2(velocity.x, -velocity.y) * 180 / Math.PI + 360) % 360;
        setHeading(prev => {
          const diff = (targetHeading - prev + 540) % 360 - 180;
          return (prev + diff * 0.1 + 360) % 360;
        });
        setYaw(heading);
      }
      
      // GPS simulation (convert position to lat/lon delta)
      const basePos = { x: 10, y: 90 };
      const deltaX = (dronePosition.x - basePos.x) * 0.0001; // ~11m per 0.0001 degree
      const deltaY = -(dronePosition.y - basePos.y) * 0.0001;
      setGpsCoordinates({
        lat: 37.7749 + deltaY,
        lon: -122.4194 + deltaX
      });
      setGpsAccuracy(1.2 + Math.random() * 0.6); // 1.2-1.8m typical accuracy
      setGpsSatellites(Math.max(8, Math.min(16, 12 + Math.floor(Math.random() * 5) - 2)));
      
      // Power consumption (realistic for 1-2kg drone)
      const baseCurrent = armed ? 2.5 : 0.1; // Amperes
      const flightCurrent = flying ? baseCurrent + (throttle / 100) * 25 : baseCurrent;
      const windCurrent = (windSpeed / 10) * 2;
      const shapeCurrent = (1 / multipliers.efficiency) * 3;
      const totalCurrent = flightCurrent + windCurrent + shapeCurrent + (graspMode ? 1.5 : 0);
      setCurrent(totalCurrent);
      
      // Voltage drop under load (4S LiPo: 16.8V full, 14.0V empty)
      const voltageDropUnderLoad = totalCurrent * 0.05;
      const batteryVoltage = 14.0 + (battery / 100) * 2.8 - voltageDropUnderLoad;
      setVoltage(batteryVoltage);
      
      // Cell voltages (4S configuration)
      const avgCellVoltage = batteryVoltage / 4;
      setBatteryCell([
        avgCellVoltage + (Math.random() - 0.5) * 0.05,
        avgCellVoltage + (Math.random() - 0.5) * 0.05,
        avgCellVoltage + (Math.random() - 0.5) * 0.05,
        avgCellVoltage + (Math.random() - 0.5) * 0.05
      ]);
      
      // Power consumption in Watts
      setPowerConsumption(voltageDropUnderLoad > 0 ? batteryVoltage * totalCurrent : 0);
      
      // Battery drain (Amp-hours)
      const drainRate = totalCurrent / 36000; // Convert to % per 0.1 second
      setBattery(prev => Math.max(prev - drainRate, 0));
      
      // Signal degradation with distance and obstacles
      const distanceFromBase = calculateDistance(dronePosition, { x: 10, y: 90 });
      const signalLoss = Math.min(distanceFromBase * 0.12, 25);
      const altitudeLoss = Math.min(altitude * 0.05, 10);
      setSignalStrength(Math.max(100 - signalLoss - altitudeLoss, 40));
      
      // Environmental simulation
      setWindSpeed(prev => Math.max(0, prev + (Math.random() - 0.5) * 0.2));
      setWindDirection(prev => (prev + (Math.random() - 0.5) * 5 + 360) % 360);
      setTemperature(22 - altitude * 0.15); // Temperature drops with altitude
      setPressure(1013.25 * Math.pow(1 - altitude / 44330, 5.255)); // Barometric formula
      
      // IMU sensors
      setAcceleration({
        x: velocity.x * 0.5,
        y: velocity.y * 0.5,
        z: 9.81 + (flying ? verticalSpeed * 0.3 : 0)
      });
      
      setGyroscope({
        x: pitch * 0.0174533, // Convert to rad/s
        y: roll * 0.0174533,
        z: (yaw - heading) * 0.01
      });
      
      // Flight controller stats
      setCpuLoad(15 + (flying ? 25 : 0) + (graspMode ? 10 : 0) + Math.random() * 5);
      setLoopTime(250 + Math.random() * 50);
      
      // Failsafe monitoring
      if (battery < 20) setFailsafeStatus('LOW_BATTERY');
      else if (signalStrength < 50) setFailsafeStatus('WEAK_SIGNAL');
      else if (altitude > 120) setFailsafeStatus('MAX_ALT');
      else setFailsafeStatus('OK');
      
    }, 100);

    return () => clearInterval(physicsLoop);
  }, [flying, velocity, dronePosition, altitude, shapeMode, armed, throttle, motorRPM, windSpeed, heading, yaw, verticalSpeed, graspMode, battery, signalStrength]);

  // Auto simulation sequence with realistic movement
  useEffect(() => {
    if (!autoSimulation) return;

    const sequence = async () => {
      switch(simulationStep) {
        case 0:
          addLog('🚁 Initializing systems...');
          setTimeout(() => setSimulationStep(1), 1500);
          break;
        case 1:
          addLog('⚡ Arming motors...');
          setArmed(true);
          setTimeout(() => setSimulationStep(2), 1500);
          break;
        case 2:
          addLog('🚀 Taking off...');
          setFlying(true);
          setTimeout(() => setSimulationStep(3), 2500);
          break;
        case 3:
          addLog('🎯 Flying to package location...');
          moveToPosition(packagePosition);
          setTimeout(() => setSimulationStep(4), 3000);
          break;
        case 4:
          addLog('📍 Package location reached');
          setTimeout(() => setSimulationStep(5), 1500);
          break;
        case 5:
          addLog('🔄 Morphing to Wide-Grasp mode...');
          handleShapeChange('wide-grasp');
          setTimeout(() => setSimulationStep(6), 2500);
          break;
        case 6:
          addLog('📦 Grasping package...');
          setGraspMode(true);
          setPackageGrabbed(true);
          setDeliveryStatus('grasped');
          setTimeout(() => setSimulationStep(7), 2000);
          break;
        case 7:
          addLog('🔄 Morphing to Compact mode for speed...');
          handleShapeChange('compact');
          setTimeout(() => setSimulationStep(8), 2500);
          break;
        case 8:
          addLog('✈️ Flying to delivery location...');
          moveToPosition(targetPosition);
          setTimeout(() => setSimulationStep(9), 4000);
          break;
        case 9:
          addLog('📍 Delivery location reached');
          setTimeout(() => setSimulationStep(10), 1500);
          break;
        case 10:
          addLog('🔄 Morphing to Precision mode...');
          handleShapeChange('precision');
          setTimeout(() => setSimulationStep(11), 2500);
          break;
        case 11:
          addLog('📤 Releasing package...');
          setGraspMode(false);
          setPackageGrabbed(false);
          setDeliveryStatus('delivered');
          setTimeout(() => setSimulationStep(12), 2000);
          break;
        case 12:
          addLog('🏠 Returning to base...');
          moveToPosition({ x: 10, y: 90 });
          setTimeout(() => setSimulationStep(13), 3500);
          break;
        case 13:
          addLog('🔄 Morphing to Standard mode...');
          handleShapeChange('standard');
          setTimeout(() => setSimulationStep(14), 2500);
          break;
        case 14:
          addLog('🛬 Landing...');
          setFlying(false);
          setTimeout(() => setSimulationStep(15), 2000);
          break;
        case 15:
          addLog('✅ Mission completed successfully!');
          setArmed(false);
          setDeliveryStatus('ready');
          setTimeout(() => {
            setAutoSimulation(false);
            setSimulationStep(0);
          }, 2000);
          break;
      }
    };

    sequence();
  }, [autoSimulation, simulationStep]);

  // Movement animation
  const moveToPosition = (target) => {
    const startPos = { ...dronePosition };
    const distance = calculateDistance(startPos, target);
    const duration = (distance / getShapeMultipliers().speed) * 30;
    const steps = 60;
    let currentStep = 0;

    const moveInterval = setInterval(() => {
      currentStep++;
      const progress = currentStep / steps;
      const easeProgress = 1 - Math.pow(1 - progress, 3); // Ease out cubic
      
      setDronePosition({
        x: startPos.x + (target.x - startPos.x) * easeProgress,
        y: startPos.y + (target.y - startPos.y) * easeProgress
      });

      if (currentStep >= steps) {
        clearInterval(moveInterval);
        setDronePosition(target);
      }
    }, duration / steps);
  };

  // Manual controls
  const moveManually = (direction) => {
    if (!flying || autoSimulation) return;
    
    const multipliers = getShapeMultipliers();
    const moveSpeed = 3 * multipliers.speed;
    
    setDronePosition(prev => {
      let newPos = { ...prev };
      switch(direction) {
        case 'up': newPos.y = Math.max(prev.y - moveSpeed, 5); break;
        case 'down': newPos.y = Math.min(prev.y + moveSpeed, 95); break;
        case 'left': newPos.x = Math.max(prev.x - moveSpeed, 5); break;
        case 'right': newPos.x = Math.min(prev.x + moveSpeed, 95); break;
      }
      return newPos;
    });
    
    // Set velocity for speed calculation
    const directions = { up: [0, -1], down: [0, 1], left: [-1, 0], right: [1, 0] };
    const [dx, dy] = directions[direction];
    setVelocity({ x: dx * moveSpeed, y: dy * moveSpeed });
    
    // Clear velocity after movement
    setTimeout(() => setVelocity({ x: 0, y: 0 }), 200);
  };

  // Morph animation
  useEffect(() => {
    if (morphProgress > 0 && morphProgress < 100) {
      const timer = setTimeout(() => setMorphProgress(prev => Math.min(prev + 4, 100)), 40);
      return () => clearTimeout(timer);
    }
  }, [morphProgress]);

  const handleShapeChange = (mode) => {
    if (shapeMode === mode) return;
    addLog(`🔄 Morphing to ${mode.split('-').map(w => w.charAt(0).toUpperCase() + w.slice(1)).join(' ')} mode...`);
    setShapeMode(mode);
    setMorphProgress(1);
  };

  const toggleGrasp = () => {
    if (!flying) {
      addLog('⚠️ Must be flying to grasp objects');
      return;
    }
    
    const distToPackage = calculateDistance(dronePosition, packagePosition);
    
    if (!graspMode && distToPackage > 10) {
      addLog('⚠️ Too far from package! Distance: ' + distToPackage.toFixed(1) + ' units');
      return;
    }
    
    if (!graspMode && packageGrabbed) {
      addLog('⚠️ Package already grabbed!');
      return;
    }
    
    const newGraspState = !graspMode;
    setGraspMode(newGraspState);
    
    if (newGraspState && distToPackage <= 10) {
      setPackageGrabbed(true);
      setDeliveryStatus('grasped');
      addLog('📦 Package grasped successfully');
    } else if (!newGraspState) {
      setPackageGrabbed(false);
      setDeliveryStatus('released');
      addLog('� Package released');
    }
  };

  const startSimulation = () => {
    if (autoSimulation) return;
    addLog('🎬 Starting automated delivery mission...');
    addLog('📊 Initializing flight telemetry systems...');
    setAutoSimulation(true);
    setSimulationStep(0);
    setDeliveryStatus('ready');
    setPackageGrabbed(false);
    setDronePosition({ x: 10, y: 90 });
    setBattery(100);
    setFlightMode('AUTO');
    setTotalFlightTime(0);
    setMaxAltitudeReached(0);
  };

  const stopSimulation = () => {
    addLog('⏸️ Simulation stopped');
    addLog(`📊 Total flight time: ${totalFlightTime.toFixed(1)}s | Max altitude: ${maxAltitudeReached.toFixed(1)}m`);
    setAutoSimulation(false);
    setSimulationStep(0);
    setFlying(false);
    setArmed(false);
    setGraspMode(false);
    setPackageGrabbed(false);
    setVelocity({ x: 0, y: 0 });
    setFlightMode('MANUAL');
  };

  // Export telemetry data (for real-world testing)
  const exportTelemetry = () => {
    const telemetryData = {
      timestamp: new Date().toISOString(),
      position: { ...dronePosition, altitude },
      gps: { ...gpsCoordinates, accuracy: gpsAccuracy, satellites: gpsSatellites },
      orientation: { pitch, roll, yaw, heading },
      velocity: { ground: groundSpeed, vertical: verticalSpeed },
      power: { voltage, current, watts: powerConsumption, battery, cells: batteryCell },
      motors: { rpm: motorRPM, temperatures: motorTemp },
      environment: { wind: { speed: windSpeed, direction: windDirection }, temperature, pressure, humidity },
      sensors: { acceleration, gyroscope, magnetometer },
      flightController: { cpuLoad, loopTime, mode: flightMode, failsafe: failsafeStatus },
      stats: { flightTime: totalFlightTime, maxAltitude: maxAltitudeReached, throttle },
      shapeMode,
      graspMode,
      packageGrabbed
    };
    
    const dataStr = JSON.stringify(telemetryData, null, 2);
    const dataBlob = new Blob([dataStr], { type: 'application/json' });
    const url = URL.createObjectURL(dataBlob);
    const link = document.createElement('a');
    link.href = url;
    link.download = `drone-telemetry-${Date.now()}.json`;
    link.click();
    addLog('📥 Telemetry data exported');
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 via-purple-900 to-slate-900 p-4 font-sans">
      {/* Header */}
      <div className="max-w-7xl mx-auto">
        <div className="bg-black/40 backdrop-blur-xl border border-purple-500/30 rounded-2xl p-4 mb-4 shadow-2xl shadow-purple-500/20">
          <div className="flex items-center justify-between flex-wrap gap-4">
            <div className="flex items-center gap-3">
              <div className="w-12 h-12 bg-gradient-to-br from-purple-500 to-pink-500 rounded-xl flex items-center justify-center">
                <Radio className="text-white" size={24} />
              </div>
              <div>
                <h1 className="text-2xl font-bold text-white tracking-tight">ShapeShifter Drone</h1>
                <p className="text-purple-300 text-sm">Professional Flight Control System v3.0</p>
              </div>
            </div>
            <div className="flex gap-4 items-center flex-wrap">
              <button
                onClick={autoSimulation ? stopSimulation : startSimulation}
                className={`px-6 py-3 rounded-xl font-bold flex items-center gap-2 transition-all duration-300 ${
                  autoSimulation
                    ? 'bg-gradient-to-r from-red-500 to-orange-500 text-white shadow-lg shadow-red-500/50'
                    : 'bg-gradient-to-r from-green-500 to-emerald-500 text-white shadow-lg shadow-green-500/50'
                }`}
              >
                {autoSimulation ? <PauseCircle size={20} /> : <PlayCircle size={20} />}
                {autoSimulation ? 'STOP SIMULATION' : 'START AUTO SIMULATION'}
              </button>
              <button
                onClick={exportTelemetry}
                disabled={!flying && totalFlightTime === 0}
                className={`px-4 py-2 rounded-xl font-semibold flex items-center gap-2 transition-all duration-300 ${
                  flying || totalFlightTime > 0
                    ? 'bg-blue-500/20 text-blue-400 border border-blue-500/50 hover:bg-blue-500/30'
                    : 'bg-gray-500/20 text-gray-500 border border-gray-500/30 cursor-not-allowed'
                }`}
              >
                📥 Export Data
              </button>
              <div className={`flex items-center gap-2 px-3 py-1 rounded-full border ${
                signalStrength > 80 ? 'bg-green-500/20 border-green-500/50' : 
                signalStrength > 50 ? 'bg-yellow-500/20 border-yellow-500/50' : 
                'bg-red-500/20 border-red-500/50'
              }`}>
                <Wifi size={16} className={
                  signalStrength > 80 ? 'text-green-400' : 
                  signalStrength > 50 ? 'text-yellow-400' : 
                  'text-red-400'
                } />
                <span className={`text-sm font-semibold ${
                  signalStrength > 80 ? 'text-green-400' : 
                  signalStrength > 50 ? 'text-yellow-400' : 
                  'text-red-400'
                }`}>{signalStrength.toFixed(0)}%</span>
              </div>
              <div className={`flex items-center gap-2 px-3 py-1 rounded-full border ${
                battery > 50 ? 'bg-blue-500/20 border-blue-500/50' : 
                battery > 20 ? 'bg-yellow-500/20 border-yellow-500/50' : 
                'bg-red-500/20 border-red-500/50'
              }`}>
                <Battery size={16} className={
                  battery > 50 ? 'text-blue-400' : 
                  battery > 20 ? 'text-yellow-400' : 
                  'text-red-400'
                } />
                <span className={`text-sm font-semibold ${
                  battery > 50 ? 'text-blue-400' : 
                  battery > 20 ? 'text-yellow-400' : 
                  'text-red-400'
                }`}>{battery.toFixed(0)}%</span>
              </div>
              <div className="flex items-center gap-2 bg-purple-500/20 px-3 py-1 rounded-full border border-purple-500/50">
                <span className="text-purple-400 text-sm font-semibold">{flightMode}</span>
              </div>
            </div>
          </div>
        </div>

        {/* New Layout: Simulation on top, controls below */}
        <div className="grid grid-cols-1 gap-4">
          {/* Main Simulation View - Full Width */}
          <div className="bg-black/40 backdrop-blur-xl border border-purple-500/30 rounded-2xl p-6 shadow-2xl">
            <h2 className="text-xl font-bold text-white mb-4 flex items-center gap-2">
              <Camera className="text-blue-400" />
              Simulation View
            </h2>
            
            <div className="aspect-video bg-gradient-to-br from-slate-800 to-slate-900 rounded-xl flex items-center justify-center relative overflow-hidden border-2 border-purple-500/30">
              {/* Grid overlay */}
              <div className="absolute inset-0 opacity-20">
                {[...Array(10)].map((_, i) => (
                  <div key={`h${i}`} className="absolute w-full border-t border-purple-500" style={{ top: `${i * 10}%` }} />
                ))}
                {[...Array(10)].map((_, i) => (
                  <div key={`v${i}`} className="absolute h-full border-l border-purple-500" style={{ left: `${i * 10}%` }} />
                ))}
              </div>

              {/* Base station */}
              <div 
                className="absolute w-10 h-10 bg-purple-500/30 border-2 border-purple-400 rounded-lg flex items-center justify-center"
                style={{ 
                  left: '10%', 
                  top: '90%',
                  transform: 'translate(-50%, -50%)'
                }}
              >
                <div className="w-2 h-2 bg-purple-400 rounded-full animate-pulse" />
              </div>

              {/* Package pickup location */}
              <div 
                className="absolute w-8 h-8 bg-blue-500/50 border-2 border-blue-400 rounded flex items-center justify-center transition-all duration-500"
                style={{ 
                  left: `${packagePosition.x}%`, 
                  top: `${packagePosition.y}%`,
                  transform: 'translate(-50%, -50%)',
                  opacity: packageGrabbed ? 0.3 : 1
                }}
              >
                <Package size={16} className="text-blue-300" />
              </div>

              {/* Delivery target location */}
              <div 
                className="absolute w-12 h-12 border-2 border-dashed border-green-400 rounded-full flex items-center justify-center"
                style={{ 
                  left: `${targetPosition.x}%`, 
                  top: `${targetPosition.y}%`,
                  transform: 'translate(-50%, -50%)'
                }}
              >
                <div className="w-4 h-4 bg-green-400 rounded-full animate-pulse" />
              </div>

              {/* Drone visualization */}
              <div 
                className="absolute transition-all duration-300 ease-out"
                style={{ 
                  left: `${dronePosition.x}%`, 
                  top: `${dronePosition.y}%`,
                  transform: 'translate(-50%, -50%)'
                }}
              >
                <div className={`w-16 h-16 border-4 rounded-full flex items-center justify-center transition-all duration-500 ${
                  armed ? 'border-green-400 shadow-lg shadow-green-400/50' : 'border-purple-400'
                }`}>
                  <Radio size={24} className={flying ? 'text-green-400' : 'text-purple-400'} />
                </div>
                
                {/* Rotor indicators with realistic rotation */}
                {[0, 90, 180, 270].map((angle, idx) => (
                  <div
                    key={angle}
                    className={`absolute w-6 h-6 rounded-full transition-all duration-300 ${
                      flying ? 'bg-green-400 shadow-lg shadow-green-400/50' : 'bg-purple-400'
                    }`}
                    style={{
                      top: '50%',
                      left: '50%',
                      transform: `translate(-50%, -50%) rotate(${angle}deg) translateY(-30px) rotate(${flying ? rotorRotation + idx * 90 : 0}deg)`,
                      transition: flying ? 'transform 0.05s linear' : 'all 0.3s'
                    }}
                  />
                ))}

                {/* Package attached indicator */}
                {packageGrabbed && (
                  <div className="absolute -bottom-8 left-1/2 transform -translate-x-1/2">
                    <Package size={20} className="text-yellow-400 animate-bounce" />
                  </div>
                )}
              </div>

              {/* Status overlay */}
              <div className="absolute top-4 left-4 right-4 flex justify-between">
                <div className="bg-black/60 px-3 py-1 rounded-full text-green-400 text-sm font-semibold">
                  {flying ? '● FLYING' : '○ STANDBY'}
                </div>
                <div className="bg-black/60 px-3 py-1 rounded-full text-purple-400 text-sm font-semibold">
                  {shapeMode.toUpperCase()}
                </div>
              </div>

              {/* Legend */}
              <div className="absolute bottom-4 left-4 bg-black/60 p-2 rounded-lg text-xs">
                <div className="flex items-center gap-2 text-blue-300">
                  <Package size={12} /> Pickup
                </div>
                <div className="flex items-center gap-2 text-green-300">
                  <div className="w-3 h-3 bg-green-400 rounded-full" /> Target
                </div>
              </div>
            </div>

            {/* Primary Telemetry - Horizontal Below Simulation */}
            <div className="grid grid-cols-6 gap-3 mt-4">
              <div className="bg-white/5 p-3 rounded-lg border border-purple-500/20">
                <div className="text-xs text-purple-300">Altitude</div>
                <div className="text-xl font-bold text-white">{altitude.toFixed(1)}m</div>
                <div className="text-xs text-purple-400">↑ {verticalSpeed.toFixed(1)} m/s</div>
              </div>
              <div className="bg-white/5 p-3 rounded-lg border border-purple-500/20">
                <div className="text-xs text-purple-300">Ground Speed</div>
                <div className="text-xl font-bold text-white">{groundSpeed.toFixed(1)}m/s</div>
                <div className="text-xs text-purple-400">{(groundSpeed * 3.6).toFixed(1)} km/h</div>
              </div>
              <div className="bg-white/5 p-3 rounded-lg border border-purple-500/20">
                <div className="text-xs text-purple-300">Distance</div>
                <div className="text-xl font-bold text-white">
                  {(calculateDistance(dronePosition, { x: 10, y: 90 }) * 0.1).toFixed(1)}km
                </div>
                <div className="text-xs text-purple-400">from base</div>
              </div>
              <div className="bg-white/5 p-3 rounded-lg border border-purple-500/20">
                <div className="text-xs text-purple-300">Throttle</div>
                <div className="text-xl font-bold text-white">{throttle.toFixed(0)}%</div>
                <div className="text-xs text-purple-400">power output</div>
              </div>
              <div className="bg-white/5 p-3 rounded-lg border border-purple-500/20">
                <div className="text-xs text-purple-300">Flight Time</div>
                <div className="text-xl font-bold text-white">{totalFlightTime.toFixed(0)}s</div>
                <div className="text-xs text-purple-400">elapsed</div>
              </div>
              <div className="bg-white/5 p-3 rounded-lg border border-purple-500/20">
                <div className="text-xs text-purple-300">Max Alt</div>
                <div className="text-xl font-bold text-white">{maxAltitudeReached.toFixed(1)}m</div>
                <div className="text-xs text-purple-400">record</div>
              </div>
            </div>
          </div>
        </div>

          {/* Control Panels Below - 3 Column Grid */}
          <div className="grid grid-cols-1 lg:grid-cols-3 gap-4">
            {/* Left Column - Shape & Grasp */}
            <div className="space-y-4">
              <div className="bg-black/40 backdrop-blur-xl border border-purple-500/30 rounded-2xl p-6 shadow-2xl">
                <h2 className="text-xl font-bold text-white mb-4 flex items-center gap-2">
                  <Zap className="text-yellow-400" />
                  Morphing Control
                </h2>
                
                <div className="space-y-3">
                  {['standard', 'wide-grasp', 'precision', 'compact'].map((mode) => (
                    <button
                      key={mode}
                      onClick={() => handleShapeChange(mode)}
                      className={`w-full py-3 px-4 rounded-xl font-semibold transition-all duration-300 ${
                        shapeMode === mode
                          ? 'bg-gradient-to-r from-purple-500 to-pink-500 text-white shadow-lg shadow-purple-500/50 scale-105'
                          : 'bg-white/10 text-purple-200 hover:bg-white/20'
                      }`}
                    >
                      {mode.split('-').map(w => w.charAt(0).toUpperCase() + w.slice(1)).join(' ')} Mode
                    </button>
                  ))}
                </div>

                {morphProgress > 0 && morphProgress < 100 && (
                  <div className="mt-4">
                    <div className="flex justify-between text-sm text-purple-300 mb-2">
                      <span>Morphing...</span>
                      <span>{morphProgress}%</span>
                    </div>
                    <div className="w-full bg-white/10 rounded-full h-3 overflow-hidden">
                      <div 
                        className="h-full bg-gradient-to-r from-purple-500 to-pink-500 transition-all duration-100 rounded-full"
                        style={{ width: `${morphProgress}%` }}
                      />
                    </div>
                  </div>
                )}
              </div>

              {/* Grasp Control */}
              <div className="bg-black/40 backdrop-blur-xl border border-purple-500/30 rounded-2xl p-6 shadow-2xl">
                <h2 className="text-xl font-bold text-white mb-4 flex items-center gap-2">
                  <Package className="text-green-400" />
                  Grasp System
                </h2>
                
                <button
                  onClick={toggleGrasp}
                  className={`w-full py-4 px-4 rounded-xl font-bold text-lg transition-all duration-300 ${
                    graspMode
                      ? 'bg-gradient-to-r from-green-500 to-emerald-500 text-white shadow-lg shadow-green-500/50'
                      : 'bg-gradient-to-r from-orange-500 to-red-500 text-white shadow-lg shadow-orange-500/50'
                  }`}
                >
                  {graspMode ? '✓ Object Grasped' : '○ Ready to Grasp'}
                </button>

                <div className="mt-4 p-3 bg-white/5 rounded-lg border border-purple-500/20">
                  <div className="text-sm text-purple-300">Delivery Status:</div>
                  <div className="text-lg font-bold text-white capitalize">{deliveryStatus}</div>
                </div>
              </div>

              {/* Mission Log */}
              <div className="bg-black/40 backdrop-blur-xl border border-purple-500/30 rounded-2xl p-6 shadow-2xl">
                <h2 className="text-xl font-bold text-white mb-4">Mission Log</h2>
                <div className="space-y-2 h-40 overflow-y-auto">
                  {missionLog.length === 0 ? (
                    <div className="text-purple-300 text-sm italic">No activity yet...</div>
                  ) : (
                    missionLog.map((log, idx) => (
                      <div key={idx} className="text-green-400 text-xs font-mono bg-black/30 p-2 rounded">
                        {log}
                      </div>
                    ))
                  )}
                </div>
              </div>
            </div>

          {/* Center Column - Advanced Telemetry Panel */}
          <div className="bg-black/40 backdrop-blur-xl border border-purple-500/30 rounded-2xl p-4 shadow-2xl">
            <div className="bg-black/40 backdrop-blur-xl border border-blue-500/30 rounded-xl p-4">
              <div className="flex items-center justify-between mb-3">
                <h3 className="text-sm font-bold text-blue-300">Flight Telemetry</h3>
                <div className={`text-xs px-2 py-1 rounded ${
                  failsafeStatus === 'OK' ? 'bg-green-500/20 text-green-400' : 'bg-red-500/20 text-red-400'
                }`}>
                  {failsafeStatus}
                </div>
              </div>
              
              <div className="grid grid-cols-2 gap-3 text-xs">
                {/* Orientation */}
                <div className="bg-white/5 p-2 rounded">
                  <div className="text-purple-300 mb-1">Orientation</div>
                  <div className="text-white">Pitch: {pitch.toFixed(1)}°</div>
                  <div className="text-white">Roll: {roll.toFixed(1)}°</div>
                  <div className="text-white">Yaw: {yaw.toFixed(1)}°</div>
                </div>

                {/* GPS */}
                <div className="bg-white/5 p-2 rounded">
                  <div className="text-purple-300 mb-1">GPS ({gpsSatellites} sats)</div>
                  <div className="text-white">{gpsCoordinates.lat.toFixed(6)}°N</div>
                  <div className="text-white">{Math.abs(gpsCoordinates.lon).toFixed(6)}°W</div>
                  <div className="text-green-400">±{gpsAccuracy.toFixed(1)}m</div>
                </div>

                {/* Power */}
                <div className="bg-white/5 p-2 rounded">
                  <div className="text-purple-300 mb-1">Power System</div>
                  <div className="text-white">Voltage: {voltage.toFixed(2)}V</div>
                  <div className="text-white">Current: {current.toFixed(2)}A</div>
                  <div className="text-yellow-400">{powerConsumption.toFixed(0)}W</div>
                </div>

                {/* Motors */}
                <div className="bg-white/5 p-2 rounded">
                  <div className="text-purple-300 mb-1">Motors (RPM)</div>
                  <div className="grid grid-cols-2 gap-1">
                    <div className="text-white text-xs">M1: {motorRPM[0]}</div>
                    <div className="text-white text-xs">M2: {motorRPM[1]}</div>
                    <div className="text-white text-xs">M3: {motorRPM[2]}</div>
                    <div className="text-white text-xs">M4: {motorRPM[3]}</div>
                  </div>
                </div>

                {/* Environment */}
                <div className="bg-white/5 p-2 rounded">
                  <div className="text-purple-300 mb-1">Environment</div>
                  <div className="text-white">Wind: {windSpeed.toFixed(1)} m/s @ {windDirection.toFixed(0)}°</div>
                  <div className="text-white">Temp: {temperature.toFixed(1)}°C</div>
                  <div className="text-white">{pressure.toFixed(1)} hPa</div>
                </div>

                {/* Flight Controller */}
                <div className="bg-white/5 p-2 rounded">
                  <div className="text-purple-300 mb-1">Flight Controller</div>
                  <div className="text-white">CPU: {cpuLoad.toFixed(0)}%</div>
                  <div className="text-white">Loop: {loopTime.toFixed(0)}μs</div>
                  <div className="text-white">Mode: {flightMode}</div>
                </div>

                {/* Battery Cells */}
                <div className="bg-white/5 p-2 rounded">
                  <div className="text-purple-300 mb-1">Battery Cells (4S)</div>
                  <div className="grid grid-cols-2 gap-1">
                    {batteryCell.map((v, i) => (
                      <div key={i} className={`text-xs ${v < 3.5 ? 'text-red-400' : 'text-green-400'}`}>
                        S{i+1}: {v.toFixed(2)}V
                      </div>
                    ))}
                  </div>
                </div>

                {/* IMU Sensors */}
                <div className="bg-white/5 p-2 rounded">
                  <div className="text-purple-300 mb-1">IMU Sensors</div>
                  <div className="text-white text-xs">Accel: {acceleration.z.toFixed(2)} m/s²</div>
                  <div className="text-white text-xs">Gyro: {gyroscope.z.toFixed(3)} rad/s</div>
                  <div className="text-white text-xs">Hdg: {heading.toFixed(0)}°</div>
                </div>

                {/* Flight Stats */}
                <div className="bg-white/5 p-2 rounded col-span-2">
                  <div className="text-purple-300 mb-1">Flight Statistics</div>
                  <div className="flex justify-between">
                    <span className="text-white">Flight Time: {totalFlightTime.toFixed(1)}s</span>
                    <span className="text-white">Max Alt: {maxAltitudeReached.toFixed(1)}m</span>
                    <span className="text-white">Throttle: {throttle.toFixed(0)}%</span>
                  </div>
                </div>

                {/* Motor Temps */}
                <div className="bg-white/5 p-2 rounded col-span-2">
                  <div className="text-purple-300 mb-1">Motor Temperatures</div>
                  <div className="flex justify-between gap-2">
                    {motorTemp.map((temp, i) => (
                      <div key={i} className="flex-1">
                        <div className={`text-xs ${temp > 70 ? 'text-red-400' : temp > 50 ? 'text-yellow-400' : 'text-green-400'}`}>
                          M{i+1}: {temp.toFixed(0)}°C
                        </div>
                        <div className="w-full bg-black/30 h-1 rounded mt-1">
                          <div 
                            className={`h-full rounded ${temp > 70 ? 'bg-red-500' : temp > 50 ? 'bg-yellow-500' : 'bg-green-500'}`}
                            style={{ width: `${(temp / 85) * 100}%` }}
                          />
                        </div>
                      </div>
                    ))}
                  </div>
                </div>
              </div>
            </div>
          </div>

          {/* Right Panel - Flight Control */}
          <div className="space-y-4">
            <div className="bg-black/40 backdrop-blur-xl border border-purple-500/30 rounded-2xl p-6 shadow-2xl">
              <h2 className="text-xl font-bold text-white mb-4 flex items-center gap-2">
                <Navigation className="text-blue-400" />
                Flight Control
              </h2>
              
              <div className="space-y-3">
                <button
                  onClick={() => setArmed(!armed)}
                  className={`w-full py-3 px-4 rounded-xl font-bold transition-all duration-300 ${
                    armed
                      ? 'bg-gradient-to-r from-red-500 to-orange-500 text-white shadow-lg shadow-red-500/50'
                      : 'bg-gradient-to-r from-green-500 to-emerald-500 text-white shadow-lg shadow-green-500/50'
                  }`}
                >
                  {armed ? '⚠ DISARM' : '✓ ARM MOTORS'}
                </button>

                <button
                  onClick={() => armed && setFlying(!flying)}
                  disabled={!armed}
                  className={`w-full py-4 px-4 rounded-xl font-bold text-lg transition-all duration-300 flex items-center justify-center gap-2 ${
                    !armed
                      ? 'bg-gray-600 text-gray-400 cursor-not-allowed'
                      : flying
                      ? 'bg-gradient-to-r from-orange-500 to-red-500 text-white shadow-lg shadow-orange-500/50'
                      : 'bg-gradient-to-r from-blue-500 to-purple-500 text-white shadow-lg shadow-blue-500/50'
                  }`}
                >
                  {flying ? <Square size={20} /> : <Play size={20} />}
                  {flying ? 'LAND' : 'TAKE OFF'}
                </button>
              </div>

              {/* Speed Control */}
              {/* Directional Controls */}
              <div className="mt-6">
                <div className="text-sm text-purple-300 mb-3">Manual Control {!flying && '(Must be flying)'}</div>
                <div className="grid grid-cols-3 gap-2">
                  <div />
                  <button 
                    onClick={() => moveManually('up')}
                    disabled={!flying || autoSimulation}
                    className={`aspect-square rounded-lg flex items-center justify-center text-white font-bold transition-all ${
                      flying && !autoSimulation ? 'bg-white/10 hover:bg-white/20' : 'bg-white/5 cursor-not-allowed opacity-50'
                    }`}
                  >
                    ▲
                  </button>
                  <div />
                  <button 
                    onClick={() => moveManually('left')}
                    disabled={!flying || autoSimulation}
                    className={`aspect-square rounded-lg flex items-center justify-center text-white font-bold transition-all ${
                      flying && !autoSimulation ? 'bg-white/10 hover:bg-white/20' : 'bg-white/5 cursor-not-allowed opacity-50'
                    }`}
                  >
                    ◄
                  </button>
                  <button 
                    onClick={() => setDronePosition({ x: 10, y: 90 })}
                    disabled={!flying || autoSimulation}
                    className={`aspect-square rounded-lg flex items-center justify-center text-white font-bold transition-all border-2 ${
                      flying && !autoSimulation 
                        ? 'bg-purple-500/30 hover:bg-purple-500/50 border-purple-500' 
                        : 'bg-purple-500/10 border-purple-500/30 cursor-not-allowed opacity-50'
                    }`}
                  >
                    ●
                  </button>
                  <button 
                    onClick={() => moveManually('right')}
                    disabled={!flying || autoSimulation}
                    className={`aspect-square rounded-lg flex items-center justify-center text-white font-bold transition-all ${
                      flying && !autoSimulation ? 'bg-white/10 hover:bg-white/20' : 'bg-white/5 cursor-not-allowed opacity-50'
                    }`}
                  >
                    ►
                  </button>
                  <div />
                  <button 
                    onClick={() => moveManually('down')}
                    disabled={!flying || autoSimulation}
                    className={`aspect-square rounded-lg flex items-center justify-center text-white font-bold transition-all ${
                      flying && !autoSimulation ? 'bg-white/10 hover:bg-white/20' : 'bg-white/5 cursor-not-allowed opacity-50'
                    }`}
                  >
                    ▼
                  </button>
                  <div />
                </div>
              </div>
            </div>

            {/* Mission Presets */}
            <div className="bg-black/40 backdrop-blur-xl border border-purple-500/30 rounded-2xl p-6 shadow-2xl">
              <h2 className="text-xl font-bold text-white mb-4 flex items-center gap-2">
                <Settings className="text-purple-400" />
                Quick Actions
              </h2>
              
              <div className="space-y-2">
                <button 
                  onClick={() => {
                    setTargetPosition({ x: 90, y: 20 });
                    addLog('🎯 Target updated to delivery zone');
                  }}
                  className="w-full py-2 px-4 rounded-lg text-sm transition-all bg-white/10 hover:bg-white/20 text-purple-200"
                >
                  � Set Delivery Target
                </button>
                <button 
                  onClick={() => {
                    if (flying) {
                      moveToPosition(packagePosition);
                      addLog('🚁 Flying to package location');
                    }
                  }}
                  disabled={!flying || autoSimulation}
                  className={`w-full py-2 px-4 rounded-lg text-sm transition-all ${
                    flying && !autoSimulation 
                      ? 'bg-white/10 hover:bg-white/20 text-purple-200' 
                      : 'bg-white/5 text-purple-400 cursor-not-allowed opacity-50'
                  }`}
                >
                  📦 Fly to Package
                </button>
                <button 
                  onClick={() => {
                    if (flying) {
                      moveToPosition(targetPosition);
                      addLog('🎯 Flying to delivery target');
                    }
                  }}
                  disabled={!flying || autoSimulation}
                  className={`w-full py-2 px-4 rounded-lg text-sm transition-all ${
                    flying && !autoSimulation 
                      ? 'bg-white/10 hover:bg-white/20 text-purple-200' 
                      : 'bg-white/5 text-purple-400 cursor-not-allowed opacity-50'
                  }`}
                >
                  🎯 Fly to Target
                </button>
                <button 
                  onClick={() => {
                    if (flying) {
                      moveToPosition({ x: 10, y: 90 });
                      addLog('🏠 Returning to base');
                    }
                  }}
                  disabled={!flying || autoSimulation}
                  className={`w-full py-2 px-4 rounded-lg text-sm transition-all ${
                    flying && !autoSimulation 
                      ? 'bg-white/10 hover:bg-white/20 text-purple-200' 
                      : 'bg-white/5 text-purple-400 cursor-not-allowed opacity-50'
                  }`}
                >
                  🏠 Return to Base
                </button>
              </div>
            </div>

            {/* Performance Stats */}
            <div className="bg-gradient-to-br from-purple-500/20 to-pink-500/20 backdrop-blur-xl border border-purple-500/50 rounded-2xl p-4 shadow-2xl">
              <h3 className="text-sm font-bold text-white mb-2">Shape Mode Effects</h3>
              <div className="space-y-1 text-xs text-purple-200">
                <div>Standard: Balanced performance</div>
                <div>Wide-Grasp: Better grasp, slower</div>
                <div>Precision: High accuracy</div>
                <div>Compact: Fast travel, less grasp</div>
              </div>
            </div>

            {/* Simulation Info */}
            <div className="bg-gradient-to-br from-green-500/20 to-emerald-500/20 backdrop-blur-xl border border-green-500/50 rounded-2xl p-4 shadow-2xl">
              <h3 className="text-sm font-bold text-white mb-2">💡 Quick Start</h3>
              <p className="text-green-200 text-xs leading-relaxed">
                Manual: ARM → TAKE OFF → Use controls to fly<br/>
                Auto: Click START AUTO SIMULATION for full demo
              </p>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
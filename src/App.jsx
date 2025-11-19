import React, { useState, useEffect, useRef } from 'react';
import { Battery, Wifi, Navigation, Package, Zap, Settings, Play, Square, Camera, Radio, PlayCircle, PauseCircle, Map as MapIcon, Globe, Home, Building2, Crosshair } from 'lucide-react';
import MapView from './components/MapView';
import ParametersPanel from './components/ParametersPanel';
import Scene3D from './components/Scene3D';
import Drone3DMiniView from './components/Drone3DMiniView';
import { pixelToGPS, gpsToPixel, calculateGPSDistance, moveGPSCoordinate, getTerrainElevation, PRESET_LOCATIONS } from './utils/gpsUtils';
import { getCurrentLocation, checkIndoorBounds, constrainToIndoorBounds, getEnvironmentPhysics, INDOOR_BOUNDS, PIDController } from './utils/environmentUtils';

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

  // NEW: Real GPS Coordinates and Map Settings
  const [baseLocation, setBaseLocation] = useState(PRESET_LOCATIONS.sanFrancisco);
  const [showMap, setShowMap] = useState(true);
  const [mapType, setMapType] = useState('satellite'); // satellite, street, terrain
  const [flightPath, setFlightPath] = useState([]);
  const [terrainElevation, setTerrainElevation] = useState(0);
  
  // NEW: Adjustable Flight Parameters
  const [parameters, setParameters] = useState({
    maxSpeed: 25, // m/s
    batteryCapacity: 5000, // mAh
    motorPower: 200, // Watts
    windStrength: 5, // m/s
    gpsAccuracy: 1.5, // meters
    signalRange: 500, // meters
    payloadWeight: 2, // kg
    maxAltitude: 120 // meters
  });

  // NEW: Drone Presets
  const dronePresets = [
    { 
      name: 'Racing', 
      icon: '🏎️',
      params: { maxSpeed: 50, batteryCapacity: 3000, motorPower: 400, windStrength: 5, gpsAccuracy: 2, signalRange: 300, payloadWeight: 0, maxAltitude: 100 }
    },
    { 
      name: 'Photography', 
      icon: '📸',
      params: { maxSpeed: 15, batteryCapacity: 8000, motorPower: 150, windStrength: 3, gpsAccuracy: 1, signalRange: 800, payloadWeight: 3, maxAltitude: 150 }
    },
    { 
      name: 'Delivery', 
      icon: '📦',
      params: { maxSpeed: 25, batteryCapacity: 5000, motorPower: 200, windStrength: 5, gpsAccuracy: 1.5, signalRange: 500, payloadWeight: 5, maxAltitude: 120 }
    },
    { 
      name: 'Inspection', 
      icon: '🔍',
      params: { maxSpeed: 10, batteryCapacity: 10000, motorPower: 100, windStrength: 2, gpsAccuracy: 0.5, signalRange: 1000, payloadWeight: 2, maxAltitude: 200 }
    }
  ];

  // NEW: Indoor/Outdoor and 3D View States
  const [isIndoor, setIsIndoor] = useState(false);
  const [indoorBounds, setIndoorBounds] = useState(INDOOR_BOUNDS.medium);
  const [view3D, setView3D] = useState(false);
  const [useCurrentLocation, setUseCurrentLocation] = useState(false);
  const [currentLocationData, setCurrentLocationData] = useState(null);
  const [locationError, setLocationError] = useState(null);

  // PID Controllers for stabilization
  const pitchPID = useRef(new PIDController(2.0, 0.1, 0.8));
  const rollPID = useRef(new PIDController(2.0, 0.1, 0.8));
  const yawPID = useRef(new PIDController(1.5, 0.05, 0.5));

  // Convert pixel positions to GPS coordinates
  const droneGPS = pixelToGPS(dronePosition.x, dronePosition.y, useCurrentLocation && currentLocationData ? currentLocationData : baseLocation);
  const baseGPS = useCurrentLocation && currentLocationData ? currentLocationData : baseLocation;
  const packageGPS = pixelToGPS(packagePosition.x, packagePosition.y, baseGPS);
  const targetGPS = pixelToGPS(targetPosition.x, targetPosition.y, baseGPS);

  // Add log entry
  const addLog = (message) => {
    const timestamp = new Date().toLocaleTimeString();
    setMissionLog(prev => [...prev.slice(-5), `${timestamp}: ${message}`]);
  };

  // Get user's location on app startup
  useEffect(() => {
    const initializeLocation = async () => {
      try {
        addLog('🌍 Initializing GPS system...');
        const location = await getCurrentLocation();
        setCurrentLocationData(location);
        setBaseLocation({ lat: location.lat, lon: location.lon, name: 'Your Location' });
        setUseCurrentLocation(true);
        setLocationError(null);
        addLog(`✅ Base station set at: ${location.lat.toFixed(6)}, ${location.lon.toFixed(6)}`);
        addLog(`📡 GPS accuracy: ±${location.accuracy.toFixed(0)}m`);
      } catch (error) {
        setLocationError(error.message);
        addLog(`⚠️ Using default location (${baseLocation.name})`);
        addLog(`💡 Enable location to use your real position`);
      }
    };
    initializeLocation();
  }, []); // Run once on mount

  // Handle parameter changes
  const handleParameterChange = (param, value) => {
    setParameters(prev => ({ ...prev, [param]: value }));
    addLog(`⚙️ Parameter updated: ${param} = ${value}`);
  };

  // Handle preset selection
  const handlePresetSelect = (preset) => {
    setParameters(preset.params);
    addLog(`🎯 Preset loaded: ${preset.name}`);
  };

  // Update flight path with GPS coordinates (throttled for performance)
  useEffect(() => {
    if (flying) {
      const interval = setInterval(() => {
        setFlightPath(prev => {
          const lastPoint = prev[prev.length - 1];
          // Only add new point if position changed significantly
          if (!lastPoint || 
              Math.abs(lastPoint.lat - droneGPS.lat) > 0.000001 || 
              Math.abs(lastPoint.lon - droneGPS.lon) > 0.000001) {
            return [...prev.slice(-100), { lat: droneGPS.lat, lon: droneGPS.lon, alt: altitude }]; // Keep last 100 points
          }
          return prev;
        });
      }, 500); // Update every 500ms
      
      return () => clearInterval(interval);
    }
  }, [flying, droneGPS, altitude]);

  // Update terrain elevation
  useEffect(() => {
    const elevation = getTerrainElevation(droneGPS);
    setTerrainElevation(elevation);
  }, [droneGPS]);

  // Get current user location
  const handleGetCurrentLocation = async () => {
    try {
      addLog('📍 Getting current location...');
      const location = await getCurrentLocation();
      setCurrentLocationData(location);
      setBaseLocation({ lat: location.lat, lon: location.lon, name: 'Current Location' });
      setUseCurrentLocation(true);
      setLocationError(null);
      
      // Reset drone to base position when switching location
      setDronePosition({ x: 10, y: 90 });
      setAltitude(0);
      setFlying(false);
      setArmed(false);
      
      addLog(`✅ Location: ${location.lat.toFixed(6)}, ${location.lon.toFixed(6)} (±${location.accuracy.toFixed(0)}m)`);
    } catch (error) {
      setLocationError(error.message);
      addLog(`❌ Location error: ${error.message}`);
    }
  };

  // Toggle indoor/outdoor mode
  const handleEnvironmentToggle = () => {
    const newMode = !isIndoor;
    setIsIndoor(newMode);
    
    // Reset drone if flying
    if (flying) {
      setFlying(false);
      setArmed(false);
      setAltitude(0);
      addLog(`⚠️ Landed due to environment change`);
    }

    // Reset PID controllers
    pitchPID.current.reset();
    rollPID.current.reset();
    yawPID.current.reset();

    // Reset drone position to base
    setDronePosition({ x: 10, y: 90 });
    setPitch(0);
    setRoll(0);
    setYaw(0);

    // Apply environment-specific physics
    const envPhysics = getEnvironmentPhysics(newMode, indoorBounds);
    setParameters(prev => ({
      ...prev,
      maxSpeed: envPhysics.maxSpeed,
      maxAltitude: envPhysics.maxAltitude,
      windStrength: envPhysics.windStrength,
      gpsAccuracy: envPhysics.gpsAccuracy,
      signalRange: envPhysics.signalStrength * 10
    }));

    addLog(`🏠 Switched to ${newMode ? 'INDOOR' : 'OUTDOOR'} mode`);
  };

  // Toggle between 2D and 3D view
  const handleView3DToggle = () => {
    setView3D(prev => !prev);
    addLog(`🖼️ Switched to ${!view3D ? '3D' : '2D'} view`);
  };

  // Change indoor bounds preset
  const handleIndoorBoundsChange = (key) => {
    const preset = INDOOR_BOUNDS[key];
    if (preset) {
      setIndoorBounds(preset);
      
      // Reset drone if flying
      if (flying) {
        setFlying(false);
        setArmed(false);
        setAltitude(0);
        addLog(`⚠️ Landed due to bounds change`);
      }
      
      // Reset PID controllers
      pitchPID.current.reset();
      rollPID.current.reset();
      yawPID.current.reset();
      
      // Reset drone position
      setDronePosition({ x: 10, y: 90 });
      
      // Recalculate environment physics
      const envPhysics = getEnvironmentPhysics(isIndoor, preset);
      setParameters(prev => ({
        ...prev,
        maxSpeed: envPhysics.maxSpeed,
        maxAltitude: envPhysics.maxAltitude,
        windStrength: envPhysics.windStrength,
        gpsAccuracy: envPhysics.gpsAccuracy
      }));
      addLog(`📐 Indoor bounds set to ${key}`);
    }
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
      // Reset failsafe when landed
      if (failsafeStatus !== 'OK') {
        setFailsafeStatus('OK');
      }
      return;
    }

    const physicsLoop = setInterval(() => {
      const multipliers = getShapeMultipliers();
      
      // Get environment-specific physics parameters
      const envPhysics = getEnvironmentPhysics(isIndoor, indoorBounds);
      
      // Flight time tracking
      setTotalFlightTime(prev => prev + 0.1);
      
      // Rotor animation - speed based on throttle
      const rotorSpeed = 10 + (throttle / 100) * 40;
      setRotorRotation(prev => (prev + rotorSpeed) % 360);
      
      // Motor RPM calculation (realistic quadcopter range: 0-15000 RPM)
      const baseRPM = armed ? 3000 : 0;
      const flightRPM = flying ? baseRPM + (throttle / 100) * 12000 : baseRPM;
      const windEffect = envPhysics.windStrength * 50; // Use environment wind
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
      
      // Throttle - only auto-manage in simulation mode, otherwise manual hover
      if (autoSimulation) {
        const targetAlt = 50;
        const altError = targetAlt - altitude;
        const newThrottle = Math.max(0, Math.min(100, 50 + altError * 2));
        setThrottle(newThrottle);
      } else {
        // Manual mode: maintain hover throttle around 50% or user-controlled
        setThrottle(prev => {
          const hoverThrottle = 50 + (altitude > 0 ? 5 : 0); // Slight boost when airborne
          return prev === 0 ? hoverThrottle : prev;
        });
      }
      
      // Altitude physics with vertical speed - using environment max altitude
      const climbRate = (throttle - 50) / 10; // m/s
      setVerticalSpeed(climbRate);
      setAltitude(prev => {
        const newAlt = Math.max(0, Math.min(prev + climbRate * 0.1, envPhysics.maxAltitude));
        setMaxAltitudeReached(max => Math.max(max, newAlt));
        return newAlt;
      });
      
      // Calculate speeds using environment maxSpeed
      const speedScale = envPhysics.maxSpeed / 25; // Scale relative to default 25 m/s
      const currentSpeed = Math.sqrt(velocity.x * velocity.x + velocity.y * velocity.y) * 10 * multipliers.speed * speedScale;
      setSpeed(currentSpeed);
      setGroundSpeed(currentSpeed);
      
      // Pitch and Roll based on movement direction (affected by payload weight)
      const stabilityFactor = 1 + (parameters.payloadWeight / 10); // More weight = slower response
      const targetPitch = -velocity.y * 5 / stabilityFactor;
      const targetRoll = velocity.x * 5 / stabilityFactor;
      
      // Apply smooth transition instead of PID (which was causing runaway)
      setPitch(prev => {
        const diff = targetPitch - prev;
        return prev + diff * 0.3; // Smooth damping
      });
      setRoll(prev => {
        const diff = targetRoll - prev;
        return prev + diff * 0.3; // Smooth damping
      });
      
      // Yaw and Heading updates with smooth damping
      if (velocity.x !== 0 || velocity.y !== 0) {
        const targetHeading = (Math.atan2(velocity.x, -velocity.y) * 180 / Math.PI + 360) % 360;
        setHeading(prev => {
          const diff = (targetHeading - prev + 540) % 360 - 180;
          return (prev + diff * 0.1 + 360) % 360;
        });
        
        // Smooth yaw transition
        setYaw(prev => {
          const diff = (heading - prev + 540) % 360 - 180;
          return (prev + diff * 0.2 + 360) % 360;
        });
      }
      
      // Indoor bounds enforcement
      if (isIndoor) {
        // Convert map position (0-100) to 3D meters (centered at 0,0)
        const dronePos3D = {
          x: (dronePosition.x - 50) * 0.1,  // X position in meters
          y: altitude,                        // Y (altitude) in meters
          z: (dronePosition.y - 50) * 0.1   // Z position in meters
        };
        
        const boundsCheck = checkIndoorBounds(dronePos3D, indoorBounds);
        if (!boundsCheck.withinBounds) {
          // Constrain position - returns 3D position in meters
          const constrained = constrainToIndoorBounds(dronePos3D, indoorBounds);
          
          // Convert back to map coordinates (0-100)
          setDronePosition({
            x: 50 + constrained.x / 0.1,
            y: 50 + constrained.z / 0.1
          });
          setAltitude(constrained.y);
          
          // Log collision
          if (boundsCheck.violations.x) addLog('⚠️ Wall collision detected (X)');
          if (boundsCheck.violations.z) addLog('⚠️ Wall collision detected (Z)');
          if (boundsCheck.violations.y) addLog('⚠️ Ceiling/floor collision detected');
        }
      }
      
      // GPS simulation (convert position to lat/lon delta)
      const basePos = { x: 10, y: 90 };
      const deltaX = (dronePosition.x - basePos.x) * 0.0001; // ~11m per 0.0001 degree
      const deltaY = -(dronePosition.y - basePos.y) * 0.0001;
      // Use actual GPS coordinates with configurable base location and environment accuracy
      setGpsCoordinates(droneGPS);
      setGpsAccuracy(envPhysics.gpsAccuracy * (0.8 + Math.random() * 0.4)); // ±20% variation
      setGpsSatellites(Math.max(isIndoor ? 4 : 8, Math.min(16, 12 + Math.floor(Math.random() * 5) - 2)));
      
      // Power consumption - using adjustable motor power parameter
      const motorPowerScale = parameters.motorPower / 200; // Scale relative to default 200W
      const baseCurrent = armed ? 2.5 * motorPowerScale : 0.1; // Amperes
      const flightCurrent = flying ? baseCurrent + (throttle / 100) * 25 * motorPowerScale : baseCurrent;
      const windCurrent = (envPhysics.windStrength / 10) * 2; // Use environment wind
      const shapeCurrent = (1 / multipliers.efficiency) * 3;
      const payloadCurrent = parameters.payloadWeight * 0.5; // Current increases with payload
      const totalCurrent = flightCurrent + windCurrent + shapeCurrent + payloadCurrent + (graspMode ? 1.5 : 0);
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
      
      // Battery drain - using adjustable battery capacity and environment multiplier
      const capacityScale = parameters.batteryCapacity / 5000; // Scale relative to default 5000mAh
      const drainRate = (totalCurrent / 36000) / capacityScale * envPhysics.batteryDrainMultiplier; // Apply environment drain
      setBattery(prev => {
        const newBattery = Math.max(prev - drainRate, 0);
        
        // Battery warnings
        if (newBattery <= 20 && prev > 20) {
          addLog('⚠️ LOW BATTERY: 20% remaining!');
          setFailsafeStatus('LOW_BATTERY');
        }
        if (newBattery <= 10 && prev > 10) {
          addLog('🚨 CRITICAL BATTERY: 10% - Return to base!');
          setFailsafeStatus('CRITICAL_BATTERY');
        }
        if (newBattery <= 5 && prev > 5) {
          addLog('🚨 EMERGENCY: Auto-landing initiated!');
          setFailsafeStatus('EMERGENCY_LAND');
          // Auto-land
          setThrottle(0);
          setFlying(false);
          setArmed(false);
          if (autoSimulation) {
            setAutoSimulation(false);
          }
        }
        
        return newBattery;
      });
      
      // Signal degradation - using adjustable signal range
      const distanceFromBase = calculateDistance(dronePosition, { x: 10, y: 90 });
      const maxRange = parameters.signalRange;
      const signalLoss = Math.min((distanceFromBase / maxRange) * 60, 50);
      const altitudeLoss = Math.min(altitude * 0.05, 10);
      setSignalStrength(Math.max(100 - signalLoss - altitudeLoss, 40));
      
      // Environmental simulation - using environment wind strength
      setWindSpeed(prev => Math.max(0, Math.min(envPhysics.windStrength + (Math.random() - 0.5) * 2, envPhysics.windStrength * 1.5)));
      setWindDirection(prev => (prev + (Math.random() - 0.5) * 5 + 360) % 360);
      setTemperature(isIndoor ? 22 : 22 - altitude * 0.15 + terrainElevation * 0.01); // Indoor constant temp
      setPressure(1013.25 * Math.pow(1 - (altitude + terrainElevation) / 44330, 5.255)); // Barometric formula with terrain
      
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
      
      // Failsafe monitoring (priority order: battery > signal > altitude)
      if (battery <= 5) {
        setFailsafeStatus('EMERGENCY_LAND');
      } else if (battery <= 10) {
        setFailsafeStatus('CRITICAL_BATTERY');
      } else if (battery <= 20) {
        setFailsafeStatus('LOW_BATTERY');
      } else if (signalStrength < 40) {
        setFailsafeStatus('SIGNAL_LOST');
      } else if (signalStrength < 50) {
        setFailsafeStatus('WEAK_SIGNAL');
      } else if (altitude > parameters.maxAltitude) {
        setFailsafeStatus('MAX_ALT');
      } else {
        setFailsafeStatus('OK');
      }
      
    }, 100);

    return () => clearInterval(physicsLoop);
  }, [flying, shapeMode, isIndoor]);

  // Auto simulation sequence with realistic movement and morphing
  useEffect(() => {
    if (!autoSimulation) return;

    const sequence = async () => {
      switch(simulationStep) {
        case 0:
          addLog('🚁 Initializing flight systems...');
          setShapeMode('standard');
          setTimeout(() => setSimulationStep(1), 1500);
          break;
        case 1:
          addLog('⚡ Arming motors...');
          setArmed(true);
          setTimeout(() => setSimulationStep(2), 1500);
          break;
        case 2:
          addLog('🚀 Taking off to cruise altitude...');
          setFlying(true);
          setThrottle(65); // Climb
          setTimeout(() => setSimulationStep(3), 2500);
          break;
        case 3:
          addLog('🎯 Flying to package location...');
          setThrottle(55); // Cruise
          moveToPosition(packagePosition);
          setTimeout(() => setSimulationStep(4), 4000);
          break;
        case 4:
          addLog('📍 Hovering above package...');
          setThrottle(50); // Hover
          setTimeout(() => setSimulationStep(5), 1500);
          break;
        case 5:
          addLog('🔄 Morphing to compact grasp configuration...');
          handleShapeChange('compact');
          setTimeout(() => setSimulationStep(6), 2000);
          break;
        case 6:
          addLog('⬇️ Descending to package level...');
          setThrottle(40); // Descend
          setTimeout(() => setSimulationStep(7), 2000);
          break;
        case 7:
          addLog('📦 Engaging grasp mode and securing package...');
          setGraspMode(true);
          setPackageGrabbed(true);
          setDeliveryStatus('grasped');
          setThrottle(50); // Stop descent
          setTimeout(() => setSimulationStep(8), 2500);
          break;
        case 8:
          addLog('⬆️ Ascending with package...');
          setThrottle(60); // Climb with payload
          setTimeout(() => setSimulationStep(9), 2000);
          break;
        case 9:
          addLog('🔄 Morphing to standard flight configuration...');
          handleShapeChange('standard');
          setTimeout(() => setSimulationStep(10), 2000);
          break;
        case 10:
          addLog('✈️ Flying to delivery location...');
          setThrottle(55); // Cruise
          moveToPosition(targetPosition);
          setTimeout(() => setSimulationStep(11), 4500);
          break;
        case 11:
          addLog('📍 Hovering above delivery point...');
          setThrottle(50); // Hover
          setTimeout(() => setSimulationStep(12), 1500);
          break;
        case 12:
          addLog('🔄 Morphing to placement configuration...');
          handleShapeChange('compact');
          setTimeout(() => setSimulationStep(13), 2000);
          break;
        case 13:
          addLog('⬇️ Descending to delivery altitude...');
          setThrottle(40); // Descend
          setTimeout(() => setSimulationStep(14), 2000);
          break;
        case 14:
          addLog('📤 Releasing package at delivery point...');
          setGraspMode(false);
          setPackageGrabbed(false);
          setDeliveryStatus('delivered');
          setThrottle(50); // Stop descent
          setTimeout(() => setSimulationStep(15), 2000);
          break;
        case 15:
          addLog('⬆️ Ascending from delivery point...');
          setThrottle(60); // Climb
          setTimeout(() => setSimulationStep(16), 2000);
          break;
        case 16:
          addLog('🔄 Morphing back to standard configuration...');
          handleShapeChange('standard');
          setTimeout(() => setSimulationStep(17), 2000);
          break;
        case 17:
          addLog('🏠 Returning to home base...');
          setThrottle(55); // Cruise
          moveToPosition({ x: 10, y: 90 });
          setTimeout(() => setSimulationStep(18), 4000);
          break;
        case 18:
          addLog('� Base location reached, preparing to land...');
          setThrottle(50); // Hover
          setTimeout(() => setSimulationStep(19), 1500);
          break;
        case 19:
          addLog('🛬 Landing...');
          setThrottle(35); // Descend to land
          setTimeout(() => setSimulationStep(20), 2500);
          break;
        case 20:
          addLog('✅ Touchdown! Mission completed successfully!');
          setFlying(false);
          setThrottle(0);
          setArmed(false);
          setDeliveryStatus('ready');
          addLog(`📊 Total flight time: ${totalFlightTime.toFixed(1)}s | Max altitude: ${maxAltitudeReached.toFixed(1)}m`);
          setTimeout(() => {
            setAutoSimulation(false);
            setSimulationStep(0);
          }, 2000);
          break;
      }
    };

    sequence();
  }, [autoSimulation, simulationStep]);

  // Movement animation - smooth velocity-based movement
  const moveToPosition = (target) => {
    const startPos = { ...dronePosition };
    const distance = calculateDistance(startPos, target);
    const multipliers = getShapeMultipliers();
    const maxSpeed = parameters.maxSpeed * multipliers.speed / 25; // Scale speed
    
    // Calculate direction
    const dx = target.x - startPos.x;
    const dy = target.y - startPos.y;
    const angle = Math.atan2(dy, dx);
    
    // Smooth acceleration and deceleration
    const accelDistance = distance * 0.3; // 30% of distance for accel/decel
    let currentStep = 0;
    const updateInterval = 50; // 20 FPS for smooth movement
    const totalSteps = Math.max(30, Math.floor(distance * 2)); // Scale steps with distance
    
    const moveInterval = setInterval(() => {
      currentStep++;
      const progress = currentStep / totalSteps;
      
      // Smooth S-curve for acceleration/deceleration
      let speedFactor;
      if (progress < 0.3) {
        // Accelerate (ease in)
        speedFactor = Math.pow(progress / 0.3, 2);
      } else if (progress > 0.7) {
        // Decelerate (ease out)
        speedFactor = Math.pow((1 - progress) / 0.3, 2);
      } else {
        // Cruise at max speed
        speedFactor = 1;
      }
      
      // Calculate velocity
      const speed = maxSpeed * speedFactor;
      const vx = Math.cos(angle) * speed;
      const vy = Math.sin(angle) * speed;
      
      setVelocity({ x: vx * 0.3, y: vy * 0.3 }); // Scale for visual effect
      
      setDronePosition(prev => {
        const newX = startPos.x + dx * progress;
        const newY = startPos.y + dy * progress;
        
        // Check if reached target
        const distRemaining = calculateDistance({ x: newX, y: newY }, target);
        if (distRemaining < 1 || currentStep >= totalSteps) {
          clearInterval(moveInterval);
          setVelocity({ x: 0, y: 0 });
          return target;
        }
        
        return { x: newX, y: newY };
      });
    }, updateInterval);
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
              <button
                onClick={() => setShowMap(!showMap)}
                className="px-4 py-2 rounded-xl font-semibold flex items-center gap-2 transition-all duration-300 bg-purple-500/20 text-purple-400 border border-purple-500/50 hover:bg-purple-500/30"
              >
                {showMap ? <Camera size={16} /> : <MapIcon size={16} />}
                {showMap ? 'Grid View' : 'Map View'}
              </button>
              <select
                value={Object.keys(PRESET_LOCATIONS).find(key => PRESET_LOCATIONS[key].lat === baseLocation.lat) || 'custom'}
                onChange={(e) => {
                  const location = PRESET_LOCATIONS[e.target.value];
                  if (location) {
                    setBaseLocation(location);
                    addLog(`📍 Location changed to ${location.name}`);
                  }
                }}
                className="px-3 py-2 rounded-xl bg-black/40 text-purple-300 border border-purple-500/50 text-sm font-semibold cursor-pointer hover:bg-black/60 transition-all"
              >
                {Object.entries(PRESET_LOCATIONS).map(([key, loc]) => (
                  <option key={key} value={key}>{loc.name}</option>
                ))}
              </select>
              {/* Environment / View controls */}
              <div className="flex items-center gap-2 flex-wrap">
                <button
                  onClick={handleEnvironmentToggle}
                  title={isIndoor ? 'Switch to Outdoor' : 'Switch to Indoor'}
                  className={`px-3 py-2 rounded-xl text-sm font-semibold flex items-center gap-1 transition-all ${isIndoor ? 'bg-yellow-500 text-black hover:bg-yellow-600' : 'bg-indigo-500 text-white hover:bg-indigo-600'}`}
                >
                  {isIndoor ? <Building2 size={16} /> : <Globe size={16} />}
                  <span>{isIndoor ? 'Indoor' : 'Outdoor'}</span>
                </button>

                <button
                  onClick={handleView3DToggle}
                  title={view3D ? 'Switch to 2D view' : 'Switch to 3D view'}
                  className={`px-3 py-2 rounded-xl text-sm font-semibold flex items-center gap-1 transition-all ${view3D ? 'bg-purple-500 text-white hover:bg-purple-600' : 'bg-black/40 text-purple-300 border border-purple-500/50 hover:bg-black/60'}`}
                >
                  <Camera size={16} />
                  <span>{view3D ? '3D' : '2D'}</span>
                </button>

                <button
                  onClick={handleGetCurrentLocation}
                  title={useCurrentLocation ? "Using your location" : "Get current location"}
                  className={`px-3 py-2 rounded-xl text-sm font-semibold flex items-center gap-1 transition-all ${
                    useCurrentLocation 
                      ? 'bg-green-500 text-white hover:bg-green-600' 
                      : 'bg-black/40 text-purple-300 border border-purple-500/50 hover:bg-black/60'
                  }`}
                >
                  <Crosshair size={16} />
                  <span>{useCurrentLocation ? '📍 Your Location' : 'Get GPS'}</span>
                </button>

                {isIndoor && (
                  <select
                    value={Object.keys(INDOOR_BOUNDS).find(k => INDOOR_BOUNDS[k] === indoorBounds) || 'medium'}
                    onChange={(e) => handleIndoorBoundsChange(e.target.value)}
                    className="px-3 py-2 rounded-xl bg-black/40 text-purple-300 border border-purple-500/50 text-sm font-semibold cursor-pointer hover:bg-black/60 transition-all capitalize"
                  >
                    {Object.keys(INDOOR_BOUNDS).map(key => (
                      <option key={key} value={key} className="capitalize">{key} Room</option>
                    ))}
                  </select>
                )}
              </div>
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
                'bg-red-500/20 border-red-500/50 animate-pulse'
              }`}>
                <Battery size={16} className={`${
                  battery > 50 ? 'text-blue-400' : 
                  battery > 20 ? 'text-yellow-400' : 
                  'text-red-400'
                } ${battery <= 10 ? 'animate-bounce' : ''}`} />
                <span className={`text-sm font-semibold ${
                  battery > 50 ? 'text-blue-400' : 
                  battery > 20 ? 'text-yellow-400' : 
                  'text-red-400'
                }`}>
                  {battery.toFixed(0)}%
                  {battery <= 20 && battery > 10 && ' ⚠️'}
                  {battery <= 10 && ' 🚨'}
                </span>
              </div>
              <div className="flex items-center gap-2 bg-purple-500/20 px-3 py-1 rounded-full border border-purple-500/50">
                <span className="text-purple-400 text-sm font-semibold">{flightMode}</span>
              </div>
            </div>
          </div>
        </div>

        {/* New Layout: Simulation with 3D Mini View */}
        <div className="grid grid-cols-1 lg:grid-cols-3 gap-4 mb-4">
          {/* Primary View - 2/3 width */}
          <div className="lg:col-span-2 bg-black/40 backdrop-blur-xl border border-purple-500/30 rounded-2xl p-6 shadow-2xl">
            <div className="flex items-center justify-between mb-4">
              <h2 className="text-xl font-bold text-white flex items-center gap-2">
                {showMap ? (
                  <>
                    <Globe className="text-blue-400" />
                    Satellite Map View
                  </>
                ) : view3D ? (
                  <>
                    <Camera className="text-purple-400" />
                    3D Simulation View
                  </>
                ) : (
                  <>
                    <Camera className="text-blue-400" />
                    Grid Simulation View
                  </>
                )}
              </h2>
              {showMap && (
                <div className="flex gap-2">
                  {['satellite', 'street', 'terrain'].map((type) => (
                    <button
                      key={type}
                      onClick={() => setMapType(type)}
                      className={`px-3 py-1 rounded-lg text-xs font-semibold transition-all ${
                        mapType === type
                          ? 'bg-purple-500 text-white'
                          : 'bg-white/10 text-purple-300 hover:bg-white/20'
                      }`}
                    >
                      {type.charAt(0).toUpperCase() + type.slice(1)}
                    </button>
                  ))}
                </div>
              )}
            </div>
            
            <div className="aspect-video bg-gradient-to-br from-slate-800 to-slate-900 rounded-xl flex items-center justify-center relative overflow-hidden border-2 border-purple-500/30">
              {showMap ? (
                <MapView
                  dronePosition={droneGPS}
                  basePosition={baseGPS}
                  packagePosition={packageGPS}
                  targetPosition={targetGPS}
                  flightPath={flightPath}
                  flying={flying}
                  altitude={altitude}
                  mapType={mapType}
                />
              ) : (
                view3D ? (
                  <div className="w-full h-full">
                    <Scene3D
                      dronePosition={{ x: dronePosition.x, y: dronePosition.y, altitude }}
                      droneRotation={[pitch * Math.PI / 180, roll * Math.PI / 180, yaw * Math.PI / 180]}
                      flying={flying}
                      armed={armed}
                      throttle={throttle}
                      shapeMode={shapeMode}
                      graspMode={graspMode}
                      isIndoor={isIndoor}
                      indoorBounds={indoorBounds}
                      packageGrabbed={packageGrabbed}
                    />
                  </div>
                ) : ( <>
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
                className="absolute transition-all duration-200 ease-out"
                style={{ 
                  left: `${dronePosition.x}%`, 
                  top: `${dronePosition.y}%`,
                  transform: `translate(-50%, -50%) ${altitude > 0 ? `scale(${1 + altitude * 0.002})` : 'scale(1)'}`
                }}
              >
                {/* Central body */}
                <div className={`w-16 h-16 border-4 rounded-full flex items-center justify-center transition-all duration-500 ${
                  armed ? 'border-green-400 shadow-lg shadow-green-400/50' : 'border-purple-400'
                }`}>
                  <Radio size={24} className={flying ? 'text-green-400' : 'text-purple-400'} />
                </div>
                
                {/* Morphing arms with rotors - showing current shape mode */}
                {(() => {
                  // Define arm positions based on shape mode
                  const armConfigs = {
                    standard: [
                      { angle: 45, distance: 30 },   // Front-right
                      { angle: 135, distance: 30 },  // Back-right
                      { angle: 225, distance: 30 },  // Back-left
                      { angle: 315, distance: 30 }   // Front-left
                    ],
                    compact: [
                      { angle: 45, distance: 22 },   // Closer X pattern
                      { angle: 135, distance: 22 },
                      { angle: 225, distance: 22 },
                      { angle: 315, distance: 22 }
                    ],
                    'wide-grasp': [
                      { angle: 0, distance: 35 },    // Wider spread
                      { angle: 90, distance: 35 },
                      { angle: 180, distance: 35 },
                      { angle: 270, distance: 35 }
                    ]
                  };
                  
                  const currentConfig = armConfigs[shapeMode] || armConfigs.standard;
                  
                  return currentConfig.map((config, idx) => (
                    <div key={idx}>
                      {/* Morphing arm */}
                      <div
                        className={`absolute w-1 transition-all duration-500 ${
                          graspMode ? 'bg-orange-400' : 'bg-purple-400/60'
                        }`}
                        style={{
                          top: '50%',
                          left: '50%',
                          height: `${config.distance - 6}px`,
                          transformOrigin: 'top center',
                          transform: `translate(-50%, -50%) rotate(${config.angle}deg) translateY(-${config.distance/2}px)`
                        }}
                      />
                      {/* Rotor with realistic rotation */}
                      <div
                        className={`absolute w-6 h-6 rounded-full transition-all duration-500 ${
                          flying ? 'bg-green-400 shadow-lg shadow-green-400/50' : 'bg-purple-400'
                        }`}
                        style={{
                          top: '50%',
                          left: '50%',
                          transform: `translate(-50%, -50%) rotate(${config.angle}deg) translateY(-${config.distance}px) rotate(${flying ? rotorRotation + idx * 90 : 0}deg)`,
                          transition: flying ? 'transform 0.05s linear' : 'all 0.5s'
                        }}
                      />
                    </div>
                  ));
                })()}

                {/* Altitude shadow indicator */}
                {altitude > 0 && (
                  <div 
                    className="absolute w-16 h-16 bg-black rounded-full blur-xl transition-all duration-200"
                    style={{
                      top: '50%',
                      left: '50%',
                      transform: `translate(-50%, -50%)`,
                      opacity: Math.min(0.3, altitude / 100),
                      scale: 1 + altitude * 0.01
                    }}
                  />
                )}

                {/* Package attached indicator */}
                {packageGrabbed && (
                  <div className="absolute -bottom-8 left-1/2 transform -translate-x-1/2">
                    <Package size={20} className="text-yellow-400 animate-bounce" />
                  </div>
                )}
              </div>

              {/* Status overlay */}
              <div className="absolute top-4 left-4 right-4 flex justify-between items-center">
                <div className="bg-black/60 px-3 py-1 rounded-full text-green-400 text-sm font-semibold">
                  {flying ? '● FLYING' : '○ STANDBY'}
                </div>
                <div className="flex gap-2">
                  <div className={`bg-black/60 px-3 py-1 rounded-full text-sm font-semibold ${
                    shapeMode === 'standard' ? 'text-blue-400' : 
                    shapeMode === 'compact' ? 'text-purple-400' : 'text-yellow-400'
                  }`}>
                    {shapeMode.toUpperCase()}
                  </div>
                  {graspMode && (
                    <div className="bg-black/60 px-3 py-1 rounded-full text-orange-400 text-sm font-semibold animate-pulse">
                      🤏 GRASPING
                    </div>
                  )}
                </div>
              </div>

              {/* Legend & Info */}
              <div className="absolute bottom-4 left-4 bg-black/60 p-3 rounded-lg text-xs space-y-2">
                <div className="flex items-center gap-2 text-blue-300">
                  <Package size={12} /> Pickup Point
                </div>
                <div className="flex items-center gap-2 text-green-300">
                  <div className="w-3 h-3 bg-green-400 rounded-full" /> Delivery Target
                </div>
                <div className="border-t border-white/20 pt-2 text-purple-300">
                  Alt: {altitude.toFixed(1)}m
                </div>
                {packageGrabbed && (
                  <div className="text-yellow-400 font-semibold">
                    📦 Package Secured
                  </div>
                )}
              </div>
              </>
                )
              )}
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

          {/* 3D Mini View - 1/3 width */}
          <div className="lg:col-span-1">
            <div className="bg-black/40 backdrop-blur-xl border border-purple-500/30 rounded-2xl p-4 shadow-2xl h-full">
              <h3 className="text-lg font-bold text-white mb-3 flex items-center gap-2">
                <Camera className="text-purple-400" size={20} />
                3D Drone View
              </h3>
              <div className="aspect-square">
                <Drone3DMiniView
                  dronePosition={dronePosition}
                  droneRotation={[pitch * Math.PI / 180, roll * Math.PI / 180, yaw * Math.PI / 180]}
                  flying={flying}
                  armed={armed}
                  throttle={throttle}
                  shapeMode={shapeMode}
                  graspMode={graspMode}
                  altitude={altitude}
                />
              </div>
              <div className="mt-3 space-y-2">
                <div className="flex justify-between text-sm">
                  <span className="text-purple-300">Pitch:</span>
                  <span className="text-white font-semibold">{pitch.toFixed(1)}°</span>
                </div>
                <div className="flex justify-between text-sm">
                  <span className="text-purple-300">Roll:</span>
                  <span className="text-white font-semibold">{roll.toFixed(1)}°</span>
                </div>
                <div className="flex justify-between text-sm">
                  <span className="text-purple-300">Yaw:</span>
                  <span className="text-white font-semibold">{yaw.toFixed(1)}°</span>
                </div>
              </div>
            </div>
          </div>
        </div>

          {/* Control Panels Below - 4 Column Grid */}
          <div className="grid grid-cols-1 lg:grid-cols-4 gap-4">
            {/* Parameters Panel Column */}
            <div className="lg:col-span-1">
              <ParametersPanel
                parameters={parameters}
                onParameterChange={handleParameterChange}
                presets={dronePresets}
                onPresetSelect={handlePresetSelect}
              />
            </div>

            {/* Shape & Grasp Column */}
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
                  <div className="text-purple-300 mb-1">Power & Battery</div>
                  <div className="text-white">Voltage: {voltage.toFixed(2)}V</div>
                  <div className="text-white">Current: {current.toFixed(2)}A</div>
                  <div className="text-yellow-400">{powerConsumption.toFixed(0)}W</div>
                  <div className={`text-sm mt-1 ${battery > 20 ? 'text-green-400' : 'text-red-400'}`}>
                    Battery: {battery.toFixed(1)}%
                    {flying && (
                      <span className="text-xs ml-1">
                        ({(battery / (current / 5)).toFixed(0)}min)
                      </span>
                    )}
                  </div>
                  <div className="grid grid-cols-4 gap-1 mt-1">
                    {batteryCell.map((cell, i) => (
                      <div key={i} className={`text-xs ${cell > 3.7 ? 'text-green-300' : cell > 3.5 ? 'text-yellow-300' : 'text-red-300'}`}>
                        {cell.toFixed(2)}V
                      </div>
                    ))}
                  </div>
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
                  <div className={`text-sm mt-1 font-semibold ${
                    failsafeStatus === 'OK' ? 'text-green-400' : 
                    failsafeStatus.includes('LOW') ? 'text-yellow-400' : 
                    'text-red-400 animate-pulse'
                  }`}>
                    {failsafeStatus === 'OK' ? '✓ All Systems OK' : `⚠️ ${failsafeStatus.replace(/_/g, ' ')}`}
                  </div>
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
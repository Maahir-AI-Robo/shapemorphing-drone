import { useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import { Box, Sphere, Cylinder } from '@react-three/drei';
import * as THREE from 'three';

export default function Drone3D({ 
  position = [0, 0, 0], 
  rotation = [0, 0, 0],
  flying = false,
  armed = false,
  throttle = 0,
  shapeMode = 'standard',
  graspMode = false
}) {
  const droneRef = useRef();
  const propellerRefs = [useRef(), useRef(), useRef(), useRef()];
  
  // Animate propellers
  useFrame((state, delta) => {
    if (flying && droneRef.current) {
      const rotationSpeed = (throttle / 100) * 50 * delta;
      
      propellerRefs.forEach((ref, index) => {
        if (ref.current) {
          // Alternate rotation direction for stability
          const direction = index % 2 === 0 ? 1 : -1;
          ref.current.rotation.y += rotationSpeed * direction;
        }
      });
      
      // Add subtle hover animation
      droneRef.current.position.y += Math.sin(state.clock.elapsedTime * 2) * 0.002;
    }
  });

  // Color based on state
  const bodyColor = flying ? '#4ade80' : armed ? '#fbbf24' : '#a855f7';
  const ledColor = armed ? '#ef4444' : '#6b7280';

  // Shape mode affects drone structure
  const getShapeScale = () => {
    switch(shapeMode) {
      case 'wide-grasp': return [1.3, 0.9, 1.3]; // Wider, flatter
      case 'precision': return [0.9, 1.1, 0.9]; // Narrower, taller
      case 'compact': return [0.8, 0.8, 0.8]; // Smaller overall
      default: return [1, 1, 1];
    }
  };

  const shapeScale = getShapeScale();

  return (
    <group ref={droneRef} position={position} rotation={rotation}>
      {/* Main Body */}
      <Box args={[0.6 * shapeScale[0], 0.2 * shapeScale[1], 0.6 * shapeScale[2]]} position={[0, 0, 0]}>
        <meshStandardMaterial color={bodyColor} metalness={0.7} roughness={0.3} />
      </Box>

      {/* Arms (4 quadcopter arms) */}
      {[
        [0.5, 0, 0.5],   // Front-right
        [-0.5, 0, 0.5],  // Front-left
        [-0.5, 0, -0.5], // Back-left
        [0.5, 0, -0.5]   // Back-right
      ].map((armPos, index) => (
        <group key={`arm-${index}`} position={armPos}>
          {/* Arm */}
          <Cylinder args={[0.03, 0.03, 0.4 * shapeScale[0]]} position={[0, 0, 0]} rotation={[0, 0, Math.PI / 2]}>
            <meshStandardMaterial color="#1f2937" metalness={0.9} roughness={0.1} />
          </Cylinder>
          
          {/* Motor */}
          <Cylinder args={[0.08, 0.08, 0.1]} position={[0.2 * shapeScale[0], 0, 0]}>
            <meshStandardMaterial color="#374151" metalness={0.8} roughness={0.2} />
          </Cylinder>
          
          {/* Propeller */}
          <group ref={propellerRefs[index]} position={[0.2 * shapeScale[0], 0.08, 0]}>
            <Box args={[0.4, 0.01, 0.05]}>
              <meshStandardMaterial 
                color={flying ? '#60a5fa' : '#9ca3af'} 
                transparent 
                opacity={flying ? 0.6 : 0.9}
                metalness={0.5}
                roughness={0.3}
              />
            </Box>
            <Box args={[0.05, 0.01, 0.4]}>
              <meshStandardMaterial 
                color={flying ? '#60a5fa' : '#9ca3af'} 
                transparent 
                opacity={flying ? 0.6 : 0.9}
                metalness={0.5}
                roughness={0.3}
              />
            </Box>
          </group>

          {/* LED Indicator */}
          <Sphere args={[0.03]} position={[0.1 * shapeScale[0], 0, 0]}>
            <meshStandardMaterial 
              color={ledColor} 
              emissive={ledColor}
              emissiveIntensity={armed ? 2 : 0.5}
            />
          </Sphere>
        </group>
      ))}

      {/* Camera/Sensor Pod */}
      <Sphere args={[0.12]} position={[0, -0.15, 0.3]}>
        <meshStandardMaterial color="#111827" metalness={0.9} roughness={0.1} />
      </Sphere>

      {/* GPS Antenna */}
      <Cylinder args={[0.02, 0.02, 0.1]} position={[0, 0.15, 0]}>
        <meshStandardMaterial color="#ef4444" emissive="#ef4444" emissiveIntensity={0.5} />
      </Cylinder>

      {/* Grasp Mechanism (when active) */}
      {graspMode && (
        <group position={[0, -0.25, 0]}>
          <Box args={[0.15, 0.05, 0.15]}>
            <meshStandardMaterial color="#fbbf24" metalness={0.6} roughness={0.4} />
          </Box>
          <Cylinder args={[0.02, 0.02, 0.15]} position={[-0.08, -0.1, 0]}>
            <meshStandardMaterial color="#78716c" metalness={0.7} roughness={0.3} />
          </Cylinder>
          <Cylinder args={[0.02, 0.02, 0.15]} position={[0.08, -0.1, 0]}>
            <meshStandardMaterial color="#78716c" metalness={0.7} roughness={0.3} />
          </Cylinder>
        </group>
      )}

      {/* Battery Pack (visible on underside) */}
      <Box args={[0.3, 0.08, 0.2]} position={[0, -0.12, 0]}>
        <meshStandardMaterial color="#1e40af" metalness={0.6} roughness={0.4} />
      </Box>
    </group>
  );
}

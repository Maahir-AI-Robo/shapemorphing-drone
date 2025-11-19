import { Canvas } from '@react-three/fiber';
import { OrbitControls, Grid, Environment, PerspectiveCamera, Sky } from '@react-three/drei';
import Drone3D from './Drone3D';
import * as THREE from 'three';

// Indoor Room Component
function IndoorRoom({ bounds }) {
  return (
    <group>
      {/* Floor */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0, 0]} receiveShadow>
        <planeGeometry args={[bounds.width / 10, bounds.depth / 10]} />
        <meshStandardMaterial color="#e5e7eb" roughness={0.8} />
      </mesh>

      {/* Walls */}
      {/* Back Wall */}
      <mesh position={[0, bounds.height / 20, -bounds.depth / 20]} receiveShadow>
        <boxGeometry args={[bounds.width / 10, bounds.height / 10, 0.1]} />
        <meshStandardMaterial color="#f3f4f6" roughness={0.9} />
      </mesh>

      {/* Left Wall */}
      <mesh position={[-bounds.width / 20, bounds.height / 20, 0]} receiveShadow>
        <boxGeometry args={[0.1, bounds.height / 10, bounds.depth / 10]} />
        <meshStandardMaterial color="#f3f4f6" roughness={0.9} />
      </mesh>

      {/* Right Wall */}
      <mesh position={[bounds.width / 20, bounds.height / 20, 0]} receiveShadow>
        <boxGeometry args={[0.1, bounds.height / 10, bounds.depth / 10]} />
        <meshStandardMaterial color="#f3f4f6" roughness={0.9} />
      </mesh>

      {/* Ceiling */}
      <mesh rotation={[Math.PI / 2, 0, 0]} position={[0, bounds.height / 10, 0]} receiveShadow>
        <planeGeometry args={[bounds.width / 10, bounds.depth / 10]} />
        <meshStandardMaterial color="#ffffff" roughness={0.7} />
      </mesh>

      {/* Windows */}
      <mesh position={[bounds.width / 30, bounds.height / 15, -bounds.depth / 20 + 0.05]}>
        <boxGeometry args={[2, 1.5, 0.05]} />
        <meshStandardMaterial color="#93c5fd" transparent opacity={0.3} roughness={0.1} metalness={0.9} />
      </mesh>

      {/* Door */}
      <mesh position={[-bounds.width / 20 + 0.05, bounds.height / 30, bounds.depth / 30]}>
        <boxGeometry args={[0.05, 2, 1]} />
        <meshStandardMaterial color="#92400e" roughness={0.7} />
      </mesh>

      {/* Furniture - Table */}
      <group position={[bounds.width / 30, 0, bounds.depth / 40]}>
        <mesh position={[0, 0.4, 0]}>
          <boxGeometry args={[1.5, 0.05, 1]} />
          <meshStandardMaterial color="#78716c" roughness={0.6} />
        </mesh>
        {/* Table Legs */}
        {[[0.7, 0, 0.45], [0.7, 0, -0.45], [-0.7, 0, 0.45], [-0.7, 0, -0.45]].map((pos, i) => (
          <mesh key={i} position={pos}>
            <cylinderGeometry args={[0.03, 0.03, 0.8]} />
            <meshStandardMaterial color="#57534e" />
          </mesh>
        ))}
      </group>

      {/* Package on table */}
      <mesh position={[bounds.width / 30, 0.45, bounds.depth / 40]} castShadow>
        <boxGeometry args={[0.2, 0.2, 0.2]} />
        <meshStandardMaterial color="#92400e" roughness={0.8} />
      </mesh>

      {/* Delivery target marker on floor */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[-bounds.width / 40, 0.01, -bounds.depth / 50]}>
        <ringGeometry args={[0.3, 0.4, 32]} />
        <meshStandardMaterial color="#10b981" emissive="#10b981" emissiveIntensity={0.5} />
      </mesh>

      {/* Indoor Lighting */}
      <pointLight position={[0, bounds.height / 10 - 0.5, 0]} intensity={1} castShadow />
      <pointLight position={[bounds.width / 25, bounds.height / 15, bounds.depth / 25]} intensity={0.5} />
      <ambientLight intensity={0.4} />
    </group>
  );
}

// Outdoor Terrain Component
function OutdoorTerrain() {
  return (
    <group>
      {/* Ground Plane */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.1, 0]} receiveShadow>
        <planeGeometry args={[50, 50]} />
        <meshStandardMaterial color="#10b981" roughness={0.9} />
      </mesh>

      {/* Buildings */}
      {/* Building 1 */}
      <mesh position={[-8, 3, -10]} castShadow receiveShadow>
        <boxGeometry args={[4, 6, 4]} />
        <meshStandardMaterial color="#6b7280" roughness={0.7} />
      </mesh>

      {/* Building 2 */}
      <mesh position={[10, 4, -8]} castShadow receiveShadow>
        <boxGeometry args={[5, 8, 5]} />
        <meshStandardMaterial color="#4b5563" roughness={0.7} />
      </mesh>

      {/* Building 3 */}
      <mesh position={[-12, 2.5, 8]} castShadow receiveShadow>
        <boxGeometry args={[3, 5, 3]} />
        <meshStandardMaterial color="#9ca3af" roughness={0.7} />
      </mesh>

      {/* Trees */}
      {[
        [5, 0, 5],
        [-6, 0, 7],
        [8, 0, 12],
        [-10, 0, -5]
      ].map((pos, i) => (
        <group key={`tree-${i}`} position={pos}>
          {/* Trunk */}
          <mesh position={[0, 0.5, 0]}>
            <cylinderGeometry args={[0.2, 0.3, 1]} />
            <meshStandardMaterial color="#78350f" roughness={0.9} />
          </mesh>
          {/* Foliage */}
          <mesh position={[0, 1.5, 0]}>
            <coneGeometry args={[0.8, 1.5, 8]} />
            <meshStandardMaterial color="#166534" roughness={0.8} />
          </mesh>
        </group>
      ))}

      {/* Landing Pad */}
      <group position={[0, 0.02, 0]}>
        <mesh rotation={[-Math.PI / 2, 0, 0]}>
          <circleGeometry args={[2, 32]} />
          <meshStandardMaterial color="#1e3a8a" roughness={0.3} />
        </mesh>
        <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.01, 0]}>
          <ringGeometry args={[1.8, 2, 32]} />
          <meshStandardMaterial color="#fbbf24" emissive="#fbbf24" emissiveIntensity={0.5} />
        </mesh>
      </group>

      {/* Package Location */}
      <mesh position={[3, 0.15, 4]} castShadow>
        <boxGeometry args={[0.3, 0.3, 0.3]} />
        <meshStandardMaterial color="#92400e" roughness={0.8} />
      </mesh>

      {/* Delivery Target */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[9, 0.02, 2]}>
        <ringGeometry args={[0.4, 0.5, 32]} />
        <meshStandardMaterial color="#10b981" emissive="#10b981" emissiveIntensity={0.8} />
      </mesh>

      {/* Outdoor Lighting */}
      <directionalLight 
        position={[10, 10, 5]} 
        intensity={1} 
        castShadow 
        shadow-mapSize-width={2048}
        shadow-mapSize-height={2048}
      />
      <ambientLight intensity={0.5} />
    </group>
  );
}

export default function Scene3D({ 
  dronePosition, 
  droneRotation,
  flying,
  armed,
  throttle,
  shapeMode,
  graspMode,
  isIndoor,
  indoorBounds
}) {
  // Convert percentage position to 3D coordinates
  const scale = isIndoor ? 0.1 : 0.5;
  const position3D = [
    (dronePosition.x - 50) * scale,
    dronePosition.altitude * 0.1 || 0.5,
    (dronePosition.y - 50) * scale
  ];

  return (
    <div className="w-full h-full" style={{ background: isIndoor ? '#f3f4f6' : '#0ea5e9' }}>
      <Canvas shadows camera={{ position: [10, 8, 10], fov: 50 }}>
        <PerspectiveCamera makeDefault position={[10, 8, 10]} />
        <OrbitControls 
          enableDamping 
          dampingFactor={0.05}
          minDistance={3}
          maxDistance={30}
          maxPolarAngle={Math.PI / 2}
        />

        {/* Environment */}
        {isIndoor ? (
          <>
            <IndoorRoom bounds={indoorBounds} />
            <fog attach="fog" args={['#f3f4f6', 10, 50]} />
          </>
        ) : (
          <>
            <Sky 
              distance={450000}
              sunPosition={[100, 20, 100]}
              inclination={0.6}
              azimuth={0.25}
            />
            <OutdoorTerrain />
            <Environment preset="city" />
            <fog attach="fog" args={['#87ceeb', 20, 100]} />
          </>
        )}

        {/* Grid Helper */}
        <Grid 
          args={[isIndoor ? 10 : 50, isIndoor ? 10 : 50]}
          cellColor="#6b7280"
          sectionColor="#4b5563"
          fadeDistance={isIndoor ? 15 : 40}
          fadeStrength={1}
        />

        {/* Drone */}
        <Drone3D 
          position={position3D}
          rotation={droneRotation}
          flying={flying}
          armed={armed}
          throttle={throttle}
          shapeMode={shapeMode}
          graspMode={graspMode}
        />
      </Canvas>
    </div>
  );
}

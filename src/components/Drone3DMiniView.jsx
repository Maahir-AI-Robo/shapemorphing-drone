import { Canvas } from '@react-three/fiber';
import { OrbitControls, PerspectiveCamera } from '@react-three/drei';
import Drone3D from './Drone3D';

export default function Drone3DMiniView({ 
  droneRotation = [0, 0, 0],
  flying = false,
  armed = false,
  throttle = 0,
  shapeMode = 'standard',
  graspMode = false,
  altitude = 0
}) {
  // Calculate realistic vertical position based on actual altitude
  // Scale altitude for mini-view visualization (1:10 scale for better viewing)
  const visualAltitudeScale = 0.08; // 10m real = 0.8m in mini view
  const baseHeight = 0.3; // Base height to keep drone visible
  const visualAltitude = Math.min(altitude * visualAltitudeScale + baseHeight, 3);
  
  const position3D = [0, visualAltitude, 0];
  
  // Calculate camera position that adapts to drone altitude
  const cameraDistance = 3.5;
  const cameraHeight = Math.max(visualAltitude, 1.5);
  const cameraPos = [cameraDistance, cameraHeight, cameraDistance];

  return (
    <div className="w-full h-full rounded-lg overflow-hidden border-2 border-purple-500/50" style={{ background: 'linear-gradient(135deg, #1e293b 0%, #0f172a 100%)' }}>
      <Canvas camera={{ position: cameraPos, fov: 50 }}>
        <PerspectiveCamera makeDefault position={cameraPos} fov={50} />
        <OrbitControls 
          enableDamping 
          dampingFactor={0.12}
          rotateSpeed={0.7}
          minDistance={2}
          maxDistance={8}
          enablePan={false}
          target={[0, visualAltitude * 0.8, 0]}
        />

        {/* Lighting */}
        <ambientLight intensity={0.7} />
        <directionalLight position={[5, 5, 5]} intensity={1.2} />
        <pointLight position={[0, 3, 0]} intensity={0.6} color="#a855f7" />
        <hemisphereLight args={['#87ceeb', '#2d3748', 0.5]} />

        {/* Ground plane for reference */}
        <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0, 0]} receiveShadow>
          <planeGeometry args={[10, 10]} />
          <meshStandardMaterial color="#1e293b" opacity={0.5} transparent />
        </mesh>

        {/* Grid */}
        <gridHelper args={[10, 10, '#4b5563', '#374151']} position={[0, 0, 0]} />

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

        {/* Altitude reference line connecting drone to ground */}
        {altitude > 0.5 && (
          <>
            {/* Vertical line showing altitude */}
            <line>
              <bufferGeometry>
                <bufferAttribute
                  attach="attributes-position"
                  count={2}
                  array={new Float32Array([
                    0, 0.02, 0,
                    0, visualAltitude, 0
                  ])}
                  itemSize={3}
                />
              </bufferGeometry>
              <lineBasicMaterial color="#a855f7" opacity={0.5} transparent linewidth={2} />
            </line>
            
            {/* Ground shadow projection with realistic size based on altitude */}
            <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.005, 0]}>
              <circleGeometry args={[0.35, 32]} />
              <meshBasicMaterial 
                color="#000000" 
                opacity={Math.max(0.2, 0.5 - altitude * 0.005)} 
                transparent 
                depthWrite={false}
              />
            </mesh>
          </>
        )}
        
        {/* Flight path trail effect */}
        {flying && (
          <mesh position={[0, position3D[1] - 0.1, 0]}>
            <sphereGeometry args={[0.05]} />
            <meshBasicMaterial color="#4ade80" opacity={0.2} transparent />
          </mesh>
        )}
        
        {/* Coordinate axes helper */}
        <group position={[0, 0.01, 0]}>
          {/* X axis - Red */}
          <mesh position={[0.3, 0, 0]} rotation={[0, 0, Math.PI / 2]}>
            <cylinderGeometry args={[0.005, 0.005, 0.6]} />
            <meshBasicMaterial color="#ef4444" />
          </mesh>
          {/* Z axis - Blue */}
          <mesh position={[0, 0, 0.3]} rotation={[Math.PI / 2, 0, 0]}>
            <cylinderGeometry args={[0.005, 0.005, 0.6]} />
            <meshBasicMaterial color="#3b82f6" />
          </mesh>
        </group>
      </Canvas>
    </div>
  );
}

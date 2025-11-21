// Coordinate system utilities for accurate 3D positioning
// Map coordinates: 0-100 grid (x: left-right, y: top-bottom)
// Scene coordinates: 3D space in meters (x: left-right, y: up-down, z: front-back)

export const MAP_SIZE = 100;
export const OUTDOOR_BOUNDS = {
  width: 100,  // meters - full outdoor area
  depth: 100   // meters
};

/**
 * Clamp map position within valid bounds
 */
export function clampMapPosition(position, margin = 5) {
  return {
    x: Math.max(margin, Math.min(MAP_SIZE - margin, position.x)),
    y: Math.max(margin, Math.min(MAP_SIZE - margin, position.y))
  };
}

/**
 * Get environment dimensions based on indoor/outdoor mode
 */
export function getEnvironmentDimensions(isIndoor, indoorBounds = null) {
  if (isIndoor && indoorBounds) {
    return { width: indoorBounds.width, depth: indoorBounds.depth };
  }
  return { ...OUTDOOR_BOUNDS };
}

/**
 * Convert map coordinates (0-100) to 3D scene coordinates (meters)
 * Map origin (0,0) = top-left, (100,100) = bottom-right
 * Scene origin (0,0) = center, +X = right, +Z = forward/down
 */
export function mapToSceneCoords(
  mapPosition,
  { isIndoor = false, indoorBounds = null, margin = 0 } = {}
) {
  if (!mapPosition) {
    return { x: 0, z: 0 };
  }

  const { width, depth } = getEnvironmentDimensions(isIndoor, indoorBounds);
  
  // Normalize to -0.5 to +0.5 range, then scale to environment dimensions
  const normalizedX = (mapPosition.x / MAP_SIZE) - 0.5;
  const normalizedZ = (mapPosition.y / MAP_SIZE) - 0.5;
  
  let x = normalizedX * width;
  let z = normalizedZ * depth;

  // Apply margin constraints for indoor environments
  if (isIndoor && indoorBounds && margin > 0) {
    const halfWidth = (indoorBounds.width / 2) - margin;
    const halfDepth = (indoorBounds.depth / 2) - margin;
    x = Math.max(-halfWidth, Math.min(halfWidth, x));
    z = Math.max(-halfDepth, Math.min(halfDepth, z));
  }

  return { x, z };
}

/**
 * Convert 3D scene coordinates (meters) back to map coordinates (0-100)
 */
export function sceneToMapCoords(
  coords,
  { isIndoor = false, indoorBounds = null, margin = 5 } = {}
) {
  if (!coords) {
    return { x: MAP_SIZE / 2, y: MAP_SIZE / 2 };
  }

  const { width, depth } = getEnvironmentDimensions(isIndoor, indoorBounds);
  
  // Normalize from meters to -0.5 to +0.5 range, then scale to map size
  const normalizedX = (coords.x / width) + 0.5;
  const normalizedZ = (coords.z / depth) + 0.5;

  return clampMapPosition({
    x: normalizedX * MAP_SIZE,
    y: normalizedZ * MAP_SIZE
  }, margin);
}

/**
 * Calculate real-world distance in meters between two map positions
 */
export function mapDistanceToMeters(
  p1,
  p2,
  { isIndoor = false, indoorBounds = null } = {}
) {
  const a = mapToSceneCoords(p1, { isIndoor, indoorBounds });
  const b = mapToSceneCoords(p2, { isIndoor, indoorBounds });
  return Math.hypot(b.x - a.x, b.z - a.z);
}

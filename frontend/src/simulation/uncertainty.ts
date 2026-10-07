export const POSITION_95_CHI_SQUARE = 5.991;
const TINY_NEGATIVE_EIGENVALUE = 1e-10;
const ZERO_EIGENVALUE = 1e-12;

export type PositionEllipse = { majorRadius: number; minorRadius: number; angleRadians: number };

export function positionUncertaintyEllipse(covariance: number[][]): PositionEllipse | null {
  if (covariance.length < 2 || covariance[0].length < 2 || covariance[1].length < 2) {
    throw new Error('position covariance must contain a 2×2 x/y block');
  }
  const a = covariance[0][0];
  const b = (covariance[0][1] + covariance[1][0]) / 2;
  const d = covariance[1][1];
  if (![a, b, d].every(Number.isFinite)) throw new Error('position covariance contains a non-finite value');
  const trace = a + d;
  const discriminant = Math.sqrt(Math.max(0, (a - d) ** 2 + 4 * b ** 2));
  const eigenvalues = [(trace + discriminant) / 2, (trace - discriminant) / 2];
  if (eigenvalues.some((value) => value < -TINY_NEGATIVE_EIGENVALUE)) {
    throw new Error('position covariance has a materially negative eigenvalue');
  }
  const major = Math.max(0, eigenvalues[0]);
  const minor = Math.max(0, eigenvalues[1]);
  if (major <= ZERO_EIGENVALUE) return null;
  return {
    majorRadius: Math.sqrt(major * POSITION_95_CHI_SQUARE),
    minorRadius: Math.sqrt(minor * POSITION_95_CHI_SQUARE),
    angleRadians: .5 * Math.atan2(2 * b, a - d),
  };
}

export function positionEllipsePoints(centerX: number, centerY: number, covariance: number[][], count = 32): [number, number][] {
  const ellipse = positionUncertaintyEllipse(covariance);
  if (!ellipse) return [];
  return Array.from({ length: count }, (_, index) => {
    const angle = index * 2 * Math.PI / count;
    const majorX = ellipse.majorRadius * Math.cos(angle);
    const minorY = ellipse.minorRadius * Math.sin(angle);
    return [
      centerX + majorX * Math.cos(ellipse.angleRadians) - minorY * Math.sin(ellipse.angleRadians),
      centerY + majorX * Math.sin(ellipse.angleRadians) + minorY * Math.cos(ellipse.angleRadians),
    ];
  });
}

package frc.robot.utils;

import edu.wpi.first.math.util.Units;

class rotationVals {
    public static final int yaw = 0;
    public static final int pitch = 1;
    public static final int roll = 2;
}

public class RingPoseEstimator {
    
    public static double[] calculatePose(int[][] boundBoxMinMax, 
                        double[] ringRotationVals, double ringDiameter, 
                            int[] camDimensions, double camFOVDeg, 
                                double[] camTransform, double[] camRotationVals) {
        
        int xCenter = ((boundBoxMinMax[0][0]) + (boundBoxMinMax[1][0])) / 2;
        int yCenter = ((boundBoxMinMax[1][0]) + (boundBoxMinMax[1][1])) / 2;

        int xNDC = (int) (((2 * xCenter / camDimensions[0]) - 1) * Math.tan(Units.degreesToRadians(camFOVDeg)));
        int yNDC = (int) (((2 * yCenter / camDimensions[1]) - 1) * Math.tan(Units.degreesToRadians(camFOVDeg)));

        double aspectRatio = camDimensions[0] / camDimensions[1];

        double camFOV_vert = 2 * Math.atan(Math.tan(Units.degreesToRadians(camFOVDeg) / 2) / aspectRatio);

        double xCamSpace = xNDC * Math.tan(camFOV_vert / 2);
        double yCamSpace = yNDC * Math.tan(camFOV_vert / 2);
        double zCamSpace = 1.0;

        double dImage = ((boundBoxMinMax[0][1] - boundBoxMinMax[0][0]) + (boundBoxMinMax[1][1] - boundBoxMinMax[1][0]));
        double d = ringDiameter / (dImage * (camDimensions[0] / 2 / Math.tan(Units.degreesToRadians(camFOVDeg) / 2)));

        double[][] RRingYaw = {
            {Math.cos(Units.degreesToRadians(ringRotationVals[rotationVals.yaw])), -Math.sin(Units.degreesToRadians(ringRotationVals[rotationVals.yaw])), 0},
            {Math.sin(Units.degreesToRadians(ringRotationVals[rotationVals.yaw])), Math.cos(Units.degreesToRadians(ringRotationVals[rotationVals.yaw])), 0},
            {0, 0, 1}
        };

        double[][] RRingPitch = {
            {Math.cos(Units.degreesToRadians(ringRotationVals[rotationVals.pitch])), 0, Math.sin(Units.degreesToRadians(ringRotationVals[rotationVals.pitch]))},
            {0, 1, 0},
            {-Math.sin(Units.degreesToRadians(ringRotationVals[rotationVals.pitch])), 0, Math.cos(Units.degreesToRadians(ringRotationVals[rotationVals.pitch]))}
        };

        double[][] RRingRoll = {
            {1, 0, 0},
            {0, Math.cos(Units.degreesToRadians(ringRotationVals[rotationVals.roll])), -Math.sin(Units.degreesToRadians(ringRotationVals[rotationVals.roll]))},
            {0, Math.sin(Units.degreesToRadians(ringRotationVals[rotationVals.roll])), Math.cos(Units.degreesToRadians(ringRotationVals[rotationVals.roll]))}
        };

        double[][] RRing = multiplyMatrix(3, 3, RRingRoll,
                                            3, 3, multiplyMatrix(3, 3, RRingPitch,     
                                                                        3, 3, RRingYaw));
        double[] camSpace = {xCamSpace, yCamSpace, zCamSpace};
        double[] RingRotation = multiplyMatrixVector(RRing, camSpace);
        
        double[][] RCamYaw = {
            {Math.cos(Units.degreesToRadians(camRotationVals[rotationVals.yaw])), -Math.sin(Units.degreesToRadians(camRotationVals[rotationVals.yaw])), 0},
            {Math.sin(Units.degreesToRadians(camRotationVals[rotationVals.yaw])), Math.cos(Units.degreesToRadians(camRotationVals[rotationVals.yaw])), 0},
            {0, 0, 1}
        };

        double[][] RCamPitch = {
            {Math.cos(Units.degreesToRadians(camRotationVals[rotationVals.pitch])), 0, Math.sin(Units.degreesToRadians(camRotationVals[rotationVals.pitch]))},
            {0, 1, 0},
            {-Math.sin(Units.degreesToRadians(camRotationVals[rotationVals.pitch])), 0, Math.cos(Units.degreesToRadians(camRotationVals[rotationVals.pitch]))}
        };

        double[][] RCamRoll = {
            {1, 0, 0},
            {0, Math.cos(Units.degreesToRadians(camRotationVals[rotationVals.roll])), -Math.sin(Units.degreesToRadians(camRotationVals[rotationVals.roll]))},
            {0, Math.sin(Units.degreesToRadians(camRotationVals[rotationVals.roll])), Math.cos(Units.degreesToRadians(camRotationVals[rotationVals.roll]))}
        };

        double[][] RCam = multiplyMatrix(3, 3, RCamRoll, 3, 3, multiplyMatrix(3, 3, RCamPitch, 3, 3, RCamYaw));

        double[] camRotation = multiplyMatrixVector(RCam, RingRotation);

        double worldX = camTransform[0] + d * camRotation[0];
        double worldY = camTransform[1] + d * camRotation[1];
        double worldZ = camTransform[2] + d * camRotation[2];

        double[] result = {worldX, worldY, worldZ};

        return result;

    }

    private static double[][] multiplyMatrix(int r1, int c1, double[][] matrix1, int r2, int c2, double[][] matrix2) {
        double[][] product = new double[r1][c2];
        for(int i = 0; i < r1; i++) {
            for (int j = 0; j < c2; j++) {
                for (int k = 0; k < c1; k++) {
                    product[i][j] += matrix1[i][k] * matrix2[k][j];
                }
            }
        }

        return product;
    }

    private static double[] multiplyMatrixVector(double[][] matrix, double[] vector) {
        double[] result = new double[3];

        for (int i = 0; i < 3; i++) {
            result[i] = 0;
            for (int j = 0; j < 3; j++) {
                result[i] += matrix[i][j] * vector[j];
            }
        }

        return result;
    }
}

package org.firstinspires.ftc.teamcode.drive.advanced;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class DetectionPipeline extends OpenCvPipeline {
    private static final Scalar BLUE = new Scalar(0, 0, 255);

    private static final int THRESHOLD = 120;

    Mat luminosityMat = new Mat();
    Mat extractionMat = new Mat();

    private static int gridSize = 3;
    private static int cols,rows;
    private static int numberOfElements = 0;

    private void inputToLuminosity(Mat input) {
        Imgproc.cvtColor(input, extractionMat, Imgproc.COLOR_RGB2YCrCb); // convert rgb to chroma and luminosity

        Core.extractChannel(extractionMat, luminosityMat, 2);
    }

    private Mat[]   allMats = new Mat[256];
    private boolean[] validZones = new boolean[1001];
    
    @Override
    public void init(Mat input)
    {
        inputToLuminosity(input);
        
        cols = input.cols();
        rows = input.rows();

        for(int i = 0; i < gridSize * gridSize;i++)
            validZones[i] = false;
    }
    
    @Override
    public Mat processFrame(Mat input) 
    {
        inputToLuminosity(input);

        numberOfElements = 0;

        for(int i = 0; i < cols;i+= cols / gridSize)
        {
            for(int j = 0; j < rows; j += rows / gridSize)
            {
                int topLeftX = i;
                int topLeftY = j;

                int botRightX = i + cols / gridSize - 1;
                int botRightY = j + rows / gridSize - 1;

                if(botRightX >= cols || botRightY >= rows) continue;

                Point p1 = new Point(topLeftX,topLeftY);

                Point p2 = new Point(botRightX, botRightY);

                Point center = new Point((topLeftX + botRightX) / 2 - 25, (topLeftY + botRightY) / 2);

                allMats[numberOfElements] = luminosityMat.submat(new Rect(p1,p2));

                String zoneName = Integer.toString((numberOfElements + 1));

                Imgproc.putText(input, zoneName, center , Imgproc.FONT_ITALIC , 1, BLUE,1);

                Imgproc.rectangle(input, p1, p2, BLUE, 2);

                validZones[numberOfElements] = Core.mean(allMats[numberOfElements]).val[0] < THRESHOLD;

                numberOfElements++;
            }

        }

        return input;
    }

   // public boolean[] getValidZones() { return validZones; }
    public void setGridSize(int gridSize) { this.gridSize = gridSize;}

    public int getGridSize() { return this.gridSize;}


    public ZoneType getZoneType(int zone) // zone parameter IS index 1 based
    {

//        if(zone <= gridSize * gridSize / 3)
//            return ZoneType.E_LEFT;
//        else if(zone <= gridSize * gridSize / 3 * 2)
//            return ZoneType.E_CENTER;
//        else
//            return ZoneType.E_RIGHT;

        if(zone <= 10)
            return ZoneType.E_LEFT;
        else if(zone <= 15)
            return ZoneType.E_CENTER;
        else
            return ZoneType.E_RIGHT;
    }

    public int getRow(int zone) // zone parameter IS index 1 based
    {
        int row = zone % 5;

        if (row == 0) return gridSize;

        return row;
    }

    public int getColumn(int zone) // zone parameter IS index 1 based
    {
        int column = zone / gridSize;

        if(zone % gridSize != 0)
        {
            column = column + 1;
        }

        return column;
    }


    public int getDuckZone() // returns zone INDEX 1 BASED
    {
        int bestZone = 0;
        double bestAverage = 10000;
        for(int i = 0; i < allMats.length;i++) {
            if(!validZones[i]) continue;

            double current_avg = Core.mean(allMats[i]).val[0];

            if(current_avg < bestAverage)
            {
                bestAverage = current_avg;
                bestZone = i + 1;
            }
        }

        return bestZone;

    }
    public int getBestZone(ZoneType preferredZone) { // index 1 based zones
        int bestZone = 0;
        int bestRow = 0;
        ZoneType bestZoneType = ZoneType.E_NONE;

        boolean foundCenterZone = false;
        boolean foundPrefferedZone = false;

        for(int i = 0; i < allMats.length;i++) {
            if(!validZones[i]) continue;

            ZoneType zoneType = getZoneType(i + 1);

            int row = getRow(i + 1);
            int column = getColumn(i + 1);
            if(row > bestRow)
            {
                bestZone = i + 1;
                bestZoneType = zoneType;
                bestRow = row;
            }
            else if(row == bestRow)
            {
                if(zoneType == ZoneType.E_CENTER){
                    bestZone = i + 1;
                    bestZoneType = zoneType;
                    bestRow = row;
                }
                else if(zoneType == bestZoneType)
                {
                    if(Math.abs(column - gridSize) < Math.abs(getColumn(bestZone) - gridSize)) // adica e mai aproape de centru
                    {
                        bestZone = i + 1;
                        bestZoneType = zoneType;
                        bestRow = row;
                    }
                }
                else if(zoneType == preferredZone && bestZoneType != ZoneType.E_CENTER)
                {
                    bestZone = i + 1;
                    bestZoneType = zoneType;
                    bestRow = row;
                }

            }
        }

        return bestZone;
    }

    public boolean isZoneValid(int zone) { return validZones[zone - 1]; }; // index 1 based

    public double getZoneLuminosity(int zone) { return Core.mean(allMats[zone - 1]).val[0]; } // index 1 based

    public enum ZoneType
    {
        E_NONE,E_LEFT,E_RIGHT,E_CENTER
    }

}

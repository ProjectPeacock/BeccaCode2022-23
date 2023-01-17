package org.firstinspires.ftc.teamcode.Libs;

public class AutoParams {
    /*

    PARAMETER MAP - PLEASE READ BEFORE EDITING

    This file contains all the parameters for BlueTerminalAuto and RedTerminalAuto.
    This file also contains all Lift encoder values for cone retrieval from the stack.

    If making a new auto, include "AutoParams params = new AutoParams();" within the class.

    All headings are converted to radians here, so there is no need to convert them in the main autonomous programs.

     */

    //lift hieghts for grabbing cones from stack
    public final int cycle1=10;
    public final int cycle2=20;
    public final int cycle3=30;
    public final int cycle4=40;
    public final int cycle5=50;

    //staring pose X and Y
    public final double startPoseX=-64;
    public final double startPoseY=-35.25;

//preload parameters
    //scoring position for preload on mid pole X, Y, and Heading
    public final double preloadMidX=-36;
    public final double preloadMidY=-30.5;
    public final double preloadMidHeading=Math.toRadians(30);

    //amount to drive forward for final preload pole alignment
    public final double preloadMidForward=4;

    //amount to drive backward for preload retreat
    public final double preloadMidBackward=8;

    //amount to turn to reorient to original heading after scoring preload
    public final double preloadReorientHeading=-60;

    //waypoint to cone stack X, Y, and Heading
    public final double preloadWaypointX=-12;
    public final double preloadWaypointY=-45;
    public final double preloadWaypointHeading=Math.toRadians(270);

//mid pole cycle parameters
    //reverse dist from cone stack
    public final double coneStackReverse=12;

    //mid pole cycle scoring position X, Y, and Heading
    public final double cycleMidX=-16;
    public final double cycleMidY=-32.5;
    public final double cycleMidHeading=Math.toRadians(130);
    public final double cycleMidEndHeading=Math.toRadians(270);

    //distance to reverse from pole in mid cycle
    public final double cycleMidReverse=6;

    //cone retrieval position X, Y, and Heading
    public final double coneStackAlignX=-12;
    public final double coneStackAlignY=-45;
    public final double coneStackAlignHeading=Math.toRadians(270);
    public final double coneStackAlignEndHeading=Math.toRadians(270);

    //approach dist to cone stack
    public final double coneStackForward=14.5;

//high pole cycle parameters
    //X, Y, and Heading of mid pole cycle scoring position
    public final double cycleHighX=-8;
    public final double cycleHighY=-30.5;
    public final double cycleHighHeading=Math.toRadians(40);
    public final double cycleHighEndHeading=Math.toRadians(270);

//parking parameters
    //park 1 reverse dist
    public final double park1Reverse=3;

    //park 1 X, Y, and Heading
    public final double park1X=-12;
    public final double park1Y=-12;
    public final double park1Heading=Math.toRadians(0);
    public final double park1EndHeading=Math.toRadians(90);

    //park 2 reverse dist
    public final double park2Reverse=8;

    //park 2 turn amount
    public final double park2HeadingAdjust=Math.toRadians(-40);

    //park 3 reverse dist
    public final double park3Reverse=6;

    //park 3 X, Y, and Heading
    public final double park3X=-12;
    public final double park3Y=-45;
    public final double park3Heading=Math.toRadians(270);
    public final double park3EndHeading=Math.toRadians(270);

    //park 3 forward dist
    public final double park3Forward=14.5;

    //park 3 turn amount
    public final double park3Turn=Math.toRadians(90);
}

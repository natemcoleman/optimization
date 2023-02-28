import matplotlib.pyplot as plt
from scipy.optimize import minimize
import CrossFrameOptimizationLibrary
import CrossFrameConstraints
import PointsAndLinesClass
import PolyInteractor
from math import cos, sin, pi
import numpy as np
import csv

global coords

def ModifyPolygon(listOfPoints):
    newPoints = PolyInteractor.CreatePolygon(listOfPoints)
    listOfPoints = []

    if len(newPoints) > 4:
        firstPoint = PointsAndLinesClass.ClassPoint(newPoints[0][0], newPoints[0][1])
        secondPoint = PointsAndLinesClass.ClassPoint(newPoints[1][0], newPoints[1][1])
        thirdPoint = PointsAndLinesClass.ClassPoint(newPoints[2][0], newPoints[2][1])
        fourthPoint = PointsAndLinesClass.ClassPoint(newPoints[3][0], newPoints[3][1])

        listOfPoints.append(firstPoint)
        listOfPoints.append(secondPoint)
        listOfPoints.append(thirdPoint)
        listOfPoints.append(fourthPoint)
    else:
        firstPoint = PointsAndLinesClass.ClassPoint(newPoints[0][0], newPoints[0][1])
        secondPoint = PointsAndLinesClass.ClassPoint(newPoints[1][0], newPoints[1][1])
        thirdPoint = PointsAndLinesClass.ClassPoint(newPoints[2][0], newPoints[2][1])

        listOfPoints.append(firstPoint)
        listOfPoints.append(secondPoint)
        listOfPoints.append(thirdPoint)

    return listOfPoints

def CreateCSV(panels):
   with open('test.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Panel", "X1", "Y1", "X2", "Y2"])
        for i in range(len(panels)):
            for j in range(len(panels[i])):
                xvals1 = panels[i][j].points[0].x
                yvals1 = panels[i][j].points[0].y
                xvals2 = panels[i][j].points[1].x
                yvals2 = panels[i][j].points[1].y
                writer.writerow([i+1, xvals1, yvals1, xvals2, yvals2])


def rotate_point_wrt_center(point_to_be_rotated, angle, center_point=(2, 1)):

    angle = np.deg2rad(angle)

    # print("point to be rotated:", point_to_be_rotated)

    xnew = cos(angle) * (point_to_be_rotated[0] - center_point[0]) - sin(angle) * (
                point_to_be_rotated[1] - center_point[1]) + center_point[0]

    ynew = sin(angle) * (point_to_be_rotated[0] - center_point[0]) + cos(angle) * (
                point_to_be_rotated[1] - center_point[1]) + center_point[1]

    return xnew, ynew

def plotShape(linesToPlot, numberOfPolygonLines, xGuessPoints, yGuessPoints, axisLimits, plotPointGuesses, boolOptions):
    plotEntireFlasher = True

    for p in range(len(linesToPlot)):
        if p < numberOfPolygonLines:
            plt.plot(linesToPlot[p].points[0].x, linesToPlot[p].points[0].y, 'r*')
            plt.plot(linesToPlot[p].points[1].x, linesToPlot[p].points[1].y, 'r*')
            if boolOptions[7]:
                plt.plot([linesToPlot[p].points[0].x, linesToPlot[p].points[1].x], [linesToPlot[p].points[0].y, linesToPlot[p].points[1].y], 'b--')
            # print("")
        else:
            plt.plot(linesToPlot[p].points[0].x, linesToPlot[p].points[0].y, 'go')
            plt.plot(linesToPlot[p].points[1].x, linesToPlot[p].points[1].y, 'go')
            plt.plot([linesToPlot[p].points[0].x, linesToPlot[p].points[1].x], [linesToPlot[p].points[0].y, linesToPlot[p].points[1].y], 'g-')


    if boolOptions[8]:
        for p in range(len(linesToPlot)):
            if p < numberOfPolygonLines:
                rotatedX1, rotatedY1 = rotate_point_wrt_center((linesToPlot[p].points[0].x, linesToPlot[p].points[0].y), 90)
                rotatedX2, rotatedY2 = rotate_point_wrt_center((linesToPlot[p].points[1].x, linesToPlot[p].points[1].y), 90)
                plt.plot(rotatedX1, rotatedY1, 'r*')
                plt.plot(rotatedX2, rotatedY2, 'r*')
                if boolOptions[7]:
                    plt.plot([rotatedX1, rotatedX2], [rotatedY1, rotatedY2], 'b--')
            else:
                rotatedX1, rotatedY1 = rotate_point_wrt_center((linesToPlot[p].points[0].x, linesToPlot[p].points[0].y), 90)
                rotatedX2, rotatedY2 = rotate_point_wrt_center((linesToPlot[p].points[1].x, linesToPlot[p].points[1].y), 90)
                plt.plot(rotatedX1, rotatedY1, 'go')
                plt.plot(rotatedX2, rotatedY2, 'go')
                plt.plot([rotatedX1, rotatedX2],
                         [rotatedY1, rotatedY2], 'g-')

        for p in range(len(linesToPlot)):
            if p < numberOfPolygonLines:
                rotatedX1, rotatedY1 = rotate_point_wrt_center((linesToPlot[p].points[0].x, linesToPlot[p].points[0].y), 180)
                rotatedX2, rotatedY2 = rotate_point_wrt_center((linesToPlot[p].points[1].x, linesToPlot[p].points[1].y), 180)
                plt.plot(rotatedX1, rotatedY1, 'r*')
                plt.plot(rotatedX2, rotatedY2, 'r*')
                if boolOptions[7]:
                    plt.plot([rotatedX1, rotatedX2], [rotatedY1, rotatedY2], 'b--')
            else:
                rotatedX1, rotatedY1 = rotate_point_wrt_center((linesToPlot[p].points[0].x, linesToPlot[p].points[0].y), 180)
                rotatedX2, rotatedY2 = rotate_point_wrt_center((linesToPlot[p].points[1].x, linesToPlot[p].points[1].y), 180)
                plt.plot(rotatedX1, rotatedY1, 'go')
                plt.plot(rotatedX2, rotatedY2, 'go')
                plt.plot([rotatedX1, rotatedX2],
                         [rotatedY1, rotatedY2], 'g-')
        for p in range(len(linesToPlot)):
            if p < numberOfPolygonLines:
                rotatedX1, rotatedY1 = rotate_point_wrt_center((linesToPlot[p].points[0].x, linesToPlot[p].points[0].y), 270)
                rotatedX2, rotatedY2 = rotate_point_wrt_center((linesToPlot[p].points[1].x, linesToPlot[p].points[1].y), 270)
                plt.plot(rotatedX1, rotatedY1, 'r*')
                plt.plot(rotatedX2, rotatedY2, 'r*')
                if boolOptions[7]:
                    plt.plot([rotatedX1, rotatedX2], [rotatedY1, rotatedY2], 'b--')
            else:
                rotatedX1, rotatedY1 = rotate_point_wrt_center((linesToPlot[p].points[0].x, linesToPlot[p].points[0].y), 270)
                rotatedX2, rotatedY2 = rotate_point_wrt_center((linesToPlot[p].points[1].x, linesToPlot[p].points[1].y), 270)
                plt.plot(rotatedX1, rotatedY1, 'go')
                plt.plot(rotatedX2, rotatedY2, 'go')
                plt.plot([rotatedX1, rotatedX2],
                         [rotatedY1, rotatedY2], 'g-')


    if plotPointGuesses:
        for plotLength in range(len(xGuessPoints)):
            plt.plot(xGuessPoints[plotLength], yGuessPoints[plotLength], 'r.')

    ax = plt.gca()
    ax.set_aspect(1)

    plt.xlabel('Y')
    plt.ylabel('X')
    xAxisBuffer = abs(axisLimits[1] - axisLimits[0]) * 0.05
    yAxisBuffer = abs(axisLimits[3] - axisLimits[2]) * 0.05

    # plt.xlim([axisLimits[0] - xAxisBuffer, axisLimits[1] + xAxisBuffer])
    # plt.ylim([axisLimits[2] - yAxisBuffer, axisLimits[3] + yAxisBuffer])

    plt.show()


def OptimizePolygon(listOfPoints, boolOptions, minDistances, crossSectionLengths, material, crossSection):
    connectToMiddlePoint = boolOptions[0]
    plotPointGuesses = boolOptions[1]
    allowModifyPolygon = boolOptions[2]
    tryAnotherPoint = boolOptions[3]
    multipleGuesses = boolOptions[4]
    allowSelectBeginPoint = boolOptions[5]
    boolOptions[7] = True
    boolOptions[8] = False

    shapeBaseLength = crossSectionLengths[0]  # meters, if square or rectangle
    shapeBaseHeight = crossSectionLengths[1]  # meters, if rectangle
    shapeBaseDiameter = crossSectionLengths[2]  # meters, if circle

    if allowModifyPolygon:
        listOfPoints = ModifyPolygon(listOfPoints)

    def onclick(event):
        global ix, iy
        ix, iy = event.xdata, event.ydata

        coords.append((ix, iy))

        if len(coords) == 1:
            fig.canvas.mpl_disconnect(cid)
            plt.close(1)
        return

    if connectToMiddlePoint:
        def functionToMinimize(optimalPoints):
            point5New = PointsAndLinesClass.ClassPoint(optimalPoints[0], optimalPoints[1])
            point6New = PointsAndLinesClass.ClassPoint(optimalPoints[2], optimalPoints[3])
            point7New = PointsAndLinesClass.ClassPoint(optimalPoints[4], optimalPoints[5])
            point8New = PointsAndLinesClass.ClassPoint(optimalPoints[6], optimalPoints[7])
            if len(optimalPoints) > 9:
                point9New = PointsAndLinesClass.ClassPoint(optimalPoints[8], optimalPoints[9])
            if plotPointGuesses:
                x5GuessPoints.append(optimalPoints[0])
                y5GuessPoints.append(optimalPoints[1])
                x6GuessPoints.append(optimalPoints[2])
                y6GuessPoints.append(optimalPoints[3])
                x7GuessPoints.append(optimalPoints[4])
                y7GuessPoints.append(optimalPoints[5])
                x8GuessPoints.append(optimalPoints[6])
                y8GuessPoints.append(optimalPoints[7])
                if len(optimalPoints) > 9:
                    x9GuessPoints.append(optimalPoints[8])
                    y9GuessPoints.append(optimalPoints[9])

            line5Opt = PointsAndLinesClass.ClassLine(point6New, point5New)
            line6Opt = PointsAndLinesClass.ClassLine(point7New, point5New)
            line7Opt = PointsAndLinesClass.ClassLine(point8New, point5New)
            pathLinesNew = [line5Opt, line6Opt, line7Opt]

            if len(optimalPoints) > 9:
                line8Opt = PointsAndLinesClass.ClassLine(point9New, point5New)
                pathLinesNew.append(line8Opt)

            # return CrossFrameOptimizationLibrary.CalculateStiffnessOfPanels(pathLinesNew, listOfPoints)
            return CrossFrameOptimizationLibrary.GetMassOfAllLines(pathLinesNew, A, rho)
            # return -GetMassOfAllLines(pathLinesNew)
            # return TempStiffnessCalc(pathLinesNew)


        line1 = PointsAndLinesClass.ClassLine(listOfPoints[0], listOfPoints[1])
        line2 = PointsAndLinesClass.ClassLine(listOfPoints[1], listOfPoints[2])

        if len(listOfPoints) > 3:
            line3 = PointsAndLinesClass.ClassLine(listOfPoints[2], listOfPoints[3])
            line4 = PointsAndLinesClass.ClassLine(listOfPoints[0], listOfPoints[3])
            polygonLines = [line1, line2, line3, line4]

            point9InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line4)

        else:
            line3 = PointsAndLinesClass.ClassLine(listOfPoints[2], listOfPoints[0])
            polygonLines = [line1, line2, line3]

        minX, maxX, minY, maxY = CrossFrameOptimizationLibrary.FindAxisLimits(listOfPoints)
        axisLimits = [minX, maxX, minY, maxY]

        point6InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line1)
        point7InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line2)
        point8InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line3)

        A, Ix, Iy = CrossFrameOptimizationLibrary.GetPropertiesOfSections(crossSection, shapeBaseLength,
                                                                          shapeBaseHeight, shapeBaseDiameter)

        rho, E, Sy, Su = CrossFrameOptimizationLibrary.GetMaterialProperties(material)

        while tryAnotherPoint:
            if allowSelectBeginPoint:
                xValuesToPlotShape = []
                yValuesToPlotShape = []
                xValuesToPlotPath = []
                yValuesToPlotPath = []

                for p in range(len(polygonLines)):
                    plt.plot(polygonLines[p].points[0].x, polygonLines[p].points[0].y, 'r*')
                    plt.plot(polygonLines[p].points[1].x, polygonLines[p].points[1].y, 'r*')
                    xValuesToPlotShape.append(polygonLines[p].points[0].x)
                    xValuesToPlotShape.append(polygonLines[p].points[1].x)
                    yValuesToPlotShape.append(polygonLines[p].points[0].y)
                    yValuesToPlotShape.append(polygonLines[p].points[1].y)

                if len(polygonLines) == 4:
                    xValuesToPlotShape.pop()
                    yValuesToPlotShape.pop()

                plt.plot(xValuesToPlotShape, yValuesToPlotShape, 'b--')
                plt.plot(xValuesToPlotPath, yValuesToPlotPath, 'g-')
                ax = plt.gca()
                ax.set_aspect(1)
                plt.xlabel('Y')
                plt.ylabel('X')
                fig = plt.figure(1)

                coords = []
                cid = fig.canvas.mpl_connect('button_press_event', onclick)
                plt.show()
                point5InitialGuess = [coords[0]]
            else:
                point5InitialGuess = [((minX + maxX) / 2, (minY + maxY) / 2)]  # First guess is in middle of bounds

            if len(listOfPoints) > 3:
                initialPointsGuesses = [point5InitialGuess, point6InitialGuess, point7InitialGuess, point8InitialGuess,
                                        point9InitialGuess]
            else:
                initialPointsGuesses = [point5InitialGuess, point6InitialGuess, point7InitialGuess, point8InitialGuess]

            if plotPointGuesses:
                x5GuessPoints = []
                y5GuessPoints = []
                x6GuessPoints = []
                y6GuessPoints = []
                x7GuessPoints = []
                y7GuessPoints = []
                x8GuessPoints = []
                y8GuessPoints = []
                if len(listOfPoints) > 3:
                    x9GuessPoints = []
                    y9GuessPoints = []

            isSinglePanel = True

            opt = {'maxiter': 1000}
            result = minimize(functionToMinimize, initialPointsGuesses,
                              constraints=CrossFrameConstraints.GetConstraintsWithMiddle(polygonLines, listOfPoints,
                                                                                         minDistances, isSinglePanel), options=opt)
            print("message:", result.message)
            print("success:", result.success)
            print("iterations:", result.nit)
            print("fun:", result.fun)

            point5 = PointsAndLinesClass.ClassPoint(result.x[0], result.x[1])
            point6 = PointsAndLinesClass.ClassPoint(result.x[2], result.x[3])
            point7 = PointsAndLinesClass.ClassPoint(result.x[4], result.x[5])
            point8 = PointsAndLinesClass.ClassPoint(result.x[6], result.x[7])
            if len(listOfPoints) > 3:
                point9 = PointsAndLinesClass.ClassPoint(result.x[8], result.x[9])

            line5 = PointsAndLinesClass.ClassLine(point6, point5)
            line6 = PointsAndLinesClass.ClassLine(point7, point5)
            line7 = PointsAndLinesClass.ClassLine(point8, point5)
            pathLines = [line5, line6, line7]
            if len(listOfPoints) > 3:
                line8 = PointsAndLinesClass.ClassLine(point9, point5)
                pathLines.append(line8)
                if plotPointGuesses:
                    xGuessPoints = [x5GuessPoints, x6GuessPoints, x7GuessPoints, x8GuessPoints, x9GuessPoints]
                    yGuessPoints = [y5GuessPoints, y6GuessPoints, y7GuessPoints, y8GuessPoints, y9GuessPoints]
                else:
                    xGuessPoints = []
                    yGuessPoints = []
            else:
                if plotPointGuesses:
                    xGuessPoints = [x5GuessPoints, x6GuessPoints, x7GuessPoints, x8GuessPoints]
                    yGuessPoints = [y5GuessPoints, y6GuessPoints, y7GuessPoints, y8GuessPoints]
                else:
                    xGuessPoints = []
                    yGuessPoints = []

            plotLines = polygonLines.copy()
            plotLines.extend(pathLines)

            bisectionIntersectionPoints, nonBisectionIntersectionPoints = CrossFrameOptimizationLibrary.\
                GetPanelBisectionAndPathLineIntersectionPointsForAPanel(listOfPoints, plotLines)
            ##Temp
            for p in range(len(polygonLines)):
                plt.plot(polygonLines[p].points[0].x, polygonLines[p].points[0].y, 'r*')
                plt.plot(polygonLines[p].points[1].x, polygonLines[p].points[1].y, 'r*')
                plt.plot([polygonLines[p].points[0].x, polygonLines[p].points[1].x],
                        [polygonLines[p].points[0].y, polygonLines[p].points[1].y], 'b--')
            for p in range(len(plotLines)):
                    plt.plot(plotLines[p].points[0].x, plotLines[p].points[0].y, 'go')
                    plt.plot(plotLines[p].points[1].x, plotLines[p].points[1].y, 'go')
                    plt.plot([plotLines[p].points[0].x, plotLines[p].points[1].x],
                             [plotLines[p].points[0].y, plotLines[p].points[1].y], 'g-')

            plt.plot([polygonLines[0].points[0].x, polygonLines[2].points[0].x],
                     [polygonLines[0].points[0].y, polygonLines[2].points[0].y], 'c--')

            plt.plot([polygonLines[1].points[0].x, polygonLines[3].points[1].x],
                     [polygonLines[1].points[0].y, polygonLines[3].points[1].y], 'c--')

            for p in range(len(bisectionIntersectionPoints)):
                plt.plot(bisectionIntersectionPoints[p][0], bisectionIntersectionPoints[p][1], 'ro')
            for p in range(len(nonBisectionIntersectionPoints)):
                plt.plot(nonBisectionIntersectionPoints[p][0], nonBisectionIntersectionPoints[p][1], 'ko')

            ax = plt.gca()
            ax.set_aspect(1)

            plt.xlabel('Y')
            plt.ylabel('X')
            plt.show()



            # PrintMassOfAllLines(pathLines)
            plotShape(plotLines, len(polygonLines), xGuessPoints, yGuessPoints, axisLimits, plotPointGuesses, boolOptions)

            if multipleGuesses:
                tryAnotherPoint = True
            else:
                tryAnotherPoint = False
    else:
        def functionToMinimize(optimalPoints):
            point6New = PointsAndLinesClass.ClassPoint(optimalPoints[0], optimalPoints[1])
            point7New = PointsAndLinesClass.ClassPoint(optimalPoints[2], optimalPoints[3])
            point8New = PointsAndLinesClass.ClassPoint(optimalPoints[4], optimalPoints[5])
            if len(optimalPoints) > 7:
                point9New = PointsAndLinesClass.ClassPoint(optimalPoints[6], optimalPoints[7])
            if plotPointGuesses:
                x6GuessPoints.append(optimalPoints[0])
                y6GuessPoints.append(optimalPoints[1])
                x7GuessPoints.append(optimalPoints[2])
                y7GuessPoints.append(optimalPoints[3])
                x8GuessPoints.append(optimalPoints[4])
                y8GuessPoints.append(optimalPoints[5])
                if len(optimalPoints) > 7:
                    x9GuessPoints.append(optimalPoints[6])
                    y9GuessPoints.append(optimalPoints[7])

            if boolOptions[6] is False:
                if len(optimalPoints) > 7:
                    line5Opt = PointsAndLinesClass.ClassLine(point6New, point7New)
                    line6Opt = PointsAndLinesClass.ClassLine(point7New, point8New)
                    line7Opt = PointsAndLinesClass.ClassLine(point8New, point9New)
                    line8Opt = PointsAndLinesClass.ClassLine(point9New, point6New)
                    pathLinesNew = [line5Opt, line6Opt, line7Opt, line8Opt]
                else:
                    line5Opt = PointsAndLinesClass.ClassLine(point6New, point7New)
                    line6Opt = PointsAndLinesClass.ClassLine(point7New, point8New)
                    line7Opt = PointsAndLinesClass.ClassLine(point8New, point6New)
                    pathLinesNew = [line5Opt, line6Opt, line7Opt]
            else:
                if len(optimalPoints) > 7:
                    line5Opt = PointsAndLinesClass.ClassLine(point6New, point7New)
                    line6Opt = PointsAndLinesClass.ClassLine(point7New, point9New)
                    line7Opt = PointsAndLinesClass.ClassLine(point8New, point9New)
                    # line8Opt = PointsAndLinesClass.ClassLine(point9New, point6New)
                    pathLinesNew = [line5Opt, line6Opt, line7Opt]
                else:
                    line5Opt = PointsAndLinesClass.ClassLine(point6New, point7New)
                    line6Opt = PointsAndLinesClass.ClassLine(point7New, point8New)
                    # line7Opt = PointsAndLinesClass.ClassLine(point8New, point6New)
                    pathLinesNew = [line5Opt, line6Opt]

            return CrossFrameOptimizationLibrary.GetMassOfAllLines(pathLinesNew, A, rho)
            # return -GetMassOfAllLines(pathLinesNew)
            # return TempStiffnessCalc(pathLinesNew)

        line1 = PointsAndLinesClass.ClassLine(listOfPoints[0], listOfPoints[1])
        line2 = PointsAndLinesClass.ClassLine(listOfPoints[1], listOfPoints[2])

        if len(listOfPoints) > 3:
            line3 = PointsAndLinesClass.ClassLine(listOfPoints[2], listOfPoints[3])
            line4 = PointsAndLinesClass.ClassLine(listOfPoints[0], listOfPoints[3])
            polygonLines = [line1, line2, line3, line4]

            point9InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line4)

        else:
            line3 = PointsAndLinesClass.ClassLine(listOfPoints[2], listOfPoints[0])
            polygonLines = [line1, line2, line3]

        minX, maxX, minY, maxY = CrossFrameOptimizationLibrary.FindAxisLimits(listOfPoints)
        axisLimits = [minX, maxX, minY, maxY]

        point6InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line1)
        point7InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line2)
        point8InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line3)

        A, Ix, Iy = CrossFrameOptimizationLibrary.GetPropertiesOfSections(crossSection, shapeBaseLength,
                                                                          shapeBaseHeight,
                                                                          shapeBaseDiameter)

        rho, E, Sy, Su = CrossFrameOptimizationLibrary.GetMaterialProperties(material)

        while tryAnotherPoint:
            if len(listOfPoints) > 3:
                initialPointsGuesses = [point6InitialGuess, point7InitialGuess, point8InitialGuess,
                                        point9InitialGuess]
            else:
                initialPointsGuesses = [point6InitialGuess, point7InitialGuess, point8InitialGuess]

            if plotPointGuesses:
                x6GuessPoints = []
                y6GuessPoints = []
                x7GuessPoints = []
                y7GuessPoints = []
                x8GuessPoints = []
                y8GuessPoints = []
                if len(listOfPoints) > 3:
                    x9GuessPoints = []
                    y9GuessPoints = []

            isSinglePanel = True

            opt = {'maxiter': 1000}
            result = minimize(functionToMinimize, initialPointsGuesses,
                              constraints=CrossFrameConstraints.GetConstraintsNoMiddle(polygonLines, listOfPoints,
                                                                                       minDistances, isSinglePanel),
                              options=opt)
            print(result)

            point6 = PointsAndLinesClass.ClassPoint(result.x[0], result.x[1])
            point7 = PointsAndLinesClass.ClassPoint(result.x[2], result.x[3])
            point8 = PointsAndLinesClass.ClassPoint(result.x[4], result.x[5])
            if len(listOfPoints) > 3:
                point9 = PointsAndLinesClass.ClassPoint(result.x[6], result.x[7])

            if boolOptions[6] is False:
                if len(listOfPoints) > 3:
                    line5 = PointsAndLinesClass.ClassLine(point6, point7)
                    line6 = PointsAndLinesClass.ClassLine(point7, point8)
                    line7 = PointsAndLinesClass.ClassLine(point8, point9)
                    line8 = PointsAndLinesClass.ClassLine(point9, point6)
                    pathLines = [line5, line6, line7, line8]
                else:
                    line5 = PointsAndLinesClass.ClassLine(point6, point7)
                    line6 = PointsAndLinesClass.ClassLine(point7, point8)
                    line7 = PointsAndLinesClass.ClassLine(point8, point6)
                    pathLines = [line5, line6, line7]
            else:
                if len(listOfPoints) > 3:
                    line5 = PointsAndLinesClass.ClassLine(point6, point7)
                    line6 = PointsAndLinesClass.ClassLine(point7, point9)
                    line7 = PointsAndLinesClass.ClassLine(point8, point9)
                    # line8 = PointsAndLinesClass.ClassLine(point9, point6)
                    pathLines = [line5, line6, line7]
                else:
                    line5 = PointsAndLinesClass.ClassLine(point6, point7)
                    line6 = PointsAndLinesClass.ClassLine(point7, point8)
                    # line7 = PointsAndLinesClass.ClassLine(point8, point6)
                    pathLines = [line5, line6]

            if plotPointGuesses:
                if len(listOfPoints) > 3:
                    xGuessPoints = [x6GuessPoints, x7GuessPoints, x8GuessPoints, x9GuessPoints]
                    yGuessPoints = [y6GuessPoints, y7GuessPoints, y8GuessPoints, y9GuessPoints]
                else:
                    xGuessPoints = [x6GuessPoints, x7GuessPoints, x8GuessPoints]
                    yGuessPoints = [y6GuessPoints, y7GuessPoints, y8GuessPoints]
            else:
                xGuessPoints = []
                yGuessPoints = []

            plotLines = polygonLines.copy()
            plotLines.extend(pathLines)

            bisectionIntersectionPoints, nonBisectionIntersectionPoints = CrossFrameOptimizationLibrary. \
                GetPanelBisectionAndPathLineIntersectionPointsForAPanel(listOfPoints, plotLines)
            ##Temp
            for p in range(len(polygonLines)):
                plt.plot(polygonLines[p].points[0].x, polygonLines[p].points[0].y, 'r*')
                plt.plot(polygonLines[p].points[1].x, polygonLines[p].points[1].y, 'r*')
                plt.plot([polygonLines[p].points[0].x, polygonLines[p].points[1].x],
                         [polygonLines[p].points[0].y, polygonLines[p].points[1].y], 'b--')
            for p in range(len(plotLines)):
                plt.plot(plotLines[p].points[0].x, plotLines[p].points[0].y, 'go')
                plt.plot(plotLines[p].points[1].x, plotLines[p].points[1].y, 'go')
                plt.plot([plotLines[p].points[0].x, plotLines[p].points[1].x],
                         [plotLines[p].points[0].y, plotLines[p].points[1].y], 'g-')

            plt.plot([polygonLines[0].points[0].x, polygonLines[2].points[0].x],
                     [polygonLines[0].points[0].y, polygonLines[2].points[0].y], 'c--')

            plt.plot([polygonLines[1].points[0].x, polygonLines[3].points[1].x],
                     [polygonLines[1].points[0].y, polygonLines[3].points[1].y], 'c--')

            for p in range(len(bisectionIntersectionPoints)):
                plt.plot(bisectionIntersectionPoints[p][0], bisectionIntersectionPoints[p][1], 'ro')
            for p in range(len(nonBisectionIntersectionPoints)):
                plt.plot(nonBisectionIntersectionPoints[p][0], nonBisectionIntersectionPoints[p][1], 'ko')

            ax = plt.gca()
            ax.set_aspect(1)

            plt.xlabel('Y')
            plt.ylabel('X')
            plt.show()

            # PrintMassOfAllLines(pathLines)
            plotShape(plotLines, len(polygonLines), xGuessPoints, yGuessPoints, axisLimits, plotPointGuesses, boolOptions)

            if multipleGuesses:
                tryAnotherPoint = True
            else:
                tryAnotherPoint = False


def Optimize22Gore(listOfPoints, boolOptions, minDistances, crossSectionLengths, material, crossSection):
    connectToMiddlePoint = boolOptions[0]
    plotPointGuesses = boolOptions[1]
    tryAnotherPoint = boolOptions[3]
    multipleGuesses = boolOptions[4]

    shapeBaseLength = crossSectionLengths[0]  # meters, if square or rectangle
    shapeBaseHeight = crossSectionLengths[1]  # meters, if rectangle
    shapeBaseDiameter = crossSectionLengths[2]  # meters, if circle

    panelCorners = [[listOfPoints[0], listOfPoints[1], listOfPoints[2], listOfPoints[3]],
                    [listOfPoints[4], listOfPoints[5], listOfPoints[6], listOfPoints[7]],
                    [listOfPoints[8], listOfPoints[9], listOfPoints[10], listOfPoints[11]],
                    [listOfPoints[12], listOfPoints[13], listOfPoints[14], listOfPoints[15]],
                    [listOfPoints[16], listOfPoints[17], listOfPoints[18], listOfPoints[19]],
                    [listOfPoints[20], listOfPoints[21], listOfPoints[22], listOfPoints[23]],
                    [listOfPoints[24], listOfPoints[25], listOfPoints[26], listOfPoints[27]],
                    [listOfPoints[28], listOfPoints[29], listOfPoints[30], listOfPoints[31]],
                    [listOfPoints[32], listOfPoints[33], listOfPoints[34], listOfPoints[35]],
                    [listOfPoints[36], listOfPoints[37], listOfPoints[38], listOfPoints[39]],
                    [listOfPoints[40], listOfPoints[41], listOfPoints[42]],
                    [listOfPoints[43], listOfPoints[44], listOfPoints[45]],
                    [listOfPoints[46], listOfPoints[47], listOfPoints[48]],
                    [listOfPoints[49], listOfPoints[50], listOfPoints[51]]]

    if connectToMiddlePoint:
        def functionToMinimize(optimalPoints):
            point5New = PointsAndLinesClass.ClassPoint(optimalPoints[0], optimalPoints[1])
            point6New = PointsAndLinesClass.ClassPoint(optimalPoints[2], optimalPoints[3])
            point7New = PointsAndLinesClass.ClassPoint(optimalPoints[4], optimalPoints[5])
            point8New = PointsAndLinesClass.ClassPoint(optimalPoints[6], optimalPoints[7])
            point9New = PointsAndLinesClass.ClassPoint(optimalPoints[8], optimalPoints[9])

            point10New = PointsAndLinesClass.ClassPoint(optimalPoints[10], optimalPoints[11])
            point11New = PointsAndLinesClass.ClassPoint(optimalPoints[12], optimalPoints[13])
            point12New = PointsAndLinesClass.ClassPoint(optimalPoints[14], optimalPoints[15])
            point13New = PointsAndLinesClass.ClassPoint(optimalPoints[16], optimalPoints[17])

            point14New = PointsAndLinesClass.ClassPoint(optimalPoints[18], optimalPoints[19])
            point15New = PointsAndLinesClass.ClassPoint(optimalPoints[20], optimalPoints[21])
            point16New = PointsAndLinesClass.ClassPoint(optimalPoints[22], optimalPoints[23])
            point17New = PointsAndLinesClass.ClassPoint(optimalPoints[24], optimalPoints[25])

            point18New = PointsAndLinesClass.ClassPoint(optimalPoints[26], optimalPoints[27])
            point19New = PointsAndLinesClass.ClassPoint(optimalPoints[28], optimalPoints[29])
            point20New = PointsAndLinesClass.ClassPoint(optimalPoints[30], optimalPoints[31])
            point21New = PointsAndLinesClass.ClassPoint(optimalPoints[32], optimalPoints[33])

            point22New = PointsAndLinesClass.ClassPoint(optimalPoints[34], optimalPoints[35])
            point23New = PointsAndLinesClass.ClassPoint(optimalPoints[36], optimalPoints[37])
            point24New = PointsAndLinesClass.ClassPoint(optimalPoints[38], optimalPoints[39])
            point25New = PointsAndLinesClass.ClassPoint(optimalPoints[40], optimalPoints[41])

            point26New = PointsAndLinesClass.ClassPoint(optimalPoints[42], optimalPoints[43])
            point27New = PointsAndLinesClass.ClassPoint(optimalPoints[44], optimalPoints[45])
            point28New = PointsAndLinesClass.ClassPoint(optimalPoints[46], optimalPoints[47])

            point29New = PointsAndLinesClass.ClassPoint(optimalPoints[48], optimalPoints[49])
            point30New = PointsAndLinesClass.ClassPoint(optimalPoints[50], optimalPoints[51])
            point31New = PointsAndLinesClass.ClassPoint(optimalPoints[52], optimalPoints[53])
            point32New = PointsAndLinesClass.ClassPoint(optimalPoints[54], optimalPoints[55])

            point33New = PointsAndLinesClass.ClassPoint(optimalPoints[56], optimalPoints[57])
            point34New = PointsAndLinesClass.ClassPoint(optimalPoints[58], optimalPoints[59])
            point35New = PointsAndLinesClass.ClassPoint(optimalPoints[60], optimalPoints[61])
            point36New = PointsAndLinesClass.ClassPoint(optimalPoints[62], optimalPoints[63])

            point37New = PointsAndLinesClass.ClassPoint(optimalPoints[64], optimalPoints[65])
            point38New = PointsAndLinesClass.ClassPoint(optimalPoints[66], optimalPoints[67])
            point39New = PointsAndLinesClass.ClassPoint(optimalPoints[68], optimalPoints[69])
            point40New = PointsAndLinesClass.ClassPoint(optimalPoints[70], optimalPoints[71])

            point41New = PointsAndLinesClass.ClassPoint(optimalPoints[72], optimalPoints[73])
            point42New = PointsAndLinesClass.ClassPoint(optimalPoints[74], optimalPoints[75])
            point43New = PointsAndLinesClass.ClassPoint(optimalPoints[76], optimalPoints[77])

            point44New = PointsAndLinesClass.ClassPoint(optimalPoints[78], optimalPoints[79])
            point45New = PointsAndLinesClass.ClassPoint(optimalPoints[80], optimalPoints[81])
            point46New = PointsAndLinesClass.ClassPoint(optimalPoints[82], optimalPoints[83])

            point47New = PointsAndLinesClass.ClassPoint(optimalPoints[84], optimalPoints[85])
            point48New = PointsAndLinesClass.ClassPoint(optimalPoints[86], optimalPoints[87])
            point49New = PointsAndLinesClass.ClassPoint(optimalPoints[88], optimalPoints[89])

            point50New = PointsAndLinesClass.ClassPoint(optimalPoints[90], optimalPoints[91])
            point51New = PointsAndLinesClass.ClassPoint(optimalPoints[92], optimalPoints[93])

            line5Opt = PointsAndLinesClass.ClassLine(point6New, point5New)
            line6Opt = PointsAndLinesClass.ClassLine(point7New, point5New)
            line7Opt = PointsAndLinesClass.ClassLine(point8New, point5New)
            line8Opt = PointsAndLinesClass.ClassLine(point9New, point5New)

            line9Opt = PointsAndLinesClass.ClassLine(point11New, point10New)
            line10Opt = PointsAndLinesClass.ClassLine(point9New, point10New)
            line11Opt = PointsAndLinesClass.ClassLine(point12New, point10New)
            line12Opt = PointsAndLinesClass.ClassLine(point13New, point10New)

            line13Opt = PointsAndLinesClass.ClassLine(point15New, point14New)
            line14Opt = PointsAndLinesClass.ClassLine(point13New, point14New)
            line15Opt = PointsAndLinesClass.ClassLine(point16New, point14New)
            line16Opt = PointsAndLinesClass.ClassLine(point17New, point14New)

            line17Opt = PointsAndLinesClass.ClassLine(point19New, point18New)
            line18Opt = PointsAndLinesClass.ClassLine(point17New, point18New)
            line19Opt = PointsAndLinesClass.ClassLine(point20New, point18New)
            line20Opt = PointsAndLinesClass.ClassLine(point21New, point18New)

            line21Opt = PointsAndLinesClass.ClassLine(point8New, point22New)
            line22Opt = PointsAndLinesClass.ClassLine(point23New, point22New)
            line23Opt = PointsAndLinesClass.ClassLine(point24New, point22New)
            line24Opt = PointsAndLinesClass.ClassLine(point25New, point22New)

            line25Opt = PointsAndLinesClass.ClassLine(point12New, point26New)
            line26Opt = PointsAndLinesClass.ClassLine(point25New, point26New)
            line27Opt = PointsAndLinesClass.ClassLine(point27New, point26New)
            line28Opt = PointsAndLinesClass.ClassLine(point28New, point26New)

            line29Opt = PointsAndLinesClass.ClassLine(point30New, point29New)
            line30Opt = PointsAndLinesClass.ClassLine(point27New, point29New)
            line31Opt = PointsAndLinesClass.ClassLine(point31New, point29New)
            line32Opt = PointsAndLinesClass.ClassLine(point32New, point29New)

            line33Opt = PointsAndLinesClass.ClassLine(point34New, point33New)
            line34Opt = PointsAndLinesClass.ClassLine(point32New, point33New)
            line35Opt = PointsAndLinesClass.ClassLine(point35New, point33New)
            line36Opt = PointsAndLinesClass.ClassLine(point36New, point33New)

            line37Opt = PointsAndLinesClass.ClassLine(point38New, point37New)
            line38Opt = PointsAndLinesClass.ClassLine(point39New, point37New)
            line39Opt = PointsAndLinesClass.ClassLine(point40New, point37New)
            line40Opt = PointsAndLinesClass.ClassLine(point34New, point37New)

            line41Opt = PointsAndLinesClass.ClassLine(point42New, point41New)
            line42Opt = PointsAndLinesClass.ClassLine(point20New, point41New)
            line43Opt = PointsAndLinesClass.ClassLine(point38New, point41New)
            line44Opt = PointsAndLinesClass.ClassLine(point43New, point41New)

            line45Opt = PointsAndLinesClass.ClassLine(point16New, point45New)
            line46Opt = PointsAndLinesClass.ClassLine(point28New, point45New)
            line47Opt = PointsAndLinesClass.ClassLine(point44New, point45New)

            line48Opt = PointsAndLinesClass.ClassLine(point44New, point46New)
            line49Opt = PointsAndLinesClass.ClassLine(point30New, point46New)
            line50Opt = PointsAndLinesClass.ClassLine(point39New, point46New)

            line51Opt = PointsAndLinesClass.ClassLine(point42New, point49New)
            line52Opt = PointsAndLinesClass.ClassLine(point47New, point49New)
            line53Opt = PointsAndLinesClass.ClassLine(point48New, point49New)

            line54Opt = PointsAndLinesClass.ClassLine(point51New, point50New)
            line55Opt = PointsAndLinesClass.ClassLine(point21New, point50New)
            line56Opt = PointsAndLinesClass.ClassLine(point48New, point50New)

            pathLinesNew = [line5Opt, line6Opt, line7Opt, line8Opt,
                            line9Opt, line10Opt, line11Opt, line12Opt,
                            line13Opt, line14Opt, line15Opt, line16Opt,
                            line17Opt, line18Opt, line19Opt, line20Opt,
                            line21Opt, line22Opt, line23Opt, line24Opt,
                            line25Opt, line26Opt, line27Opt, line28Opt,
                            line29Opt, line30Opt, line31Opt, line32Opt,
                            line33Opt, line34Opt, line35Opt, line36Opt,
                            line37Opt, line38Opt, line39Opt, line40Opt,
                            line41Opt, line42Opt, line43Opt, line44Opt,
                            line45Opt, line46Opt, line47Opt,
                            line48Opt, line49Opt, line50Opt,
                            line51Opt, line52Opt, line53Opt,
                            line54Opt, line55Opt, line56Opt]

            panelPathLines = [[line5Opt, line6Opt, line7Opt, line8Opt],
                            [line9Opt,  line10Opt, line11Opt, line12Opt],
                            [line13Opt, line14Opt, line15Opt, line16Opt],
                            [line17Opt, line18Opt, line19Opt, line20Opt],
                            [line21Opt, line22Opt, line23Opt, line24Opt],
                            [line25Opt, line26Opt, line27Opt, line28Opt],
                            [line29Opt, line30Opt, line31Opt, line32Opt],
                            [line33Opt, line34Opt, line35Opt, line36Opt],
                            [line37Opt, line38Opt, line39Opt, line40Opt],
                            [line41Opt, line42Opt, line43Opt, line44Opt],
                            [line45Opt, line46Opt, line47Opt],
                            [line48Opt, line49Opt, line50Opt],
                            [line51Opt, line52Opt, line53Opt],
                            [line54Opt, line55Opt, line56Opt]]

            return CrossFrameOptimizationLibrary.GetMassOfAllLines(pathLinesNew, A, rho)
            # return -CrossFrameOptimizationLibrary.GetMassOfAllLines(pathLinesNew, A, rho)
            # return TempStiffnessCalc(pathLinesNew)

        line1 = PointsAndLinesClass.ClassLine(listOfPoints[0], listOfPoints[1])
        line2 = PointsAndLinesClass.ClassLine(listOfPoints[1], listOfPoints[2])
        line3 = PointsAndLinesClass.ClassLine(listOfPoints[2], listOfPoints[3])
        line4 = PointsAndLinesClass.ClassLine(listOfPoints[0], listOfPoints[3])

        line5 = PointsAndLinesClass.ClassLine(listOfPoints[4], listOfPoints[0])
        line6 = line4
        line7 = PointsAndLinesClass.ClassLine(listOfPoints[3], listOfPoints[5])
        line8 = PointsAndLinesClass.ClassLine(listOfPoints[4], listOfPoints[5])

        line9 = PointsAndLinesClass.ClassLine(listOfPoints[6], listOfPoints[4])
        line10 = line8
        line11 = PointsAndLinesClass.ClassLine(listOfPoints[5], listOfPoints[7])
        line12 = PointsAndLinesClass.ClassLine(listOfPoints[6], listOfPoints[7])

        line13 = PointsAndLinesClass.ClassLine(listOfPoints[8], listOfPoints[6])
        line14 = line12
        line15 = PointsAndLinesClass.ClassLine(listOfPoints[7], listOfPoints[9])
        line16 = PointsAndLinesClass.ClassLine(listOfPoints[8], listOfPoints[9])

        line17 = line3
        line18 = PointsAndLinesClass.ClassLine(listOfPoints[2], listOfPoints[12])
        line19 = PointsAndLinesClass.ClassLine(listOfPoints[12], listOfPoints[13])
        line20 = PointsAndLinesClass.ClassLine(listOfPoints[3], listOfPoints[13])

        line21 = PointsAndLinesClass.ClassLine(listOfPoints[13], listOfPoints[14])
        line22 = PointsAndLinesClass.ClassLine(listOfPoints[5], listOfPoints[14])

        line23 = PointsAndLinesClass.ClassLine(listOfPoints[16], listOfPoints[14])
        line24 = PointsAndLinesClass.ClassLine(listOfPoints[13], listOfPoints[15])
        line25 = PointsAndLinesClass.ClassLine(listOfPoints[16], listOfPoints[15])

        line26 = PointsAndLinesClass.ClassLine(listOfPoints[18], listOfPoints[16])
        line27 = PointsAndLinesClass.ClassLine(listOfPoints[15], listOfPoints[17])
        line28 = PointsAndLinesClass.ClassLine(listOfPoints[18], listOfPoints[17])

        line29 = PointsAndLinesClass.ClassLine(listOfPoints[19], listOfPoints[7])
        line30 = PointsAndLinesClass.ClassLine(listOfPoints[7], listOfPoints[16])
        line31 = PointsAndLinesClass.ClassLine(listOfPoints[19], listOfPoints[18])

        line32 = PointsAndLinesClass.ClassLine(listOfPoints[11], listOfPoints[9])
        line33 = PointsAndLinesClass.ClassLine(listOfPoints[11], listOfPoints[19])

        line34 = PointsAndLinesClass.ClassLine(listOfPoints[19], listOfPoints[7])
        line35 = PointsAndLinesClass.ClassLine(listOfPoints[7], listOfPoints[16])
        line36 = PointsAndLinesClass.ClassLine(listOfPoints[19], listOfPoints[18])

        line37 = PointsAndLinesClass.ClassLine(listOfPoints[11], listOfPoints[9])
        line38 = PointsAndLinesClass.ClassLine(listOfPoints[11], listOfPoints[19])

        line39 = PointsAndLinesClass.ClassLine(listOfPoints[7], listOfPoints[14])

        line40 = PointsAndLinesClass.ClassLine(listOfPoints[10], listOfPoints[9])
        line41 = PointsAndLinesClass.ClassLine(listOfPoints[10], listOfPoints[11])
        line42 = PointsAndLinesClass.ClassLine(listOfPoints[10], listOfPoints[8])

        polygonLines = [line1, line2, line3, line4, line5, line6, line7, line8, line9, line10, line11, line12,
                        line13, line14, line15, line16, line17, line18, line19, line20,
                        line21, line22, line23, line24, line25, line26, line27, line28, line29, line30, line31,
                        line32, line33, line34, line35, line36, line37, line38, line39, line40, line41, line42]

        panel1 = [line1, line2, line3, line4]
        panel2 = [line5, line4, line7, line8]
        panel3 = [line9, line8, line11, line12]
        panel4 = [line13, line12, line15, line16]
        panel5 = [line3, line18, line19, line20]
        panel6 = [line7, line20, line21, line22]
        panel7 = [line23, line21, line24, line25]
        panel8 = [line26, line25, line27, line28]
        panel9 = [line29, line30, line26, line31]
        panel10 = [line32, line15, line29, line33]
        panel11 = [line11, line22, line34]
        panel12 = [line34, line23, line30]
        panel13 = [line40, line41, line37]
        panel14 = [line42, line40, line16]


        minX1, maxX1, minY1, maxY1 = CrossFrameOptimizationLibrary.FindAxisLimitsOfLines(panel1)
        minX2, maxX2, minY2, maxY2 = CrossFrameOptimizationLibrary.FindAxisLimitsOfLines(panel2)
        minX3, maxX3, minY3, maxY3 = CrossFrameOptimizationLibrary.FindAxisLimitsOfLines(panel3)
        minX4, maxX4, minY4, maxY4 = CrossFrameOptimizationLibrary.FindAxisLimitsOfLines(panel4)
        minX5, maxX5, minY5, maxY5 = CrossFrameOptimizationLibrary.FindAxisLimitsOfLines(panel5)
        minX6, maxX6, minY6, maxY6 = CrossFrameOptimizationLibrary.FindAxisLimitsOfLines(panel6)
        minX7, maxX7, minY7, maxY7 = CrossFrameOptimizationLibrary.FindAxisLimitsOfLines(panel7)
        minX8, maxX8, minY8, maxY8 = CrossFrameOptimizationLibrary.FindAxisLimitsOfLines(panel8)
        minX9, maxX9, minY9, maxY9 = CrossFrameOptimizationLibrary.FindAxisLimitsOfLines(panel9)
        minX10, maxX10, minY10, maxY10 = CrossFrameOptimizationLibrary.FindAxisLimitsOfLines(panel10)
        minX11, maxX11, minY11, maxY11 = CrossFrameOptimizationLibrary.FindAxisLimitsOfLines(panel11)
        minX12, maxX12, minY12, maxY12 = CrossFrameOptimizationLibrary.FindAxisLimitsOfLines(panel12)
        minX13, maxX13, minY13, maxY13 = CrossFrameOptimizationLibrary.FindAxisLimitsOfLines(panel13)
        minX14, maxX14, minY14, maxY14 = CrossFrameOptimizationLibrary.FindAxisLimitsOfLines(panel14)

        minX, maxX, minY, maxY = CrossFrameOptimizationLibrary.FindAxisLimits(listOfPoints)
        axisLimits = [minX, maxX, minY, maxY]

        point6InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line1)
        point7InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line2)
        point8InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line3)
        point9InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line4)

        point11InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line5)
        point12InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line7)
        point13InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line8)

        point15InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line9)
        point16InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line11)
        point17InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line12)

        point19InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line13)
        point20InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line15)
        point21InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line16)

        point23InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line18)
        point24InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line19)
        point25InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line20)

        point27InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line21)
        point28InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line22)

        point30InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line23)
        point31InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line24)
        point32InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line25)

        point34InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line26)
        point35InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line27)
        point36InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line28)

        point38InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line29)
        point39InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line30)
        point40InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line31)

        point42InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line32)
        point43InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line33)

        point44InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line39)

        point47InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line41)
        point48InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line40)

        point51InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line42)

        point5InitialGuess = [((minX1 + maxX1) / 2, (minY1 + maxY1) / 2)]  # First guess is in middle of bounds
        point10InitialGuess = [((minX2 + maxX2) / 2, (minY2 + maxY2) / 2)]  # First guess is in middle of bounds
        point14InitialGuess = [((minX3 + maxX3) / 2, (minY3 + maxY3) / 2)]  # First guess is in middle of bounds
        point18InitialGuess = [((minX4 + maxX4) / 2, (minY4 + maxY4) / 2)]  # First guess is in middle of bounds
        point22InitialGuess = [((minX5 + maxX5) / 2, (minY5 + maxY5) / 2)]  # First guess is in middle of bounds
        point26InitialGuess = [((minX6 + maxX6) / 2, (minY6 + maxY6) / 2)]  # First guess is in middle of bounds
        point29InitialGuess = [((minX7 + maxX7) / 2, (minY7 + maxY7) / 2)]  # First guess is in middle of bounds
        point33InitialGuess = [((minX8 + maxX8) / 2, (minY8 + maxY8) / 2)]  # First guess is in middle of bounds
        point37InitialGuess = [((minX9 + maxX9) / 2, (minY9 + maxY9) / 2)]  # First guess is in middle of bounds
        point41InitialGuess = [((minX10 + maxX10) / 2, (minY10 + maxY10) / 2)]  # First guess is in middle of bounds
        point45InitialGuess = [((minX11 + maxX11) / 2, (minY11 + maxY11) / 2)]  # First guess is in middle of bounds
        point46InitialGuess = [((minX12 + maxX12) / 2, (minY12 + maxY12) / 2)]  # First guess is in middle of bounds
        point49InitialGuess = [((minX13 + maxX13) / 2, (minY13 + maxY13) / 2)]  # First guess is in middle of bounds
        point50InitialGuess = [((minX14 + maxX14) / 2, (minY14 + maxY14) / 2)]  # First guess is in middle of bounds

        initialPointsGuesses = [point5InitialGuess, point6InitialGuess, point7InitialGuess, point8InitialGuess, point9InitialGuess,
                                point10InitialGuess, point11InitialGuess, point12InitialGuess, point13InitialGuess,
                                point14InitialGuess, point15InitialGuess, point16InitialGuess, point17InitialGuess,
                                point18InitialGuess, point19InitialGuess, point20InitialGuess, point21InitialGuess,
                                point22InitialGuess, point23InitialGuess, point24InitialGuess, point25InitialGuess,
                                point26InitialGuess, point27InitialGuess, point28InitialGuess,
                                point29InitialGuess, point30InitialGuess, point31InitialGuess, point32InitialGuess,
                                point33InitialGuess, point34InitialGuess, point35InitialGuess, point36InitialGuess,
                                point37InitialGuess, point38InitialGuess, point39InitialGuess, point40InitialGuess,
                                point41InitialGuess, point42InitialGuess, point43InitialGuess,
                                point44InitialGuess, point45InitialGuess, point46InitialGuess,
                                point47InitialGuess, point48InitialGuess, point49InitialGuess,
                                point50InitialGuess, point51InitialGuess]

        A, Ix, Iy = CrossFrameOptimizationLibrary.GetPropertiesOfSections(crossSection, shapeBaseLength,
                                                                          shapeBaseHeight, shapeBaseDiameter)

        rho, E, Sy, Su = CrossFrameOptimizationLibrary.GetMaterialProperties(material)

        while tryAnotherPoint:
            isSinglePanel = False

            opt = {'maxiter': 1000}
            result = minimize(functionToMinimize, initialPointsGuesses,
                              constraints=CrossFrameConstraints.GetConstraintsWithMiddle(polygonLines, listOfPoints,
                                                                                         minDistances, isSinglePanel), options=opt)
            # print(result)
            print("message:", result.message)
            print("success:", result.success)
            print("iterations:", result.nit)
            print("fun:", result.fun)

            point5 = PointsAndLinesClass.ClassPoint(result.x[0], result.x[1])
            point6 = PointsAndLinesClass.ClassPoint(result.x[2], result.x[3])
            point7 = PointsAndLinesClass.ClassPoint(result.x[4], result.x[5])
            point8 = PointsAndLinesClass.ClassPoint(result.x[6], result.x[7])
            point9 = PointsAndLinesClass.ClassPoint(result.x[8], result.x[9])

            point10 = PointsAndLinesClass.ClassPoint(result.x[10], result.x[11])
            point11 = PointsAndLinesClass.ClassPoint(result.x[12], result.x[13])
            point12 = PointsAndLinesClass.ClassPoint(result.x[14], result.x[15])
            point13 = PointsAndLinesClass.ClassPoint(result.x[16], result.x[17])

            point14 = PointsAndLinesClass.ClassPoint(result.x[18], result.x[19])
            point15 = PointsAndLinesClass.ClassPoint(result.x[20], result.x[21])
            point16 = PointsAndLinesClass.ClassPoint(result.x[22], result.x[23])
            point17 = PointsAndLinesClass.ClassPoint(result.x[24], result.x[25])

            point18 = PointsAndLinesClass.ClassPoint(result.x[26], result.x[27])
            point19 = PointsAndLinesClass.ClassPoint(result.x[28], result.x[29])
            point20 = PointsAndLinesClass.ClassPoint(result.x[30], result.x[31])
            point21 = PointsAndLinesClass.ClassPoint(result.x[32], result.x[33])

            point22 = PointsAndLinesClass.ClassPoint(result.x[34], result.x[35])
            point23 = PointsAndLinesClass.ClassPoint(result.x[36], result.x[37])
            point24 = PointsAndLinesClass.ClassPoint(result.x[38], result.x[39])
            point25 = PointsAndLinesClass.ClassPoint(result.x[40], result.x[41])

            point26 = PointsAndLinesClass.ClassPoint(result.x[42], result.x[43])
            point27 = PointsAndLinesClass.ClassPoint(result.x[44], result.x[45])
            point28 = PointsAndLinesClass.ClassPoint(result.x[46], result.x[47])

            point29 = PointsAndLinesClass.ClassPoint(result.x[48], result.x[49])
            point30 = PointsAndLinesClass.ClassPoint(result.x[50], result.x[51])
            point31 = PointsAndLinesClass.ClassPoint(result.x[52], result.x[53])
            point32 = PointsAndLinesClass.ClassPoint(result.x[54], result.x[55])

            point33 = PointsAndLinesClass.ClassPoint(result.x[56], result.x[57])
            point34 = PointsAndLinesClass.ClassPoint(result.x[58], result.x[59])
            point35 = PointsAndLinesClass.ClassPoint(result.x[60], result.x[61])
            point36 = PointsAndLinesClass.ClassPoint(result.x[62], result.x[63])

            point37 = PointsAndLinesClass.ClassPoint(result.x[64], result.x[65])
            point38 = PointsAndLinesClass.ClassPoint(result.x[66], result.x[67])
            point39 = PointsAndLinesClass.ClassPoint(result.x[68], result.x[69])
            point40 = PointsAndLinesClass.ClassPoint(result.x[70], result.x[71])

            point41 = PointsAndLinesClass.ClassPoint(result.x[72], result.x[73])
            point42 = PointsAndLinesClass.ClassPoint(result.x[74], result.x[75])
            point43 = PointsAndLinesClass.ClassPoint(result.x[76], result.x[77])

            point44 = PointsAndLinesClass.ClassPoint(result.x[78], result.x[79])
            point45 = PointsAndLinesClass.ClassPoint(result.x[80], result.x[81])
            point46 = PointsAndLinesClass.ClassPoint(result.x[82], result.x[83])

            point47 = PointsAndLinesClass.ClassPoint(result.x[84], result.x[85])
            point48 = PointsAndLinesClass.ClassPoint(result.x[86], result.x[87])
            point49 = PointsAndLinesClass.ClassPoint(result.x[88], result.x[89])

            point50 = PointsAndLinesClass.ClassPoint(result.x[90], result.x[91])
            point51 = PointsAndLinesClass.ClassPoint(result.x[92], result.x[93])

            line5 = PointsAndLinesClass.ClassLine(point6, point5)
            line6 = PointsAndLinesClass.ClassLine(point7, point5)
            line7 = PointsAndLinesClass.ClassLine(point8, point5)
            line8 = PointsAndLinesClass.ClassLine(point9, point5)

            line9 = PointsAndLinesClass.ClassLine(point11, point10)
            line10 = PointsAndLinesClass.ClassLine(point9, point10)
            line11 = PointsAndLinesClass.ClassLine(point12, point10)
            line12 = PointsAndLinesClass.ClassLine(point13, point10)

            line13 = PointsAndLinesClass.ClassLine(point15, point14)
            line14 = PointsAndLinesClass.ClassLine(point13, point14)
            line15 = PointsAndLinesClass.ClassLine(point16, point14)
            line16 = PointsAndLinesClass.ClassLine(point17, point14)

            line17 = PointsAndLinesClass.ClassLine(point19, point18)
            line18 = PointsAndLinesClass.ClassLine(point17, point18)
            line19 = PointsAndLinesClass.ClassLine(point20, point18)
            line20 = PointsAndLinesClass.ClassLine(point21, point18)

            line21 = PointsAndLinesClass.ClassLine(point8, point22)
            line22 = PointsAndLinesClass.ClassLine(point23, point22)
            line23 = PointsAndLinesClass.ClassLine(point24, point22)
            line24 = PointsAndLinesClass.ClassLine(point25, point22)

            line25 = PointsAndLinesClass.ClassLine(point12, point26)
            line26 = PointsAndLinesClass.ClassLine(point25, point26)
            line27 = PointsAndLinesClass.ClassLine(point27, point26)
            line28 = PointsAndLinesClass.ClassLine(point28, point26)

            line29 = PointsAndLinesClass.ClassLine(point30, point29)
            line30 = PointsAndLinesClass.ClassLine(point27, point29)
            line31 = PointsAndLinesClass.ClassLine(point31, point29)
            line32 = PointsAndLinesClass.ClassLine(point32, point29)

            line33 = PointsAndLinesClass.ClassLine(point34, point33)
            line34 = PointsAndLinesClass.ClassLine(point32, point33)
            line35 = PointsAndLinesClass.ClassLine(point35, point33)
            line36 = PointsAndLinesClass.ClassLine(point36, point33)

            line37 = PointsAndLinesClass.ClassLine(point38, point37)
            line38 = PointsAndLinesClass.ClassLine(point34, point37)
            line39 = PointsAndLinesClass.ClassLine(point39, point37)
            line40 = PointsAndLinesClass.ClassLine(point40, point37)

            line41 = PointsAndLinesClass.ClassLine(point42, point41)
            line42 = PointsAndLinesClass.ClassLine(point20, point41)
            line43 = PointsAndLinesClass.ClassLine(point38, point41)
            line44 = PointsAndLinesClass.ClassLine(point43, point41)

            line45 = PointsAndLinesClass.ClassLine(point16, point45)
            line46 = PointsAndLinesClass.ClassLine(point28, point45)
            line47 = PointsAndLinesClass.ClassLine(point44, point45)

            line48 = PointsAndLinesClass.ClassLine(point44, point46)
            line49 = PointsAndLinesClass.ClassLine(point30, point46)
            line50 = PointsAndLinesClass.ClassLine(point39, point46)

            line51 = PointsAndLinesClass.ClassLine(point42, point49)
            line52 = PointsAndLinesClass.ClassLine(point48, point49)
            line53 = PointsAndLinesClass.ClassLine(point47, point49)

            line54 = PointsAndLinesClass.ClassLine(point51, point50)
            line55 = PointsAndLinesClass.ClassLine(point21, point50)
            line56 = PointsAndLinesClass.ClassLine(point48, point50)

            pathLines = [line5, line6, line7, line8, line9, line10, line11, line12, line13, line14, line15, line16,
                         line17, line18, line19, line20, line21, line22, line23, line24, line25, line26, line27, line28,
                         line29, line30, line31, line32, line33, line34, line35, line36, line37, line38, line39, line40,
                         line41, line42, line43, line44, line45, line46, line47, line48,
                         line49, line50, line51, line52, line53, line54, line55, line56]

            panel1 = [line5, line6, line7, line8]
            panel2 = [line9, line10, line11, line12]
            panel3 = [line13, line14, line15, line16]
            panel4 = [line17, line18, line19, line20]
            panel5 = [line21, line22, line23, line24]
            panel6 = [line25, line26, line27, line28]
            panel7 = [line29, line30, line31, line32]
            panel8 = [line33, line34, line35, line36]
            panel9 = [line37, line38, line39, line40]
            panel10 = [line41, line42, line43, line44]
            panel11 = [line45, line46, line47]
            panel12 = [line48, line49, line50]
            panel13 = [line51, line52, line53]
            panel14 = [line54, line55, line56]

            panels = [panel1, panel2, panel3, panel4, panel5, panel6, panel7, panel8, panel9, panel10, panel11, panel12,
                      panel13, panel14]

            xGuessPoints = []
            yGuessPoints = []

            plotLines = polygonLines.copy()
            plotLines.extend(pathLines)


            CreateCSV(panels)
            # PrintMassOfAllLines(pathLines)
            plotShape(plotLines, len(polygonLines), xGuessPoints, yGuessPoints, axisLimits, plotPointGuesses, boolOptions)

            tryAnotherPoint = False

    else:
        def functionToMinimize(optimalPoints):
            point6New = PointsAndLinesClass.ClassPoint(optimalPoints[0], optimalPoints[1])
            point7New = PointsAndLinesClass.ClassPoint(optimalPoints[2], optimalPoints[3])
            point8New = PointsAndLinesClass.ClassPoint(optimalPoints[4], optimalPoints[5])
            point9New = PointsAndLinesClass.ClassPoint(optimalPoints[6], optimalPoints[7])

            point11New = PointsAndLinesClass.ClassPoint(optimalPoints[8], optimalPoints[9])
            point12New = PointsAndLinesClass.ClassPoint(optimalPoints[10], optimalPoints[11])
            point13New = PointsAndLinesClass.ClassPoint(optimalPoints[12], optimalPoints[13])

            point15New = PointsAndLinesClass.ClassPoint(optimalPoints[14], optimalPoints[15])
            point16New = PointsAndLinesClass.ClassPoint(optimalPoints[16], optimalPoints[17])
            point17New = PointsAndLinesClass.ClassPoint(optimalPoints[18], optimalPoints[19])

            point19New = PointsAndLinesClass.ClassPoint(optimalPoints[20], optimalPoints[21])
            point20New = PointsAndLinesClass.ClassPoint(optimalPoints[22], optimalPoints[23])
            point21New = PointsAndLinesClass.ClassPoint(optimalPoints[24], optimalPoints[25])

            point23New = PointsAndLinesClass.ClassPoint(optimalPoints[26], optimalPoints[27])
            point24New = PointsAndLinesClass.ClassPoint(optimalPoints[28], optimalPoints[29])
            point25New = PointsAndLinesClass.ClassPoint(optimalPoints[30], optimalPoints[31])

            point27New = PointsAndLinesClass.ClassPoint(optimalPoints[32], optimalPoints[33])
            point28New = PointsAndLinesClass.ClassPoint(optimalPoints[34], optimalPoints[35])

            point30New = PointsAndLinesClass.ClassPoint(optimalPoints[36], optimalPoints[37])
            point31New = PointsAndLinesClass.ClassPoint(optimalPoints[38], optimalPoints[39])
            point32New = PointsAndLinesClass.ClassPoint(optimalPoints[40], optimalPoints[41])

            point34New = PointsAndLinesClass.ClassPoint(optimalPoints[42], optimalPoints[43])
            point35New = PointsAndLinesClass.ClassPoint(optimalPoints[44], optimalPoints[45])
            point36New = PointsAndLinesClass.ClassPoint(optimalPoints[46], optimalPoints[47])

            point38New = PointsAndLinesClass.ClassPoint(optimalPoints[48], optimalPoints[49])
            point39New = PointsAndLinesClass.ClassPoint(optimalPoints[50], optimalPoints[51])
            point40New = PointsAndLinesClass.ClassPoint(optimalPoints[52], optimalPoints[53])

            point42New = PointsAndLinesClass.ClassPoint(optimalPoints[54], optimalPoints[55])
            point43New = PointsAndLinesClass.ClassPoint(optimalPoints[56], optimalPoints[57])

            point44New = PointsAndLinesClass.ClassPoint(optimalPoints[58], optimalPoints[59])

            point47New = PointsAndLinesClass.ClassPoint(optimalPoints[60], optimalPoints[61])
            point48New = PointsAndLinesClass.ClassPoint(optimalPoints[62], optimalPoints[63])

            point51New = PointsAndLinesClass.ClassPoint(optimalPoints[64], optimalPoints[65])

            if boolOptions[6] is False:
                line5Opt = PointsAndLinesClass.ClassLine(point6New, point7New)
                line6Opt = PointsAndLinesClass.ClassLine(point7New, point8New)
                line7Opt = PointsAndLinesClass.ClassLine(point8New, point9New)
                line8Opt = PointsAndLinesClass.ClassLine(point9New, point6New)

                line9Opt = PointsAndLinesClass.ClassLine(point11New, point9New)
                line11Opt = PointsAndLinesClass.ClassLine(point12New, point13New)
                line12Opt = PointsAndLinesClass.ClassLine(point13New, point11New)
                line13Opt = PointsAndLinesClass.ClassLine(point9New, point12New)

                line14Opt = PointsAndLinesClass.ClassLine(point15New, point13New)
                line15Opt = PointsAndLinesClass.ClassLine(point13New, point16New)
                line16Opt = PointsAndLinesClass.ClassLine(point16New, point17New)
                line17Opt = PointsAndLinesClass.ClassLine(point17New, point15New)

                line18Opt = PointsAndLinesClass.ClassLine(point19New, point17New)
                line19Opt = PointsAndLinesClass.ClassLine(point17New, point20New)
                line20Opt = PointsAndLinesClass.ClassLine(point20New, point21New)
                line21Opt = PointsAndLinesClass.ClassLine(point21New, point19New)

                line22Opt = PointsAndLinesClass.ClassLine(point8New, point23New)
                line23Opt = PointsAndLinesClass.ClassLine(point23New, point24New)
                line24Opt = PointsAndLinesClass.ClassLine(point24New, point25New)
                line25Opt = PointsAndLinesClass.ClassLine(point25New, point8New)

                line26Opt = PointsAndLinesClass.ClassLine(point12New, point25New)
                line27Opt = PointsAndLinesClass.ClassLine(point25New, point27New)
                line28Opt = PointsAndLinesClass.ClassLine(point27New, point28New)
                line29Opt = PointsAndLinesClass.ClassLine(point28New, point12New)

                line30Opt = PointsAndLinesClass.ClassLine(point30New, point27New)
                line31Opt = PointsAndLinesClass.ClassLine(point27New, point31New)
                line32Opt = PointsAndLinesClass.ClassLine(point31New, point32New)
                line33Opt = PointsAndLinesClass.ClassLine(point32New, point30New)

                line34Opt = PointsAndLinesClass.ClassLine(point34New, point32New)
                line35Opt = PointsAndLinesClass.ClassLine(point32New, point35New)
                line36Opt = PointsAndLinesClass.ClassLine(point35New, point36New)
                line37Opt = PointsAndLinesClass.ClassLine(point36New, point34New)

                line38Opt = PointsAndLinesClass.ClassLine(point38New, point39New)
                line39Opt = PointsAndLinesClass.ClassLine(point39New, point34New)
                line40Opt = PointsAndLinesClass.ClassLine(point34New, point40New)
                line41Opt = PointsAndLinesClass.ClassLine(point40New, point38New)

                line42Opt = PointsAndLinesClass.ClassLine(point42New, point20New)
                line43Opt = PointsAndLinesClass.ClassLine(point20New, point38New)
                line44Opt = PointsAndLinesClass.ClassLine(point38New, point43New)
                line45Opt = PointsAndLinesClass.ClassLine(point43New, point42New)

                line46Opt = PointsAndLinesClass.ClassLine(point16New, point28New)
                line47Opt = PointsAndLinesClass.ClassLine(point28New, point44New)
                line48Opt = PointsAndLinesClass.ClassLine(point44New, point16New)

                line49Opt = PointsAndLinesClass.ClassLine(point44New, point30New)
                line50Opt = PointsAndLinesClass.ClassLine(point30New, point39New)
                line51Opt = PointsAndLinesClass.ClassLine(point39New, point44New)

                line52Opt = PointsAndLinesClass.ClassLine(point47New, point48New)
                line53Opt = PointsAndLinesClass.ClassLine(point48New, point42New)
                line54Opt = PointsAndLinesClass.ClassLine(point42New, point47New)

                line55Opt = PointsAndLinesClass.ClassLine(point51New, point21New)
                line56Opt = PointsAndLinesClass.ClassLine(point21New, point48New)
                line57Opt = PointsAndLinesClass.ClassLine(point48New, point51New)

                pathLinesNew = [line5Opt, line6Opt, line7Opt, line8Opt, line9Opt, line11Opt, line12Opt, line13Opt,
                                line14Opt, line15Opt, line16Opt, line17Opt, line18Opt, line19Opt, line20Opt, line21Opt,
                                line22Opt, line23Opt, line24Opt, line25Opt, line26Opt, line27Opt, line28Opt, line29Opt,
                                line30Opt, line31Opt, line32Opt, line33Opt, line34Opt, line35Opt, line36Opt, line37Opt,
                                line38Opt, line39Opt, line40Opt, line41Opt, line42Opt, line43Opt, line44Opt, line45Opt,
                                line46Opt, line47Opt, line48Opt, line49Opt, line50Opt, line51Opt,
                                line52Opt, line53Opt, line54Opt, line55Opt, line56Opt, line57Opt]

                panelPathLines = [[line5Opt, line6Opt, line7Opt, line8Opt],
                                [line9Opt, line11Opt, line12Opt, line13Opt],
                                [line14Opt, line15Opt, line16Opt, line17Opt],
                                [line18Opt, line19Opt, line20Opt, line21Opt],
                                [line22Opt, line23Opt, line24Opt, line25Opt],
                                [line26Opt, line27Opt, line28Opt, line29Opt],
                                [line30Opt, line31Opt, line32Opt, line33Opt],
                                [line34Opt, line35Opt, line36Opt, line37Opt],
                                [line38Opt, line39Opt, line40Opt, line41Opt],
                                [line42Opt, line43Opt, line44Opt, line45Opt],
                                [line46Opt, line47Opt, line48Opt],
                                [line49Opt, line50Opt, line51Opt],
                                [line52Opt, line53Opt, line54Opt],
                                [line55Opt, line56Opt, line57Opt]]

            else:
                line5Opt = PointsAndLinesClass.ClassLine(point6New, point7New)
                line6Opt = PointsAndLinesClass.ClassLine(point9New, point7New)
                line7Opt = PointsAndLinesClass.ClassLine(point8New, point9New)

                line9Opt = PointsAndLinesClass.ClassLine(point11New, point9New)
                line11Opt = PointsAndLinesClass.ClassLine(point9New, point13New)
                line12Opt = PointsAndLinesClass.ClassLine(point13New, point12New)

                line14Opt = PointsAndLinesClass.ClassLine(point15New, point13New)
                line15Opt = PointsAndLinesClass.ClassLine(point13New, point17New)
                line16Opt = PointsAndLinesClass.ClassLine(point17New, point16New)

                line18Opt = PointsAndLinesClass.ClassLine(point19New, point17New)
                line19Opt = PointsAndLinesClass.ClassLine(point17New, point21New)
                line20Opt = PointsAndLinesClass.ClassLine(point21New, point20New)

                line22Opt = PointsAndLinesClass.ClassLine(point8New, point23New)
                line23Opt = PointsAndLinesClass.ClassLine(point23New, point25New)
                line24Opt = PointsAndLinesClass.ClassLine(point24New, point25New)

                line26Opt = PointsAndLinesClass.ClassLine(point12New, point25New)
                line27Opt = PointsAndLinesClass.ClassLine(point28New, point25New)
                line28Opt = PointsAndLinesClass.ClassLine(point27New, point28New)

                line30Opt = PointsAndLinesClass.ClassLine(point30New, point27New)
                line31Opt = PointsAndLinesClass.ClassLine(point31New, point30New)
                line32Opt = PointsAndLinesClass.ClassLine(point31New, point32New)

                line34Opt = PointsAndLinesClass.ClassLine(point34New, point32New)
                line35Opt = PointsAndLinesClass.ClassLine(point32New, point36New)
                line36Opt = PointsAndLinesClass.ClassLine(point35New, point36New)

                line38Opt = PointsAndLinesClass.ClassLine(point38New, point39New)
                line39Opt = PointsAndLinesClass.ClassLine(point39New, point40New)
                line40Opt = PointsAndLinesClass.ClassLine(point34New, point40New)

                line42Opt = PointsAndLinesClass.ClassLine(point42New, point20New)
                line43Opt = PointsAndLinesClass.ClassLine(point42New, point38New)
                line44Opt = PointsAndLinesClass.ClassLine(point38New, point43New)

                line46Opt = PointsAndLinesClass.ClassLine(point44New, point28New)
                line47Opt = PointsAndLinesClass.ClassLine(point16New, point44New)

                line49Opt = PointsAndLinesClass.ClassLine(point44New, point30New)
                line50Opt = PointsAndLinesClass.ClassLine(point44New, point39New)

                line52Opt = PointsAndLinesClass.ClassLine(point42New, point48New)
                line53Opt = PointsAndLinesClass.ClassLine(point48New, point47New)

                line55Opt = PointsAndLinesClass.ClassLine(point48New, point21New)
                line56Opt = PointsAndLinesClass.ClassLine(point51New, point48New)

                pathLinesNew = [line5Opt, line6Opt, line7Opt, line9Opt, line11Opt, line12Opt,
                                line14Opt, line15Opt, line16Opt, line18Opt, line19Opt, line20Opt,
                                line22Opt, line23Opt, line24Opt, line26Opt, line27Opt, line28Opt,
                                line30Opt, line31Opt, line32Opt, line34Opt, line35Opt, line36Opt,
                                line38Opt, line39Opt, line40Opt, line42Opt, line43Opt, line44Opt,
                                line46Opt, line47Opt, line49Opt, line50Opt,
                                line52Opt, line53Opt, line55Opt, line56Opt]

                panelPathLines = [[line5Opt, line6Opt, line7Opt],
                                [line9Opt, line11Opt, line12Opt],
                                [line14Opt, line15Opt, line16Opt],
                                [line18Opt, line19Opt, line20Opt],
                                [line22Opt, line23Opt, line24Opt],
                                [line26Opt, line27Opt, line28Opt],
                                [line30Opt, line31Opt, line32Opt],
                                [line34Opt, line35Opt, line36Opt],
                                [line38Opt, line39Opt, line40Opt],
                                [line42Opt, line43Opt, line44Opt],
                                [line46Opt, line47Opt],
                                [line49Opt, line50Opt],
                                [line52Opt, line53Opt],
                                [line55Opt, line56Opt]]

            return CrossFrameOptimizationLibrary.GetMassOfAllLines(pathLinesNew, A, rho)
            # return -GetMassOfAllLines(pathLinesNew)
            # return TempStiffnessCalc(pathLinesNew)

        line1 = PointsAndLinesClass.ClassLine(listOfPoints[0], listOfPoints[1])
        line2 = PointsAndLinesClass.ClassLine(listOfPoints[1], listOfPoints[2])
        line3 = PointsAndLinesClass.ClassLine(listOfPoints[2], listOfPoints[3])
        line4 = PointsAndLinesClass.ClassLine(listOfPoints[0], listOfPoints[3])

        line5 = PointsAndLinesClass.ClassLine(listOfPoints[4], listOfPoints[0])
        line6 = line4
        line7 = PointsAndLinesClass.ClassLine(listOfPoints[3], listOfPoints[5])
        line8 = PointsAndLinesClass.ClassLine(listOfPoints[4], listOfPoints[5])

        line9 = PointsAndLinesClass.ClassLine(listOfPoints[6], listOfPoints[4])
        line10 = line8
        line11 = PointsAndLinesClass.ClassLine(listOfPoints[5], listOfPoints[7])
        line12 = PointsAndLinesClass.ClassLine(listOfPoints[6], listOfPoints[7])

        line13 = PointsAndLinesClass.ClassLine(listOfPoints[8], listOfPoints[6])
        line14 = line12
        line15 = PointsAndLinesClass.ClassLine(listOfPoints[7], listOfPoints[9])
        line16 = PointsAndLinesClass.ClassLine(listOfPoints[8], listOfPoints[9])

        line17 = line3
        line18 = PointsAndLinesClass.ClassLine(listOfPoints[2], listOfPoints[12])
        line19 = PointsAndLinesClass.ClassLine(listOfPoints[12], listOfPoints[13])
        line20 = PointsAndLinesClass.ClassLine(listOfPoints[3], listOfPoints[13])

        line21 = PointsAndLinesClass.ClassLine(listOfPoints[13], listOfPoints[14])
        line22 = PointsAndLinesClass.ClassLine(listOfPoints[5], listOfPoints[14])

        line23 = PointsAndLinesClass.ClassLine(listOfPoints[16], listOfPoints[14])
        line24 = PointsAndLinesClass.ClassLine(listOfPoints[13], listOfPoints[15])
        line25 = PointsAndLinesClass.ClassLine(listOfPoints[16], listOfPoints[15])

        line26 = PointsAndLinesClass.ClassLine(listOfPoints[18], listOfPoints[16])
        line27 = PointsAndLinesClass.ClassLine(listOfPoints[15], listOfPoints[17])
        line28 = PointsAndLinesClass.ClassLine(listOfPoints[18], listOfPoints[17])

        line29 = PointsAndLinesClass.ClassLine(listOfPoints[19], listOfPoints[7])
        line30 = PointsAndLinesClass.ClassLine(listOfPoints[7], listOfPoints[16])
        line31 = PointsAndLinesClass.ClassLine(listOfPoints[19], listOfPoints[18])

        line32 = PointsAndLinesClass.ClassLine(listOfPoints[11], listOfPoints[9])
        line33 = PointsAndLinesClass.ClassLine(listOfPoints[11], listOfPoints[19])

        line34 = PointsAndLinesClass.ClassLine(listOfPoints[19], listOfPoints[7])
        line35 = PointsAndLinesClass.ClassLine(listOfPoints[7], listOfPoints[16])
        line36 = PointsAndLinesClass.ClassLine(listOfPoints[19], listOfPoints[18])

        line37 = PointsAndLinesClass.ClassLine(listOfPoints[11], listOfPoints[9])
        line38 = PointsAndLinesClass.ClassLine(listOfPoints[11], listOfPoints[19])

        line39 = PointsAndLinesClass.ClassLine(listOfPoints[7], listOfPoints[14])

        line40 = PointsAndLinesClass.ClassLine(listOfPoints[10], listOfPoints[9])
        line41 = PointsAndLinesClass.ClassLine(listOfPoints[10], listOfPoints[11])
        line42 = PointsAndLinesClass.ClassLine(listOfPoints[10], listOfPoints[8])

        polygonLines = [line1, line2, line3, line4, line5, line6, line7, line8, line9, line10, line11, line12,
                        line13, line14, line15, line16, line17, line18, line19, line20, line21, line22,
                        line23, line24, line25, line26, line27, line28, line29, line30, line31, line32, line33,
                        line34, line35, line36, line37, line38, line39, line40, line41, line42]

        minX, maxX, minY, maxY = CrossFrameOptimizationLibrary.FindAxisLimits(listOfPoints)
        axisLimits = [minX, maxX, minY, maxY]

        point6InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line1)
        point7InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line2)
        point8InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line3)
        point9InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line4)
        point11InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line5)
        point12InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line7)
        point13InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line8)
        point15InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line9)
        point16InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line11)
        point17InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line12)
        point19InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line13)
        point20InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line15)
        point21InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line16)

        point23InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line18)
        point24InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line19)
        point25InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line20)

        point27InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line21)
        point28InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line22)

        point30InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line23)
        point31InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line24)
        point32InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line25)

        point34InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line26)
        point35InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line27)
        point36InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line28)

        point38InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line29)
        point39InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line30)
        point40InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line31)

        point42InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line32)
        point43InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line33)

        point44InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line39)

        point47InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line41)
        point48InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line40)

        point51InitialGuess = CrossFrameOptimizationLibrary.GetMidpointOfLine(line42)


        initialPointsGuesses = [point6InitialGuess, point7InitialGuess, point8InitialGuess, point9InitialGuess,
                                point11InitialGuess, point12InitialGuess, point13InitialGuess,
                                point15InitialGuess, point16InitialGuess, point17InitialGuess,
                                point19InitialGuess, point20InitialGuess, point21InitialGuess,
                                point23InitialGuess, point24InitialGuess, point25InitialGuess,
                                point27InitialGuess, point28InitialGuess,
                                point30InitialGuess, point31InitialGuess, point32InitialGuess,
                                point34InitialGuess, point35InitialGuess, point36InitialGuess,
                                point38InitialGuess, point39InitialGuess, point40InitialGuess,
                                point42InitialGuess, point43InitialGuess,
                                point44InitialGuess, point47InitialGuess, point48InitialGuess,
                                point51InitialGuess]

        A, Ix, Iy = CrossFrameOptimizationLibrary.GetPropertiesOfSections(crossSection, shapeBaseLength,
                                                                          shapeBaseHeight, shapeBaseDiameter)

        rho, E, Sy, Su = CrossFrameOptimizationLibrary.GetMaterialProperties(material)

        while tryAnotherPoint:
            isSinglePanel = False

            opt = {'maxiter': 1000}
            result = minimize(functionToMinimize, initialPointsGuesses,
                              constraints=CrossFrameConstraints.GetConstraintsNoMiddle(polygonLines, listOfPoints,
                                                                                         minDistances, isSinglePanel),
                              options=opt)
            # print(result)
            print("message:", result.message)
            print("success:", result.success)
            print("iterations:", result.nit)
            print("fun:", result.fun)


            point6 = PointsAndLinesClass.ClassPoint(result.x[0], result.x[1])
            point7 = PointsAndLinesClass.ClassPoint(result.x[2], result.x[3])
            point8 = PointsAndLinesClass.ClassPoint(result.x[4], result.x[5])
            point9 = PointsAndLinesClass.ClassPoint(result.x[6], result.x[7])
            point11 = PointsAndLinesClass.ClassPoint(result.x[8], result.x[9])
            point12 = PointsAndLinesClass.ClassPoint(result.x[10], result.x[11])
            point13 = PointsAndLinesClass.ClassPoint(result.x[12], result.x[13])
            point15 = PointsAndLinesClass.ClassPoint(result.x[14], result.x[15])
            point16 = PointsAndLinesClass.ClassPoint(result.x[16], result.x[17])
            point17 = PointsAndLinesClass.ClassPoint(result.x[18], result.x[19])
            point19 = PointsAndLinesClass.ClassPoint(result.x[20], result.x[21])
            point20 = PointsAndLinesClass.ClassPoint(result.x[22], result.x[23])
            point21 = PointsAndLinesClass.ClassPoint(result.x[24], result.x[25])

            point23 = PointsAndLinesClass.ClassPoint(result.x[26], result.x[27])
            point24 = PointsAndLinesClass.ClassPoint(result.x[28], result.x[29])
            point25 = PointsAndLinesClass.ClassPoint(result.x[30], result.x[31])

            point27 = PointsAndLinesClass.ClassPoint(result.x[32], result.x[33])
            point28 = PointsAndLinesClass.ClassPoint(result.x[34], result.x[35])

            point30 = PointsAndLinesClass.ClassPoint(result.x[36], result.x[37])
            point31 = PointsAndLinesClass.ClassPoint(result.x[38], result.x[39])
            point32 = PointsAndLinesClass.ClassPoint(result.x[40], result.x[41])

            point34 = PointsAndLinesClass.ClassPoint(result.x[42], result.x[43])
            point35 = PointsAndLinesClass.ClassPoint(result.x[44], result.x[45])
            point36 = PointsAndLinesClass.ClassPoint(result.x[46], result.x[47])

            point38 = PointsAndLinesClass.ClassPoint(result.x[48], result.x[49])
            point39 = PointsAndLinesClass.ClassPoint(result.x[50], result.x[51])
            point40 = PointsAndLinesClass.ClassPoint(result.x[52], result.x[53])

            point42 = PointsAndLinesClass.ClassPoint(result.x[54], result.x[55])
            point43 = PointsAndLinesClass.ClassPoint(result.x[56], result.x[57])

            point44 = PointsAndLinesClass.ClassPoint(result.x[58], result.x[59])

            point47 = PointsAndLinesClass.ClassPoint(result.x[60], result.x[61])
            point48 = PointsAndLinesClass.ClassPoint(result.x[62], result.x[63])

            point51 = PointsAndLinesClass.ClassPoint(result.x[64], result.x[65])

            if boolOptions[6] is False:
                line5 = PointsAndLinesClass.ClassLine(point6, point7)
                line6 = PointsAndLinesClass.ClassLine(point7, point8)
                line7 = PointsAndLinesClass.ClassLine(point8, point9)
                line8 = PointsAndLinesClass.ClassLine(point9, point6)

                line9 = PointsAndLinesClass.ClassLine(point11, point9)
                line10 = PointsAndLinesClass.ClassLine(point9, point12)
                line11 = PointsAndLinesClass.ClassLine(point12, point13)
                line12 = PointsAndLinesClass.ClassLine(point13, point11)

                line13 = PointsAndLinesClass.ClassLine(point15, point13)
                line14 = PointsAndLinesClass.ClassLine(point13, point16)
                line15 = PointsAndLinesClass.ClassLine(point16, point17)
                line16 = PointsAndLinesClass.ClassLine(point17, point15)

                line17 = PointsAndLinesClass.ClassLine(point19, point17)
                line18 = PointsAndLinesClass.ClassLine(point17, point20)
                line19 = PointsAndLinesClass.ClassLine(point20, point21)
                line20 = PointsAndLinesClass.ClassLine(point21, point19)

                line21 = PointsAndLinesClass.ClassLine(point8, point23)
                line22 = PointsAndLinesClass.ClassLine(point23, point24)
                line23 = PointsAndLinesClass.ClassLine(point24, point25)
                line24 = PointsAndLinesClass.ClassLine(point25, point8)

                line25 = PointsAndLinesClass.ClassLine(point12, point25)
                line26 = PointsAndLinesClass.ClassLine(point25, point27)
                line27 = PointsAndLinesClass.ClassLine(point27, point28)
                line28 = PointsAndLinesClass.ClassLine(point28, point12)

                line29 = PointsAndLinesClass.ClassLine(point30, point27)
                line30 = PointsAndLinesClass.ClassLine(point27, point31)
                line31 = PointsAndLinesClass.ClassLine(point31, point32)
                line32 = PointsAndLinesClass.ClassLine(point32, point30)

                line33 = PointsAndLinesClass.ClassLine(point34, point32)
                line34 = PointsAndLinesClass.ClassLine(point32, point35)
                line35 = PointsAndLinesClass.ClassLine(point35, point36)
                line36 = PointsAndLinesClass.ClassLine(point36, point34)

                line38 = PointsAndLinesClass.ClassLine(point38, point39)
                line39 = PointsAndLinesClass.ClassLine(point39, point34)
                line40 = PointsAndLinesClass.ClassLine(point34, point40)
                line41 = PointsAndLinesClass.ClassLine(point40, point38)

                line42 = PointsAndLinesClass.ClassLine(point42, point20)
                line43 = PointsAndLinesClass.ClassLine(point20, point38)
                line44 = PointsAndLinesClass.ClassLine(point38, point43)
                line45 = PointsAndLinesClass.ClassLine(point43, point42)

                line46 = PointsAndLinesClass.ClassLine(point16, point28)
                line47 = PointsAndLinesClass.ClassLine(point28, point44)
                line48 = PointsAndLinesClass.ClassLine(point44, point16)

                line49 = PointsAndLinesClass.ClassLine(point44, point30)
                line50 = PointsAndLinesClass.ClassLine(point30, point39)
                line51 = PointsAndLinesClass.ClassLine(point39, point44)

                line52 = PointsAndLinesClass.ClassLine(point48, point42)
                line53 = PointsAndLinesClass.ClassLine(point42, point47)
                line54 = PointsAndLinesClass.ClassLine(point47, point48)

                line55 = PointsAndLinesClass.ClassLine(point51, point21)
                line56 = PointsAndLinesClass.ClassLine(point21, point48)
                line57 = PointsAndLinesClass.ClassLine(point48, point51)

                panel1 = [line5, line6, line7, line8]
                panel2 = [line9, line10, line11, line12]
                panel3 = [line13, line14, line15, line16]
                panel4 = [line17, line18, line19, line20]
                panel5 = [line21, line22, line23, line24]
                panel6 = [line25, line26, line27, line28]
                panel7 = [line29, line30, line31, line32]
                panel8 = [line33, line34, line35, line36]
                panel9 = [line38, line39, line40, line41]
                panel10 = [line42, line43, line44, line45]
                panel11 = [line46, line47, line48]
                panel12 = [line49, line50, line51]
                panel13 = [line52, line53, line54]
                panel14 = [line55, line56, line57]

                pathLines = [line5, line6, line7, line8,
                             line9, line10, line11, line12,
                             line13, line14, line15, line16,
                             line17, line18, line19, line20,
                             line21, line22, line23, line24,
                             line25, line26, line27, line28,
                             line29, line30, line31, line32,
                             line33, line34, line35, line36,
                             line38, line39, line40, line41,
                             line42, line43, line44, line45,
                             line46, line47, line48, line49, line50, line51,
                             line52, line53, line54,
                             line55, line56, line57]
            else:
                line5 = PointsAndLinesClass.ClassLine(point6, point7)
                line6 = PointsAndLinesClass.ClassLine(point9, point7)
                line7 = PointsAndLinesClass.ClassLine(point8, point9)

                line9 = PointsAndLinesClass.ClassLine(point11, point9)
                line11 = PointsAndLinesClass.ClassLine(point9, point13)
                line12 = PointsAndLinesClass.ClassLine(point13, point12)

                line14 = PointsAndLinesClass.ClassLine(point15, point13)
                line15 = PointsAndLinesClass.ClassLine(point13, point17)
                line16 = PointsAndLinesClass.ClassLine(point17, point16)

                line18 = PointsAndLinesClass.ClassLine(point19, point17)
                line19 = PointsAndLinesClass.ClassLine(point17, point21)
                line20 = PointsAndLinesClass.ClassLine(point21, point20)

                line22 = PointsAndLinesClass.ClassLine(point8, point23)
                line23 = PointsAndLinesClass.ClassLine(point23, point25)
                line24 = PointsAndLinesClass.ClassLine(point24, point25)

                line26 = PointsAndLinesClass.ClassLine(point12, point25)
                line27 = PointsAndLinesClass.ClassLine(point28, point25)
                line28 = PointsAndLinesClass.ClassLine(point27, point28)

                line30 = PointsAndLinesClass.ClassLine(point30, point27)
                line31 = PointsAndLinesClass.ClassLine(point31, point30)
                line32 = PointsAndLinesClass.ClassLine(point31, point32)

                line34 = PointsAndLinesClass.ClassLine(point34, point32)
                line35 = PointsAndLinesClass.ClassLine(point32, point36)
                line36 = PointsAndLinesClass.ClassLine(point35, point36)

                line38 = PointsAndLinesClass.ClassLine(point38, point39)
                line39 = PointsAndLinesClass.ClassLine(point39, point40)
                line40 = PointsAndLinesClass.ClassLine(point34, point40)

                line42 = PointsAndLinesClass.ClassLine(point42, point20)
                line43 = PointsAndLinesClass.ClassLine(point42, point38)
                line44 = PointsAndLinesClass.ClassLine(point38, point43)

                line46 = PointsAndLinesClass.ClassLine(point44, point28)
                line47 = PointsAndLinesClass.ClassLine(point16, point44)

                line49 = PointsAndLinesClass.ClassLine(point44, point30)
                line50 = PointsAndLinesClass.ClassLine(point44, point39)

                line52 = PointsAndLinesClass.ClassLine(point42, point48)
                line53 = PointsAndLinesClass.ClassLine(point48, point47)

                line55 = PointsAndLinesClass.ClassLine(point48, point21)
                line56 = PointsAndLinesClass.ClassLine(point51, point48)

                panel1 = [line5, line6, line7]
                panel2 = [line9, line11, line12]
                panel3 = [line14, line15, line16]
                panel4 = [line18, line19, line20]
                panel5 = [line22, line23, line24]
                panel6 = [line26, line27, line28]
                panel7 = [line30, line31, line32]
                panel8 = [line34, line35, line36]
                panel9 = [line38, line39, line40]
                panel10 = [line42, line43, line44]
                panel11 = [line46, line47]
                panel12 = [line49, line50]
                panel13 = [line52, line53]
                panel14 = [line55, line56]

                pathLines = [line5, line6, line7,
                             line9, line11, line12,
                             line14, line15, line16,
                             line18, line19, line20,
                             line22, line23, line24,
                             line26, line27, line28,
                             line30, line31, line32,
                             line34, line35, line36,
                             line38, line39, line40,
                             line42, line43, line44,
                             line46, line47,
                             line49, line50,
                             line52, line53,
                             line55, line56]

            panels = [panel1, panel2, panel3, panel4, panel5, panel6, panel7, panel8, panel9, panel10, panel11, panel12,
                      panel13, panel14]

            xGuessPoints = []
            yGuessPoints = []

            plotLines = polygonLines.copy()
            plotLines.extend(pathLines)

            CreateCSV(panels)
            # PrintMassOfAllLines(pathLines)
            plotShape(plotLines, len(polygonLines), xGuessPoints, yGuessPoints, axisLimits, plotPointGuesses, boolOptions)

            tryAnotherPoint = False

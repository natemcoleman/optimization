import matplotlib.pyplot as plt
from scipy.optimize import minimize
import CrossFrameOptimizationLibrary
import CrossFrameConstraints
import PointsAndLinesClass
import PolyInteractor

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


def plotShape(linesToPlot, numberOfPolygonLines, xGuessPoints, yGuessPoints, axisLimits, plotPointGuesses, connectToMiddlePoint):
    # print("Number of path lines:", len(linesToPlot)-numberOfPolygonLines)
    for p in range(len(linesToPlot)):
        if p < numberOfPolygonLines:
            plt.plot(linesToPlot[p].points[0].x, linesToPlot[p].points[0].y, 'r*')
            plt.plot(linesToPlot[p].points[1].x, linesToPlot[p].points[1].y, 'r*')
            plt.plot([linesToPlot[p].points[0].x, linesToPlot[p].points[1].x], [linesToPlot[p].points[0].y, linesToPlot[p].points[1].y], 'b--')

        else:
            # print("p for path line:", p)
            plt.plot(linesToPlot[p].points[0].x, linesToPlot[p].points[0].y, 'go')
            plt.plot(linesToPlot[p].points[1].x, linesToPlot[p].points[1].y, 'go')
            plt.plot([linesToPlot[p].points[0].x, linesToPlot[p].points[1].x], [linesToPlot[p].points[0].y, linesToPlot[p].points[1].y], 'g-')

    if plotPointGuesses:
        for plotLength in range(len(xGuessPoints)-1):
            plt.plot(xGuessPoints[plotLength], yGuessPoints[plotLength], 'c.')

    # plt.plot(xGuessPoints[0], yGuessPoints[0], 'c.')
        # plt.plot(xGuessPoints[1], yGuessPoints[1], 'm.')
        # plt.plot(xGuessPoints[2], yGuessPoints[2], 'y.')
        # plt.plot(xGuessPoints[3], yGuessPoints[3], 'm.')
        # plt.plot(xGuessPoints[4], yGuessPoints[4], 'y.')
        # plt.plot(xGuessPoints[5], yGuessPoints[5], 'm.')
        # plt.plot(xGuessPoints[6], yGuessPoints[6], 'y.')
        # plt.plot(xGuessPoints[7], yGuessPoints[7], 'm.')
        # if connectToMiddlePoint:
        #     plt.plot(xGuessPoints[8], yGuessPoints[8], 'y.')
        #     plt.plot(xGuessPoints[9], yGuessPoints[9], 'm.')

    ax = plt.gca()
    ax.set_aspect(1)

    plt.xlabel('Y')
    plt.ylabel('X')
    xAxisBuffer = abs(axisLimits[1] - axisLimits[0]) * 0.05
    yAxisBuffer = abs(axisLimits[3] - axisLimits[2]) * 0.05

    plt.xlim([axisLimits[0] - xAxisBuffer, axisLimits[1] + xAxisBuffer])
    plt.ylim([axisLimits[2] - yAxisBuffer, axisLimits[3] + yAxisBuffer])

    plt.show()


def OptimizePolygon(listOfPoints, boolOptions, minDistances, crossSectionLengths, material, crossSection):
    connectToMiddlePoint = boolOptions[0]
    plotPointGuesses = boolOptions[1]
    allowModifyPolygon = boolOptions[2]
    tryAnotherPoint = boolOptions[3]
    multipleGuesses = boolOptions[4]
    allowSelectBeginPoint = boolOptions[5]

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
            print(result)

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

            # PrintMassOfAllLines(pathLines)
            plotShape(plotLines, len(polygonLines), xGuessPoints, yGuessPoints, axisLimits, plotPointGuesses, connectToMiddlePoint)

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

            # PrintMassOfAllLines(pathLines)
            plotShape(plotLines, len(polygonLines), xGuessPoints, yGuessPoints, axisLimits, plotPointGuesses, connectToMiddlePoint)

            if multipleGuesses:
                tryAnotherPoint = True
            else:
                tryAnotherPoint = False


def Optimize22Gore(listOfPoints, boolOptions, minDistances, crossSectionLengths, material, crossSection):
    connectToMiddlePoint = boolOptions[0]
    plotPointGuesses = boolOptions[1]
    # allowModifyPolygon = boolOptions[2]
    tryAnotherPoint = boolOptions[3]
    multipleGuesses = boolOptions[4]
    # allowSelectBeginPoint = boolOptions[5]

    shapeBaseLength = crossSectionLengths[0]  # meters, if square or rectangle
    shapeBaseHeight = crossSectionLengths[1]  # meters, if rectangle
    shapeBaseDiameter = crossSectionLengths[2]  # meters, if circle

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

            if plotPointGuesses:
                x5GuessPoints.append(optimalPoints[0])
                y5GuessPoints.append(optimalPoints[1])
                x6GuessPoints.append(optimalPoints[2])
                y6GuessPoints.append(optimalPoints[3])
                x7GuessPoints.append(optimalPoints[4])
                y7GuessPoints.append(optimalPoints[5])
                x8GuessPoints.append(optimalPoints[6])
                y8GuessPoints.append(optimalPoints[7])
                x9GuessPoints.append(optimalPoints[8])
                y9GuessPoints.append(optimalPoints[9])
                x10GuessPoints.append(optimalPoints[10])
                y10GuessPoints.append(optimalPoints[11])
                x11GuessPoints.append(optimalPoints[12])
                y11GuessPoints.append(optimalPoints[13])
                x12GuessPoints.append(optimalPoints[14])
                y12GuessPoints.append(optimalPoints[15])
                x13GuessPoints.append(optimalPoints[16])
                y13GuessPoints.append(optimalPoints[17])
                x14GuessPoints.append(optimalPoints[18])
                y14GuessPoints.append(optimalPoints[19])
                x15GuessPoints.append(optimalPoints[20])
                y15GuessPoints.append(optimalPoints[21])
                x16GuessPoints.append(optimalPoints[22])
                y16GuessPoints.append(optimalPoints[23])
                x17GuessPoints.append(optimalPoints[24])
                y17GuessPoints.append(optimalPoints[25])


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

            pathLinesNew = [line5Opt, line6Opt, line7Opt, line8Opt, line9Opt, line10Opt, line11Opt, line12Opt, line13Opt, line14Opt, line15Opt, line16Opt]

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

        polygonLines = [line1, line2, line3, line4, line5, line6, line7, line8, line9, line10, line11, line12]
        panel1 = [line1, line2, line3, line4]
        panel2 = [line5, line6, line7, line8]
        panel3 = [line9, line10, line11, line12]

        minX1, maxX1, minY1, maxY1 = CrossFrameOptimizationLibrary.FindAxisLimitsOfLines(panel1)
        minX2, maxX2, minY2, maxY2 = CrossFrameOptimizationLibrary.FindAxisLimitsOfLines(panel2)
        minX3, maxX3, minY3, maxY3 = CrossFrameOptimizationLibrary.FindAxisLimitsOfLines(panel3)


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


        point5InitialGuess = [((minX1 + maxX1) / 2, (minY1 + maxY1) / 2)]  # First guess is in middle of bounds
        point10InitialGuess = [((minX2 + maxX2) / 2, (minY2 + maxY2) / 2)]  # First guess is in middle of bounds
        point14InitialGuess = [((minX3 + maxX3) / 2, (minY3 + maxY3) / 2)]  # First guess is in middle of bounds

        initialPointsGuesses = [point5InitialGuess, point6InitialGuess, point7InitialGuess, point8InitialGuess, point9InitialGuess, point10InitialGuess, point11InitialGuess, point12InitialGuess, point13InitialGuess, point14InitialGuess, point15InitialGuess, point16InitialGuess, point17InitialGuess]

        A, Ix, Iy = CrossFrameOptimizationLibrary.GetPropertiesOfSections(crossSection, shapeBaseLength,
                                                                          shapeBaseHeight, shapeBaseDiameter)

        rho, E, Sy, Su = CrossFrameOptimizationLibrary.GetMaterialProperties(material)

        while tryAnotherPoint:

            if plotPointGuesses:
                x5GuessPoints = []
                y5GuessPoints = []
                x6GuessPoints = []
                y6GuessPoints = []
                x7GuessPoints = []
                y7GuessPoints = []
                x8GuessPoints = []
                y8GuessPoints = []
                x9GuessPoints = []
                y9GuessPoints = []
                x10GuessPoints = []
                y10GuessPoints = []
                x11GuessPoints = []
                y11GuessPoints = []
                x12GuessPoints = []
                y12GuessPoints = []
                x13GuessPoints = []
                y13GuessPoints = []
                x14GuessPoints = []
                y14GuessPoints = []
                x15GuessPoints = []
                y15GuessPoints = []
                x16GuessPoints = []
                y16GuessPoints = []
                x17GuessPoints = []
                y17GuessPoints = []

            isSinglePanel = False

            opt = {'maxiter': 1000}
            result = minimize(functionToMinimize, initialPointsGuesses,
                              constraints=CrossFrameConstraints.GetConstraintsWithMiddle(polygonLines, listOfPoints,
                                                                                         minDistances, isSinglePanel), options=opt)
            print(result)

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

            pathLines = [line5, line6, line7, line8, line9, line10, line11, line12, line13, line14, line15, line16]

            if plotPointGuesses:
                xGuessPoints = [x5GuessPoints, x6GuessPoints, x7GuessPoints, x8GuessPoints, x9GuessPoints, x10GuessPoints, x11GuessPoints, x12GuessPoints, x13GuessPoints, x14GuessPoints, x15GuessPoints, x16GuessPoints, x17GuessPoints]
                yGuessPoints = [y5GuessPoints, y6GuessPoints, y7GuessPoints, y8GuessPoints, y9GuessPoints, y10GuessPoints, y11GuessPoints, y12GuessPoints, y13GuessPoints, y14GuessPoints, y15GuessPoints, y16GuessPoints, y17GuessPoints]
            else:
                xGuessPoints = []
                yGuessPoints = []

            plotLines = polygonLines.copy()
            plotLines.extend(pathLines)

            # PrintMassOfAllLines(pathLines)
            plotShape(plotLines, len(polygonLines), xGuessPoints, yGuessPoints, axisLimits, plotPointGuesses, connectToMiddlePoint)

            if multipleGuesses:
                tryAnotherPoint = True
            else:
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

            if plotPointGuesses:
                x6GuessPoints.append(optimalPoints[0])
                y6GuessPoints.append(optimalPoints[1])
                x7GuessPoints.append(optimalPoints[2])
                y7GuessPoints.append(optimalPoints[3])
                x8GuessPoints.append(optimalPoints[4])
                y8GuessPoints.append(optimalPoints[5])
                x9GuessPoints.append(optimalPoints[6])
                y9GuessPoints.append(optimalPoints[7])
                x11GuessPoints.append(optimalPoints[8])
                y11GuessPoints.append(optimalPoints[9])
                x12GuessPoints.append(optimalPoints[10])
                y12GuessPoints.append(optimalPoints[11])
                x13GuessPoints.append(optimalPoints[12])
                y13GuessPoints.append(optimalPoints[13])
                x15GuessPoints.append(optimalPoints[14])
                y15GuessPoints.append(optimalPoints[15])
                x16GuessPoints.append(optimalPoints[16])
                y16GuessPoints.append(optimalPoints[17])
                x17GuessPoints.append(optimalPoints[18])
                y17GuessPoints.append(optimalPoints[19])

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

            pathLinesNew = [line5Opt, line6Opt, line7Opt, line8Opt, line9Opt, line11Opt, line12Opt, line13Opt, line14Opt, line15Opt, line16Opt, line17Opt]

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

        polygonLines = [line1, line2, line3, line4, line5, line6, line7, line8, line9, line10, line11, line12]
        panel1 = [line1, line2, line3, line4]
        panel2 = [line5, line6, line7, line8]
        panel3 = [line9, line10, line11, line12]

        minX1, maxX1, minY1, maxY1 = CrossFrameOptimizationLibrary.FindAxisLimitsOfLines(panel1)
        minX2, maxX2, minY2, maxY2 = CrossFrameOptimizationLibrary.FindAxisLimitsOfLines(panel2)
        minX3, maxX3, minY3, maxY3 = CrossFrameOptimizationLibrary.FindAxisLimitsOfLines(panel3)

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


        initialPointsGuesses = [point6InitialGuess, point7InitialGuess, point8InitialGuess,
                                point9InitialGuess, point11InitialGuess, point12InitialGuess,
                                point13InitialGuess, point15InitialGuess, point16InitialGuess, point17InitialGuess]

        A, Ix, Iy = CrossFrameOptimizationLibrary.GetPropertiesOfSections(crossSection, shapeBaseLength,
                                                                          shapeBaseHeight, shapeBaseDiameter)

        rho, E, Sy, Su = CrossFrameOptimizationLibrary.GetMaterialProperties(material)

        while tryAnotherPoint:

            if plotPointGuesses:
                x6GuessPoints = []
                y6GuessPoints = []
                x7GuessPoints = []
                y7GuessPoints = []
                x8GuessPoints = []
                y8GuessPoints = []
                x9GuessPoints = []
                y9GuessPoints = []
                x11GuessPoints = []
                y11GuessPoints = []
                x12GuessPoints = []
                y12GuessPoints = []
                x13GuessPoints = []
                y13GuessPoints = []
                x15GuessPoints = []
                y15GuessPoints = []
                x16GuessPoints = []
                y16GuessPoints = []
                x17GuessPoints = []
                y17GuessPoints = []

            isSinglePanel = False

            opt = {'maxiter': 1000}
            result = minimize(functionToMinimize, initialPointsGuesses,
                              constraints=CrossFrameConstraints.GetConstraintsNoMiddle(polygonLines, listOfPoints,
                                                                                         minDistances, isSinglePanel),
                              options=opt)
            print(result)

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


            pathLines = [line5, line6, line7, line8, line9, line10, line11, line12, line13, line14, line15, line16]

            if plotPointGuesses:
                xGuessPoints = [x6GuessPoints, x7GuessPoints, x8GuessPoints, x9GuessPoints,
                                x11GuessPoints, x12GuessPoints, x13GuessPoints, x15GuessPoints, x16GuessPoints, x17GuessPoints]
                yGuessPoints = [y6GuessPoints, y7GuessPoints, y8GuessPoints, y9GuessPoints,
                                y11GuessPoints, y12GuessPoints, y13GuessPoints, y15GuessPoints, y16GuessPoints, y17GuessPoints]
            else:
                xGuessPoints = []
                yGuessPoints = []

            plotLines = polygonLines.copy()
            plotLines.extend(pathLines)

            # PrintMassOfAllLines(pathLines)
            plotShape(plotLines, len(polygonLines), xGuessPoints, yGuessPoints, axisLimits, plotPointGuesses, connectToMiddlePoint)

            if multipleGuesses:
                tryAnotherPoint = True
            else:
                tryAnotherPoint = False
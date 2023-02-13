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
        def plotShape(linesToPlot, numberOfPolygonLines):
            xValuesToPlotShape = []
            yValuesToPlotShape = []
            xValuesToPlotPath = []
            yValuesToPlotPath = []
            for p in range(len(linesToPlot)):
                if p < numberOfPolygonLines:
                    plt.plot(linesToPlot[p].points[0].x, linesToPlot[p].points[0].y, 'r*')
                    plt.plot(linesToPlot[p].points[1].x, linesToPlot[p].points[1].y, 'r*')
                    xValuesToPlotShape.append(linesToPlot[p].points[0].x)
                    xValuesToPlotShape.append(linesToPlot[p].points[1].x)
                    yValuesToPlotShape.append(linesToPlot[p].points[0].y)
                    yValuesToPlotShape.append(linesToPlot[p].points[1].y)
                else:
                    plt.plot(linesToPlot[p].points[0].x, linesToPlot[p].points[0].y, 'go')
                    plt.plot(linesToPlot[p].points[1].x, linesToPlot[p].points[1].y, 'go')
                    xValuesToPlotPath.append(linesToPlot[p].points[0].x)
                    xValuesToPlotPath.append(linesToPlot[p].points[1].x)
                    yValuesToPlotPath.append(linesToPlot[p].points[0].y)
                    yValuesToPlotPath.append(linesToPlot[p].points[1].y)

            if len(polygonLines) == 4:
                xValuesToPlotShape.pop()
                yValuesToPlotShape.pop()

                xValuesToPlotPath.pop()
                yValuesToPlotPath.pop()

            plt.plot(xValuesToPlotShape, yValuesToPlotShape, 'b--')
            plt.plot(xValuesToPlotPath, yValuesToPlotPath, 'g-')

            if plotPointGuesses:
                plt.plot(x5GuessPoints, y5GuessPoints, 'c.')
                plt.plot(x6GuessPoints, y6GuessPoints, 'm.')
                plt.plot(x7GuessPoints, y7GuessPoints, 'y.')
                plt.plot(x8GuessPoints, y8GuessPoints, 'm.')
                if numberOfPolygonLines > 3:
                    plt.plot(x9GuessPoints, y9GuessPoints, 'y.')

            ax = plt.gca()
            ax.set_aspect(1)

            plt.xlabel('Y')
            plt.ylabel('X')

            xAxisBuffer = abs(maxX - minX) * 0.05
            yAxisBuffer = abs(maxY - minY) * 0.05

            plt.xlim([minX - xAxisBuffer, maxX + xAxisBuffer])
            plt.ylim([minY - yAxisBuffer, maxY + yAxisBuffer])

            plt.show()

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

            opt = {'maxiter': 1000}
            result = minimize(functionToMinimize, initialPointsGuesses,
                              constraints=CrossFrameConstraints.GetConstraintsWithMiddle(polygonLines, listOfPoints,
                                                                                         minDistances), options=opt)
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
            if len(listOfPoints) > 3:
                line8 = PointsAndLinesClass.ClassLine(point9, point5)
                pathLines = [line5, line6, line7, line8]
            else:
                pathLines = [line5, line6, line7]

            plotLines = polygonLines.copy()
            plotLines.extend(pathLines)

            # PrintMassOfAllLines(pathLines)
            plotShape(plotLines, len(polygonLines))

            if multipleGuesses:
                tryAnotherPoint = True
            else:
                tryAnotherPoint = False
    else:
        def plotShape(linesToPlot, numberOfPolygonLines):
            xValuesToPlotShape = []
            yValuesToPlotShape = []
            xValuesToPlotPath = []
            yValuesToPlotPath = []
            for p in range(len(linesToPlot)):
                if p < numberOfPolygonLines:
                    plt.plot(linesToPlot[p].points[0].x, linesToPlot[p].points[0].y, 'r*')
                    plt.plot(linesToPlot[p].points[1].x, linesToPlot[p].points[1].y, 'r*')
                    xValuesToPlotShape.append(linesToPlot[p].points[0].x)
                    xValuesToPlotShape.append(linesToPlot[p].points[1].x)
                    yValuesToPlotShape.append(linesToPlot[p].points[0].y)
                    yValuesToPlotShape.append(linesToPlot[p].points[1].y)
                else:
                    plt.plot(linesToPlot[p].points[0].x, linesToPlot[p].points[0].y, 'go')
                    plt.plot(linesToPlot[p].points[1].x, linesToPlot[p].points[1].y, 'go')
                    xValuesToPlotPath.append(linesToPlot[p].points[0].x)
                    xValuesToPlotPath.append(linesToPlot[p].points[1].x)
                    yValuesToPlotPath.append(linesToPlot[p].points[0].y)
                    yValuesToPlotPath.append(linesToPlot[p].points[1].y)

            if len(polygonLines) == 4:
                xValuesToPlotShape.pop()
                yValuesToPlotShape.pop()

            plt.plot(xValuesToPlotShape, yValuesToPlotShape, 'b--')
            plt.plot(xValuesToPlotPath, yValuesToPlotPath, 'g-')

            if plotPointGuesses:
                plt.plot(x6GuessPoints, y6GuessPoints, 'm.')
                plt.plot(x7GuessPoints, y7GuessPoints, 'y.')
                plt.plot(x8GuessPoints, y8GuessPoints, 'm.')
                if numberOfPolygonLines > 3:
                    plt.plot(x9GuessPoints, y9GuessPoints, 'y.')

            ax = plt.gca()
            ax.set_aspect(1)

            plt.xlabel('Y')
            plt.ylabel('X')

            xAxisBuffer = abs(maxX - minX) * 0.05
            yAxisBuffer = abs(maxY - minY) * 0.05

            plt.xlim([minX - xAxisBuffer, maxX + xAxisBuffer])
            plt.ylim([minY - yAxisBuffer, maxY + yAxisBuffer])

            plt.show()

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

            opt = {'maxiter': 1000}
            result = minimize(functionToMinimize, initialPointsGuesses,
                              constraints=CrossFrameConstraints.GetConstraintsNoMiddle(polygonLines, listOfPoints,
                                                                                       minDistances),
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

            plotLines = polygonLines.copy()
            plotLines.extend(pathLines)

            # PrintMassOfAllLines(pathLines)
            plotShape(plotLines, len(polygonLines))

            if multipleGuesses:
                tryAnotherPoint = True
            else:
                tryAnotherPoint = False
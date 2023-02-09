import CrossFrameOptimizationLibrary
import PointsAndLinesClass

def GetConstraints():
    def constraint1(optimalPoints):
        returnVec = []

        # for distanceIndex in range(len(polygonLines)):
        #     returnVec.append(GetPointDistanceFromLine(currPoint5, polygonLines[distanceIndex])-minDistanceFromLine)

        returnVec.append(
            CrossFrameOptimizationLibrary.GetPointDistanceFromLine(PointsAndLinesClass.ClassPoint(optimalPoints[0], optimalPoints[1]),
                                                                   polygonLines[2])
            - minDistanceFromLine)

        return returnVec

    def KeepMiddleNodeMinDistanceFromCornersConstraint(optimalPoints):
        distanceBetweenPoints = []
        currentPoint5 = PointsAndLinesClass.ClassPoint(optimalPoints[0], optimalPoints[1])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(point1, currentPoint5) - minDistanceFromCorners)
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(point2, currentPoint5) - minDistanceFromCorners)
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(point3, currentPoint5) - minDistanceFromCorners)

        if len(optimalPoints) > 9:
            distanceBetweenPoints.append(
                CrossFrameOptimizationLibrary.distance(point4, currentPoint5) - minDistanceFromCorners)

        return distanceBetweenPoints

    def KeepGuessPointsMinDistanceApartConstraint(optimalPoints):
        distanceBetweenPoints = []
        currentPoint5 = PointsAndLinesClass.ClassPoint(optimalPoints[0], optimalPoints[1])
        currentPoint6 = PointsAndLinesClass.ClassPoint(optimalPoints[2], optimalPoints[3])
        currentPoint7 = PointsAndLinesClass.ClassPoint(optimalPoints[4], optimalPoints[5])
        currentPoint8 = PointsAndLinesClass.ClassPoint(optimalPoints[6], optimalPoints[7])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint6, currentPoint7) - minDistanceBetweenPathNodes)
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint7, currentPoint8) - minDistanceBetweenPathNodes)
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint6, currentPoint5) - minDistanceBetweenPathNodes)
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint7, currentPoint5) - minDistanceBetweenPathNodes)
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint8, currentPoint5) - minDistanceBetweenPathNodes)

        if len(optimalPoints) > 9:
            currentPoint9 = PointsAndLinesClass.ClassPoint(optimalPoints[8], optimalPoints[9])

            distanceBetweenPoints.append(
                CrossFrameOptimizationLibrary.distance(currentPoint8, currentPoint9) - minDistanceBetweenPathNodes)
            distanceBetweenPoints.append(
                CrossFrameOptimizationLibrary.distance(currentPoint9, currentPoint6) - minDistanceBetweenPathNodes)
            distanceBetweenPoints.append(
                CrossFrameOptimizationLibrary.distance(currentPoint9, currentPoint5) - minDistanceBetweenPathNodes)

        return distanceBetweenPoints

    def PointIsBoundedInPolygonConstraint(optimalPoints):
        returnVec = []
        yOfLine2 = line2.points[1].y + (
                    ((line2.points[0].y - line2.points[1].y) / (line2.points[0].x - line2.points[1].x))
                    * (optimalPoints[0] - line2.points[1].x))

        xOfLine1 = line1.points[1].x + (
                    ((line1.points[1].x - line1.points[0].x) / (line1.points[1].y - line1.points[0].y))
                    * (optimalPoints[1] - line1.points[1].y))

        xOfLine3 = line3.points[1].x + (
                    ((line3.points[1].x - line3.points[0].x) / (line3.points[1].y - line3.points[0].y))
                    * (optimalPoints[1] - line3.points[1].y))

        returnVec.append(yOfLine2 - optimalPoints[1])
        returnVec.append(optimalPoints[0] - xOfLine1)
        returnVec.append(xOfLine3 - optimalPoints[0])

        if len(optimalPoints) > 9:
            yOfLine4 = line4.points[1].y + (
                        ((line4.points[0].y - line4.points[1].y) / (line4.points[0].x - line4.points[1].x))
                        * (optimalPoints[0] - line4.points[1].x))
            returnVec.append(optimalPoints[1] - yOfLine4)

        return returnVec

    def PathStartPointsFallOnLines(optimalPoints):
        returnVec = []

        # point6ThatShouldBeOnLine = classPoint(optimalPoints[2], optimalPoints[3])
        # returnVec.append(distance(line1.points[0], point6ThatShouldBeOnLine) +
        # distance(point6ThatShouldBeOnLine, line1.points[1]) - distance(line1.points[0], line1.points[1]))

        # point7ThatShouldBeOnLine = classPoint(optimalPoints[4], optimalPoints[5])
        # returnVec.append(distance(line2.points[0], point7ThatShouldBeOnLine) +
        # distance(point7ThatShouldBeOnLine, line2.points[1]) - distance(line2.points[0], line2.points[1]))

        # point8ThatShouldBeOnLine = classPoint(optimalPoints[6], optimalPoints[7])
        # returnVec.append(distance(line3.points[0], point8ThatShouldBeOnLine) +
        # distance(point8ThatShouldBeOnLine, line3.points[1]) - distance(line3.points[0], line3.points[1]))

        # point9ThatShouldBeOnLine = classPoint(optimalPoints[8], optimalPoints[9])
        # returnVec.append(distance(line4.points[0], point9ThatShouldBeOnLine) +
        # distance(point9ThatShouldBeOnLine, line4.points[1]) - distance(line4.points[0], line4.points[1]))

        # yOfLine2 = line2.points[1].y + (((line2.points[0].y - line2.points[1].y) / (line2.points[0].x - line2.points[1].x))
        #                                 * (optimalPoints[0] - line2.points[1].x))
        yOfLine2 = line2.points[1].y + (
                    ((line2.points[1].y - line2.points[0].y) / (line2.points[1].x - line2.points[0].x))
                    * (optimalPoints[4] - line2.points[1].x))

        # xOfLine1 = line1.points[1].x + (((line1.points[1].x - line1.points[0].x) / (line1.points[1].y - line1.points[0].y))
        #                                 * (optimalPoints[1] - line1.points[1].y))
        xOfLine1 = line1.points[1].x + (
                    ((line1.points[1].x - line1.points[0].x) / (line1.points[1].y - line1.points[0].y))
                    * (optimalPoints[3] - line1.points[1].y))

        xOfLine3 = line3.points[0].x + (
                    ((line3.points[1].x - line3.points[0].x) / (line3.points[1].y - line3.points[0].y))
                    * (optimalPoints[7] - line3.points[0].y))

        returnVec.append(optimalPoints[2] - xOfLine1)
        returnVec.append(xOfLine3 - optimalPoints[6])
        returnVec.append(yOfLine2 - optimalPoints[5])

        if len(optimalPoints) > 9:
            yOfLine4 = line4.points[1].y + (
                        ((line4.points[0].y - line4.points[1].y) / (line4.points[0].x - line4.points[1].x))
                        * (optimalPoints[8] - line4.points[1].x))

            returnVec.append(optimalPoints[9] - yOfLine4)

        # print("optimal9:", optimalPoints[9], " yOfLine4:", yOfLine4)

        # for x in range(len(returnVec)):
        #     print(returnVec[x])

        return returnVec

    def StartPointsDoNotGoBeyondLineConstraint(optimalPoints):
        returnVec = []

        returnVec.append(line1.points[1].y - optimalPoints[3])
        returnVec.append(optimalPoints[3] - line1.points[0].y)
        returnVec.append(line3.points[0].y - optimalPoints[7])
        returnVec.append(optimalPoints[7] - line3.points[1].y)
        returnVec.append(line2.points[1].x - optimalPoints[4])
        returnVec.append(optimalPoints[4] - line2.points[0].x)
        if len(optimalPoints) > 9:
            returnVec.append(line4.points[1].x - optimalPoints[8])
            returnVec.append(optimalPoints[8] - line4.points[0].x)

        return returnVec

    con1 = {'type': 'ineq', 'fun': KeepGuessPointsMinDistanceApartConstraint}
    con2 = {'type': 'ineq', 'fun': PointIsBoundedInPolygonConstraint}
    con3 = {'type': 'eq', 'fun': PathStartPointsFallOnLines}
    con4 = {'type': 'ineq', 'fun': StartPointsDoNotGoBeyondLineConstraint}
    con5 = {'type': 'ineq', 'fun': KeepMiddleNodeMinDistanceFromCornersConstraint}
    # con6 = {'type': 'ineq', 'fun': constraint1}  # Point is a minimum distance from line

    cons = [con1, con2, con3, con4, con5]
    return cons


import CrossFrameOptimizationLibrary
import PointsAndLinesClass

def GetConstraintsWithMiddle(polygonLines, listOfPoints, minDistances, isSinglePanel):

    def KeepMiddleNodeMinDistanceFromCornersConstraintPanel1(optimalPoints):
        distanceBetweenPoints = []
        currentPoint5 = PointsAndLinesClass.ClassPoint(optimalPoints[0], optimalPoints[1])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[0], currentPoint5) - minDistances[1])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[1], currentPoint5) - minDistances[1])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[2], currentPoint5) - minDistances[1])

        if len(optimalPoints) > 9:
            distanceBetweenPoints.append(
                CrossFrameOptimizationLibrary.distance(listOfPoints[3], currentPoint5) - minDistances[1])

        return distanceBetweenPoints

    def KeepGuessPointsMinDistanceApartConstraintPanel1(optimalPoints):
        distanceBetweenPoints = []
        currentPoint5 = PointsAndLinesClass.ClassPoint(optimalPoints[0], optimalPoints[1])
        currentPoint6 = PointsAndLinesClass.ClassPoint(optimalPoints[2], optimalPoints[3])
        currentPoint7 = PointsAndLinesClass.ClassPoint(optimalPoints[4], optimalPoints[5])
        currentPoint8 = PointsAndLinesClass.ClassPoint(optimalPoints[6], optimalPoints[7])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint6, currentPoint7) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint7, currentPoint8) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint6, currentPoint5) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint7, currentPoint5) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint8, currentPoint5) - minDistances[0])

        if len(optimalPoints) > 9:
            currentPoint9 = PointsAndLinesClass.ClassPoint(optimalPoints[8], optimalPoints[9])

            distanceBetweenPoints.append(
                CrossFrameOptimizationLibrary.distance(currentPoint8, currentPoint9) - minDistances[0])
            distanceBetweenPoints.append(
                CrossFrameOptimizationLibrary.distance(currentPoint9, currentPoint6) - minDistances[0])
            distanceBetweenPoints.append(
                CrossFrameOptimizationLibrary.distance(currentPoint9, currentPoint5) - minDistances[0])

        return distanceBetweenPoints

    def PointIsBoundedInPolygonConstraintPanel1(optimalPoints):
        returnVec = []
        yOfLine2 = polygonLines[1].points[1].y + (
                    ((polygonLines[1].points[0].y - polygonLines[1].points[1].y) / (polygonLines[1].points[0].x - polygonLines[1].points[1].x))
                    * (optimalPoints[0] - polygonLines[1].points[1].x))

        xOfLine1 = polygonLines[0].points[1].x + (
                    ((polygonLines[0].points[1].x - polygonLines[0].points[0].x) / (polygonLines[0].points[1].y - polygonLines[0].points[0].y))
                    * (optimalPoints[1] - polygonLines[0].points[1].y))

        xOfLine3 = polygonLines[2].points[1].x + (
                    ((polygonLines[2].points[1].x - polygonLines[2].points[0].x) / (polygonLines[2].points[1].y - polygonLines[2].points[0].y))
                    * (optimalPoints[1] - polygonLines[2].points[1].y))

        returnVec.append(yOfLine2 - optimalPoints[1])
        returnVec.append(optimalPoints[0] - xOfLine1)
        returnVec.append(xOfLine3 - optimalPoints[0])

        if len(optimalPoints) > 9:
            yOfLine4 = polygonLines[3].points[1].y + (
                        ((polygonLines[3].points[0].y - polygonLines[3].points[1].y) / (polygonLines[3].points[0].x - polygonLines[3].points[1].x))
                        * (optimalPoints[0] - polygonLines[3].points[1].x))
            returnVec.append(optimalPoints[1] - yOfLine4)

        return returnVec

    def PathStartPointsFallOnLinesPanel1(optimalPoints):
        returnVec = []

        yOfLine2 = polygonLines[1].points[1].y + (
                    ((polygonLines[1].points[1].y - polygonLines[1].points[0].y) / (polygonLines[1].points[1].x - polygonLines[1].points[0].x))
                    * (optimalPoints[4] - polygonLines[1].points[1].x))


        xOfLine1 = polygonLines[0].points[1].x + (
                    ((polygonLines[0].points[1].x - polygonLines[0].points[0].x) / (polygonLines[0].points[1].y - polygonLines[0].points[0].y))
                    * (optimalPoints[3] - polygonLines[0].points[1].y))

        xOfLine3 = polygonLines[2].points[0].x + (
                    ((polygonLines[2].points[1].x - polygonLines[2].points[0].x) / (polygonLines[2].points[1].y - polygonLines[2].points[0].y))
                    * (optimalPoints[7] - polygonLines[2].points[0].y))

        returnVec.append(optimalPoints[2] - xOfLine1)
        returnVec.append(xOfLine3 - optimalPoints[6])
        returnVec.append(yOfLine2 - optimalPoints[5])

        if len(optimalPoints) > 9:
            yOfLine4 = polygonLines[3].points[1].y + (
                        ((polygonLines[3].points[0].y - polygonLines[3].points[1].y) / (polygonLines[3].points[0].x - polygonLines[3].points[1].x))
                        * (optimalPoints[8] - polygonLines[3].points[1].x))

            returnVec.append(optimalPoints[9] - yOfLine4)

        return returnVec

    def StartPointsDoNotGoBeyondLineConstraintPanel1(optimalPoints):
        returnVec = []

        returnVec.append(polygonLines[0].points[1].y - optimalPoints[3])
        returnVec.append(optimalPoints[3] - polygonLines[0].points[0].y)
        returnVec.append(polygonLines[2].points[0].y - optimalPoints[7])
        returnVec.append(optimalPoints[7] - polygonLines[2].points[1].y)
        returnVec.append(polygonLines[1].points[1].x - optimalPoints[4])
        returnVec.append(optimalPoints[4] - polygonLines[1].points[0].x)
        if len(optimalPoints) > 9:
            returnVec.append(polygonLines[3].points[1].x - optimalPoints[8])
            returnVec.append(optimalPoints[8] - polygonLines[3].points[0].x)

        return returnVec

    def KeepMiddleNodeMinDistanceFromCornersConstraintPanel2(optimalPoints):

        # for e in range(len(optimalPoints)):
        #     print("optimalPoint", e, ": ", optimalPoints[e])

        distanceBetweenPoints = []
        currentPoint10 = PointsAndLinesClass.ClassPoint(optimalPoints[10], optimalPoints[11])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[0], currentPoint10) - minDistances[1])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[3], currentPoint10) - minDistances[1])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[4], currentPoint10) - minDistances[1])
        distanceBetweenPoints.append(
                CrossFrameOptimizationLibrary.distance(listOfPoints[5], currentPoint10) - minDistances[1])

        return distanceBetweenPoints

    def KeepGuessPointsMinDistanceApartConstraintPanel2(optimalPoints):
        distanceBetweenPoints = []
        currentPoint10 = PointsAndLinesClass.ClassPoint(optimalPoints[10], optimalPoints[11])
        currentPoint11 = PointsAndLinesClass.ClassPoint(optimalPoints[12], optimalPoints[13])
        currentPoint12 = PointsAndLinesClass.ClassPoint(optimalPoints[14], optimalPoints[15])
        currentPoint13 = PointsAndLinesClass.ClassPoint(optimalPoints[16], optimalPoints[17])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint11, currentPoint12) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint12, currentPoint13) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint11, currentPoint10) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint12, currentPoint10) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint13, currentPoint10) - minDistances[0])

        return distanceBetweenPoints

    def PointIsBoundedInPolygonConstraintPanel2(optimalPoints):
        returnVec = []

        xOfLine5 = polygonLines[4].points[1].x + (
                ((polygonLines[4].points[1].x - polygonLines[4].points[0].x) / (
                        polygonLines[4].points[1].y - polygonLines[4].points[0].y))
                * (optimalPoints[11] - polygonLines[4].points[1].y))

        yOfLine6 = polygonLines[5].points[1].y + (
                ((polygonLines[5].points[0].y - polygonLines[5].points[1].y) / (
                            polygonLines[5].points[0].x - polygonLines[5].points[1].x))
                * (optimalPoints[10] - polygonLines[5].points[1].x))

        xOfLine7 = polygonLines[6].points[1].x + (
                ((polygonLines[6].points[1].x - polygonLines[6].points[0].x) / (
                            polygonLines[6].points[1].y - polygonLines[6].points[0].y))
                * (optimalPoints[11] - polygonLines[6].points[1].y))

        yOfLine8 = polygonLines[7].points[1].y + (
                    ((polygonLines[7].points[0].y - polygonLines[7].points[1].y) / (
                                polygonLines[7].points[0].x - polygonLines[7].points[1].x))
                    * (optimalPoints[10] - polygonLines[7].points[1].x))


        returnVec.append(yOfLine6 - optimalPoints[11])
        returnVec.append(optimalPoints[10] - xOfLine5)
        returnVec.append(xOfLine7 - optimalPoints[10])
        returnVec.append(optimalPoints[11] - yOfLine8)

        return returnVec

    def PathStartPointsFallOnLinesPanel2(optimalPoints):
        returnVec = []

        xOfLine5 = polygonLines[4].points[1].x + (
                ((polygonLines[4].points[1].x - polygonLines[4].points[0].x) / (
                        polygonLines[4].points[1].y - polygonLines[4].points[0].y))
                * (optimalPoints[13] - polygonLines[4].points[1].y))

        # yOfLine6 = polygonLines[5].points[1].y + (
        #         ((polygonLines[5].points[0].y - polygonLines[5].points[1].y) / (
        #                 polygonLines[5].points[0].x - polygonLines[5].points[1].x))
        #         * (optimalPoints[8] - polygonLines[5].points[1].x))

        xOfLine7 = polygonLines[6].points[1].x + (
                ((polygonLines[6].points[1].x - polygonLines[6].points[0].x) / (
                        polygonLines[6].points[1].y - polygonLines[6].points[0].y))
                * (optimalPoints[15] - polygonLines[6].points[1].y))

        yOfLine8 = polygonLines[7].points[1].y + (
                ((polygonLines[7].points[0].y - polygonLines[7].points[1].y) / (
                        polygonLines[7].points[0].x - polygonLines[7].points[1].x))
                * (optimalPoints[16] - polygonLines[7].points[1].x))


        returnVec.append(optimalPoints[12] - xOfLine5)
        returnVec.append(xOfLine7 - optimalPoints[14])
        # returnVec.append(yOfLine6 - optimalPoints[9])
        returnVec.append(optimalPoints[17] - yOfLine8)

        return returnVec

    def StartPointsDoNotGoBeyondLineConstraintPanel2(optimalPoints):
        returnVec = []

        returnVec.append(polygonLines[4].points[1].y - optimalPoints[13])
        returnVec.append(optimalPoints[13] - polygonLines[4].points[0].y)
        returnVec.append(polygonLines[6].points[0].y - optimalPoints[15])
        returnVec.append(optimalPoints[15] - polygonLines[6].points[1].y)
        returnVec.append(polygonLines[5].points[1].x - optimalPoints[8])
        returnVec.append(optimalPoints[8] - polygonLines[5].points[0].x)
        returnVec.append(polygonLines[7].points[1].x - optimalPoints[16])
        returnVec.append(optimalPoints[16] - polygonLines[7].points[0].x)

        return returnVec

    def KeepGuessPointsMinDistanceApartConstraintPanel3(optimalPoints):
        distanceBetweenPoints = []

        currentPoint13 = PointsAndLinesClass.ClassPoint(optimalPoints[16], optimalPoints[17])
        currentPoint14 = PointsAndLinesClass.ClassPoint(optimalPoints[18], optimalPoints[19])
        currentPoint15 = PointsAndLinesClass.ClassPoint(optimalPoints[20], optimalPoints[21])
        currentPoint16 = PointsAndLinesClass.ClassPoint(optimalPoints[22], optimalPoints[23])
        currentPoint17 = PointsAndLinesClass.ClassPoint(optimalPoints[24], optimalPoints[25])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint14, currentPoint15) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint14, currentPoint16) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint14, currentPoint17) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint14, currentPoint13) - minDistances[0])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint17, currentPoint15) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint17, currentPoint16) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint13, currentPoint15) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint13, currentPoint16) - minDistances[0])

        return distanceBetweenPoints

    def PathStartPointsFallOnLinesPanel3(optimalPoints):
        returnVec = []
        # print("polygonLines:", len(polygonLines))

        xOfLine9 = polygonLines[8].points[1].x + (
                ((polygonLines[8].points[1].x - polygonLines[8].points[0].x) / (
                        polygonLines[8].points[1].y - polygonLines[8].points[0].y))
                * (optimalPoints[21] - polygonLines[8].points[1].y))

        xOfLine11 = polygonLines[10].points[0].x + (
                ((polygonLines[10].points[1].x - polygonLines[10].points[0].x) / (
                            polygonLines[10].points[1].y - polygonLines[10].points[0].y))
                * (optimalPoints[23] - polygonLines[10].points[0].y))

        yOfLine12 = polygonLines[11].points[1].y + (
                ((polygonLines[11].points[0].y - polygonLines[11].points[1].y) / (
                        polygonLines[11].points[0].x - polygonLines[11].points[1].x))
                * (optimalPoints[24] - polygonLines[11].points[1].x))

        returnVec.append(optimalPoints[20] - xOfLine9)
        returnVec.append(xOfLine11 - optimalPoints[22])
        returnVec.append(optimalPoints[25] - yOfLine12)

        return returnVec

    def StartPointsDoNotGoBeyondLineConstraintPanel3(optimalPoints):
        returnVec = []

        returnVec.append(polygonLines[8].points[1].y - optimalPoints[21])
        returnVec.append(optimalPoints[21] - polygonLines[8].points[0].y)

        returnVec.append(polygonLines[10].points[0].y - optimalPoints[23])
        returnVec.append(optimalPoints[23] - polygonLines[10].points[1].y)

        returnVec.append(polygonLines[11].points[1].x - optimalPoints[24])
        returnVec.append(optimalPoints[24] - polygonLines[11].points[0].x)

        return returnVec


    def KeepMiddleNodeMinDistanceFromCornersConstraintPanel3(optimalPoints):
        distanceBetweenPoints = []
        currentPoint14 = PointsAndLinesClass.ClassPoint(optimalPoints[18], optimalPoints[19])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[6], currentPoint14) - minDistances[1])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[4], currentPoint14) - minDistances[1])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[5], currentPoint14) - minDistances[1])
        distanceBetweenPoints.append(
                CrossFrameOptimizationLibrary.distance(listOfPoints[7], currentPoint14) - minDistances[1])

        return distanceBetweenPoints

    con1 = {'type': 'ineq', 'fun': KeepGuessPointsMinDistanceApartConstraintPanel1}
    con2 = {'type': 'eq', 'fun': PathStartPointsFallOnLinesPanel1}
    con3 = {'type': 'ineq', 'fun': StartPointsDoNotGoBeyondLineConstraintPanel1}

    con4 = {'type': 'ineq', 'fun': KeepGuessPointsMinDistanceApartConstraintPanel2}
    con5 = {'type': 'eq', 'fun': PathStartPointsFallOnLinesPanel2}
    con6 = {'type': 'ineq', 'fun': StartPointsDoNotGoBeyondLineConstraintPanel2}

    con7 = {'type': 'ineq', 'fun': KeepGuessPointsMinDistanceApartConstraintPanel3}
    con8 = {'type': 'eq', 'fun': PathStartPointsFallOnLinesPanel3}
    con9 = {'type': 'ineq', 'fun': StartPointsDoNotGoBeyondLineConstraintPanel3}
    con10 = {'type': 'ineq', 'fun': KeepMiddleNodeMinDistanceFromCornersConstraintPanel3}


    if isSinglePanel:
        cons = [con1, con2, con3]
    else:
        cons = [con1, con2, con3, con4, con5, con6, con7, con8, con9, con10]

    return cons


def GetConstraintsNoMiddle(polygonLines, listOfPoints, minDistances, isSinglePanel):
    def KeepGuessPointsMinDistanceApartConstraintPanel1(optimalPoints):
        distanceBetweenPoints = []

        if len(optimalPoints) > 7:
            currentPoint6 = PointsAndLinesClass.ClassPoint(optimalPoints[0], optimalPoints[1])
            currentPoint7 = PointsAndLinesClass.ClassPoint(optimalPoints[2], optimalPoints[3])
            currentPoint8 = PointsAndLinesClass.ClassPoint(optimalPoints[4], optimalPoints[5])
            currentPoint9 = PointsAndLinesClass.ClassPoint(optimalPoints[6], optimalPoints[7])
            distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint6, currentPoint7) - minDistances[0])
            distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint7, currentPoint8) - minDistances[0])
            distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint8, currentPoint9) - minDistances[0])
            distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint9, currentPoint6) - minDistances[0])
        else:
            currentPoint6 = PointsAndLinesClass.ClassPoint(optimalPoints[0], optimalPoints[1])
            currentPoint7 = PointsAndLinesClass.ClassPoint(optimalPoints[2], optimalPoints[3])
            currentPoint8 = PointsAndLinesClass.ClassPoint(optimalPoints[4], optimalPoints[5])
            distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint6, currentPoint7) - minDistances[0])
            distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint7, currentPoint8) - minDistances[0])
            distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint8, currentPoint6) - minDistances[0])

        return distanceBetweenPoints

    def PathStartPointsFallOnLinesPanel1(optimalPoints):
        returnVec = []

        yOfLine2 = polygonLines[1].points[1].y + (
                    ((polygonLines[1].points[1].y - polygonLines[1].points[0].y) / (polygonLines[1].points[1].x - polygonLines[1].points[0].x))
                    * (optimalPoints[2] - polygonLines[1].points[1].x))

        xOfLine1 = polygonLines[0].points[1].x + (
                    ((polygonLines[0].points[1].x - polygonLines[0].points[0].x) / (polygonLines[0].points[1].y - polygonLines[0].points[0].y))
                    * (optimalPoints[1] - polygonLines[0].points[1].y))

        xOfLine3 = polygonLines[2].points[0].x + (
                    ((polygonLines[2].points[1].x - polygonLines[2].points[0].x) / (polygonLines[2].points[1].y - polygonLines[2].points[0].y))
                    * (optimalPoints[5] - polygonLines[2].points[0].y))

        returnVec.append(optimalPoints[0] - xOfLine1)
        returnVec.append(xOfLine3 - optimalPoints[4])
        returnVec.append(yOfLine2 - optimalPoints[3])

        if len(optimalPoints) > 7:
            yOfLine4 = polygonLines[3].points[1].y + (
                        ((polygonLines[3].points[0].y - polygonLines[3].points[1].y) / (polygonLines[3].points[0].x - polygonLines[3].points[1].x))
                        * (optimalPoints[6] - polygonLines[3].points[1].x))

            returnVec.append(optimalPoints[7] - yOfLine4)

        return returnVec

    def StartPointsDoNotGoBeyondLineConstraintPanel1(optimalPoints):
        returnVec = []

        returnVec.append(polygonLines[0].points[1].y - optimalPoints[1])
        returnVec.append(optimalPoints[1] - polygonLines[0].points[0].y)
        returnVec.append(polygonLines[2].points[0].y - optimalPoints[5])
        returnVec.append(optimalPoints[5] - polygonLines[2].points[1].y)
        returnVec.append(polygonLines[1].points[1].x - optimalPoints[2])
        returnVec.append(optimalPoints[2] - polygonLines[1].points[0].x)
        if len(optimalPoints) > 7:
            returnVec.append(polygonLines[3].points[1].x - optimalPoints[6])
            returnVec.append(optimalPoints[6] - polygonLines[3].points[0].x)

        return returnVec



    def KeepGuessPointsMinDistanceApartConstraintPanel2(optimalPoints):
        distanceBetweenPoints = []

        currentPoint11 = PointsAndLinesClass.ClassPoint(optimalPoints[8], optimalPoints[9])
        currentPoint12 = PointsAndLinesClass.ClassPoint(optimalPoints[10], optimalPoints[11])
        currentPoint13 = PointsAndLinesClass.ClassPoint(optimalPoints[12], optimalPoints[13])
        currentPoint9 = PointsAndLinesClass.ClassPoint(optimalPoints[6], optimalPoints[7])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint11, currentPoint9) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint9, currentPoint12) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint12, currentPoint13) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint13, currentPoint11) - minDistances[0])

        return distanceBetweenPoints

    def PathStartPointsFallOnLinesPanel2(optimalPoints):
        returnVec = []

        xOfLine5 = polygonLines[4].points[1].x + (
                ((polygonLines[4].points[1].x - polygonLines[4].points[0].x) / (
                            polygonLines[4].points[1].y - polygonLines[4].points[0].y))
                * (optimalPoints[9] - polygonLines[4].points[1].y))

        xOfLine7 = polygonLines[6].points[0].x + (
                    ((polygonLines[6].points[1].x - polygonLines[6].points[0].x) / (polygonLines[6].points[1].y - polygonLines[6].points[0].y))
                    * (optimalPoints[11] - polygonLines[6].points[0].y))

        yOfLine8 = polygonLines[7].points[1].y + (
                ((polygonLines[7].points[0].y - polygonLines[7].points[1].y) / (
                            polygonLines[7].points[0].x - polygonLines[7].points[1].x))
                * (optimalPoints[12] - polygonLines[7].points[1].x))

        returnVec.append(optimalPoints[8] - xOfLine5)
        returnVec.append(xOfLine7 - optimalPoints[10])
        returnVec.append(optimalPoints[13] - yOfLine8)

        return returnVec

    def StartPointsDoNotGoBeyondLineConstraintPanel2(optimalPoints):
        returnVec = []

        returnVec.append(polygonLines[4].points[1].y - optimalPoints[9])
        returnVec.append(optimalPoints[9] - polygonLines[4].points[0].y)
        returnVec.append(polygonLines[6].points[0].y - optimalPoints[11])
        returnVec.append(optimalPoints[11] - polygonLines[6].points[1].y)
        returnVec.append(polygonLines[7].points[1].x - optimalPoints[12])
        returnVec.append(optimalPoints[12] - polygonLines[7].points[0].x)

        return returnVec



    def KeepGuessPointsMinDistanceApartConstraintPanel3(optimalPoints):
        distanceBetweenPoints = []

        currentPoint13 = PointsAndLinesClass.ClassPoint(optimalPoints[12], optimalPoints[13])
        currentPoint15 = PointsAndLinesClass.ClassPoint(optimalPoints[14], optimalPoints[15])
        currentPoint16 = PointsAndLinesClass.ClassPoint(optimalPoints[16], optimalPoints[17])
        currentPoint17 = PointsAndLinesClass.ClassPoint(optimalPoints[18], optimalPoints[19])

        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint13, currentPoint15) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint13, currentPoint16) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint17, currentPoint15) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint17, currentPoint16) - minDistances[0])

        return distanceBetweenPoints

    def PathStartPointsFallOnLinesPanel3(optimalPoints):
        returnVec = []

        xOfLine9 = polygonLines[8].points[1].x + (
                ((polygonLines[8].points[1].x - polygonLines[8].points[0].x) / (
                            polygonLines[8].points[1].y - polygonLines[8].points[0].y))
                * (optimalPoints[15] - polygonLines[8].points[1].y))

        xOfLine11 = polygonLines[10].points[0].x + (
                    ((polygonLines[10].points[1].x - polygonLines[10].points[0].x) / (polygonLines[10].points[1].y - polygonLines[10].points[0].y))
                    * (optimalPoints[17] - polygonLines[10].points[0].y))

        yOfLine12 = polygonLines[11].points[1].y + (
                ((polygonLines[11].points[0].y - polygonLines[11].points[1].y) / (
                            polygonLines[11].points[0].x - polygonLines[11].points[1].x))
                * (optimalPoints[18] - polygonLines[11].points[1].x))

        returnVec.append(optimalPoints[14] - xOfLine9)
        returnVec.append(xOfLine11 - optimalPoints[16])
        returnVec.append(optimalPoints[19] - yOfLine12)

        return returnVec

    def StartPointsDoNotGoBeyondLineConstraintPanel3(optimalPoints):
        returnVec = []

        returnVec.append(polygonLines[8].points[1].y - optimalPoints[15])
        returnVec.append(optimalPoints[15] - polygonLines[8].points[0].y)

        returnVec.append(polygonLines[10].points[0].y - optimalPoints[17])
        returnVec.append(optimalPoints[17] - polygonLines[10].points[1].y)

        returnVec.append(polygonLines[11].points[1].x - optimalPoints[18])
        returnVec.append(optimalPoints[18] - polygonLines[11].points[0].x)

        return returnVec


    con1 = {'type': 'ineq', 'fun': KeepGuessPointsMinDistanceApartConstraintPanel1}
    con2 = {'type': 'eq', 'fun': PathStartPointsFallOnLinesPanel1}
    con3 = {'type': 'ineq', 'fun': StartPointsDoNotGoBeyondLineConstraintPanel1}

    con4 = {'type': 'ineq', 'fun': KeepGuessPointsMinDistanceApartConstraintPanel2}
    con5 = {'type': 'eq', 'fun': PathStartPointsFallOnLinesPanel2}
    con6 = {'type': 'ineq', 'fun': StartPointsDoNotGoBeyondLineConstraintPanel2}

    con7 = {'type': 'ineq', 'fun': KeepGuessPointsMinDistanceApartConstraintPanel3}
    con8 = {'type': 'eq', 'fun': PathStartPointsFallOnLinesPanel3}
    con9 = {'type': 'ineq', 'fun': StartPointsDoNotGoBeyondLineConstraintPanel3}

    if isSinglePanel:
        cons = [con1, con2, con3]
    else:
        cons = [con1, con2, con3, con4, con5, con6, con7, con8, con9]

    return cons



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


    def KeepGuessPointsMinDistanceApartConstraintPanel4(optimalPoints):
        distanceBetweenPoints = []

        currentPoint17 = PointsAndLinesClass.ClassPoint(optimalPoints[24], optimalPoints[25])
        currentPoint18 = PointsAndLinesClass.ClassPoint(optimalPoints[26], optimalPoints[27])
        currentPoint19 = PointsAndLinesClass.ClassPoint(optimalPoints[28], optimalPoints[29])
        currentPoint20 = PointsAndLinesClass.ClassPoint(optimalPoints[30], optimalPoints[31])
        currentPoint21 = PointsAndLinesClass.ClassPoint(optimalPoints[32], optimalPoints[33])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint18, currentPoint17) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint18, currentPoint19) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint18, currentPoint20) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint18, currentPoint21) - minDistances[0])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint17, currentPoint19) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint17, currentPoint20) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint21, currentPoint19) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint21, currentPoint20) - minDistances[0])

        return distanceBetweenPoints

    def PathStartPointsFallOnLinesPanel4(optimalPoints):
        returnVec = []

        xOfLine13 = polygonLines[12].points[1].x + (
                ((polygonLines[12].points[1].x - polygonLines[12].points[0].x) / (
                        polygonLines[12].points[1].y - polygonLines[12].points[0].y))
                * (optimalPoints[29] - polygonLines[12].points[1].y))

        xOfLine15 = polygonLines[14].points[0].x + (
                ((polygonLines[14].points[1].x - polygonLines[14].points[0].x) / (
                            polygonLines[14].points[1].y - polygonLines[14].points[0].y))
                * (optimalPoints[31] - polygonLines[14].points[0].y))

        yOfLine16 = polygonLines[15].points[1].y + (
                ((polygonLines[15].points[0].y - polygonLines[15].points[1].y) / (
                        polygonLines[15].points[0].x - polygonLines[15].points[1].x))
                * (optimalPoints[32] - polygonLines[15].points[1].x))

        returnVec.append(optimalPoints[28] - xOfLine13)
        returnVec.append(xOfLine15 - optimalPoints[30])
        returnVec.append(optimalPoints[33] - yOfLine16)

        return returnVec

    def StartPointsDoNotGoBeyondLineConstraintPanel4(optimalPoints):
        returnVec = []

        returnVec.append(polygonLines[12].points[1].y - optimalPoints[29])
        returnVec.append(optimalPoints[29] - polygonLines[12].points[0].y)

        returnVec.append(polygonLines[14].points[0].y - optimalPoints[31])
        returnVec.append(optimalPoints[31] - polygonLines[14].points[1].y)

        returnVec.append(polygonLines[15].points[1].x - optimalPoints[32])
        returnVec.append(optimalPoints[32] - polygonLines[15].points[0].x)

        return returnVec

    def KeepMiddleNodeMinDistanceFromCornersConstraintPanel4(optimalPoints):
        distanceBetweenPoints = []
        currentPoint18 = PointsAndLinesClass.ClassPoint(optimalPoints[26], optimalPoints[27])
        currentPoint20 = PointsAndLinesClass.ClassPoint(optimalPoints[30], optimalPoints[31])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[6], currentPoint18) - minDistances[1])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[8], currentPoint18) - minDistances[1])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[9], currentPoint18) - minDistances[1])
        distanceBetweenPoints.append(
                CrossFrameOptimizationLibrary.distance(listOfPoints[7], currentPoint18) - minDistances[1])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[9], currentPoint20) - minDistances[1])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[7], currentPoint20) - minDistances[1])

        return distanceBetweenPoints


    def KeepGuessPointsMinDistanceApartConstraintPanel5(optimalPoints):
        distanceBetweenPoints = []

        currentPoint22 = PointsAndLinesClass.ClassPoint(optimalPoints[34], optimalPoints[35])
        currentPoint8 = PointsAndLinesClass.ClassPoint(optimalPoints[6], optimalPoints[7])
        currentPoint23 = PointsAndLinesClass.ClassPoint(optimalPoints[36], optimalPoints[37])
        currentPoint24 = PointsAndLinesClass.ClassPoint(optimalPoints[38], optimalPoints[39])
        currentPoint25 = PointsAndLinesClass.ClassPoint(optimalPoints[40], optimalPoints[41])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint22, currentPoint8) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint22, currentPoint23) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint22, currentPoint24) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint22, currentPoint25) - minDistances[0])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint23, currentPoint8) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint23, currentPoint24) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint25, currentPoint8) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint25, currentPoint24) - minDistances[0])

        return distanceBetweenPoints

    def PathStartPointsFallOnLinesPanel5(optimalPoints):
        returnVec = []

        yOfLine18 = polygonLines[17].points[1].y + (
                ((polygonLines[17].points[0].y - polygonLines[17].points[1].y) / (
                        polygonLines[17].points[0].x - polygonLines[17].points[1].x))
                * (optimalPoints[36] - polygonLines[17].points[1].x))

        xOfLine19 = polygonLines[18].points[0].x + (
                ((polygonLines[18].points[1].x - polygonLines[18].points[0].x) / (
                            polygonLines[18].points[1].y - polygonLines[18].points[0].y))
                * (optimalPoints[39] - polygonLines[18].points[0].y))

        yOfLine20 = polygonLines[19].points[1].y + (
                ((polygonLines[19].points[0].y - polygonLines[19].points[1].y) / (
                        polygonLines[19].points[0].x - polygonLines[19].points[1].x))
                * (optimalPoints[40] - polygonLines[19].points[1].x))

        returnVec.append(yOfLine18 - optimalPoints[37])
        returnVec.append(xOfLine19 - optimalPoints[38])
        returnVec.append(optimalPoints[41] - yOfLine20)

        return returnVec

    def StartPointsDoNotGoBeyondLineConstraintPanel5(optimalPoints):
        returnVec = []

        returnVec.append(polygonLines[17].points[1].x - optimalPoints[36])
        returnVec.append(optimalPoints[36] - polygonLines[17].points[0].x)

        returnVec.append(polygonLines[18].points[0].y - optimalPoints[39])
        returnVec.append(optimalPoints[39] - polygonLines[18].points[1].y)

        returnVec.append(polygonLines[19].points[1].x - optimalPoints[40])
        returnVec.append(optimalPoints[40] - polygonLines[19].points[0].x)

        return returnVec

    def KeepMiddleNodeMinDistanceFromCornersConstraintPanel5(optimalPoints):
        distanceBetweenPoints = []
        currentPoint22 = PointsAndLinesClass.ClassPoint(optimalPoints[34], optimalPoints[35])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[2], currentPoint22) - minDistances[1])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[3], currentPoint22) - minDistances[1])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[12], currentPoint22) - minDistances[1])
        distanceBetweenPoints.append(
                CrossFrameOptimizationLibrary.distance(listOfPoints[13], currentPoint22) - minDistances[1])

        return distanceBetweenPoints


    def KeepGuessPointsMinDistanceApartConstraintPanel6(optimalPoints):
        distanceBetweenPoints = []

        currentPoint26 = PointsAndLinesClass.ClassPoint(optimalPoints[42], optimalPoints[43])
        currentPoint12 = PointsAndLinesClass.ClassPoint(optimalPoints[14], optimalPoints[15])
        currentPoint25 = PointsAndLinesClass.ClassPoint(optimalPoints[40], optimalPoints[41])
        currentPoint27 = PointsAndLinesClass.ClassPoint(optimalPoints[44], optimalPoints[45])
        currentPoint28 = PointsAndLinesClass.ClassPoint(optimalPoints[46], optimalPoints[47])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint26, currentPoint12) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint26, currentPoint25) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint26, currentPoint27) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint26, currentPoint28) - minDistances[0])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint25, currentPoint12) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint25, currentPoint27) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint28, currentPoint12) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint28, currentPoint27) - minDistances[0])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[14], currentPoint27) - minDistances[1])

        return distanceBetweenPoints

    def PathStartPointsFallOnLinesPanel6(optimalPoints):
        returnVec = []

        xOfLine21 = polygonLines[20].points[0].x + (
                ((polygonLines[20].points[1].x - polygonLines[20].points[0].x) / (
                            polygonLines[20].points[1].y - polygonLines[20].points[0].y))
                * (optimalPoints[45] - polygonLines[20].points[0].y))

        yOfLine22 = polygonLines[21].points[1].y + (
                ((polygonLines[21].points[0].y - polygonLines[21].points[1].y) / (
                        polygonLines[21].points[0].x - polygonLines[21].points[1].x))
                * (optimalPoints[46] - polygonLines[21].points[1].x))

        returnVec.append(xOfLine21 - optimalPoints[44])
        returnVec.append(optimalPoints[47] - yOfLine22)

        return returnVec

    def StartPointsDoNotGoBeyondLineConstraintPanel6(optimalPoints):
        returnVec = []

        returnVec.append(polygonLines[20].points[0].y - optimalPoints[45])
        returnVec.append(optimalPoints[45] - polygonLines[20].points[1].y)

        returnVec.append(polygonLines[21].points[1].x - optimalPoints[46])
        returnVec.append(optimalPoints[46] - polygonLines[21].points[0].x)

        return returnVec

    def KeepMiddleNodeMinDistanceFromCornersConstraintPanel6(optimalPoints):
        distanceBetweenPoints = []
        currentPoint26 = PointsAndLinesClass.ClassPoint(optimalPoints[42], optimalPoints[43])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[3], currentPoint26) - minDistances[1])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[5], currentPoint26) - minDistances[1])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[13], currentPoint26) - minDistances[1])
        distanceBetweenPoints.append(
                CrossFrameOptimizationLibrary.distance(listOfPoints[14], currentPoint26) - minDistances[1])

        return distanceBetweenPoints


    def KeepGuessPointsMinDistanceApartConstraintPanel7(optimalPoints):
        distanceBetweenPoints = []

        currentPoint29 = PointsAndLinesClass.ClassPoint(optimalPoints[48], optimalPoints[49])
        currentPoint30 = PointsAndLinesClass.ClassPoint(optimalPoints[50], optimalPoints[51])
        currentPoint27 = PointsAndLinesClass.ClassPoint(optimalPoints[44], optimalPoints[45])
        currentPoint31 = PointsAndLinesClass.ClassPoint(optimalPoints[52], optimalPoints[53])
        currentPoint32 = PointsAndLinesClass.ClassPoint(optimalPoints[54], optimalPoints[55])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint29, currentPoint30) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint29, currentPoint27) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint29, currentPoint31) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint29, currentPoint32) - minDistances[0])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint27, currentPoint30) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint27, currentPoint31) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint32, currentPoint30) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint32, currentPoint31) - minDistances[0])

        return distanceBetweenPoints

    def PathStartPointsFallOnLinesPanel7(optimalPoints):
        returnVec = []

        xOfLine23 = polygonLines[22].points[0].x + (
                ((polygonLines[22].points[1].x - polygonLines[22].points[0].x) / (
                            polygonLines[22].points[1].y - polygonLines[22].points[0].y))
                * (optimalPoints[51] - polygonLines[22].points[0].y))

        xOfLine24 = polygonLines[23].points[0].x + (
                ((polygonLines[23].points[1].x - polygonLines[23].points[0].x) / (
                        polygonLines[23].points[1].y - polygonLines[23].points[0].y))
                * (optimalPoints[53] - polygonLines[23].points[0].y))

        yOfLine25 = polygonLines[24].points[1].y + (
                ((polygonLines[24].points[0].y - polygonLines[24].points[1].y) / (
                        polygonLines[24].points[0].x - polygonLines[24].points[1].x))
                * (optimalPoints[54] - polygonLines[24].points[1].x))


        returnVec.append(optimalPoints[50] - xOfLine23)
        returnVec.append(xOfLine24 - optimalPoints[52])
        returnVec.append(optimalPoints[55] - yOfLine25)

        return returnVec

    def StartPointsDoNotGoBeyondLineConstraintPanel7(optimalPoints):
        returnVec = []

        returnVec.append(polygonLines[22].points[1].y - optimalPoints[51])
        returnVec.append(optimalPoints[51] - polygonLines[22].points[0].y)

        returnVec.append(polygonLines[23].points[0].y - optimalPoints[53])
        returnVec.append(optimalPoints[53] - polygonLines[23].points[1].y)

        returnVec.append(polygonLines[24].points[1].x - optimalPoints[54])
        returnVec.append(optimalPoints[54] - polygonLines[24].points[0].x)

        return returnVec

    def KeepMiddleNodeMinDistanceFromCornersConstraintPanel7(optimalPoints):
        distanceBetweenPoints = []
        currentPoint29 = PointsAndLinesClass.ClassPoint(optimalPoints[48], optimalPoints[49])
        currentPoint32 = PointsAndLinesClass.ClassPoint(optimalPoints[54], optimalPoints[55])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[14], currentPoint29) - minDistances[1])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[13], currentPoint29) - minDistances[1])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[16], currentPoint29) - minDistances[1])
        distanceBetweenPoints.append(
                CrossFrameOptimizationLibrary.distance(listOfPoints[15], currentPoint29) - minDistances[1])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[16], currentPoint32) - minDistances[1])

        return distanceBetweenPoints


    def KeepGuessPointsMinDistanceApartConstraintPanel8(optimalPoints):
        distanceBetweenPoints = []

        currentPoint33 = PointsAndLinesClass.ClassPoint(optimalPoints[56], optimalPoints[57])
        currentPoint34 = PointsAndLinesClass.ClassPoint(optimalPoints[58], optimalPoints[59])
        currentPoint32 = PointsAndLinesClass.ClassPoint(optimalPoints[54], optimalPoints[55])
        currentPoint35 = PointsAndLinesClass.ClassPoint(optimalPoints[60], optimalPoints[61])
        currentPoint36 = PointsAndLinesClass.ClassPoint(optimalPoints[62], optimalPoints[63])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint33, currentPoint34) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint33, currentPoint32) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint33, currentPoint35) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint33, currentPoint36) - minDistances[0])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint32, currentPoint34) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint32, currentPoint35) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint36, currentPoint34) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint36, currentPoint35) - minDistances[0])

        return distanceBetweenPoints

    def PathStartPointsFallOnLinesPanel8(optimalPoints):
        returnVec = []

        xOfLine26 = polygonLines[25].points[0].x + (
                ((polygonLines[25].points[1].x - polygonLines[25].points[0].x) / (
                            polygonLines[25].points[1].y - polygonLines[25].points[0].y))
                * (optimalPoints[59] - polygonLines[25].points[0].y))

        xOfLine27 = polygonLines[26].points[0].x + (
                ((polygonLines[26].points[1].x - polygonLines[26].points[0].x) / (
                        polygonLines[26].points[1].y - polygonLines[26].points[0].y))
                * (optimalPoints[61] - polygonLines[26].points[0].y))

        yOfLine28 = polygonLines[27].points[1].y + (
                ((polygonLines[27].points[0].y - polygonLines[27].points[1].y) / (
                        polygonLines[27].points[0].x - polygonLines[27].points[1].x))
                * (optimalPoints[62] - polygonLines[27].points[1].x))


        returnVec.append(optimalPoints[58] - xOfLine26)
        returnVec.append(xOfLine27 - optimalPoints[60])
        returnVec.append(optimalPoints[63] - yOfLine28)

        return returnVec

    def StartPointsDoNotGoBeyondLineConstraintPanel8(optimalPoints):
        returnVec = []

        returnVec.append(polygonLines[25].points[1].y - optimalPoints[59])
        returnVec.append(optimalPoints[59] - polygonLines[25].points[0].y)

        returnVec.append(polygonLines[26].points[0].y - optimalPoints[61])
        returnVec.append(optimalPoints[61] - polygonLines[26].points[1].y)

        returnVec.append(polygonLines[27].points[1].x - optimalPoints[62])
        returnVec.append(optimalPoints[62] - polygonLines[27].points[0].x)

        return returnVec

    def KeepMiddleNodeMinDistanceFromCornersConstraintPanel8(optimalPoints):
        distanceBetweenPoints = []
        currentPoint33 = PointsAndLinesClass.ClassPoint(optimalPoints[56], optimalPoints[57])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[16], currentPoint33) - minDistances[1])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[18], currentPoint33) - minDistances[1])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[15], currentPoint33) - minDistances[1])
        distanceBetweenPoints.append(
                CrossFrameOptimizationLibrary.distance(listOfPoints[17], currentPoint33) - minDistances[1])

        return distanceBetweenPoints


    def KeepGuessPointsMinDistanceApartConstraintPanel9(optimalPoints):
        distanceBetweenPoints = []

        currentPoint37 = PointsAndLinesClass.ClassPoint(optimalPoints[64], optimalPoints[65])
        currentPoint38 = PointsAndLinesClass.ClassPoint(optimalPoints[66], optimalPoints[67])
        currentPoint39 = PointsAndLinesClass.ClassPoint(optimalPoints[68], optimalPoints[69])
        currentPoint34 = PointsAndLinesClass.ClassPoint(optimalPoints[58], optimalPoints[59])
        currentPoint40 = PointsAndLinesClass.ClassPoint(optimalPoints[70], optimalPoints[71])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint37, currentPoint34) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint37, currentPoint39) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint37, currentPoint38) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint37, currentPoint40) - minDistances[0])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint39, currentPoint34) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint39, currentPoint38) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint40, currentPoint34) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint40, currentPoint38) - minDistances[0])

        return distanceBetweenPoints

    def PathStartPointsFallOnLinesPanel9(optimalPoints):
        returnVec = []

        xOfLine29 = polygonLines[28].points[0].x + (
                ((polygonLines[28].points[1].x - polygonLines[28].points[0].x) / (
                            polygonLines[28].points[1].y - polygonLines[28].points[0].y))
                * (optimalPoints[67] - polygonLines[28].points[0].y))

        yOfLine30 = polygonLines[29].points[1].y + (
                ((polygonLines[29].points[0].y - polygonLines[29].points[1].y) / (
                        polygonLines[29].points[0].x - polygonLines[29].points[1].x))
                * (optimalPoints[68] - polygonLines[29].points[1].x))

        yOfLine31 = polygonLines[30].points[1].y + (
                ((polygonLines[30].points[0].y - polygonLines[30].points[1].y) / (
                        polygonLines[30].points[0].x - polygonLines[30].points[1].x))
                * (optimalPoints[70] - polygonLines[30].points[1].x))


        returnVec.append(optimalPoints[66] - xOfLine29)
        returnVec.append(yOfLine30 - optimalPoints[69])
        returnVec.append(optimalPoints[71] - yOfLine31)

        return returnVec

    def StartPointsDoNotGoBeyondLineConstraintPanel9(optimalPoints):
        returnVec = []

        returnVec.append(polygonLines[28].points[1].y - optimalPoints[67])
        returnVec.append(optimalPoints[67] - polygonLines[28].points[0].y)

        returnVec.append(polygonLines[29].points[1].x - optimalPoints[68])
        returnVec.append(optimalPoints[68] - polygonLines[29].points[0].x)

        returnVec.append(polygonLines[30].points[1].x - optimalPoints[70])
        returnVec.append(optimalPoints[70] - polygonLines[30].points[0].x)

        return returnVec

    def KeepMiddleNodeMinDistanceFromCornersConstraintPanel9(optimalPoints):
        distanceBetweenPoints = []
        currentPoint37 = PointsAndLinesClass.ClassPoint(optimalPoints[64], optimalPoints[65])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[7], currentPoint37) - minDistances[1])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[16], currentPoint37) - minDistances[1])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[19], currentPoint37) - minDistances[1])
        distanceBetweenPoints.append(
                CrossFrameOptimizationLibrary.distance(listOfPoints[18], currentPoint37) - minDistances[1])

        return distanceBetweenPoints


    def KeepGuessPointsMinDistanceApartConstraintPanel10(optimalPoints):
        distanceBetweenPoints = []

        currentPoint41 = PointsAndLinesClass.ClassPoint(optimalPoints[72], optimalPoints[73])
        currentPoint42 = PointsAndLinesClass.ClassPoint(optimalPoints[74], optimalPoints[75])
        currentPoint43 = PointsAndLinesClass.ClassPoint(optimalPoints[76], optimalPoints[77])
        currentPoint20 = PointsAndLinesClass.ClassPoint(optimalPoints[30], optimalPoints[31])
        currentPoint38 = PointsAndLinesClass.ClassPoint(optimalPoints[66], optimalPoints[67])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint41, currentPoint42) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint41, currentPoint43) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint41, currentPoint20) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint41, currentPoint38) - minDistances[0])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint43, currentPoint42) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint43, currentPoint38) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint20, currentPoint42) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint20, currentPoint38) - minDistances[0])

        return distanceBetweenPoints

    def PathStartPointsFallOnLinesPanel10(optimalPoints):
        returnVec = []

        xOfLine32 = polygonLines[31].points[0].x + (
                ((polygonLines[31].points[1].x - polygonLines[31].points[0].x) / (
                            polygonLines[31].points[1].y - polygonLines[31].points[0].y))
                * (optimalPoints[75] - polygonLines[31].points[0].y))

        yOfLine33 = polygonLines[32].points[1].y + (
                ((polygonLines[32].points[0].y - polygonLines[32].points[1].y) / (
                        polygonLines[32].points[0].x - polygonLines[32].points[1].x))
                * (optimalPoints[76] - polygonLines[32].points[1].x))


        returnVec.append(optimalPoints[74] - xOfLine32)
        returnVec.append(optimalPoints[77] - yOfLine33)

        return returnVec

    def StartPointsDoNotGoBeyondLineConstraintPanel10(optimalPoints):
        returnVec = []

        returnVec.append(polygonLines[31].points[1].y - optimalPoints[75])
        returnVec.append(optimalPoints[75] - polygonLines[31].points[0].y)

        returnVec.append(polygonLines[32].points[1].x - optimalPoints[76])
        returnVec.append(optimalPoints[76] - polygonLines[32].points[0].x)

        return returnVec

    def KeepMiddleNodeMinDistanceFromCornersConstraintPanel10(optimalPoints):
        distanceBetweenPoints = []
        currentPoint41 = PointsAndLinesClass.ClassPoint(optimalPoints[72], optimalPoints[73])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[11], currentPoint41) - minDistances[1])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[9], currentPoint41) - minDistances[1])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[7], currentPoint41) - minDistances[1])
        distanceBetweenPoints.append(
                CrossFrameOptimizationLibrary.distance(listOfPoints[19], currentPoint41) - minDistances[1])

        return distanceBetweenPoints

    def KeepGoresAligned(optimalPoints):
        distanceBetweenPoints = []
        currentPoint6 = PointsAndLinesClass.ClassPoint(optimalPoints[2], optimalPoints[3])
        currentPoint11 = PointsAndLinesClass.ClassPoint(optimalPoints[12], optimalPoints[13])
        currentPoint15 = PointsAndLinesClass.ClassPoint(optimalPoints[20], optimalPoints[21])
        currentPoint19 = PointsAndLinesClass.ClassPoint(optimalPoints[28], optimalPoints[29])

        currentPoint36 = PointsAndLinesClass.ClassPoint(optimalPoints[62], optimalPoints[63])
        currentPoint40 = PointsAndLinesClass.ClassPoint(optimalPoints[70], optimalPoints[71])
        currentPoint43 = PointsAndLinesClass.ClassPoint(optimalPoints[76], optimalPoints[77])
        currentPoint47 = PointsAndLinesClass.ClassPoint(optimalPoints[84], optimalPoints[85])

        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(listOfPoints[1], currentPoint6) -
                                     CrossFrameOptimizationLibrary.distance(listOfPoints[17], currentPoint36))

        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(listOfPoints[0], currentPoint11) -
                                     CrossFrameOptimizationLibrary.distance(listOfPoints[18], currentPoint40))

        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(listOfPoints[4], currentPoint15) -
                                     CrossFrameOptimizationLibrary.distance(listOfPoints[19], currentPoint43))


        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(listOfPoints[6], currentPoint19) -
                                     CrossFrameOptimizationLibrary.distance(listOfPoints[11], currentPoint47))

        return distanceBetweenPoints


    def KeepGuessPointsMinDistanceApartConstraintPanel11(optimalPoints):
        distanceBetweenPoints = []

        currentPoint45 = PointsAndLinesClass.ClassPoint(optimalPoints[80], optimalPoints[81])
        currentPoint16 = PointsAndLinesClass.ClassPoint(optimalPoints[22], optimalPoints[23])
        currentPoint28 = PointsAndLinesClass.ClassPoint(optimalPoints[46], optimalPoints[47])
        currentPoint44 = PointsAndLinesClass.ClassPoint(optimalPoints[78], optimalPoints[79])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint45, currentPoint16) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint45, currentPoint28) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint45, currentPoint44) - minDistances[0])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint16, currentPoint28) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint28, currentPoint44) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint44, currentPoint16) - minDistances[0])

        return distanceBetweenPoints

    def PathStartPointsFallOnLinesPanel11(optimalPoints):
        returnVec = []

        xOfLine34 = polygonLines[38].points[0].x + (
                ((polygonLines[38].points[1].x - polygonLines[38].points[0].x) / (
                            polygonLines[38].points[1].y - polygonLines[38].points[0].y))
                * (optimalPoints[79] - polygonLines[38].points[0].y))

        returnVec.append(optimalPoints[78] - xOfLine34)

        return returnVec

    def StartPointsDoNotGoBeyondLineConstraintPanel11(optimalPoints):
        returnVec = []

        returnVec.append(polygonLines[38].points[1].y - optimalPoints[79])
        returnVec.append(optimalPoints[79] - polygonLines[38].points[0].y)

        return returnVec

    def KeepMiddleNodeMinDistanceFromCornersConstraintPanel11(optimalPoints):
        distanceBetweenPoints = []
        currentPoint45 = PointsAndLinesClass.ClassPoint(optimalPoints[80], optimalPoints[81])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[7], currentPoint45) - minDistances[1])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[5], currentPoint45) - minDistances[1])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[14], currentPoint45) - minDistances[1])

        return distanceBetweenPoints


    def KeepGuessPointsMinDistanceApartConstraintPanel12(optimalPoints):
        distanceBetweenPoints = []

        currentPoint46 = PointsAndLinesClass.ClassPoint(optimalPoints[82], optimalPoints[83])
        currentPoint30 = PointsAndLinesClass.ClassPoint(optimalPoints[50], optimalPoints[51])
        currentPoint39 = PointsAndLinesClass.ClassPoint(optimalPoints[68], optimalPoints[69])
        currentPoint44 = PointsAndLinesClass.ClassPoint(optimalPoints[78], optimalPoints[79])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint46, currentPoint30) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint46, currentPoint39) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint46, currentPoint44) - minDistances[0])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint30, currentPoint39) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint39, currentPoint44) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint44, currentPoint30) - minDistances[0])

        return distanceBetweenPoints

    def KeepMiddleNodeMinDistanceFromCornersConstraintPanel12(optimalPoints):
        distanceBetweenPoints = []
        currentPoint46 = PointsAndLinesClass.ClassPoint(optimalPoints[82], optimalPoints[83])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[7], currentPoint46) - minDistances[1])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[16], currentPoint46) - minDistances[1])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[14], currentPoint46) - minDistances[1])

        return distanceBetweenPoints


    def KeepGuessPointsMinDistanceApartConstraintPanel13(optimalPoints):
        distanceBetweenPoints = []

        currentPoint49 = PointsAndLinesClass.ClassPoint(optimalPoints[88], optimalPoints[89])
        currentPoint47 = PointsAndLinesClass.ClassPoint(optimalPoints[84], optimalPoints[85])
        currentPoint48 = PointsAndLinesClass.ClassPoint(optimalPoints[86], optimalPoints[87])
        currentPoint42 = PointsAndLinesClass.ClassPoint(optimalPoints[74], optimalPoints[75])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint49, currentPoint47) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint49, currentPoint48) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint49, currentPoint42) - minDistances[0])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint47, currentPoint48) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint48, currentPoint42) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint42, currentPoint47) - minDistances[0])

        return distanceBetweenPoints

    def PathStartPointsFallOnLinesPanel13(optimalPoints):
        returnVec = []

        yOfLine36 = polygonLines[40].points[1].y + (
                ((polygonLines[40].points[0].y - polygonLines[40].points[1].y) / (
                        polygonLines[40].points[0].x - polygonLines[40].points[1].x))
                * (optimalPoints[84] - polygonLines[40].points[1].x))

        xOfLine37 = polygonLines[39].points[0].x + (
                ((polygonLines[39].points[1].x - polygonLines[39].points[0].x) / (
                            polygonLines[39].points[1].y - polygonLines[39].points[0].y))
                * (optimalPoints[87] - polygonLines[39].points[0].y))

        returnVec.append(optimalPoints[85] - yOfLine36)
        returnVec.append(optimalPoints[86] - xOfLine37)

        return returnVec

    def StartPointsDoNotGoBeyondLineConstraintPanel13(optimalPoints):
        returnVec = []

        returnVec.append(polygonLines[39].points[1].y - optimalPoints[85])
        returnVec.append(optimalPoints[85] - polygonLines[39].points[0].y)

        returnVec.append(polygonLines[40].points[1].x - optimalPoints[86])
        returnVec.append(optimalPoints[86] - polygonLines[40].points[0].x)

        return returnVec

    def KeepMiddleNodeMinDistanceFromCornersConstraintPanel13(optimalPoints):
        distanceBetweenPoints = []
        currentPoint49 = PointsAndLinesClass.ClassPoint(optimalPoints[88], optimalPoints[89])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[9], currentPoint49) - minDistances[1])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[10], currentPoint49) - minDistances[1])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[11], currentPoint49) - minDistances[1])

        return distanceBetweenPoints


    def KeepGuessPointsMinDistanceApartConstraintPanel14(optimalPoints):
        distanceBetweenPoints = []

        currentPoint21 = PointsAndLinesClass.ClassPoint(optimalPoints[32], optimalPoints[33])
        currentPoint48 = PointsAndLinesClass.ClassPoint(optimalPoints[86], optimalPoints[87])
        currentPoint50 = PointsAndLinesClass.ClassPoint(optimalPoints[90], optimalPoints[91])
        currentPoint51 = PointsAndLinesClass.ClassPoint(optimalPoints[92], optimalPoints[93])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint50, currentPoint21) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint50, currentPoint48) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint50, currentPoint51) - minDistances[0])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint21, currentPoint48) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint48, currentPoint51) - minDistances[0])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(currentPoint51, currentPoint21) - minDistances[0])

        return distanceBetweenPoints

    def PathStartPointsFallOnLinesPanel14(optimalPoints):
        returnVec = []

        xOfLine35 = polygonLines[41].points[0].x + (
                ((polygonLines[41].points[1].x - polygonLines[41].points[0].x) / (
                        polygonLines[41].points[1].y - polygonLines[41].points[0].y))
                * (optimalPoints[93] - polygonLines[41].points[0].y))

        returnVec.append(optimalPoints[92] - xOfLine35)

        return returnVec

    def StartPointsDoNotGoBeyondLineConstraintPanel14(optimalPoints):
        returnVec = []

        returnVec.append(polygonLines[41].points[1].y - optimalPoints[93])
        returnVec.append(optimalPoints[93] - polygonLines[41].points[0].y)

        return returnVec

    def KeepMiddleNodeMinDistanceFromCornersConstraintPanel14(optimalPoints):
        distanceBetweenPoints = []
        currentPoint50 = PointsAndLinesClass.ClassPoint(optimalPoints[92], optimalPoints[93])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[8], currentPoint50) - minDistances[1])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[10], currentPoint50) - minDistances[1])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[9], currentPoint50) - minDistances[1])

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

    con11 = {'type': 'ineq', 'fun': KeepGuessPointsMinDistanceApartConstraintPanel4}
    con12 = {'type': 'eq', 'fun': PathStartPointsFallOnLinesPanel4}
    con13 = {'type': 'ineq', 'fun': StartPointsDoNotGoBeyondLineConstraintPanel4}
    con14 = {'type': 'ineq', 'fun': KeepMiddleNodeMinDistanceFromCornersConstraintPanel4}

    con15 = {'type': 'ineq', 'fun': KeepGuessPointsMinDistanceApartConstraintPanel5}
    con16 = {'type': 'eq', 'fun': PathStartPointsFallOnLinesPanel5}
    con17 = {'type': 'ineq', 'fun': StartPointsDoNotGoBeyondLineConstraintPanel5}
    con18 = {'type': 'ineq', 'fun': KeepMiddleNodeMinDistanceFromCornersConstraintPanel5}

    con19 = {'type': 'ineq', 'fun': KeepGuessPointsMinDistanceApartConstraintPanel6}
    con20 = {'type': 'eq', 'fun': PathStartPointsFallOnLinesPanel6}
    con21 = {'type': 'ineq', 'fun': StartPointsDoNotGoBeyondLineConstraintPanel6}
    con22 = {'type': 'ineq', 'fun': KeepMiddleNodeMinDistanceFromCornersConstraintPanel6}

    con23 = {'type': 'ineq', 'fun': KeepGuessPointsMinDistanceApartConstraintPanel7}
    con24 = {'type': 'eq', 'fun': PathStartPointsFallOnLinesPanel7}
    con25 = {'type': 'ineq', 'fun': StartPointsDoNotGoBeyondLineConstraintPanel7}
    con26 = {'type': 'ineq', 'fun': KeepMiddleNodeMinDistanceFromCornersConstraintPanel7}

    con27 = {'type': 'ineq', 'fun': KeepGuessPointsMinDistanceApartConstraintPanel8}
    con28 = {'type': 'eq', 'fun': PathStartPointsFallOnLinesPanel8}
    con29 = {'type': 'ineq', 'fun': StartPointsDoNotGoBeyondLineConstraintPanel8}
    con30 = {'type': 'ineq', 'fun': KeepMiddleNodeMinDistanceFromCornersConstraintPanel8}

    con31 = {'type': 'ineq', 'fun': KeepGuessPointsMinDistanceApartConstraintPanel9}
    con32 = {'type': 'eq', 'fun': PathStartPointsFallOnLinesPanel9}
    con33 = {'type': 'ineq', 'fun': StartPointsDoNotGoBeyondLineConstraintPanel9}
    con34 = {'type': 'ineq', 'fun': KeepMiddleNodeMinDistanceFromCornersConstraintPanel9}

    con35 = {'type': 'ineq', 'fun': KeepGuessPointsMinDistanceApartConstraintPanel10}
    con36 = {'type': 'eq', 'fun': PathStartPointsFallOnLinesPanel10}
    con37 = {'type': 'ineq', 'fun': StartPointsDoNotGoBeyondLineConstraintPanel10}
    con38 = {'type': 'ineq', 'fun': KeepMiddleNodeMinDistanceFromCornersConstraintPanel10}

    con39 = {'type': 'eq', 'fun': KeepGoresAligned}

    con40 = {'type': 'ineq', 'fun': KeepGuessPointsMinDistanceApartConstraintPanel11}
    con41 = {'type': 'eq', 'fun': PathStartPointsFallOnLinesPanel11}
    con42 = {'type': 'ineq', 'fun': StartPointsDoNotGoBeyondLineConstraintPanel11}
    con43 = {'type': 'ineq', 'fun': KeepMiddleNodeMinDistanceFromCornersConstraintPanel11}

    con44 = {'type': 'ineq', 'fun': KeepGuessPointsMinDistanceApartConstraintPanel12}
    con45 = {'type': 'ineq', 'fun': KeepMiddleNodeMinDistanceFromCornersConstraintPanel12}

    con46 = {'type': 'ineq', 'fun': KeepGuessPointsMinDistanceApartConstraintPanel13}
    con47 = {'type': 'eq', 'fun': PathStartPointsFallOnLinesPanel13}
    con48 = {'type': 'ineq', 'fun': StartPointsDoNotGoBeyondLineConstraintPanel13}
    con49 = {'type': 'ineq', 'fun': KeepMiddleNodeMinDistanceFromCornersConstraintPanel13}

    con50 = {'type': 'ineq', 'fun': KeepGuessPointsMinDistanceApartConstraintPanel14}
    con51 = {'type': 'eq', 'fun': PathStartPointsFallOnLinesPanel14}
    con52 = {'type': 'ineq', 'fun': StartPointsDoNotGoBeyondLineConstraintPanel14}
    con53 = {'type': 'ineq', 'fun': KeepMiddleNodeMinDistanceFromCornersConstraintPanel14}

    if isSinglePanel:
        cons = [con1, con2, con3]
    else:
        cons = [con1, con2, con3, con4, con5, con6, con7, con8, con9, con10, con11, con12, con13, con14,
                con15, con16, con17, con18, con19, con20, con21, con22, con23, con24, con25, con26,
                con27, con28, con29, con30, con31, con32, con33, con34, con35, con36, con37, con38, con39,
                con40, con41, con42, con43, con44, con45, con46, con47, con48, con49, con50, con51, con52, con53]

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


    def KeepGuessPointsMinDistanceApartConstraintPanel4(optimalPoints):
        distanceBetweenPoints = []

        currentPoint17 = PointsAndLinesClass.ClassPoint(optimalPoints[18], optimalPoints[19])
        currentPoint19 = PointsAndLinesClass.ClassPoint(optimalPoints[20], optimalPoints[21])
        currentPoint20 = PointsAndLinesClass.ClassPoint(optimalPoints[22], optimalPoints[23])
        currentPoint21 = PointsAndLinesClass.ClassPoint(optimalPoints[24], optimalPoints[25])

        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint21, currentPoint19) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint21, currentPoint20) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint17, currentPoint19) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint17, currentPoint20) - minDistances[0])

        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(listOfPoints[9], currentPoint20) - minDistances[1])

        return distanceBetweenPoints

    def PathStartPointsFallOnLinesPanel4(optimalPoints):
        returnVec = []

        xOfLine13 = polygonLines[12].points[1].x + (
                ((polygonLines[12].points[1].x - polygonLines[12].points[0].x) / (
                            polygonLines[12].points[1].y - polygonLines[12].points[0].y))
                * (optimalPoints[21] - polygonLines[12].points[1].y))

        xOfLine15 = polygonLines[14].points[0].x + (
                    ((polygonLines[14].points[1].x - polygonLines[14].points[0].x) / (polygonLines[14].points[1].y - polygonLines[14].points[0].y))
                    * (optimalPoints[23] - polygonLines[14].points[0].y))

        yOfLine16 = polygonLines[15].points[1].y + (
                ((polygonLines[15].points[0].y - polygonLines[15].points[1].y) / (
                            polygonLines[15].points[0].x - polygonLines[15].points[1].x))
                * (optimalPoints[24] - polygonLines[15].points[1].x))

        returnVec.append(optimalPoints[20] - xOfLine13)
        returnVec.append(xOfLine15 - optimalPoints[22])
        returnVec.append(optimalPoints[25] - yOfLine16)

        return returnVec

    def StartPointsDoNotGoBeyondLineConstraintPanel4(optimalPoints):
        returnVec = []

        returnVec.append(polygonLines[12].points[1].y - optimalPoints[21])
        returnVec.append(optimalPoints[21] - polygonLines[12].points[0].y)

        returnVec.append(polygonLines[14].points[0].y - optimalPoints[23])
        returnVec.append(optimalPoints[23] - polygonLines[15].points[1].y)

        returnVec.append(polygonLines[15].points[1].x - optimalPoints[24])
        returnVec.append(optimalPoints[24] - polygonLines[15].points[0].x)

        return returnVec


    def KeepGuessPointsMinDistanceApartConstraintPanel5(optimalPoints):
        distanceBetweenPoints = []

        currentPoint8 = PointsAndLinesClass.ClassPoint(optimalPoints[4], optimalPoints[5])
        currentPoint23 = PointsAndLinesClass.ClassPoint(optimalPoints[26], optimalPoints[27])
        currentPoint24 = PointsAndLinesClass.ClassPoint(optimalPoints[28], optimalPoints[29])
        currentPoint25 = PointsAndLinesClass.ClassPoint(optimalPoints[30], optimalPoints[31])

        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint23, currentPoint8) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint23, currentPoint24) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint25, currentPoint8) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint25, currentPoint24) - minDistances[0])

        return distanceBetweenPoints

    def PathStartPointsFallOnLinesPanel5(optimalPoints):
        returnVec = []

        yOfLine18 = polygonLines[17].points[1].y + (
                ((polygonLines[17].points[0].y - polygonLines[17].points[1].y) / (
                        polygonLines[17].points[0].x - polygonLines[17].points[1].x))
                * (optimalPoints[26] - polygonLines[17].points[1].x))

        xOfLine19 = polygonLines[18].points[0].x + (
                    ((polygonLines[18].points[1].x - polygonLines[18].points[0].x) / (
                            polygonLines[18].points[1].y - polygonLines[18].points[0].y))
                    * (optimalPoints[29] - polygonLines[18].points[0].y))

        yOfLine20 = polygonLines[19].points[1].y + (
                ((polygonLines[19].points[0].y - polygonLines[19].points[1].y) / (
                            polygonLines[19].points[0].x - polygonLines[19].points[1].x))
                * (optimalPoints[30] - polygonLines[19].points[1].x))

        returnVec.append(yOfLine18 - optimalPoints[27])
        returnVec.append(xOfLine19 - optimalPoints[28])
        returnVec.append(optimalPoints[31] - yOfLine20)

        return returnVec

    def StartPointsDoNotGoBeyondLineConstraintPanel5(optimalPoints):
        returnVec = []

        returnVec.append(polygonLines[17].points[1].x - optimalPoints[26])
        returnVec.append(optimalPoints[26] - polygonLines[17].points[0].x)

        returnVec.append(polygonLines[18].points[0].y - optimalPoints[29])
        returnVec.append(optimalPoints[29] - polygonLines[18].points[1].y)

        returnVec.append(polygonLines[19].points[1].x - optimalPoints[30])
        returnVec.append(optimalPoints[30] - polygonLines[19].points[0].x)

        return returnVec


    def KeepGuessPointsMinDistanceApartConstraintPanel6(optimalPoints):
        distanceBetweenPoints = []

        currentPoint12 = PointsAndLinesClass.ClassPoint(optimalPoints[10], optimalPoints[11])
        currentPoint25 = PointsAndLinesClass.ClassPoint(optimalPoints[30], optimalPoints[31])
        currentPoint27 = PointsAndLinesClass.ClassPoint(optimalPoints[32], optimalPoints[33])
        currentPoint28 = PointsAndLinesClass.ClassPoint(optimalPoints[34], optimalPoints[35])

        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint25, currentPoint12) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint25, currentPoint27) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint28, currentPoint12) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint28, currentPoint27) - minDistances[0])

        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(listOfPoints[14], currentPoint27) - minDistances[1])

        return distanceBetweenPoints

    def PathStartPointsFallOnLinesPanel6(optimalPoints):
        returnVec = []

        xOfLine21 = polygonLines[20].points[0].x + (
                    ((polygonLines[20].points[1].x - polygonLines[20].points[0].x) / (
                            polygonLines[20].points[1].y - polygonLines[20].points[0].y))
                    * (optimalPoints[33] - polygonLines[20].points[0].y))

        yOfLine22 = polygonLines[21].points[1].y + (
                ((polygonLines[21].points[0].y - polygonLines[21].points[1].y) / (
                            polygonLines[21].points[0].x - polygonLines[21].points[1].x))
                * (optimalPoints[34] - polygonLines[21].points[1].x))

        returnVec.append(xOfLine21 - optimalPoints[32])
        returnVec.append(optimalPoints[35] - yOfLine22)

        return returnVec

    def StartPointsDoNotGoBeyondLineConstraintPanel6(optimalPoints):
        returnVec = []

        returnVec.append(polygonLines[20].points[0].y - optimalPoints[33])
        returnVec.append(optimalPoints[33] - polygonLines[20].points[1].y)

        returnVec.append(polygonLines[21].points[1].x - optimalPoints[34])
        returnVec.append(optimalPoints[34] - polygonLines[21].points[0].x)

        return returnVec


    def KeepGuessPointsMinDistanceApartConstraintPanel7(optimalPoints):
        distanceBetweenPoints = []

        currentPoint30 = PointsAndLinesClass.ClassPoint(optimalPoints[36], optimalPoints[37])
        currentPoint27 = PointsAndLinesClass.ClassPoint(optimalPoints[32], optimalPoints[33])
        currentPoint31 = PointsAndLinesClass.ClassPoint(optimalPoints[38], optimalPoints[39])
        currentPoint32 = PointsAndLinesClass.ClassPoint(optimalPoints[40], optimalPoints[41])

        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint30, currentPoint27) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint27, currentPoint31) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint31, currentPoint32) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint32, currentPoint30) - minDistances[0])

        return distanceBetweenPoints

    def PathStartPointsFallOnLinesPanel7(optimalPoints):
        returnVec = []

        xOfLine23 = polygonLines[22].points[0].x + (
                    ((polygonLines[22].points[1].x - polygonLines[22].points[0].x) / (
                            polygonLines[22].points[1].y - polygonLines[22].points[0].y))
                    * (optimalPoints[37] - polygonLines[22].points[0].y))

        xOfLine24 = polygonLines[23].points[0].x + (
                ((polygonLines[23].points[1].x - polygonLines[23].points[0].x) / (
                        polygonLines[23].points[1].y - polygonLines[23].points[0].y))
                * (optimalPoints[39] - polygonLines[23].points[0].y))

        yOfLine25 = polygonLines[24].points[1].y + (
                ((polygonLines[24].points[0].y - polygonLines[24].points[1].y) / (
                            polygonLines[24].points[0].x - polygonLines[24].points[1].x))
                * (optimalPoints[40] - polygonLines[24].points[1].x))

        returnVec.append(optimalPoints[36] - xOfLine23)
        returnVec.append(xOfLine24 - optimalPoints[38])
        returnVec.append(optimalPoints[41] - yOfLine25)

        return returnVec

    def StartPointsDoNotGoBeyondLineConstraintPanel7(optimalPoints):
        returnVec = []

        returnVec.append(polygonLines[22].points[1].y - optimalPoints[37])
        returnVec.append(optimalPoints[37] - polygonLines[22].points[0].y)

        returnVec.append(polygonLines[23].points[0].y - optimalPoints[39])
        returnVec.append(optimalPoints[39] - polygonLines[23].points[1].y)

        returnVec.append(polygonLines[24].points[1].x - optimalPoints[40])
        returnVec.append(optimalPoints[40] - polygonLines[24].points[0].x)

        return returnVec


    def KeepGuessPointsMinDistanceApartConstraintPanel8(optimalPoints):
        distanceBetweenPoints = []

        currentPoint32 = PointsAndLinesClass.ClassPoint(optimalPoints[40], optimalPoints[41])
        currentPoint34 = PointsAndLinesClass.ClassPoint(optimalPoints[42], optimalPoints[43])
        currentPoint35 = PointsAndLinesClass.ClassPoint(optimalPoints[44], optimalPoints[45])
        currentPoint36 = PointsAndLinesClass.ClassPoint(optimalPoints[46], optimalPoints[47])

        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint34, currentPoint32) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint32, currentPoint35) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint35, currentPoint36) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint36, currentPoint34) - minDistances[0])

        return distanceBetweenPoints

    def PathStartPointsFallOnLinesPanel8(optimalPoints):
        returnVec = []

        xOfLine26 = polygonLines[25].points[0].x + (
                    ((polygonLines[25].points[1].x - polygonLines[25].points[0].x) / (
                            polygonLines[25].points[1].y - polygonLines[25].points[0].y))
                    * (optimalPoints[43] - polygonLines[25].points[0].y))

        xOfLine27 = polygonLines[26].points[0].x + (
                ((polygonLines[26].points[1].x - polygonLines[26].points[0].x) / (
                        polygonLines[26].points[1].y - polygonLines[26].points[0].y))
                * (optimalPoints[45] - polygonLines[26].points[0].y))

        yOfLine28 = polygonLines[27].points[1].y + (
                ((polygonLines[27].points[0].y - polygonLines[27].points[1].y) / (
                            polygonLines[27].points[0].x - polygonLines[27].points[1].x))
                * (optimalPoints[46] - polygonLines[27].points[1].x))

        returnVec.append(optimalPoints[42] - xOfLine26)
        returnVec.append(xOfLine27 - optimalPoints[44])
        returnVec.append(optimalPoints[47] - yOfLine28)

        return returnVec

    def StartPointsDoNotGoBeyondLineConstraintPanel8(optimalPoints):
        returnVec = []

        returnVec.append(polygonLines[25].points[1].y - optimalPoints[43])
        returnVec.append(optimalPoints[43] - polygonLines[25].points[0].y)

        returnVec.append(polygonLines[26].points[0].y - optimalPoints[45])
        returnVec.append(optimalPoints[45] - polygonLines[26].points[1].y)

        returnVec.append(polygonLines[27].points[1].x - optimalPoints[46])
        returnVec.append(optimalPoints[46] - polygonLines[27].points[0].x)

        return returnVec


    def KeepGuessPointsMinDistanceApartConstraintPanel9(optimalPoints):
        distanceBetweenPoints = []

        currentPoint38 = PointsAndLinesClass.ClassPoint(optimalPoints[48], optimalPoints[49])
        currentPoint39 = PointsAndLinesClass.ClassPoint(optimalPoints[50], optimalPoints[51])
        currentPoint34 = PointsAndLinesClass.ClassPoint(optimalPoints[42], optimalPoints[43])
        currentPoint40 = PointsAndLinesClass.ClassPoint(optimalPoints[52], optimalPoints[53])

        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint38, currentPoint39) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint39, currentPoint34) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint34, currentPoint40) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint40, currentPoint38) - minDistances[0])

        return distanceBetweenPoints

    def PathStartPointsFallOnLinesPanel9(optimalPoints):
        returnVec = []

        xOfLine29 = polygonLines[28].points[0].x + (
                    ((polygonLines[28].points[1].x - polygonLines[28].points[0].x) / (
                            polygonLines[28].points[1].y - polygonLines[28].points[0].y))
                    * (optimalPoints[49] - polygonLines[28].points[0].y))

        yOfLine30 = polygonLines[29].points[1].y + (
                ((polygonLines[29].points[0].y - polygonLines[29].points[1].y) / (
                        polygonLines[29].points[0].x - polygonLines[29].points[1].x))
                * (optimalPoints[50] - polygonLines[29].points[1].x))

        yOfLine31 = polygonLines[30].points[1].y + (
                ((polygonLines[30].points[0].y - polygonLines[30].points[1].y) / (
                            polygonLines[30].points[0].x - polygonLines[30].points[1].x))
                * (optimalPoints[52] - polygonLines[30].points[1].x))

        returnVec.append(optimalPoints[48] - xOfLine29)
        returnVec.append(yOfLine30 - optimalPoints[51])
        returnVec.append(optimalPoints[53] - yOfLine31)

        return returnVec

    def StartPointsDoNotGoBeyondLineConstraintPanel9(optimalPoints):
        returnVec = []

        returnVec.append(polygonLines[28].points[1].y - optimalPoints[49])
        returnVec.append(optimalPoints[49] - polygonLines[28].points[0].y)

        returnVec.append(polygonLines[29].points[1].x - optimalPoints[50])
        returnVec.append(optimalPoints[50] - polygonLines[29].points[0].x)

        returnVec.append(polygonLines[30].points[1].x - optimalPoints[52])
        returnVec.append(optimalPoints[52] - polygonLines[30].points[0].x)

        return returnVec


    def KeepGuessPointsMinDistanceApartConstraintPanel10(optimalPoints):
        distanceBetweenPoints = []

        currentPoint42 = PointsAndLinesClass.ClassPoint(optimalPoints[54], optimalPoints[55])
        currentPoint20 = PointsAndLinesClass.ClassPoint(optimalPoints[22], optimalPoints[23])
        currentPoint38 = PointsAndLinesClass.ClassPoint(optimalPoints[48], optimalPoints[49])
        currentPoint43 = PointsAndLinesClass.ClassPoint(optimalPoints[56], optimalPoints[57])

        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint42, currentPoint20) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint20, currentPoint38) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint38, currentPoint43) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint43, currentPoint42) - minDistances[0])

        return distanceBetweenPoints

    def PathStartPointsFallOnLinesPanel10(optimalPoints):
        returnVec = []

        xOfLine32 = polygonLines[31].points[0].x + (
                    ((polygonLines[31].points[1].x - polygonLines[31].points[0].x) / (
                            polygonLines[31].points[1].y - polygonLines[31].points[0].y))
                    * (optimalPoints[55] - polygonLines[31].points[0].y))

        yOfLine33 = polygonLines[32].points[1].y + (
                ((polygonLines[32].points[0].y - polygonLines[32].points[1].y) / (
                            polygonLines[32].points[0].x - polygonLines[32].points[1].x))
                * (optimalPoints[56] - polygonLines[32].points[1].x))

        returnVec.append(optimalPoints[54] - xOfLine32)
        returnVec.append(optimalPoints[57] - yOfLine33)

        return returnVec

    def StartPointsDoNotGoBeyondLineConstraintPanel10(optimalPoints):
        returnVec = []

        returnVec.append(polygonLines[31].points[1].y - optimalPoints[55])
        returnVec.append(optimalPoints[55] - polygonLines[31].points[0].y)

        returnVec.append(polygonLines[32].points[1].x - optimalPoints[56])
        returnVec.append(optimalPoints[56] - polygonLines[32].points[0].x)

        return returnVec

    def KeepGoresAligned(optimalPoints):
        distanceBetweenPoints = []
        currentPoint6 = PointsAndLinesClass.ClassPoint(optimalPoints[0], optimalPoints[1])
        currentPoint11 = PointsAndLinesClass.ClassPoint(optimalPoints[8], optimalPoints[9])
        currentPoint15 = PointsAndLinesClass.ClassPoint(optimalPoints[14], optimalPoints[15])
        currentPoint19 = PointsAndLinesClass.ClassPoint(optimalPoints[20], optimalPoints[21])

        currentPoint36 = PointsAndLinesClass.ClassPoint(optimalPoints[46], optimalPoints[47])
        currentPoint40 = PointsAndLinesClass.ClassPoint(optimalPoints[52], optimalPoints[53])
        currentPoint43 = PointsAndLinesClass.ClassPoint(optimalPoints[56], optimalPoints[57])
        currentPoint47 = PointsAndLinesClass.ClassPoint(optimalPoints[60], optimalPoints[61])

        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(listOfPoints[1], currentPoint6) -
                                     CrossFrameOptimizationLibrary.distance(listOfPoints[17], currentPoint36))

        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(listOfPoints[0], currentPoint11) -
                                     CrossFrameOptimizationLibrary.distance(listOfPoints[18], currentPoint40))

        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(listOfPoints[4], currentPoint15) -
                                     CrossFrameOptimizationLibrary.distance(listOfPoints[19], currentPoint43))

        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(listOfPoints[6], currentPoint19) -
                                     CrossFrameOptimizationLibrary.distance(listOfPoints[11], currentPoint47))

        return distanceBetweenPoints


    def KeepGuessPointsMinDistanceApartConstraintPanel11And12(optimalPoints):
        distanceBetweenPoints = []

        currentPoint16 = PointsAndLinesClass.ClassPoint(optimalPoints[16], optimalPoints[17])
        currentPoint28 = PointsAndLinesClass.ClassPoint(optimalPoints[34], optimalPoints[35])
        currentPoint44 = PointsAndLinesClass.ClassPoint(optimalPoints[58], optimalPoints[59])
        currentPoint30 = PointsAndLinesClass.ClassPoint(optimalPoints[36], optimalPoints[37])
        currentPoint39 = PointsAndLinesClass.ClassPoint(optimalPoints[50], optimalPoints[51])


        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint16, currentPoint28) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint28, currentPoint44) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint44, currentPoint16) - minDistances[0])

        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint44, currentPoint30) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint30, currentPoint39) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint39, currentPoint44) - minDistances[0])

        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(listOfPoints[7], currentPoint44) - minDistances[1])

        return distanceBetweenPoints

    def PathStartPointsFallOnLinesPanel11And12(optimalPoints):
        returnVec = []

        yOfLine34 = polygonLines[38].points[1].y + (
                ((polygonLines[38].points[0].y - polygonLines[38].points[1].y) / (
                        polygonLines[38].points[0].x - polygonLines[38].points[1].x))
                * (optimalPoints[58] - polygonLines[38].points[1].x))

        returnVec.append(yOfLine34 - optimalPoints[59])

        return returnVec

    def StartPointsDoNotGoBeyondLineConstraintPanel11And12(optimalPoints):
        returnVec = []

        returnVec.append(polygonLines[38].points[1].y - optimalPoints[59])
        returnVec.append(optimalPoints[59] - polygonLines[38].points[0].y)

        return returnVec


    def KeepGuessPointsMinDistanceApartConstraintPanel13And14(optimalPoints):
        distanceBetweenPoints = []

        currentPoint21 = PointsAndLinesClass.ClassPoint(optimalPoints[24], optimalPoints[25])
        currentPoint42 = PointsAndLinesClass.ClassPoint(optimalPoints[54], optimalPoints[55])
        currentPoint47 = PointsAndLinesClass.ClassPoint(optimalPoints[60], optimalPoints[61])
        currentPoint48 = PointsAndLinesClass.ClassPoint(optimalPoints[62], optimalPoints[63])
        currentPoint51 = PointsAndLinesClass.ClassPoint(optimalPoints[64], optimalPoints[65])


        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint42, currentPoint47) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint47, currentPoint48) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint48, currentPoint42) - minDistances[0])

        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint51, currentPoint21) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint21, currentPoint48) - minDistances[0])
        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(currentPoint48, currentPoint51) - minDistances[0])

        distanceBetweenPoints.append(CrossFrameOptimizationLibrary.distance(listOfPoints[10], currentPoint51) - minDistances[1])

        return distanceBetweenPoints

    def PathStartPointsFallOnLinesPanel13And14(optimalPoints):
        returnVec = []

        yOfLine36 = polygonLines[40].points[1].y + (
                ((polygonLines[40].points[0].y - polygonLines[40].points[1].y) / (
                        polygonLines[40].points[0].x - polygonLines[40].points[1].x))
                * (optimalPoints[60] - polygonLines[40].points[1].x))

        xOfLine37 = polygonLines[39].points[0].x + (
                ((polygonLines[39].points[1].x - polygonLines[39].points[0].x) / (
                        polygonLines[39].points[1].y - polygonLines[39].points[0].y))
                * (optimalPoints[63] - polygonLines[39].points[0].y))

        xOfLine38 = polygonLines[41].points[0].x + (
                ((polygonLines[41].points[1].x - polygonLines[41].points[0].x) / (
                        polygonLines[41].points[1].y - polygonLines[41].points[0].y))
                * (optimalPoints[65] - polygonLines[41].points[0].y))

        returnVec.append(optimalPoints[61] - yOfLine36)
        returnVec.append(optimalPoints[62] - xOfLine37)
        returnVec.append(optimalPoints[64] - xOfLine38)


        return returnVec

    def StartPointsDoNotGoBeyondLineConstraintPanel13And14(optimalPoints):
        returnVec = []

        returnVec.append(polygonLines[39].points[1].y - optimalPoints[63])
        returnVec.append(optimalPoints[63] - polygonLines[39].points[0].y)

        returnVec.append(polygonLines[40].points[1].x - optimalPoints[60])
        returnVec.append(optimalPoints[60] - polygonLines[40].points[0].x)

        returnVec.append(polygonLines[41].points[1].y - optimalPoints[65])
        returnVec.append(optimalPoints[65] - polygonLines[41].points[0].y)

        return returnVec


    def KeepGuessPointsAwayFromVerticiesSinglePanel(optimalPoints):
        distanceBetweenPoints = []

        currentPoint6 = PointsAndLinesClass.ClassPoint(optimalPoints[0], optimalPoints[1])
        currentPoint7 = PointsAndLinesClass.ClassPoint(optimalPoints[2], optimalPoints[3])
        currentPoint8 = PointsAndLinesClass.ClassPoint(optimalPoints[4], optimalPoints[5])
        currentPoint9 = PointsAndLinesClass.ClassPoint(optimalPoints[6], optimalPoints[7])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[0], currentPoint6) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[1], currentPoint6) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[2], currentPoint7) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[1], currentPoint7) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[2], currentPoint8) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[3], currentPoint8) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[0], currentPoint9) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[3], currentPoint9) - minDistances[3])

        return distanceBetweenPoints

    def KeepGuessPointsAwayFromVerticies(optimalPoints):
        distanceBetweenPoints = []

        # currentPoint6 = PointsAndLinesClass.ClassPoint(optimalPoints[0], optimalPoints[1])
        # currentPoint7 = PointsAndLinesClass.ClassPoint(optimalPoints[2], optimalPoints[3])
        # currentPoint8 = PointsAndLinesClass.ClassPoint(optimalPoints[4], optimalPoints[5])
        # currentPoint9 = PointsAndLinesClass.ClassPoint(optimalPoints[6], optimalPoints[7])
        currentPoint11 = PointsAndLinesClass.ClassPoint(optimalPoints[8], optimalPoints[9])
        currentPoint12 = PointsAndLinesClass.ClassPoint(optimalPoints[10], optimalPoints[11])
        currentPoint13 = PointsAndLinesClass.ClassPoint(optimalPoints[12], optimalPoints[13])
        currentPoint15 = PointsAndLinesClass.ClassPoint(optimalPoints[14], optimalPoints[15])
        currentPoint16 = PointsAndLinesClass.ClassPoint(optimalPoints[16], optimalPoints[17])
        currentPoint17 = PointsAndLinesClass.ClassPoint(optimalPoints[18], optimalPoints[19])
        currentPoint19 = PointsAndLinesClass.ClassPoint(optimalPoints[20], optimalPoints[21])
        currentPoint20 = PointsAndLinesClass.ClassPoint(optimalPoints[22], optimalPoints[23])
        currentPoint21 = PointsAndLinesClass.ClassPoint(optimalPoints[24], optimalPoints[25])
        currentPoint23 = PointsAndLinesClass.ClassPoint(optimalPoints[26], optimalPoints[27])
        currentPoint24 = PointsAndLinesClass.ClassPoint(optimalPoints[28], optimalPoints[29])
        currentPoint25 = PointsAndLinesClass.ClassPoint(optimalPoints[30], optimalPoints[31])
        currentPoint25 = PointsAndLinesClass.ClassPoint(optimalPoints[30], optimalPoints[31])
        currentPoint27 = PointsAndLinesClass.ClassPoint(optimalPoints[32], optimalPoints[33])
        currentPoint28 = PointsAndLinesClass.ClassPoint(optimalPoints[34], optimalPoints[35])
        currentPoint30 = PointsAndLinesClass.ClassPoint(optimalPoints[36], optimalPoints[37])
        currentPoint31 = PointsAndLinesClass.ClassPoint(optimalPoints[38], optimalPoints[39])
        currentPoint32 = PointsAndLinesClass.ClassPoint(optimalPoints[40], optimalPoints[41])
        currentPoint34 = PointsAndLinesClass.ClassPoint(optimalPoints[42], optimalPoints[43])
        currentPoint35 = PointsAndLinesClass.ClassPoint(optimalPoints[44], optimalPoints[45])
        currentPoint36 = PointsAndLinesClass.ClassPoint(optimalPoints[46], optimalPoints[47])
        currentPoint38 = PointsAndLinesClass.ClassPoint(optimalPoints[48], optimalPoints[49])
        currentPoint39 = PointsAndLinesClass.ClassPoint(optimalPoints[50], optimalPoints[51])
        currentPoint40 = PointsAndLinesClass.ClassPoint(optimalPoints[52], optimalPoints[53])
        currentPoint42 = PointsAndLinesClass.ClassPoint(optimalPoints[54], optimalPoints[55])
        currentPoint43 = PointsAndLinesClass.ClassPoint(optimalPoints[56], optimalPoints[57])
        currentPoint44 = PointsAndLinesClass.ClassPoint(optimalPoints[58], optimalPoints[59])
        currentPoint47 = PointsAndLinesClass.ClassPoint(optimalPoints[60], optimalPoints[61])
        currentPoint48 = PointsAndLinesClass.ClassPoint(optimalPoints[36], optimalPoints[37])
        currentPoint51 = PointsAndLinesClass.ClassPoint(optimalPoints[64], optimalPoints[65])

        # distanceBetweenPoints.append(
        #     CrossFrameOptimizationLibrary.distance(listOfPoints[0], currentPoint6) - minDistances[3])
        # distanceBetweenPoints.append(
        #     CrossFrameOptimizationLibrary.distance(listOfPoints[1], currentPoint6) - minDistances[3])
        # distanceBetweenPoints.append(
        #     CrossFrameOptimizationLibrary.distance(listOfPoints[2], currentPoint7) - minDistances[3])
        # distanceBetweenPoints.append(
        #     CrossFrameOptimizationLibrary.distance(listOfPoints[1], currentPoint7) - minDistances[3])
        # distanceBetweenPoints.append(
        #     CrossFrameOptimizationLibrary.distance(listOfPoints[2], currentPoint8) - minDistances[3])
        # distanceBetweenPoints.append(
        #     CrossFrameOptimizationLibrary.distance(listOfPoints[3], currentPoint8) - minDistances[3])
        # distanceBetweenPoints.append(
        #     CrossFrameOptimizationLibrary.distance(listOfPoints[0], currentPoint9) - minDistances[3])
        # distanceBetweenPoints.append(
        #     CrossFrameOptimizationLibrary.distance(listOfPoints[3], currentPoint9) - minDistances[3])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[0], currentPoint11) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[4], currentPoint11) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[3], currentPoint12) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[5], currentPoint12) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[4], currentPoint13) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[5], currentPoint13) - minDistances[3])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[6], currentPoint15) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[4], currentPoint15) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[7], currentPoint16) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[5], currentPoint16) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[6], currentPoint17) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[7], currentPoint17) - minDistances[3])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[6], currentPoint19) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[8], currentPoint19) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[7], currentPoint20) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[9], currentPoint20) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[8], currentPoint21) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[9], currentPoint21) - minDistances[3])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[2], currentPoint23) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[12], currentPoint23) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[12], currentPoint24) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[13], currentPoint24) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[3], currentPoint25) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[13], currentPoint25) - minDistances[3])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[13], currentPoint27) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[14], currentPoint27) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[5], currentPoint28) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[14], currentPoint28) - minDistances[3])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[14], currentPoint30) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[16], currentPoint30) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[13], currentPoint31) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[15], currentPoint31) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[16], currentPoint32) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[15], currentPoint32) - minDistances[3])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[18], currentPoint34) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[16], currentPoint34) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[17], currentPoint35) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[15], currentPoint35) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[18], currentPoint36) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[17], currentPoint36) - minDistances[3])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[7], currentPoint38) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[19], currentPoint38) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[7], currentPoint39) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[16], currentPoint39) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[19], currentPoint40) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[18], currentPoint40) - minDistances[3])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[9], currentPoint42) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[11], currentPoint42) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[11], currentPoint43) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[19], currentPoint43) - minDistances[3])

        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[7], currentPoint44) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[14], currentPoint44) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[10], currentPoint47) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[11], currentPoint47) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[10], currentPoint48) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[9], currentPoint48) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[8], currentPoint51) - minDistances[3])
        distanceBetweenPoints.append(
            CrossFrameOptimizationLibrary.distance(listOfPoints[10], currentPoint51) - minDistances[3])

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

    con10 = {'type': 'ineq', 'fun': KeepGuessPointsMinDistanceApartConstraintPanel4}
    con11 = {'type': 'eq', 'fun': PathStartPointsFallOnLinesPanel4}
    con12 = {'type': 'ineq', 'fun': StartPointsDoNotGoBeyondLineConstraintPanel4}

    con13 = {'type': 'ineq', 'fun': KeepGuessPointsMinDistanceApartConstraintPanel5}
    con14 = {'type': 'eq', 'fun': PathStartPointsFallOnLinesPanel5}
    con15 = {'type': 'ineq', 'fun': StartPointsDoNotGoBeyondLineConstraintPanel5}

    con16 = {'type': 'ineq', 'fun': KeepGuessPointsMinDistanceApartConstraintPanel6}
    con17 = {'type': 'eq', 'fun': PathStartPointsFallOnLinesPanel6}
    con18 = {'type': 'ineq', 'fun': StartPointsDoNotGoBeyondLineConstraintPanel6}

    con19 = {'type': 'ineq', 'fun': KeepGuessPointsMinDistanceApartConstraintPanel7}
    con20 = {'type': 'eq', 'fun': PathStartPointsFallOnLinesPanel7}
    con21 = {'type': 'ineq', 'fun': StartPointsDoNotGoBeyondLineConstraintPanel7}

    con22 = {'type': 'ineq', 'fun': KeepGuessPointsMinDistanceApartConstraintPanel8}
    con23 = {'type': 'eq', 'fun': PathStartPointsFallOnLinesPanel8}
    con24 = {'type': 'ineq', 'fun': StartPointsDoNotGoBeyondLineConstraintPanel8}

    con25 = {'type': 'ineq', 'fun': KeepGuessPointsMinDistanceApartConstraintPanel9}
    con26 = {'type': 'eq', 'fun': PathStartPointsFallOnLinesPanel9}
    con27 = {'type': 'ineq', 'fun': StartPointsDoNotGoBeyondLineConstraintPanel9}

    con28 = {'type': 'ineq', 'fun': KeepGuessPointsMinDistanceApartConstraintPanel10}
    con29 = {'type': 'eq', 'fun': PathStartPointsFallOnLinesPanel10}
    con30 = {'type': 'ineq', 'fun': StartPointsDoNotGoBeyondLineConstraintPanel10}

    con31 = {'type': 'eq', 'fun': KeepGoresAligned}

    con32 = {'type': 'ineq', 'fun': KeepGuessPointsMinDistanceApartConstraintPanel11And12}
    con33 = {'type': 'eq', 'fun': PathStartPointsFallOnLinesPanel11And12}
    con34 = {'type': 'ineq', 'fun': StartPointsDoNotGoBeyondLineConstraintPanel11And12}


    con35 = {'type': 'ineq', 'fun': KeepGuessPointsMinDistanceApartConstraintPanel13And14}
    con36 = {'type': 'eq', 'fun': PathStartPointsFallOnLinesPanel13And14}
    con37 = {'type': 'ineq', 'fun': StartPointsDoNotGoBeyondLineConstraintPanel13And14}

    con38 = {'type': 'ineq', 'fun': KeepGuessPointsAwayFromVerticies}
    con39 = {'type': 'ineq', 'fun': KeepGuessPointsAwayFromVerticiesSinglePanel}

    if isSinglePanel:
        cons = [con1, con2, con3, con39]
    else:
        cons = [con1, con2, con3, con4, con5, con6, con7, con8, con9, con10, con11, con12,
                con13, con14, con15, con16, con17, con18, con19, con20, con21, con22, con23, con24,
                con25, con26, con27, con28, con29, con30, con31, con32, con33, con34, con35, con36, con37, con38, con39]

    return cons



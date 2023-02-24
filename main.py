# Python script for optimization final project

import OptimizeFlasherPanel
import PointsAndLinesClass


connectToMiddlePoint = True
zPanel = False
plotFlasherPanels = True
plotEntireFlasher = True

plotPointGuesses = True
allowModifyPolygon = False
tryAnotherPoint = True
multipleGuesses = False
allowSelectBeginPoint = True


# IF TRIANGULAR PANEL, LEAVE FOURTH POINT EMPTY
firstPoint = (0.3125, 0.497)
secondPoint = (0.08333, 1.6287)
thirdPoint = (1.875, 1.7126)
# fourthPoint = ()
fourthPoint = (1.8958, 0.5398)

shapeBaseLength = 0.05  # meters, if square or rectangle
shapeBaseHeight = 0.05  # meters, if rectangle
shapeBaseDiameter = 0.05  # meters, if circle

minDistanceBetweenPathNodes = 0.25
minDistanceFromCorners = 0.1
minDistanceFromLine = 0.05
minEdgePointDistanceFromCorners = 0.25

materials = ["Aluminum", "Steel", "ABS"]
crossSectionShapes = ["Square", "Round", "Rectangle", "I-Beam"]
material = materials[0]
crossSection = crossSectionShapes[0]


flasherPoint1 = (1.5556, 6.2593)
flasherPoint2 = (.55556, 8.9259)
flasherPoint3 = (3.3333, 10.074)
flasherPoint4 = (4.1111, 7.1111)
flasherPoint5 = (2.1481, 4.2593)
flasherPoint6 = (4.5185, 4.8519)
flasherPoint7 = (2.4074, 2.7407)
flasherPoint8 = (4.7037, 3.0741)
flasherPoint9 = (2.5185, 1.5556)
flasherPoint10 = (3.5556, 1.5926)
flasherPoint11 = (2.5185, 0.44444)
flasherPoint12 = (3.7407, 0.55556)
flasherPoint13 = (6.2963, 10.074)
flasherPoint14 = (6.7778, 7.4815)
flasherPoint15 = (5.6667, 4.8889)
flasherPoint16 = (8.7407, 5.1852)
flasherPoint17 = (6.407, 3.8519)
flasherPoint18 = (9.9259, 2.4444)
flasherPoint19 = (7.2593, 1.4444)
flasherPoint20 = (5.2222, 0.85185)

listOfPointsForFlasher = []
point1 = PointsAndLinesClass.ClassPoint(flasherPoint1[0], flasherPoint1[1])
point2 = PointsAndLinesClass.ClassPoint(flasherPoint2[0], flasherPoint2[1])
point3 = PointsAndLinesClass.ClassPoint(flasherPoint3[0], flasherPoint3[1])
point4 = PointsAndLinesClass.ClassPoint(flasherPoint4[0], flasherPoint4[1])

point5 = PointsAndLinesClass.ClassPoint(flasherPoint5[0], flasherPoint5[1])
point6 = PointsAndLinesClass.ClassPoint(flasherPoint6[0], flasherPoint6[1])

point7 = PointsAndLinesClass.ClassPoint(flasherPoint7[0], flasherPoint7[1])
point8 = PointsAndLinesClass.ClassPoint(flasherPoint8[0], flasherPoint8[1])

point9 = PointsAndLinesClass.ClassPoint(flasherPoint9[0], flasherPoint9[1])
point10 = PointsAndLinesClass.ClassPoint(flasherPoint10[0], flasherPoint10[1])

point11 = PointsAndLinesClass.ClassPoint(flasherPoint11[0], flasherPoint11[1])
point12 = PointsAndLinesClass.ClassPoint(flasherPoint12[0], flasherPoint12[1])

point13 = PointsAndLinesClass.ClassPoint(flasherPoint13[0], flasherPoint13[1])
point14 = PointsAndLinesClass.ClassPoint(flasherPoint14[0], flasherPoint14[1])

point15 = PointsAndLinesClass.ClassPoint(flasherPoint15[0], flasherPoint15[1])

point16 = PointsAndLinesClass.ClassPoint(flasherPoint16[0], flasherPoint16[1])
point17 = PointsAndLinesClass.ClassPoint(flasherPoint17[0], flasherPoint17[1])

point18 = PointsAndLinesClass.ClassPoint(flasherPoint18[0], flasherPoint18[1])
point19 = PointsAndLinesClass.ClassPoint(flasherPoint19[0], flasherPoint19[1])

point20 = PointsAndLinesClass.ClassPoint(flasherPoint20[0], flasherPoint20[1])



listOfPointsForFlasher.append(point1)
listOfPointsForFlasher.append(point2)
listOfPointsForFlasher.append(point3)
listOfPointsForFlasher.append(point4)

listOfPointsForFlasher.append(point5)
listOfPointsForFlasher.append(point6)

listOfPointsForFlasher.append(point7)
listOfPointsForFlasher.append(point8)

listOfPointsForFlasher.append(point9)
listOfPointsForFlasher.append(point10)

listOfPointsForFlasher.append(point11)
listOfPointsForFlasher.append(point12)

listOfPointsForFlasher.append(point13)
listOfPointsForFlasher.append(point14)

listOfPointsForFlasher.append(point15)

listOfPointsForFlasher.append(point16)
listOfPointsForFlasher.append(point17)

listOfPointsForFlasher.append(point18)
listOfPointsForFlasher.append(point19)

listOfPointsForFlasher.append(point20)


##ALL PARAMETERS ABOVE THIS LINE## #
boolOptions = [connectToMiddlePoint, plotPointGuesses, allowModifyPolygon, tryAnotherPoint, multipleGuesses, allowSelectBeginPoint, zPanel, plotFlasherPanels, plotEntireFlasher]
minDistances = [minDistanceBetweenPathNodes, minDistanceFromCorners, minDistanceFromLine, minEdgePointDistanceFromCorners]
crossSectionLengths = [shapeBaseLength, shapeBaseHeight, shapeBaseDiameter]
listOfPoints = []
point1 = PointsAndLinesClass.ClassPoint(firstPoint[0], firstPoint[1])
point2 = PointsAndLinesClass.ClassPoint(secondPoint[0], secondPoint[1])
point3 = PointsAndLinesClass.ClassPoint(thirdPoint[0], thirdPoint[1])
listOfPoints.append(point1)
listOfPoints.append(point2)
listOfPoints.append(point3)
if len(fourthPoint) != 0:
    point4 = PointsAndLinesClass.ClassPoint(fourthPoint[0], fourthPoint[1])
    listOfPoints.append(point4)



# OptimizeFlasherPanel.OptimizePolygon(listOfPoints, boolOptions, minDistances, crossSectionLengths, material, crossSection)
OptimizeFlasherPanel.Optimize22Gore(listOfPointsForFlasher, boolOptions, minDistances, crossSectionLengths, material, crossSection)

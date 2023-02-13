from math import sqrt, pi
import numpy as np
from numpy import linalg as la


def GetMaterialProperties(materialType):
    if materialType == "Aluminum":
        rho = 2800  # kg/m3
        E = 71E9  # Pascals
        Sy = 95E7  # Pascals
        Su = 110E7  # Pascals
    elif materialType == "Steel":
        rho = 7750  # kg/m3
        E = 207E9  # Pascals
        Sy = 250E7  # Pascals
        Su = 400E7  # Pascals
    elif materialType == "ABS":
        rho = 1115  # kg/m3
        E = 2E9  # Pascals
        Sy = 4E7  # Pascals, no yield strength listed
        Su = 4E7  # Pascals
    else:
        rho = -9999
        E = -9999
        Sy = -9999
        Su = -9999
        print("Invalid material selected!")

    # print('\u03C1 = ', rho)
    # print("E =", E)
    # print("Sy =", Sy)
    # print("Su =", Su)

    return rho, E, Sy, Su


def GetPropertiesOfSections(crossSectionShape, baseLength, baseHeight, diameter):
    if crossSectionShape == "Round":
        A = (pi * (diameter ** 2)) / 4
        Ix = (pi * (diameter ** 4)) / 64
        Iy = Ix
    elif crossSectionShape == "Rectangle":
        A = baseLength * baseHeight
        Ix = (baseLength * (baseHeight ** 3)) / 12
        Iy = (baseHeight * (baseLength ** 3)) / 12
    elif crossSectionShape == "Square":
        A = baseLength ** 2
        Ix = (baseLength * (baseLength ** 3)) / 12
        Iy = (baseLength * (baseLength ** 3)) / 12
    # elif crossSectionShape == "I-Beam":
        # A, Ix, Iy = OptimizeIBeam()
    else:
        A = -9999
        Ix = -9999
        Iy = -9999
        print("Invalid cross-sectional shape!")

    # print("A =", A)
    # print("Ix =", Ix)
    # print("Iy =", Iy)

    return A, Ix, Iy


def GetLengthOfLine(lineToGetLength):
    return sqrt(distance(lineToGetLength.points[0], lineToGetLength.points[1]))


def GetMassOfLine(lineToCalcMass, A, rho):
    massOfLine = GetLengthOfLine(lineToCalcMass) * A * rho  # mass in kilograms

    return massOfLine


def GetMassOfAllLines(linesToGetMass, A, rho):
    totalMass = 0
    for m in range(len(linesToGetMass)):
        totalMass += GetMassOfLine(linesToGetMass[m], A, rho)

    return totalMass


def PrintMassOfAllLines(linesToGetMass, A, rho):
    totalMass = 0
    for m in range(len(linesToGetMass)):
        totalMass += GetMassOfLine(linesToGetMass[m], A, rho)
        print("Mass of line ", m, ":", GetMassOfLine(linesToGetMass[m], A, rho), "kg")
    print("Total mass:", totalMass, "kg")


def TempStiffnessCalc(allLines, E, Ix, Iy):
    stiffness = 0
    kx = 0
    ky = 0
    gamma = 0.8517
    xLengths = []
    yLengths = []

    for stiffnessIndex in range(len(allLines)):
        xLengths.append(abs(allLines[stiffnessIndex].points[0].x - allLines[stiffnessIndex].points[1].x))
        yLengths.append(abs(allLines[stiffnessIndex].points[0].y - allLines[stiffnessIndex].points[1].y))

    for stiffnessIndex in range(len(xLengths)):
        kx += (pi*(gamma**2)*E*Ix) / (xLengths[stiffnessIndex])
        ky += (pi*(gamma**2)*E*Iy) / (yLengths[stiffnessIndex])

    stiffness = kx
    stiffness -= ky

    return stiffness



def distance(a, b):
    # print(a)
    # print(b)
    return sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)


def is_between_points(a, b, c):
    return distance(a, c) + distance(c, b) == distance(a, b)


def is_on_line(lineForDistance, pointOnLine):
    # print(lineForDistance)
    # print(point)
    return distance(lineForDistance.points[0], pointOnLine) + distance(pointOnLine,
                                                                       lineForDistance.points[1]) == distance(
        lineForDistance.points[0], lineForDistance.points[1])


def is_on_any_line(guessPoint, polygonLinesCon):
    trueStatements = [is_on_line(i, guessPoint) for i in polygonLinesCon]
    return any(trueStatements)

#
# def OptimizeIBeam():
#     def moment_of_inertia(x):
#         flangeLength, heightLength, middleThickness, flangeThickness = x
#         Iy = (((middleThickness ** 3) * heightLength) / 12) + (2 * (((flangeLength ** 3) * flangeThickness) / 12))
#         return -Iy
#
#
#     def thickness_constraint(x):
#         return np.min([x[2], x[3]]) - t_min
#
#
#     def Ix_constraint(x):
#         flangeLength, heightLength, middleThickness, flangeThickness = x
#         Ix = (((heightLength ** 3) * middleThickness) / 12) + (2 * ((((flangeThickness ** 3) * flangeLength) / 12) + (
#                 ((flangeThickness * flangeLength) * ((heightLength + flangeThickness) ** 2)) / 4)))
#         Iy = (((middleThickness ** 3) * heightLength) / 12) + (2 * (((flangeLength ** 3) * flangeThickness) / 12))
#
#         return Ix - Iy * ratioIxIy
#
#
#     def area_constraint(x):
#         flangeLength, heightLength, middleThickness, flangeThickness = x
#         A = (2 * flangeLength * flangeThickness) + (heightLength * middleThickness)
#         return A_max - A
#
#
#     def height_constraint(x):
#         flangeLength, heightLength, middleThickness, flangeThickness = x
#         return H_max - heightLength
#
#
#     def flange_constraint(x):
#         flangeLength, heightLength, middleThickness, flangeThickness = x
#         return f_max - flangeLength
#
#
#     def optimize_beam(t_min, A_min, H_max, f_max):
#         x0 = np.array([10, 20, 1, 2])  # initial guesses for flangeLength, heightLength, middleThickness, flangeThickness
#         bounds = [(0, f_max), (0, H_max), (t_min, H_max),
#                   (t_min, f_max)]  # bounds for flangeLength, heightLength, middleThickness, flangeThickness
#         constraints = [{"type": "ineq", "fun": thickness_constraint},
#                        {"type": "ineq", "fun": area_constraint},
#                        {"type": "ineq", "fun": height_constraint},
#                        {"type": "ineq", "fun": Ix_constraint},
#                        {"type": "ineq", "fun": flange_constraint}]
#         res = minimize(moment_of_inertia, x0, bounds=bounds, constraints=constraints)
#
#         return res
#
#
#     def plot_I_beam(flangeLength, heightLength, middleThickness, flangeThickness):
#         # Define the x-coordinates for the flanges and the web
#         x_flange1 = [flangeLength / 2, flangeLength / 2, -flangeLength / 2, -flangeLength / 2]
#         y_flange1 = [heightLength / 2, heightLength / 2 + flangeThickness, heightLength / 2 + flangeThickness,
#                      heightLength / 2]
#         x_flange2 = [flangeLength / 2, flangeLength / 2, -flangeLength / 2, -flangeLength / 2]
#         y_flange2 = [-heightLength / 2, -heightLength / 2 - flangeThickness, -heightLength / 2 - flangeThickness,
#                      -heightLength / 2]
#         x_web = [-middleThickness / 2, middleThickness / 2, middleThickness / 2, -middleThickness / 2]
#         y_web = [heightLength / 2, heightLength / 2, -heightLength / 2, -heightLength / 2]
#
#         # Plot the flanges and the web
#         fig = plt.figure()
#         ax = fig.add_subplot(111)
#         ax.set_aspect(1)
#         ax.fill(x_flange1, y_flange1, 'k')
#         ax.fill(x_flange2, y_flange2, 'k')
#         ax.fill(x_web, y_web, 'gray')
#         # ax.axis('off')
#         plt.show()
#
#
#     # t_min = 0.25  # inches
#     # A_max = 10  # square inches
#     # H_max = 5  # inches
#     # f_max = 5  # inches
#     # ratioIxIy = 3
#     res = optimize_beam(t_min, A_max, H_max, f_max)
#     flangeLength, heightLength, middleThickness, flangeThickness = res.x
#     Ix = (((heightLength ** 3) * middleThickness) / 12) + (2 * ((((flangeThickness ** 3) * flangeLength) / 12) + (
#                 ((flangeThickness * flangeLength) * ((heightLength + flangeThickness) ** 2)) / 4)))
#     Iy = (((middleThickness ** 3) * heightLength) / 12) + (2 * (((flangeLength ** 3) * flangeThickness) / 12))
#     A = (2 * flangeLength * flangeThickness) + (heightLength * middleThickness)
#
#     # print("Optimal flange length:", flangeLength, "inches")
#     # print("Optimal height:", heightLength, "inches")
#     # print("Optimal web thickness:", middleThickness, "inches")
#     # print("Optimal flange thickness:", flangeThickness, "inches")
#     # print("Maximized moment of inertia (y):", Iy, "inch^4")
#     # print("Maximized moment of inertia (x):", Ix, "inch^4")
#     # plot_I_beam(flangeLength, heightLength, middleThickness, flangeThickness)
#     return A, Ix, Iy
#

# def PlotOptimizedIBeam():
#     def moment_of_inertia(x):
#         flangeLength, heightLength, middleThickness, flangeThickness = x
#         Iy = (((middleThickness ** 3) * heightLength) / 12) + (2 * (((flangeLength ** 3) * flangeThickness) / 12))
#         return -Iy
#
#     def thickness_constraint(x):
#         return np.min([x[2], x[3]]) - t_min
#
#     def Ix_constraint(x):
#         flangeLength, heightLength, middleThickness, flangeThickness = x
#         Ix = (((heightLength ** 3) * middleThickness) / 12) + (2 * ((((flangeThickness ** 3) * flangeLength) / 12) + (
#                 ((flangeThickness * flangeLength) * ((heightLength + flangeThickness) ** 2)) / 4)))
#         Iy = (((middleThickness ** 3) * heightLength) / 12) + (2 * (((flangeLength ** 3) * flangeThickness) / 12))
#
#         return Ix - Iy * ratioIxIy
#
#     def area_constraint(x):
#         flangeLength, heightLength, middleThickness, flangeThickness = x
#         A = (2 * flangeLength * flangeThickness) + (heightLength * middleThickness)
#         return A_max - A
#
#     def height_constraint(x):
#         flangeLength, heightLength, middleThickness, flangeThickness = x
#         return H_max - heightLength
#
#     def flange_constraint(x):
#         flangeLength, heightLength, middleThickness, flangeThickness = x
#         return f_max - flangeLength
#
#     def optimize_beam(t_min, A_min, H_max, f_max):
#         x0 = np.array(
#             [10, 20, 1, 2])  # initial guesses for flangeLength, heightLength, middleThickness, flangeThickness
#         bounds = [(0, f_max), (0, H_max), (t_min, H_max),
#                   (t_min, f_max)]  # bounds for flangeLength, heightLength, middleThickness, flangeThickness
#         constraints = [{"type": "ineq", "fun": thickness_constraint},
#                        {"type": "ineq", "fun": area_constraint},
#                        {"type": "ineq", "fun": height_constraint},
#                        {"type": "ineq", "fun": Ix_constraint},
#                        {"type": "ineq", "fun": flange_constraint}]
#         res = minimize(moment_of_inertia, x0, bounds=bounds, constraints=constraints)
#
#         return res
#
#     def plot_I_beam(flangeLength, heightLength, middleThickness, flangeThickness):
#         # Define the x-coordinates for the flanges and the web
#         x_flange1 = [flangeLength / 2, flangeLength / 2, -flangeLength / 2, -flangeLength / 2]
#         y_flange1 = [heightLength / 2, heightLength / 2 + flangeThickness, heightLength / 2 + flangeThickness,
#                      heightLength / 2]
#         x_flange2 = [flangeLength / 2, flangeLength / 2, -flangeLength / 2, -flangeLength / 2]
#         y_flange2 = [-heightLength / 2, -heightLength / 2 - flangeThickness, -heightLength / 2 - flangeThickness,
#                      -heightLength / 2]
#         x_web = [-middleThickness / 2, middleThickness / 2, middleThickness / 2, -middleThickness / 2]
#         y_web = [heightLength / 2, heightLength / 2, -heightLength / 2, -heightLength / 2]
#
#         # Plot the flanges and the web
#         fig = plt.figure()
#         ax = fig.add_subplot(111)
#         ax.set_aspect(1)
#         ax.fill(x_flange1, y_flange1, 'k')
#         ax.fill(x_flange2, y_flange2, 'k')
#         ax.fill(x_web, y_web, 'gray')
#         # ax.axis('off')
#         plt.show()
#
#     # t_min = 0.25  # inches
#     # A_max = 10  # square inches
#     # H_max = 5  # inches
#     # f_max = 5  # inches
#     # ratioIxIy = 3
#     res = optimize_beam(t_min, A_max, H_max, f_max)
#     flangeLength, heightLength, middleThickness, flangeThickness = res.x
#     Ix = (((heightLength ** 3) * middleThickness) / 12) + (2 * ((((flangeThickness ** 3) * flangeLength) / 12) + (
#             ((flangeThickness * flangeLength) * ((heightLength + flangeThickness) ** 2)) / 4)))
#     Iy = (((middleThickness ** 3) * heightLength) / 12) + (2 * (((flangeLength ** 3) * flangeThickness) / 12))
#     A = (2 * flangeLength * flangeThickness) + (heightLength * middleThickness)
#
#     print("Optimal flange length:", flangeLength, "inches")
#     print("Optimal height:", heightLength, "inches")
#     print("Optimal web thickness:", middleThickness, "inches")
#     print("Optimal flange thickness:", flangeThickness, "inches")
#     print("Maximized moment of inertia (y):", Iy, "inch^4")
#     print("Maximized moment of inertia (x):", Ix, "inch^4")
#     plot_I_beam(flangeLength, heightLength, middleThickness, flangeThickness)
#     return A, Ix, Iy
#


def FindAxisLimits(arrayOfPoints):
    minXAxis = 10000
    maxXAxis = -10000
    minYAxis = 10000
    maxYAxis = -10000

    numberOfPointsNotOnEdge = round(len(arrayOfPoints))

    for q in range(numberOfPointsNotOnEdge):
        if arrayOfPoints[q].x > maxXAxis:
            maxXAxis = arrayOfPoints[q].x
        if arrayOfPoints[q].x < minXAxis:
            minXAxis = arrayOfPoints[q].x
        if arrayOfPoints[q].y > maxYAxis:
            maxYAxis = arrayOfPoints[q].y
        if arrayOfPoints[q].y < minYAxis:
            minYAxis = arrayOfPoints[q].y

    return minXAxis, maxXAxis, minYAxis, maxYAxis


def FindAxisLimitsOfLines(arrayOfLines):
    minXAxis = 10000
    maxXAxis = -10000
    minYAxis = 10000
    maxYAxis = -10000

    minXAxis = float(minXAxis)
    maxXAxis = float(maxXAxis)
    minYAxis = float(minYAxis)
    maxYAxis = float(maxYAxis)

    xPoints = []
    yPoints = []

    for q in range(len(arrayOfLines)):
        xPoints.append(arrayOfLines[q].points[0].x)
        xPoints.append(arrayOfLines[q].points[1].x)
        yPoints.append(arrayOfLines[q].points[0].y)
        yPoints.append(arrayOfLines[q].points[1].y)

    numberOfPointsNotOnEdge = round(len(xPoints))

    for q in range(numberOfPointsNotOnEdge):
        if xPoints[q] > maxXAxis:
            maxXAxis = xPoints[q]
        if xPoints[q] < minXAxis:
            minXAxis = xPoints[q]
        if yPoints[q] > maxYAxis:
            maxYAxis = yPoints[q]
        if yPoints[q] < minYAxis:
            minYAxis = yPoints[q]

    return minXAxis, maxXAxis, minYAxis, maxYAxis



def GetMidpointOfLine(currLine):
    return [(((currLine.points[0].x + currLine.points[1].x) / 2), ((currLine.points[0].y + currLine.points[1].y) / 2))]


def GetPointDistanceFromLine(middlePoint, polygonLine):
    p1 = np.array([polygonLine.points[0].x, polygonLine.points[0].y])
    p2 = np.array([polygonLine.points[1].x, polygonLine.points[1].y])
    p3 = np.array([middlePoint.x, middlePoint.y])

    return np.cross(p2 - p1, p3 - p1) / la.norm(p2 - p1)


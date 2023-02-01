# Initial python script for optimization final project

from math import sqrt, pi, floor, ceil, acos
import matplotlib.pyplot as plt
from scipy.optimize import minimize, brute
from tqdm import tqdm

materials = ["Aluminum", "Steel", "ABS"]
crossSectionShapes = ["Square", "Round", "Rectangle"]

shapeBaseLength = 0.05  # meters, if square or rectangle
shapeBaseHeight = 0.05  # meters, if rectangle
shapeBaseDiameter = 0.05  # meters, if circle

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
        rho = 1115 # kg/m3
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


def GetPropertiesOfSections(crossSectionShape):
    if crossSectionShape == "Round":
        A = (pi*(shapeBaseDiameter**2))/4
        Ix = (pi*(shapeBaseDiameter**4))/64
        Iy = Ix
    elif crossSectionShape == "Rectangle":
        A = shapeBaseLength*shapeBaseHeight
        Ix = (shapeBaseLength * (shapeBaseHeight ** 3)) / 12
        Iy = (shapeBaseHeight * (shapeBaseLength ** 3)) / 12
    elif crossSectionShape == "Square":
        A = shapeBaseLength**2
        Ix = (shapeBaseLength * (shapeBaseLength ** 3)) / 12
        Iy = (shapeBaseLength * (shapeBaseLength ** 3)) / 12

    # print("A =", A)
    # print("Ix =", Ix)
    # print("Iy =", Iy)

    return A, Ix, Iy


def GetLengthOfLine(lineToGetLength):
    return sqrt(distance(lineToGetLength.points[0], lineToGetLength.points[1]))


def GetMassOfLine(lineToCalcMass):
    A, Ix, Iy = GetPropertiesOfSections(crossSectionShapes[0])
    rho, E, Sy, Su = GetMaterialProperties(materials[0])
    massOfLine = GetLengthOfLine(lineToCalcMass) * A * rho  # mass in kilograms

    return massOfLine


def GetMassOfAllLines(linesToGetMass):
    totalMass = 0
    for m in range(len(linesToGetMass)):
        totalMass += GetMassOfLine(linesToGetMass[m])
        # print("Mass of line ", m, ":", GetMassOfLine(linesToGetMass[m]))

    return totalMass


def PrintMassOfAllLines(linesToGetMass):
    totalMass = 0
    for m in range(len(linesToGetMass)):
        totalMass += GetMassOfLine(linesToGetMass[m])
        print("Mass of line ", m, ":", GetMassOfLine(linesToGetMass[m]))
    print("Total mass:", totalMass)

class point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __str__(self):
        return f"x:{self.x} y:{self.y}"


class line:
    def __init__(self, firstInitialPointForLine, secondInitialPointForLine):
        self.points = [firstInitialPointForLine, secondInitialPointForLine]

    def __str__(self):
        return f"({self.points[0].x},{self.points[0].y}), ({self.points[1].x},{self.points[1].y})"


def distance(a,b):
    return sqrt((a.x - b.x)**2 + (a.y - b.y)**2)

def is_between_points(a,b,c):
    return distance(a,c) + distance(c,b) == distance(a,b)
def is_on_line(line,point):
    return distance(line.points[0],point) + distance(point,line.points[1]) == distance(line.points[0],line.points[1])

def is_on_any_line(guessPoint):
    trueStatements = [is_on_line(i, guessPoint) for i in polygonLines]
    return any(trueStatements)


def plotShape(linesToPlot,  numberOfPolygonLines):
    plt.axis('off')
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

    xValuesToPlotShape.pop()
    yValuesToPlotShape.pop()

    xValuesToPlotPath.pop()
    yValuesToPlotPath.pop()

    plt.plot(xValuesToPlotShape, yValuesToPlotShape, 'b--')
    plt.plot(xValuesToPlotPath, yValuesToPlotPath, 'g-')

    ax = plt.gca()
    ax.set_aspect(1)

    plt.xlabel('Y')
    plt.ylabel('X')
    plt.show()


def constraint1(point5Optimal):
    return -(point5Optimal[0]-minDistance)
def constraint2(point5Optimal):
    return (point5Optimal[0]-minDistance)
def constraint3(point5Optimal):
    return -(point5Optimal[1]-minDistance)
def constraint4(point5Optimal):
    return (point5Optimal[1]-minDistance)

con1 = {'type': 'ineq', 'fun': constraint1}
con2 = {'type': 'ineq', 'fun': constraint2}
con3 = {'type': 'ineq', 'fun': constraint3}
con4 = {'type': 'ineq', 'fun': constraint4}
cons = [con1, con2, con3, con4]


# pointsInitialGuess = [(0, 0), (1, 0), (1, 1), (0, 1)]
point5InitialGuess = [(0.25, 0.375)]

def functionToMinimize(point5Optimal):
    point5New = point(point5Optimal[0], point5Optimal[1])
    line5 = line(point1, point5New)
    line6 = line(point2, point5New)
    line7 = line(point3, point5New)
    line8 = line(point4, point5New)
    pathLinesNew = [line5, line6, line7, line8]

    return GetMassOfAllLines(pathLinesNew)


point1 = point(0,0)
point2 = point(0,1)
point3 = point(1,1)
point4 = point(1,0)

line1 = line(point1, point2)
line2 = line(point2, point3)
line3 = line(point3, point4)
line4 = line(point1, point4)


polygonLines = [line1, line2, line3, line4]


minDistance = 0.1
opt = {'maxiter': 500}
result = minimize(functionToMinimize, point5InitialGuess, constraints=cons, options=opt)
print(result)

point5 = point(result.x[0], result.x[1])

line5 = line(point1, point5)
line6 = line(point2, point5)
line7 = line(point3, point5)
line8 = line(point4, point5)

pathLines = [line5, line6, line7, line8]

plotLines = polygonLines.copy()
plotLines.extend(pathLines)

# print("Mass of lines:", GetMassOfAllLines(polygonLines))
# print("Mass of path lines:", GetMassOfAllLines(pathLines))
PrintMassOfAllLines(pathLines)
# for l in range(len(plotLines)):
#     print(plotLines[l])
# plotShape(polygonLines)
plotShape(plotLines, len(polygonLines))




# print("point 5 is between:", is_between_points(point1, point2,point5), ". This should be False.")
# print("point 1 is between:", is_between_points(point1, point4, point1), ". This should be True.")
# print("Point 1 is on line 1:", is_on_line(line1, point1), ". This should be True.")
# print("Point 3 is on line 1:", is_on_line(line1, point3), ". This should be False.")
# print("Point 5 is on line 4:", is_on_line(line4, point5), ". This should be False.")
# print("Point 6 is on line 4:", is_on_line(line4, point6), ". This should be True.")

# print("Point 1 is on any line", is_on_any_line(point1), ". This should be True.")
# print("Point 2 is on any line", is_on_any_line(point2), ". This should be True.")
# print("Point 3 is on any line", is_on_any_line(point3), ". This should be True.")
# print("Point 4 is on any line", is_on_any_line(point4), ". This should be True.")
# print("Point 5 is on any line", is_on_any_line(point5), ". This should be False.")
# print("Point 6 is on any line", is_on_any_line(point6), ". This should be True.")

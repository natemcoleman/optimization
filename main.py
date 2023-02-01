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
        print("Mass of line ", m, ":", GetMassOfLine(linesToGetMass[m]))

    return totalMass


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


def plotShape(polygonLinesToPlot):
    plt.axis('off')
    xValuesToPlot = []
    yValuesToPlot = []
    for p in range(len(polygonLinesToPlot)):
        plt.plot(polygonLinesToPlot[p].points[0].x, polygonLinesToPlot[p].points[0].y, 'r*')
        plt.plot(polygonLinesToPlot[p].points[1].x, polygonLinesToPlot[p].points[1].y, 'r*')
        xValuesToPlot.append(polygonLinesToPlot[p].points[0].x)
        xValuesToPlot.append(polygonLinesToPlot[p].points[1].x)
        yValuesToPlot.append(polygonLinesToPlot[p].points[0].y)
        yValuesToPlot.append(polygonLinesToPlot[p].points[1].y)

    plt.plot(xValuesToPlot, yValuesToPlot, 'b')
    ax = plt.gca()
    ax.set_aspect(1)

    plt.xlabel('Y')
    plt.ylabel('X')
    plt.show()


con1 = {'type': 'eq', 'fun': is_on_any_line}
cons = [con1]


point1 = point(0,0)
point2 = point(0,1)
point3 = point(1,1)
point4 = point(1,0)

line1 = line(point1, point2)
line2 = line(point2, point3)
line3 = line(point3, point4)
line4 = line(point1, point4)

polygonLines = [line1, line2, line3, line4]

point5 = point(0.5, 0.5)
point6 = point(0.5,0)


print("Mass of lines:", GetMassOfAllLines(polygonLines))

plotShape(polygonLines)


# print("point 5 is between:", is_between_points(point1, point2,point5), ". This should be False.")
# print("point 6 is between:", is_between_points(point1, point4,point6), ". This should be True.")
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

# print(line1)
# print(line2)
# print(line3)
# print(line4)
# print(point1)
# print(point2)
# print(point3)
# print(point4)

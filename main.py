# Initial python script for optimization final project

from math import sqrt, pi, floor, ceil
import matplotlib.pyplot as plt
from scipy.optimize import minimize, brute

sideLength = 1

def checkDistance(point1, point2):
    return sqrt(((point2[0]-point1[0])**2)+((point2[1]-point1[1])**2))

def calculateAreaEfficiency(n, circleRadius):
    return ((pi*(circleRadius**2))/(n*sideLength*sideLength))*100

def findAreaEfficiencyFromGridOrder(squareGridOrder):

    # # print("Area efficiency with ", n, " cells is ", calculateAreaEfficiency(n, circleRadius), "%")
    # # print("Overall circle efficiency with ", n, " cells is ", calculateAreaEfficiency(n, circleRadius)/0.78539816339, "%")
    #
    # # if squareGridOrder>10:
    # #     plotCurrentGrid(squareGridOrder)
    # print("with order " ,  squareGridOrder, " p is ", p)
    # narray.append(int(squareGridOrder))
    # nefficiency.append(float(calculateAreaEfficiency(p, circleRadius)))

    p = squareGridOrder ** 2
    circleCenter = (sideLength * 0.5 * squareGridOrder, sideLength * 0.5 * squareGridOrder)
    circleRadius = sideLength * 0.5 * squareGridOrder

    for i in range(int(squareGridOrder)):
        for j in range(int(squareGridOrder)):
            checkPoint = (0, 0)
            squareCenter = ((i * sideLength) + 0.5, (j * sideLength) + 0.5)

            # find if square is within circle
            if (squareCenter[0] < circleCenter[0]):
                if (squareCenter[1] < circleCenter[1]):
                    checkPoint = (squareCenter[0] + 0.5 * sideLength, squareCenter[1] + 0.5 * sideLength)
                elif (squareCenter[1] == circleCenter[1]):
                    checkPoint = (squareCenter[0] + 0.5 * sideLength, squareCenter[1])
                else:
                    checkPoint = (squareCenter[0] + 0.5 * sideLength, squareCenter[1] - 0.5 * sideLength)

            elif (squareCenter[0] == circleCenter[0]):
                if (squareCenter[1] < circleCenter[1]):
                    checkPoint = (squareCenter[0], squareCenter[1] + 0.5 * sideLength)
                elif (squareCenter[1] == circleCenter[1]):
                    checkPoint = (squareCenter[0], squareCenter[1])
                else:
                    checkPoint = (squareCenter[0], squareCenter[1] - 0.5 * sideLength)

            else:
                if (squareCenter[1] < circleCenter[1]):

                    checkPoint = (squareCenter[0] - 0.5 * sideLength, squareCenter[1] + 0.5 * sideLength)
                elif (squareCenter[1] == circleCenter[1]):
                    checkPoint = (squareCenter[0] - 0.5 * sideLength, squareCenter[1])
                else:

                    checkPoint = (squareCenter[0] - 0.5 * sideLength, squareCenter[1] - 0.5 * sideLength)

            if checkDistance(checkPoint, circleCenter) > circleRadius:
                p -= 1

    narray.append(int(squareGridOrder))
    nefficiency.append(float(calculateAreaEfficiency(p, circleRadius)))
    # print("Area efficiency with ", p, " cells in a ", squareGridOrder, " x ", squareGridOrder, " grid with ",
    #       (squareGridOrder ** 2) - p, " cells removed is ", calculateAreaEfficiency(p, circleRadius), "%")
    # print("The circle radius is ", circleRadius)

    return -calculateAreaEfficiency(p, circleRadius)

def plotCurrentGrid(squareGridOrder):
    n = squareGridOrder**2
    plt.axes()
    circleCenter = (sideLength*0.5*squareGridOrder, sideLength*0.5*squareGridOrder)
    circleRadius = sideLength*0.5*squareGridOrder
    squareValidColor = (199/255, 201/255, 199/255, 1)
    squareValidBorderColor = (0/255, 46/255, 96/255, 1)
    squareInvalidColor = (209/255, 65/255, 36/255, 1)
    circleColor = (158/255, 42/255, 43/255, 0.5)

    for i in range(int(squareGridOrder)):
        for j in range(int(squareGridOrder)):
            checkPoint = (0,0)
            squareCenter = ((i*sideLength)+0.5, (j*sideLength)+0.5)

            #find if square is within circle
            if(squareCenter[0] < circleCenter[0]):
                if(squareCenter[1] < circleCenter[1]):
                    checkPoint = (squareCenter[0] + 0.5*sideLength, squareCenter[1] + 0.5*sideLength)
                elif (squareCenter[1] == circleCenter[1]):
                    checkPoint = (squareCenter[0] + 0.5*sideLength, squareCenter[1])
                else:
                    checkPoint = (squareCenter[0] + 0.5*sideLength, squareCenter[1] - 0.5*sideLength)

            elif (squareCenter[0] == circleCenter[0]):
                if (squareCenter[1] < circleCenter[1]):
                    checkPoint = (squareCenter[0], squareCenter[1] + 0.5 * sideLength)
                elif (squareCenter[1] == circleCenter[1]):
                    checkPoint = (squareCenter[0], squareCenter[1])
                else:
                    checkPoint = (squareCenter[0], squareCenter[1] - 0.5 * sideLength)

            else:
                if(squareCenter[1] < circleCenter[1]):

                    checkPoint = (squareCenter[0] - 0.5*sideLength, squareCenter[1] + 0.5*sideLength)
                elif (squareCenter[1] == circleCenter[1]):
                    checkPoint = (squareCenter[0] - 0.5 * sideLength, squareCenter[1])
                else:

                    checkPoint = (squareCenter[0] - 0.5*sideLength, squareCenter[1] - 0.5*sideLength)

            if checkDistance(checkPoint, circleCenter) <= circleRadius:
                rectangle = plt.Rectangle((i * sideLength, j * sideLength), sideLength, sideLength, fc=squareValidColor,ec=squareValidBorderColor)
            else:
                rectangle = plt.Rectangle((i * sideLength, j * sideLength), sideLength, sideLength, fc=squareInvalidColor,ec=squareValidBorderColor)
                n -= 1
            plt.gca().add_patch(rectangle)

            # print("Distance between ", checkPoint, " and ", circleCenter, " is ", checkDistance(checkPoint, circleCenter))
            # print("Circle radius is ", circleRadius)
            # circle = plt.Circle(checkPoint, .1, color='green')
            # circle = plt.Circle(squareCenter, .1, color='green')
            # plt.gca().add_patch(circle)
            plt.axis('scaled')
    circle = plt.Circle(circleCenter, circleRadius, color=circleColor)
    plt.gca().add_patch(circle)
    plt.axis('scaled')
    plt.axis('off')
    print("Area efficiency with ", n, " cells in a ", squareGridOrder, " x " , squareGridOrder, " grid with ", (squareGridOrder**2) - n, " cells removed is ", calculateAreaEfficiency(n, circleRadius), "%")
    print("The circle radius is ", circleRadius)
    plt.show()


# def constraint1(squareGridOrder):
#     return [squareGridOrder - 2]
#
# def constraint2(squareGridOrder):
#     return [10 - squareGridOrder]

# con1 = {'type': 'ineq', 'fun': constraint1}
# con2 = {'type': 'ineq', 'fun': constraint2}
# cons = [con1, con2]
# cons = [con1]
# intialGridOrderGuess = 4

maxCells = 10000
minCells = 1
functionRanges = (slice(floor(sqrt(minCells)), floor(sqrt(maxCells)), 1),)
narray = []
nefficiency = []
# result = minimize(f, a0, constraints=cons, options = opt)
# result = minimize(findAreaEfficiencyFromGridOrder, intialGridOrderGuess, constraints=cons)
# result = minimize(findAreaEfficiencyFromGridOrder, intialGridOrderGuess)
result = brute(findAreaEfficiencyFromGridOrder, functionRanges, finish = None)

# print(result)

plt.plot(narray, nefficiency)
plt.xlabel('Number of Cells on Each Side')
plt.ylabel('Area Efficiency (%)')
plt.show()
# for k in range(3):
#     plotCurrentGrid(k+6)

plotCurrentGrid(result)

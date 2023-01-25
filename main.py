# Initial python script for optimization final project

from math import sqrt
import matplotlib.pyplot as plt
from scipy.optimize import minimize
import numpy as np

sideLength = 1

def checkDistance(point1, point2):
    return sqrt(((point2[0]-point1[0])**2)+((point2[1]-point1[1])**2))

def plotCurrentGrid(squareGridOrder):
    plt.axes()
    circleCenter = (sideLength*0.5*squareGridOrder, sideLength*0.5*squareGridOrder)
    circleRadius = sideLength*0.5*squareGridOrder
    squareValidColor = (199/255, 201/255, 199/255, 1)
    squareValidBorderColor = (0/255, 46/255, 96/255, 1)
    squareInvalidColor = (209/255, 65/255, 36/255, 1)
    circleColor = (158/255, 42/255, 43/255, 0.5)

    for i in range(squareGridOrder):
        for j in range(squareGridOrder):
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
                    print("center square found")
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
    plt.show()


# for k in range(3):
#     plotCurrentGrid(k+6)

plotCurrentGrid(11)

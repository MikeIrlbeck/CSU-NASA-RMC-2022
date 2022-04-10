#! /usr/bin/env python

import math


class point(object):
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw

    def __str__(self):
        return str(self.x) + ',' + str(self.y)


class MineLocationGenerator(object):
    class goal(object):
        def __init__(self, x, y, yaw):
            self.x = x
            self.y = y
            self.yaw = yaw

        def __str__(self):
            return str(self.x) + ', ' + str(self.y) + ', ' + str(self.yaw)

    def __init__(self, home):
        self.home = home

    def genLocations(self, distance, number):
        # self.x_array = range(0,distance,distance/20)
        self.x_array = list()
        i = 0.0
        while (i <= float(0.01 + distance)):
            self.x_array.append(i)
            i += float(distance/number)
            # print(i)
        for i in range(0,len(self.x_array)):
            self.x_array.append(-1.0 * self.x_array[i])
            # print(-1.0*self.x_array[i])

        self.goalList = list()
        negativeYGoalList = []
        for i in self.x_array:
            # the positive value
            y = math.sqrt(abs(math.pow(distance, 2) - i**2))
            yNeg = -1.0 * y
            yaw = math.atan2(y, i)
            yawNeg = math.atan2(yNeg, i)
 
            currentGoal = self.goal(self.home.x + i, self.home.y + y, yaw)
            negYGoal = self.goal(self.home.x + i, self.home.y + yNeg, yawNeg)
            self.goalList.append(currentGoal)
            negativeYGoalList.append(negYGoal)
            # print(i)
        self.goalList.extend(negativeYGoalList)

    def __str__(self):
        returnString = ''
        for i in self.goalList:
            returnString += str(i) + '\n'
        return returnString
if __name__ == '__main__':
    p1 = point(2,3, 0)
    print(p1)
    myGen = MineLocationGenerator(p1)
    myGen.genLocations(4, 5)
    print(myGen)

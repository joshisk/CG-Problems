class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class Box():
    def __init__(self, name, PointA, PointB):
        self.name = name
        self.pointA = PointA
        self.pointB = PointB

    # Function to check whether a point lies inside, on or outside the box
    # Checks if the point coordinates lie within the bounds of the box
    def PointBoxTest(self, PointQuery):
        if self.pointA.x < PointQuery.x < self.pointB.x and \
                self.pointA.y < PointQuery.y < self.pointB.y and \
                self.pointA.z < PointQuery.z < self.pointB.z:
            return 'point is inside the box'
        elif (self.pointA.x == PointQuery.x or self.pointB.x == PointQuery.x) or \
                (self.pointA.y == PointQuery.y or self.pointB.y == PointQuery.y) or \
                (self.pointA.z == PointQuery.z or self.pointB.z == PointQuery.z):
            return 'point is on the box face'
        else:
            return 'point is outside the box'

    # Because the box is assumed as axis aligned, the normal of each box face is parallel to the coordinate axes
    # Dot product of box vertices with the box normals (X, Y and Z axes)
    def Projections(self, axisx, axisy, axisz):
        minx = self.pointA.x * axisx.x + self.pointA.y * axisx.y + self.pointA.z * axisx.z
        maxx = self.pointB.x * axisx.x + self.pointB.y * axisx.y + self.pointB.z * axisx.z

        miny = self.pointA.x * axisy.x + self.pointA.y * axisy.y + self.pointA.z * axisy.z
        maxy = self.pointB.x * axisy.x + self.pointB.y * axisy.y + self.pointB.z * axisy.z

        minz = self.pointA.x * axisz.x + self.pointA.y * axisz.y + self.pointA.z * axisz.z
        maxz = self.pointB.x * axisz.x + self.pointB.y * axisz.y + self.pointB.z * axisz.z

        return [minx, maxx, miny, maxy, minz, maxz]

    # Separating axis test is applied
    # Corresponding projected values of box1 and box2 vertices on each coordinate axis are compared
    # The projection axes can be any axes; This problem uses X, Y and Z axes
    def CollisionTesting(self, boxobj2):
        axisx = Point(1, 0, 0)
        axisy = Point(0, 1, 0)
        axisz = Point(0, 0, 1)
        proj1 = self.Projections(axisx, axisy, axisz)
        proj2 = boxobj2.Projections(axisx, axisy, axisz)

        # For X-axis: proj1[0] = box1min and proj1[1] = box1max;  proj2[0] = box2min and proj2[1] = box2max
        # For Y-axis: proj1[2] = box1min and proj1[3] = box1max;  proj2[2] = box2min and proj2[3] = box2max
        # For Z-axis: proj1[4] = box1min and proj1[5] = box1max;  proj2[4] = box2min and proj2[5] = box2max

        # Algorithm: The following conditions should satisfy for all normals of box faces
        # In our case, since the boxes are axis aligned, the normals are X, Y and Z axes for each box
        # (box2max >= box1min >= box2min) or (box1max >= box2min >= box1min)
        if ((proj2[1] >= proj1[0] >= proj2[0]) or (proj1[1] >= proj2[0] >= proj1[0])) and \
                ((proj2[3] >= proj1[2] >= proj2[2]) or (proj1[3] >= proj2[2] >= proj1[2])) and \
                ((proj2[5] >= proj1[4] >= proj2[4]) or (proj1[5] >= proj2[4] >= proj1[4])):
            print('collision')
            return True
        else:
            print('no collision')
            return False

    # Function iterates through points array and for each point and the current box object, PointBoxTest is performed
    # If the point is inside or on the box, the dictionary is appended with the box object name as the key and the
    # list of point objects as its value There could be more than one point in the same box, therefore, the value is
    # defined as a list
    def BoxPointsAssociation(self, pointsarray):
        pointlist = []
        associationDictionary = {self.name: pointlist}
        for point in pointsarray:
            res = self.PointBoxTest(point)
            if res == 'point is inside the box' or res == 'point is on the box face':
                coordinates = (point.x, point.y, point.z)
                pointlist.append(coordinates)
                associationDictionary[self.name] = pointlist
                continue
            else:
                continue
        return associationDictionary


# Test cases for each problem

PointA = Point(0, 0, 0)
PointB = Point(5, 5, 5)
boxobj = Box('boxobj', PointA, PointB)
# point inside the box
PointQuery1 = Point(1, 1, 1)
string1 = boxobj.PointBoxTest(PointQuery1)
print(string1)
# point on the face of the box
PointQuery2 = Point(3, 3, 0)
string2 = boxobj.PointBoxTest(PointQuery2)
print(string2)
# point outside the box
PointQuery3 = Point(6, 6, 6)
string3 = boxobj.PointBoxTest(PointQuery3)
print(string3)

print('---------------------------------------------')

PointC = Point(0, 0, 0)
PointD = Point(5, 5, 5)
boxobj1 = Box('boxobj1', PointC, PointD)
# collision case
PointE = Point(3, 0, 0)
PointF = Point(8, 5, 5)
boxobj2 = Box('boxobj2', PointE, PointF)
boxobj1.CollisionTesting(boxobj2)
# boxes touching is a collision case
PointG = Point(5, 5, 5)
PointH = Point(15, 15, 15)
boxobj3 = Box('boxobj3', PointG, PointH)
boxobj1.CollisionTesting(boxobj3)
# no collision case
PointG = Point(6, 6, 6)
PointH = Point(15, 15, 15)
boxobj4 = Box('boxobj4', PointG, PointH)
boxobj1.CollisionTesting(boxobj4)

print('---------------------------------------------')

# box with no points in or on it
PointMin1 = Point(0, 0, 0)
PointMax1 = Point(5, 5, 5)
box1 = Box('box1', PointMin1, PointMax1)
# box with one point in it
PointMin2 = Point(0, 10, 0)
PointMax2 = Point(5, 15, 5)
box2 = Box('box2', PointMin2, PointMax2)
# box with 2 points in it
PointMin3 = Point(0, 0, 10)
PointMax3 = Point(5, 5, 15)
box3 = Box('box3', PointMin3, PointMax3)
# Define points
Point1 = Point(3, 12, 3)  # point belongs to box2
Point2 = Point(3, 3, 12)  # point belongs to box3
Point3 = Point(2, 2, 12)  # point belongs to box3
Point4 = Point(100, 100, 100)  # point is not in dictionary
PointsArray = [Point1, Point2, Point3, Point4]

# box array
boxes = [box1, box2, box3]
print('Box name and its associated Points:')
for box in boxes:
    associationdictionary = box.BoxPointsAssociation(PointsArray)
    print(associationdictionary)

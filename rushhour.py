## rushhour.py
import copy
# import sys
# sys.setrecursionlimit(10000)
## plan: have to check for cycles. 
## make use of tilepuzzle code to apply recursion
## prioritize moving main car towards goal
## new heuristic will be number of spaces left to goal. it should prioritize moving to goal so that 


# This is the starting function that is called and let's the whole process run.
def rushhour(heuristic,start):
    vehicaldirection = cardirection(start)
    path, numStatesExplored = statesearch(start,[],[],heuristic, vehicaldirection, 0, [])
    # Reverse is needed because the path is appended from goal and backwards
    path = reverse(path)
    printPath(path)
    print("Total moves: " + str(len(path)-1))
    print("Total states explored: " + str(numStatesExplored))
    return (path)

#This function is used to print out the states in the path after completion. 
def printPath(path):
    for state in path:
        for row in state:
            print(row)
        print("\n")
    return path


## This creates and returns a dictionary for all vehicles and the directions of their movement. This will be used when gen
def cardirection(start):
    carDetails = {"X":"h"}
    for row in start:
        cars = []
        for tile in range(0,len(row)):
            if row[tile] == "-" or carDetails.get(row[tile]) != None:
                continue
            cars.append(row[tile])
        
        if not cars:
            continue
        before = ""
        for car in cars:
            if car == before and carDetails[car] != "h":
                carDetails[car] = "h"
            before = car
            if carDetails.get(car) == None:
                carDetails[car] = "v"
    return carDetails
            
# This is the A* search algorithm. This is an iterative version. It uses generateNewStates to generate new states and heuristic sort to apply heuristics and sort them by f(n).
# Other than the normal A* search algorithm, the path is used to keep track of the popped front nodes from the frontier. 
# Pathfinder is used after finishing the A* algorithm to discern the optimal path from the nodes in path.
def statesearch(startNode,frontier,path,heuristic,carDetails, numStatesExplored, statesExplored):
    path.append((startNode,blockingHeuristic(startNode),0))
    statesExplored.append(startNode)
    moveNum = 1
    newStates = generateNewStates(startNode, carDetails, statesExplored)
    for stateNew in newStates:
        if stateNew not in statesExplored:
            statesExplored.append(stateNew)
    frontier = heuristicSort(newStates, frontier, moveNum, heuristic)
    # print(path)
    # print(frontier)
    
    while True:
        if frontier == []:
            return [], statesExplored
        frontNode = frontier.pop(0)
        path.append(frontNode)
        # print("path")
        # print(path)
        statesExplored.append(frontNode[0])
        if frontNode[0][2].find("XX") == 4:
            # print("win")
            break
        else:
            # moveNum+=1
            newStates = generateNewStates(frontNode[0], carDetails, statesExplored)
            for stateNewTwo in newStates:
                if stateNewTwo not in statesExplored:
                    statesExplored.append(stateNewTwo)
            frontier = heuristicSort(newStates, frontier, frontNode[2]+1, heuristic)
            # print("frontier")
            # print(frontier)
    
    numStatesExplored = len(path)-1
    truePath = pathfinder(path,carDetails)
    
    return truePath, numStatesExplored

# Pathfinder takes the popped nodes from the stateSearch and traverses from the goal state backwards to find the optimal path. FluidityCheck ensures that the path is coherent
def pathfinder(path,carDetails):
    truePath = []
    truePathCounter = 0
    pathCounter = len(path)-1
    moveNum = copy.deepcopy(path[len(path)-1][2])

    truePath.append(copy.deepcopy(path[pathCounter][0]), )
    moveNum -= 1
    pathCounter -= 1

    while pathCounter >= 0:
        if path[pathCounter][2] == moveNum:
            if fluidityCheck(path[pathCounter][0], carDetails, truePath[truePathCounter]):
                truePath.append(copy.deepcopy(path[pathCounter][0]))
                moveNum-=1
                truePathCounter+=1
        pathCounter-=1
    # print("truepath")
    # print(truePath)
    return truePath

# Heuristic sort applies a heuristic and a g(n). Along with the state in question, they form a tuple. The frontier is sorted by h(n) + g(n).
# A 0 for heuristic uses just the blocking heuristic. 1 adds on my special heuristic.
def heuristicSort(newStates,frontier,moveNum, heuristic):
    newFrontier = copy.deepcopy(frontier)
    if newStates == []:
        return newFrontier
    for state in newStates:
        if heuristic == 0:
            heuristicValue = blockingHeuristic(state) 
        else:
            heuristicValue = blockingHeuristic(state) + disFromGoalHeuristic(state)
        newFrontier.append((copy.deepcopy(state),heuristicValue, moveNum))
    if type(newFrontier[0][2]) != str:
        newFrontier.sort(key=gnPlushn)
    

    return newFrontier

# This is used for the sort function's key
def gnPlushn(state):
    return state[1]+state[2]


## This is the blocking heuristic. It checks the state's row with XX and checks for vehicles blocking the way.
def blockingHeuristic(state):
    lane = state[2]
    vehicles = ["X"]
    if lane[4] == "X" and lane[5] == "X":
        return 0
    for spot in range(lane.find("XX")+2, len(lane)):
        if lane[spot] == "-":
                continue
        unique = True
        for vehicle in vehicles:
            if lane[spot] == vehicle:
                unique = False
        if unique:
            vehicles.append(lane[spot])
    # print(len(vehicles))
    return len(vehicles)

## This is the distance from goal heuristic. It is combined with the blocking heuristic to create a new heuristic.    
def disFromGoalHeuristic(state):
    lane = state[2]
    return 4 - lane.find("XX")


##-- Here are some simple utility functions to give names to
##-- some of the list operations (borrowed from functional
##-- programming):

def reverse(st):
    return st[::-1]
    
def head(lst):
    return lst[0]

def tail(lst):
    return lst[1:]

def take(n,lst):
    return lst[0:n]

def drop(n,lst):
    return lst[n:]

def cons(item,lst):
    return [item] + lst


## Idea (check if in last string, if not, then find first occurrence, and next instances. call a function to replace the two needed spots)
def generateDownSlide(state, vehicle):
    carRow = []
    carColumn = -1
    if state[5].find(vehicle) != -1:
        return carRow
    for rowNum in range(0,5):
        position = state[rowNum].find(vehicle)
        if position != -1:
            carRow.append(rowNum)
            carColumn = position
    if state[carRow[len(carRow)-1]+1][carColumn] != "-":
        return []
    newState = copy.deepcopy(state)
    if carColumn != 5:
        newState[carRow[0]] = newState[carRow[0]][:carColumn] + "-" + newState[carRow[0]][carColumn+1:]
        newState[carRow[len(carRow)-1]+1] = newState[carRow[len(carRow)-1]+1][:carColumn] + vehicle + newState[carRow[len(carRow)-1]+1][carColumn+1:]
    else:
        newState[carRow[0]] = newState[carRow[0]][:carColumn] + "-"
        newState[carRow[len(carRow)-1]+1] = newState[carRow[len(carRow)-1]+1][:carColumn] + vehicle
    # print("down")
    # print(newState)
    return newState

# Self explanatory function that generates a slide upwards for a vehicle if the state is possible.
def generateUpSlide(state, vehicle):
    carRow = []
    carColumn = -1
    if state[0].find(vehicle) != -1:
        return carRow
    for rowNum in range(1,6):
        position = state[rowNum].find(vehicle)
        if position != -1:
            carRow.append(rowNum)
            carColumn = position
    if state[carRow[0]-1][carColumn] != "-":
        return []
    newState = copy.deepcopy(state)
    if carColumn != 5:
        newState[carRow[0]-1] = newState[carRow[0]-1][:carColumn] + vehicle + newState[carRow[0]-1][carColumn+1:]
        newState[carRow[len(carRow)-1]] = newState[carRow[len(carRow)-1]][:carColumn] + "-" + newState[carRow[len(carRow)-1]][carColumn+1:]
    else:
        newState[carRow[0]-1] = newState[carRow[0]-1][:carColumn] + vehicle
        newState[carRow[len(carRow)-1]] = newState[carRow[len(carRow)-1]][:carColumn] + "-" 
    # print("up")
    # print(newState)
    return newState

# Self explanatory function that generates a slide leftwards for a vehicle if the state is possible.
def generateLeftSlide(state, vehicle):
    car = vehicle + vehicle
    truck = vehicle + vehicle + vehicle
    vehiclePos = -1
    vehicleRow = -1
    vehicleType = "car"
    for rowNum in range(0,6):
        position = state[rowNum].find(truck)
        if position == 0:
            return []
        elif position != -1:
            vehiclePos = position
            vehicleRow = rowNum
            vehicleType = "truck"
            break
        position = state[rowNum].find(car)
        if position == 0:
            return []
        elif position != -1:
            vehiclePos = position
            vehicleRow = rowNum
            break
    
    if vehiclePos == -1 or vehicleRow == -1 or state[vehicleRow][vehiclePos-1] != "-":
        return []
    newState = copy.deepcopy(state)
    if vehicleType == "truck": 
        newState[vehicleRow] = newState[vehicleRow][:vehiclePos-1] + vehicle + newState[vehicleRow][vehiclePos:vehiclePos+2] + "-" + newState[vehicleRow][vehiclePos+3:]
    elif vehicleType == "car":
        newState[vehicleRow] = newState[vehicleRow][:vehiclePos-1] + vehicle + newState[vehicleRow][vehiclePos:vehiclePos+1] + "-" + newState[vehicleRow][vehiclePos+2:]
    # print("left")
    # print(newState)
    return newState

# Self explanatory function that generates a slide rightwards for a vehicle if the state is possible.
def generateRightSlide(state, vehicle):
    car = vehicle + vehicle
    truck = vehicle + vehicle + vehicle
    vehiclePos = -1
    vehicleRow = -1
    vehicleType = "car"
    for rowNum in range(0,6):
        position = state[rowNum].find(truck)
        
        if position != -1:
            if position == 3:
                return []
            else:
                vehiclePos = position
                vehicleRow = rowNum
                vehicleType = "truck"
                break
        position = state[rowNum].find(car)
        
        if position != -1:
            if position == 4:
                return []
            else:
                vehiclePos = position
                vehicleRow = rowNum
                break
    
    if vehiclePos == -1 or vehicleRow == -1 or (vehicleType == "truck" and state[vehicleRow][vehiclePos+3] != "-") or (vehicleType == "car" and state[vehicleRow][vehiclePos+2] != "-"):
        return []
    newState = copy.deepcopy(state)
    if vehicleType == "truck": 
        newState[vehicleRow] = newState[vehicleRow][:vehiclePos] + "-" + newState[vehicleRow][vehiclePos:vehiclePos+2] + vehicle + newState[vehicleRow][vehiclePos+4:]
    elif vehicleType == "car": 
        newState[vehicleRow] = newState[vehicleRow][:vehiclePos] + "-" + newState[vehicleRow][vehiclePos:vehiclePos+1] + vehicle + newState[vehicleRow][vehiclePos+3:]
    # print("right")
    # print(newState)
    return newState
    






# This function generates new states for a given state. To prevent cycling, it will not generate a state if it has ever been in the frontier. 
def generateNewStates(currState, carDetails, statesExplored):
    # print("current")
    # print(currState)
    vehicleList = carDetails.keys()
    newStates = []
    for vehicle in vehicleList:
        if carDetails[vehicle] == "h":
            newState = generateRightSlide(currState, vehicle)
            if newState != [] and not(newState in statesExplored):
                newStates.append(newState)
            newState = generateLeftSlide(currState, vehicle)
            if newState != [] and not(newState in statesExplored):
                newStates.append(newState)
        elif carDetails[vehicle] == "v":
            newState = generateUpSlide(currState, vehicle)
            if newState != [] and not(newState in statesExplored):
                newStates.append(newState)
            newState = generateDownSlide(currState, vehicle)
            if newState != [] and not(newState in statesExplored):
                newStates.append(newState)
    # print("new states")
    # print(newStates)
    return newStates

# This works similarly to generate new states, but will return true or false depending on if the checkState is one of the functions that can be generated from prev funciton. 
def fluidityCheck(prevState, carDetails, checkState):
    vehicleList = carDetails.keys()
    newStates = []
    for vehicle in vehicleList:
        if carDetails[vehicle] == "h":
            newState = generateRightSlide(prevState, vehicle)
            if newState != []:
                newStates.append(newState)
            newState = generateLeftSlide(prevState, vehicle)
            if newState != []:
                newStates.append(newState)
        elif carDetails[vehicle] == "v":
            newState = generateUpSlide(prevState, vehicle)
            if newState != []:
                newStates.append(newState)
            newState = generateDownSlide(prevState, vehicle)
            if newState != []:
                newStates.append(newState)
    if checkState in newStates:
        return True
    else:
        return False


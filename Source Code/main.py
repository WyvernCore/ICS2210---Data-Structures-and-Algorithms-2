from random import randrange
from random import randint
import collections

states = randrange(16,64)
allNodes = [i for i in range(states)]

print("")
print("------- Task 1 -------")
print("")
print("Number of nodes in DFA: "+str(len(allNodes)))

accepting_states = []
count = 0
flips = [randint(0,1) for r in range(states)]

for k in flips:
    if k == True:
        accepting_states.append(count)
    count+=1

def add_node(node, nodeList, adjacencyList):  # initilising nodes

    if node not in adjacencyList:
        nodeList.append(node)
    else:
        print("Error: ", node, " already preset")

    return nodeList, adjacencyList


def add_edge(firstNode, secondNode, w, nodeList, adjacencyList):  # adding edges
    tempStore = [] #TempStore used to append the edge values to the dictionary

    if firstNode in nodeList and secondNode in nodeList:
        if firstNode not in adjacencyList:
            tempStore.append([secondNode, w])
            adjacencyList[firstNode] = tempStore

        elif firstNode in adjacencyList:
            tempStore.extend(adjacencyList[firstNode]) #get previous edge
            tempStore.append([secondNode, w]) #add another edge to list of edges
            adjacencyList[firstNode] = tempStore

        else:
            print("Node not present")
    return nodeList, adjacencyList

def printNetwork(adjacencyList):
    for node in adjacencyList:
        print(node, " ---> ", [i for i in adjacencyList[node]])

myAdjacency = {}
myNode = []

for r in range(states):
    myNode, myAdjacency = add_node(r, myNode, myAdjacency)

for r in range(states): #Each state will have two outgoing edges to other random states
    outgoing = randrange(0, states - 1)
    outgoing2 = randrange(0, states - 1)

    myNode, myAdjacency = add_edge(r, outgoing, "a", myNode, myAdjacency)
    myNode, myAdjacency = add_edge(r, outgoing2, "b", myNode, myAdjacency)

print("DFA pre-Minimization:")
printNetwork(myAdjacency)
print("Accepting states: " +str(accepting_states))

visited = []  # List to keep track of visited nodes.

def bfs(visited, graph, node):
    depth = -1
    visited.append(node)
    queue = collections.deque()
    queue.append(node)
    while queue:
        currSize = len(queue)
        while currSize > 0:
            s = queue.popleft()
            currSize -= 1
            for neighbour in graph[s]:
                if neighbour[0] not in visited:
                    visited.append(neighbour[0])
                    queue.append(neighbour[0])

        depth += 1
    return depth, visited


# Driver Code
depthList, visited = bfs(visited, myAdjacency, 0)

print("")
print("------- Task 2 -------")
print("")
print("Number of states in network:", states)
print("Depth of Network:", depthList)

visited.sort()  # Deleting unreachable states
unreachable = list(set(allNodes) - set(visited))

for r in unreachable:
    del myAdjacency[r]


newFinal = list(set(accepting_states) - set(unreachable))  # Computing new final states
newNodes = list(set(allNodes) - set(unreachable))  # Retrieving all the new nodes

def partition(Final, allNodes):
    nonFinal = list(set(allNodes) - set(Final))
    return ([Final, nonFinal])


def HopCroft(adjacency, accepting, AllNodes):
    P = partition(accepting, AllNodes)  # Initial Partition
    while (True):  # keep looping till no change in partition occurs
        currP = P
        P = []
        for eachSet in currP:  # Go over every subset in the partition
            Pminor = []
            firstValue = eachSet[0]
            Pminor.append([firstValue])
            for secondValue in eachSet[1:]:
                flag = False
                y1 = adjacency[secondValue][0][0]
                y2 = adjacency[secondValue][1][0]
                for subSet in Pminor:
                    flagone = False
                    flagtwo = False
                    for s in currP:
                        x1 = adjacency[subSet[0]][0][0]
                        x2 = adjacency[subSet[0]][1][0]
                        if x1 in s and y1 in s:
                            flagone = True
                        if x2 in s and y2 in s:
                            flagtwo = True
                        if flagone and flagtwo:
                            subSet.append(secondValue)
                            flag = True
                            break
                        if flag==True:
                            break
                if not flag:
                    Pminor.append([secondValue])

            P.extend(Pminor)

        if currP == P:  # If partition doesnt change , finished
            break
    return P

def UpdateAdj(myDict, adj):
    upAdj = {}
    singlePar = []
    newTups = []
    for x in myDict:
        if len(x) > 1:  # Hopcroft's final partition has nodes partitioned together, thus combine nodes
            newNode = []

            for state in x:
                newNode.append(state)
            myTup = tuple(newNode)
            newTups.append(myTup)
            str = []

            for val in adj[myTup[0]]:
                if val[0] in myTup:
                    str.append([myTup, val[1]])
                else:
                    str.append([val[0], val[1]])
            upAdj[myTup] = str
        else:
            singlePar.append(x)

    for x in singlePar:
        str = []
        for val in adj[x[0]]:
            for s in newTups:
                if val[0] in s:
                    str.append([s, val[1]])
                    break
            else:
                str.append([val[0], val[1]])
        upAdj[x[0]] = str

    return (upAdj)
"""
adj = {
    "A": [["B", 0],["C",1]],
    "B": [["B", 0],["D",1]],
    "C": [["B", 0],["C",0]],
    "D": [["B", 0],["E",1]],
    "E": [["B", 0],["C",1]]
}

final = ["E"]
nNodes = ["A", "B", "C", "D", "E"]

ans = (HopCroft(adj, final, nNodes))
newAdj = UpdateAdj(ans, adj)

printNetwork(adj)
print("")
print(ans)
print("")
printNetwork(newAdj)

"""

FinalPartition = HopCroft(myAdjacency, newFinal, newNodes)

print("")
print("------- Task 3 -------")
print("")

mergedAdj = UpdateAdj(FinalPartition, myAdjacency)

printNetwork(mergedAdj)

print("")
print("------- Task 4 -------")
print("")

visited = []
depthList, visited = bfs(visited, mergedAdj, list(mergedAdj.keys())[0])
print("Number of states in network:", len(mergedAdj))
print("Depth of Network:", depthList)


def tarjan(V):

    indexCounter = [0]
    stack = []
    result = []
    lowLinks = {}
    index = {}

    def strongConnected(v):

        index[v] = indexCounter[0]
        lowLinks[v] = indexCounter[0]
        indexCounter[0] += 1
        stack.append(v)

        try:
            outDegree = [item[0] for item in V[v]] #For every Out-Degree nodes
        except:
            outDegree = [] #No out degrees present

        for newNode in outDegree:
            if newNode not in lowLinks:
                strongConnected(newNode) #If successor not searched, recurse
                lowLinks[v] = min(lowLinks[v], lowLinks[newNode])
            elif newNode in stack:
                lowLinks[v] = min(lowLinks[v], index[newNode])

        if lowLinks[v] == index[v]:
            connectedComponent = []
            while True: #Get all the scc and append them together
                scc = stack.pop()
                connectedComponent.append(scc)
                if scc == v:
                    break

            result.append(connectedComponent)

    for v in V:
        if v not in lowLinks:
            strongConnected(v)

    return result

print("")
print("------- Task 5 -------")
print("")
SCC = tarjan(mergedAdj)
list_len = [len(i) for i in SCC]

print("Number of SCC: " +str(len(SCC)))
print("Largest SCC in M: "+str(max(list_len)))
print("Smallest SCC in M: "+str(min(list_len)))

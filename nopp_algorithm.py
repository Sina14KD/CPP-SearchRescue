# -*- coding: utf-8 -*-

import copy
import math
import matplotlib.pyplot as plt
import numpy as np
import time

# phase one -> generating the initial path
def phaseOne_initialPath(startPoint,num_S_Moves,num_P_moves,feasibleSet,numAssignedCell):

    dPath=[]
    pTotalOdd=False
    upMoves=0
    downMoves=0

    currentCell=startPoint[:]
    path=[currentCell]
    feasibleSet.remove(currentCell)
    numAssignedCell-=1
    if num_P_moves%2!=0 : pTotalOdd=True

    for i in range(numAssignedCell):

        next_D_Move=[currentCell[0],currentCell[1]-1]
        next_U_Move=[currentCell[0],currentCell[1]+1]
        next_S_Move=[currentCell[0]+1,currentCell[1]]

        # check if D move is feasible
        if next_D_Move in feasibleSet and num_P_moves>0:

            currentCell=next_D_Move
            path.append(currentCell)
            feasibleSet.remove(currentCell)
            num_P_moves-=1
            downMoves+=1
            dPath.append('D')

        # check if S move is feasible
        elif next_S_Move in feasibleSet:

            currentCell=next_S_Move
            path.append(currentCell)
            feasibleSet.remove(currentCell)
            dPath.append('S')

        # check if U move is feasible
        elif next_U_Move in feasibleSet and num_P_moves>0 and upMoves<downMoves:

            # if total number of P move is odd, the algorithm do the following if statement to ensure that the remaining P moves is even
            if pTotalOdd:

                if upMoves<downMoves-1:

                    currentCell=next_U_Move
                    path.append(currentCell)
                    feasibleSet.remove(currentCell)
                    num_P_moves-=1
                    upMoves+=1
                    dPath.append('U')

            else:

                currentCell=next_U_Move
                path.append(currentCell)
                feasibleSet.remove(currentCell)
                num_P_moves-=1
                upMoves+=1
                dPath.append('U')

        else:

            continue

    return path,dPath,feasibleSet,num_P_moves

# phase two -> ensure that always h-index is even number, and works when the initial path terminated with U moves
def phaseTwo_gauranteeVallyIsEven(startPoint,path,dPath,feasibleSet):

    pathCopy=copy.deepcopy(path)
    ll=len(dPath)
    uIndex=len(dPath)-1
    uCount=0
    sIndex=0
    sCount=0
    size=0
    numOfU_Deleted=0

    if dPath[-1]=='U':
        while uIndex>=0:
            if dPath[uIndex]!='U':
                break
            uIndex-=1
            uCount+=1

        sIndex=uIndex
        while sIndex>=0:
            if dPath[sIndex]!='S':
                break
            sIndex-=1
            sCount+=1

        if dPath[sIndex]=='D':
            size=sCount-1
        else:
            size=sCount

        if size%2!=0:
            dPath=dPath[:uIndex+1]
            pathCopy=pathCopy[:uIndex+2]

            for n in path:

                if n not in pathCopy:
                    feasibleSet.append(n)

            numOfU_Deleted=uCount


    return  pathCopy,dPath,feasibleSet,numOfU_Deleted


# phase three is responsible to put the remaining P moves in the initial path
def phaseThree_PlacingOther_P_Moves(startPoint,path,dPath,feasibleSet,numOfRemained_P_Moves):
    index=-1

    for i in range(len(dPath)):

        if i>index:
            if dPath[i]=='S':
                if dPath[i-1]=='U' or dPath[i-1]=='S':

                    currentCell=path[i]
                    nextCandidCell=[currentCell[0],currentCell[1]+1]

                    if nextCandidCell in feasibleSet and nextCandidCell[1]<=startPoint[1]:

                        updateDpath(dPath, i, 'U')
                        path.insert(i+1,[currentCell[0]+1,currentCell[1]+1])
                        path.insert(i+1,[currentCell[0],currentCell[1]+1])
                        feasibleSet.remove([currentCell[0]+1,currentCell[1]+1])
                        feasibleSet.remove([currentCell[0],currentCell[1]+1])
                        numOfRemained_P_Moves-=2
                        index=i
                        break
                    else:
                        continue


    return path,dPath,feasibleSet,numOfRemained_P_Moves

# phase four is responsible to take all the remaining cells by the last drone
def phaseFour_FinalCheck(path,dPath,feasibleSet,numOfRemained_P_Moves):

    if len(feasibleSet)!=0:

        for i in range(len(feasibleSet)):

            lastCell=path[-1]
            nextU_Candid=[lastCell[0],lastCell[1]+1]

            if nextU_Candid in feasibleSet:
                dPath.append('U')
                path.append(nextU_Candid)

    return path,dPath,feasibleSet

def initialSettings(n,m,q):
    startPointsList=[]
    numAssignedCellsList=[]
    num_P_Moves_List=[]
    feasibleSet=[]

    for i in range(n):
        for j in range(m):
            c=[i+1,j+1]
            feasibleSet.append(c)

    for i in range(q):

        i+=1
        c=[1,m-i+1]
        startPointsList.append(c)

    startPointsList.reverse()

    num_P_Moves=math.ceil((m*n)/q)-n
    num_S_Moves=n-1
    maxNumAssignedCells=num_P_Moves+num_S_Moves+1
    numAssignedCellsList=[maxNumAssignedCells]*q
    num_P_Moves_List=[num_P_Moves]*q
    #print("note - SP: {}- S : {}- P: {}- F: {}- A: {}".format(startPointsList,num_S_Moves,num_P_Moves_List,feasibleSet,numAssignedCellsList))
    return startPointsList,num_S_Moves,num_P_Moves_List,feasibleSet,numAssignedCellsList




# generate the path for a drone and works based on the flowchart in the paper
def generateSingleDronePath(startPoint,num_S_Moves,num_P_moves,feasibleSet,numAssignedCell,isLastDrone):

    path2,dPath2,feasibleSet2,num_P_Moves2=phaseOne_initialPath(
        startPoint, num_S_Moves, num_P_moves,feasibleSet, numAssignedCell)

    path3,dPath3,feasibleSet3,numDeletedUpMoves3=phaseTwo_gauranteeVallyIsEven(
        startPoint, path2, dPath2, feasibleSet2)

    num_P_Moves3=num_P_Moves2+numDeletedUpMoves3

    # if the remaining P moves for the phase 3 is not even, the algorithm add one more P moves to make it even.
    # this section cause to generate a solution with LB+T_p in some cases.
    if num_P_Moves3%2==0:
        halfnum_P_Moves3=int(num_P_Moves3/2)
    else:

        num_P_Moves3+=1
        halfnum_P_Moves3=int(num_P_Moves3/2)

    for i in range(halfnum_P_Moves3):

        path3,dPath3,feasibleSet3,num_P_Moves3=phaseThree_PlacingOther_P_Moves(
            startPoint, path3, dPath3, feasibleSet3, num_P_Moves3)

    if isLastDrone:

        path4,dPath4,feasibleSet4=phaseFour_FinalCheck(
            path3, dPath3, feasibleSet3, num_P_Moves3)

        return path4,dPath4,feasibleSet4

    else:

        return path3,dPath3,feasibleSet3

# generate paths for all drones
def generateAllDronesPath(n,m,q):

    TotalStepsAns=[]
    finalAns=[]
    startPointsList,num_S_Moves,num_P_Moves,feasibleSet,numAssignedCell=initialSettings(n, m, q)

    for i in range(q):

        if i==q-1:
            path,dPath,feasibleSet=generateSingleDronePath(
                startPointsList[i], num_S_Moves, num_P_Moves[i], feasibleSet, numAssignedCell[i],True)

            finalAns.append(path)

        else:
            path,dPath,feasibleSet=generateSingleDronePath(
                startPointsList[i], num_S_Moves, num_P_Moves[i], feasibleSet, numAssignedCell[i],False)

            finalAns.append(path)

    return finalAns

# ******************* Utils **************************************************
def updateDpath(dPath,index_S_Move,previousUpdated_S_Move):

    if previousUpdated_S_Move=='U':

        dPath.insert(index_S_Move+1,'D')
        dPath.insert(index_S_Move,'U')

    elif previousUpdated_S_Move=='D':

        dPath.insert(index_S_Move+1,'U')
        dPath.insert(index_S_Move,'D')

    else:
        raise Exception('Wrong inputs for updateDpath Method')

    return dPath

'''
Calculate the lower bound based on proposition 2 in the paper

n: Number of cells along the length (X-axis) of the search area
m: Number of cells along the width (Y-axis) of the search area
q: Number of UAVs (Drones)
v: Airspeed of UAVs (m/s)
w: Wind speed (m/s)
d: Size of the cell's edge (m)
'''
def calculate_LB(n,m,q,v,w,d):
    T_s=d/(v+w)
    T_p=d/math.sqrt(v**2-w**2)
    LB=(n-1)*T_s+(math.ceil(m*n/q)-n)*T_p
    PT=math.ceil(m*n/q)-n
    return LB

'''
Calculate the operation time (coverage time required by the NOPP solution)

ans: Solution generated by the NOPP algorithm
v: Airspeed of UAVs (m/s)
w: Wind speed (m/s)
d: Size of the cell's edge (m)
'''
def totalObjectValue(ans,v,w,d):
    allObjectValues=[]
    for i in range(len(ans)):
        o= objectiveFunctionValueForDrone(ans[i],v,w,d)
        allObjectValues.append(o)
    return max(allObjectValues)

'''
Calculate the mission time for each UAV

ans: Path generated by the NOPP algorithm for a UAV
v: Airspeed of UAVs (m/s)
w: Wind speed (m/s)
d: Size of the cell's edge (m)
'''
def objectiveFunctionValueForDrone(ans,v,w,d):
    T_s=d/(v+w)
    T_p=d/math.sqrt(v**2-w**2)
    sMoves=0
    pMoves=0
    for i in range(len(ans)):
        if i!=len(ans)-1:
            nxtS=[ans[i][0]+1,ans[i][1]]
            nxtUp=[ans[i][0],ans[i][1]+1]
            nxtDown=[ans[i][0],ans[i][1]-1]
            if ans[i+1]==nxtS:
                sMoves+=1
            elif ans[i+1]==nxtUp or ans[i+1]==nxtDown:
                pMoves+=1
    return sMoves*T_s+pMoves*T_p

# ******************* Plot **************************************************
def plotPath(ans,n,m):

    colors = ['#0000ff', '#ff3300', '#00cc00', '#031971','#ff8000',
              '#00BDA6','#212F3D','#A4557F','#611E78','#5571A4']*(len(ans)//10+1)

    X_axis=[i-0.5 for i in range(1, n+2)]
    Y_axis=[j-0.5 for j in range(1, m+2)]

    x=[]
    y=[]
    # seprate x and y

    for a in ans:
        c=[]
        d=[]
        for b in a:
            c.append(b[0])
            d.append(b[1])
        x.append(c)
        y.append(d)

    # plot
    fig = plt.figure(dpi=300)
    ax = fig.add_subplot(1, 1, 1)
    fig.set_size_inches(10, 10*m/n)   # Set the size of the figure
    ax.set_title('drone path - q : {}'.format(len(ans)))
    for i in range(len(ans)):
        ax.scatter(np.array(x[i]),np.array(y[i]),color='#F0F0F0',s=10*300/(n*m))

    # draw arrows
    for i in range(len(x)):
        for j in range(len(x[i])):
            if j!=len(x[i])-1:
                ax.annotate("",
                    xy=(x[i][j+1],y[i][j+1]), xycoords='data',
                    xytext=(x[i][j],y[i][j]), textcoords='data',
                    arrowprops=dict(arrowstyle="->",
                                    connectionstyle="arc3",color=colors[i]),
                  )

    xTick=[]
    yTick=[]

    xAxis=0
    yAxis=0

    for a in x:
        if max(a)>xAxis:
            xAxis=max(a)

    for b in y:
        if max(b)>yAxis:
            yAxis=max(b)

    for i in range(xAxis+2):
        if i!=0:
            xTick.append(i-0.5)

    for j in range(yAxis+2):
        if j!=0:
            yTick.append(j-0.5)

    plt.xticks(xTick)
    plt.yticks(yTick)


    plt.xlim(0.5, n + 0.5)
    plt.ylim(0.5, m + 0.5)
    plt.xticks(X_axis, labels=[])
    plt.yticks(Y_axis, labels=[])
    plt.xticks(rotation='vertical')
    plt.title('{}x{} Search area with {} UAVs'.format(n, m, len(ans)))
    plt.grid(True, color='#E9E9E9')
    #plt.grid()
    #plt.savefig("myimage.eps", dpi=1200)
    plt.show()

# ******************* Test **************************************************
'''
n: Number of cells along the length (X-axis) of the search area
m: Number of cells along the width (Y-axis) of the search area
q: Number of UAVs (Drones)
v: Airspeed of UAVs (m/s)
w: Wind speed (m/s)
d: Size of the cell's edge (m)
'''

n,m,q=10,10,3
v,w,d=20,5,100

ans=generateAllDronesPath(n,m,q)

print('==========Results==========')
print('LB is : ', calculate_LB(n,m,q,v,w,d))
print('---------------------------')
print('Operation time is : ',totalObjectValue(ans,v,w,d))
print('---------------------------')
print('Solution is : ',ans)

# plot
plotPath(ans,n,m)

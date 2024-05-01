# -*- coding: utf-8 -*-
"""
install the following package

!pip install dash
!pip install dash-renderer
!pip install dash-html-components
!pip install dash-core-components
!pip install plotly
"""
import copy
import math
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
import time
import dash
from dash import dcc, html
import matplotlib.pyplot as plt
import numpy as np
from io import BytesIO
import base64
from dash.dependencies import Input, Output, State



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
    print("note - SP: {}- S : {}- P: {}- F: {}- A: {}".format(startPointsList,num_S_Moves,num_P_Moves_List,feasibleSet,numAssignedCellsList))
    return startPointsList,num_S_Moves,num_P_Moves_List,feasibleSet,numAssignedCellsList




# generate the path for a drone and works based on the flowchart in the paper
def generateSingleDronePath(startPoint,num_S_Moves,num_P_moves,feasibleSet,numAssignedCell,isLastDrone):

    StepsAns=[]

    path2,dPath2,feasibleSet2,num_P_Moves2=phaseOne_initialPath(
        startPoint, num_S_Moves, num_P_moves,feasibleSet, numAssignedCell)

    StepsAns.append(path2)

    path3,dPath3,feasibleSet3,numDeletedUpMoves3=phaseTwo_gauranteeVallyIsEven(
        startPoint, path2, dPath2, feasibleSet2)

    StepsAns.append(copy.deepcopy(path3))

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

    StepsAns.append(copy.deepcopy(path3))

    if isLastDrone:

        path4,dPath4,feasibleSet4=phaseFour_FinalCheck(
            path3, dPath3, feasibleSet3, num_P_Moves3)

        if [startPoint[0],startPoint[1]-1] in path4 and (startPoint[1]*(num_S_Moves+1))!=numAssignedCell:

            path4=correctLastDronePath_nOdd_mEven_qEven_MisEvenMultiplierOfQ(path4)

        StepsAns.append(path4)

        return path4,dPath4,feasibleSet4,StepsAns

    else:
        StepsAns.append(path3)
        return path3,dPath3,feasibleSet3,StepsAns

# generate paths for all drones
def generateAllDronesPath(n,m,q):

    TotalStepsAns=[]
    finalAns=[]
    startPointsList,num_S_Moves,num_P_Moves,feasibleSet,numAssignedCell=initialSettings(n, m, q)

    for i in range(q):

        if i==q-1:
            path,dPath,feasibleSet,stepsAns=generateSingleDronePath(
                startPointsList[i], num_S_Moves, num_P_Moves[i], feasibleSet, numAssignedCell[i],True)

            finalAns.append(path)
            TotalStepsAns.append(stepsAns)

        else:
            path,dPath,feasibleSet,stepsAns=generateSingleDronePath(
                startPointsList[i], num_S_Moves, num_P_Moves[i], feasibleSet, numAssignedCell[i],False)

            finalAns.append(path)
            TotalStepsAns.append(stepsAns)


    return finalAns,TotalStepsAns

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

def Allocation_P_Moves_Method(n,m,q):

    AllocatedNumList=[]
    Upperbound=math.ceil((n*m)/q)
    residual=(n*m)%q

    if residual==0:
        AllocatedNumList=[Upperbound-n]*q

    else:
        for i in range(residual):
            AllocatedNumList.append(Upperbound-n)

        for i in range(q-residual):
            AllocatedNumList.append(Upperbound-1-n)

    return AllocatedNumList

def deductionAvailability(n,m,q):

    residual=(n*m)%q
    deductionAvailibility=0

    if residual!=0:
        for i in range(residual):
            deductionAvailibility+=1

    return deductionAvailibility

def correctLastDronePath_nOdd_mEven_qEven_MisEvenMultiplierOfQ(path):
    k=0
    m=path[0][1]

    for p in path:
        if p[0]==1:
            k+=1
        if p[1]>m:
            m=p[1]

    newPath=[]
    newDpath=[]
    newPathDpath=[]
    startPoint=[1,m-k+1]
    newPath.append(startPoint)
    newPath.append([1,m-k+2])
    path.remove(startPoint)
    path.remove([1,m-k+2])
    newDpath.append('U')
    currentCell=newPath[-1]
    for i in range(len(path)):

        nextU=[currentCell[0],currentCell[1]+1]
        nextS=[currentCell[0]+1,currentCell[1]]
        nextD=[currentCell[0],currentCell[1]-1]

        if nextU in path:
            if newDpath[-1]=='U' or newDpath[-1]=='S':
                currentCell=nextU
                path.remove(nextU)
                newPath.append(nextU)
                newDpath.append('U')

        elif nextD in path:
            if newDpath[-1]=='D' or newDpath[-1]=='S':

                currentCell=nextD
                path.remove(nextD)
                newPath.append(nextD)
                newDpath.append('D')

        elif nextS in path:

            currentCell=nextS
            path.remove(nextS)
            newPath.append(nextS)
            newDpath.append('S')

        else:
            continue


    return newPath


def plotPath(ans):

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
    fig, ax = plt.subplots()
    #ax.set_title('drone path - q : {}'.format(len(ans)))
    for i in range(len(ans)):
        ax.scatter(np.array(x[i]),np.array(y[i]))

    # draw arrows
    for i in range(len(x)):
        for j in range(len(x[i])):
            if j!=len(x[i])-1:
                ax.annotate("",
                    xy=(x[i][j+1],y[i][j+1]), xycoords='data',
                    xytext=(x[i][j],y[i][j]), textcoords='data',
                    arrowprops=dict(arrowstyle="->",
                                    connectionstyle="arc3"),
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

    for i in range(xAxis+1):
        if i!=0:
            xTick.append(i)

    for j in range(yAxis+1):
        if j!=0:
            yTick.append(j)

    plt.xticks(xTick)
    plt.yticks(yTick)
    #plt.grid()
    #plt.savefig("myimage.pdf")
    plt.show()

def plotPath2(ans,n,m):

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

    # Save the plot to a BytesIO object
    buffer = BytesIO()
    plt.savefig(buffer, format='png')
    buffer.seek(0)

    # Encode the BytesIO object to base64
    plot_data = base64.b64encode(buffer.read()).decode('utf-8')

    return plot_data



def plotAllStepspath(n,m,ans):

    def plotStepsPath03(n,m,ans,title1,title2,colors):

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
        fig.set_size_inches(10, 10)   # Set the size of the figure
        #ax.set_title('UAV : {} , Phase : {}'.format(title1,title2),size=35)

        # Generate the coordinates for scatter plot
        x_coords = []
        y_coords = []
        for i in range(1, n + 1):
            for j in range(1, m + 1):
                x_coords.append(i)
                y_coords.append(j)

        ax.scatter(x_coords,y_coords,color='#93B9B9',alpha=0.2)
        # Set the aspect ratio and limits
        ax.set_aspect('equal')
        ax.set_xlim(0.5, n + 0.5)
        ax.set_ylim(0.5, m + 0.5)

        # Set the tick positions and labels
        ax.set_xticks(range(1, n + 1))
        ax.set_yticks(range(1, m + 1))

        # Set the tick label rotation
        plt.xticks(rotation='vertical')

        # Set the tick label font size
        plt.tick_params(axis='both', which='major', labelsize=8)

        # Set the grid lines
        #ax.grid(True, linestyle='--', linewidth=0.5)

        # Set the axis labels
        #ax.set_xlabel('X')
        #ax.set_ylabel('Y')

        # draw arrows
        for i in range(len(x)):
            for j in range(len(x[i])):
                if j!=len(x[i])-1:
                    ax.annotate("",
                        xy=(x[i][j+1],y[i][j+1]), xycoords='data',
                        xytext=(x[i][j],y[i][j]), textcoords='data',
                        arrowprops=dict(arrowstyle="->",
                                        connectionstyle="arc3",color=colors[i],mutation_scale=20),
                        )



        xTick=[]
        yTick=[]
        for i in range(n+1):
            xTick.append(i+0.5)

        for j in range(m+1):
            yTick.append(j+0.5)

        ax.set_xticks(xTick)
        ax.set_yticks(yTick)
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        ax.grid(color='lightgray', alpha=0.25)
        #ax.axvline(x=0.5, color='darkgray', linestyle='-', linewidth=4)
        #ax.axvline(x=n+0.5, color='darkgray', linestyle='-', linewidth=4)
        #ax.axhline(y=0.5, color='darkgray', linestyle='-', linewidth=4)
        #ax.axhline(y=m+0.5, color='darkgray', linestyle='-', linewidth=4)

        plt.show()

    colors = ['#0000ff', '#ff3300', '#00cc00', '#031971','#ff8000',
              '#00BDA6','#212F3D','#A4557F','#611E78','#5571A4']*(len(ans)//10+1)
    a1=[]
    plotStepsPath03(n, m, [], "", "", colors)
    for idx1,t1 in enumerate(totalStepsAns):
        time.sleep(1)  # Delay for 2 seconds
        if idx1 !=0:
            a1.append(totalStepsAns[idx1-1][-1])
            for idx2,t2 in enumerate(t1):
                time.sleep(1)  # Delay for 2 seconds
                if idx1!=0:
                    a1.append(t2)
                    plotStepsPath03(n,m,a1,idx1+1,idx2+1,colors)

                    a1.remove(t2)
                else:
                    plotStepsPath03(n,m,[t2],idx1+1,idx2+1,colors)

        else:
            for idx2,t2 in enumerate(t1):
                if idx1!=0:
                    a1.append(t2)
                    plotStepsPath03(n,m,a1,idx1+1,idx2+1,colors)

                    a1.remove(t2)
                else:
                    plotStepsPath03(n,m,[t2],idx1+1,idx2+1,colors)


# Calculate the lower bound based on the proposition 2 in the paper
def calculate_LB(n,m,q,T_s,T_p):
    LB=(n-1)*T_s+(math.ceil(m*n/q)-n)*T_p
    PT=math.ceil(m*n/q)-n

    return LB,PT

def objectiveFunctionValueForDrone(ans,T_s,T_p):
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

def totalObjectValue(ans,T_s,T_p):
    allObjectValues=[]
    for i in range(len(ans)):
        o= objectiveFunctionValueForDrone(ans[i],T_s,T_p)
        allObjectValues.append(o)
    return max(allObjectValues)




N_Control=0

app = dash.Dash(__name__)

app.layout = html.Div([
    html.Div(className='center',style={'backgroundColor': '#333683'}, children=[
         html.H1("Demo of the paper \"An exact coverage path planning algorithm for UAV-based search and rescue operations\"", style={'textAlign': 'center', 'color': '#FFFFFF'}),
         html.H2("UAVs' Path Generator", style={'textAlign': 'center', 'color': '#FFFFFF'})
    ]),
    html.Div(style={'backgroundColor': '#92B8FA'}, children=[
        html.Label("Enter value for n (number of cells align the lentgh of rectagular area): ",style={'fontSize': 20}),
        dcc.Input(id='input-n',style={'fontSize': '18px'}, type='number', value=2),
    ]),
    html.Div(style={'backgroundColor': '#92B8FA'}, children=[
        html.Label("Enter value for m (number of cells align the width of rectagular area): ",style={'fontSize': 20}),
        dcc.Input(id='input-m',style={'fontSize': '18px'}, type='number', value=2),
    ]),
    html.Div(style={'backgroundColor': '#92B8FA'}, children=[
        html.Label("Enter value for q (number of UAVs): ",style={'fontSize': 20}),
        dcc.Input(id='input-q',style={'fontSize': '18px'}, type='number', value=2),
    ]),
     html.Div(style={'backgroundColor': '#92B8FA'}, children=[
        html.Label("Enter value for cell dimension (meter): ",style={'fontSize': 20}),
        dcc.Input(id='input-cell',style={'fontSize': '18px'}, type='number', value=100),
    ]),
    html.Div(style={'backgroundColor': '#92B8FA'}, children=[
        html.Label("Enter value for UAVs' speed (m/s): ",style={'fontSize': 20}),
        dcc.Input(id='input-v', style={'fontSize': '18px'},type='number', value=20),
    ]),
    html.Div(style={'backgroundColor': '#92B8FA'}, children=[
        html.Label("Enter value for Wind speed(m/s): ",style={'fontSize': 20}),
        dcc.Input(id='input-w',style={'fontSize': '18px'}, type='number', value=5),
    ]),
    html.Div(className='center',style={'marginTop': '20px','margin-left': '20px', 'display': 'flex', 'justifyContent': 'center', 'alignItems': 'center'} ,children=[  # Wrap the button in a centered div
        html.Button('Generate Paths of the UAVs', id='calculate-btn', style={'fontSize': '18px'}, n_clicks=0),
    ] ),
    html.Div(className='center',style={'marginTop': '20px','margin-left': '20px', 'display': 'flex', 'justifyContent': 'center', 'alignItems': 'center'} ,children=[  # Wrap the button in a centered div
        html.Button('Generate solutions plot', id='plot-btn', style={'fontSize': '18px'}, n_clicks=0),
    ] ),
    html.Div(className='center',style={'marginTop': '20px','margin-left': '20px', 'display': 'flex', 'justifyContent': 'center', 'alignItems': 'center'} ,children=[  # Wrap the button in a centered div
        html.Button('Reset', id='reset-btn', style={'fontSize': '16px'}, n_clicks=0),
    ] ),

    html.Div(id='output01', style={'marginTop': '20px','margin-left': '10px','fontSize': 20}),

    html.Div(id='output', style={'marginTop': '5px'}),

    html.Div(id='output02', style={'marginTop': '20px','margin-left': '10px','fontSize': 20}),

    html.Div(id='output03', style={'marginTop': '20px','margin-left': '10px','fontSize': 20}),


])


@app.callback(
    Output('output01', 'children'),
    [Input('calculate-btn', 'n_clicks')]
)

def printRunning(n_clicks):
  if n_clicks>0:
    n_clicks=0
    return "Code is running ..."

@app.callback(
    Output('output03', 'children',allow_duplicate=True),
    [Input('plot-btn', 'n_clicks')],
    prevent_initial_call=True
)

def printPlotGenrating(n_clicks):
  if n_clicks>0:
    n_clicks=0
    return "Plot is generating ..."



@app.callback(
    [Output('output', 'children', allow_duplicate=True),
     Output('output01', 'children', allow_duplicate=True),
     Output('output03', 'children',allow_duplicate=True),
     Output('input-n', 'value'),
     Output('input-m', 'value'),
     Output('input-q', 'value'),
     Output('input-v', 'value'),
     Output('input-w', 'value'),
     Output('input-cell', 'value')],
    Input('reset-btn', 'n_clicks'),
    prevent_initial_call=True
)
def reset(n_clicks):
  if n_clicks>0:
    n_clicks=0
    return "","","",2,2,2,20,5,100






@app.callback(
    Output('output', 'children'),
    [Input('calculate-btn', 'n_clicks')],
    [State('input-n', 'value'),
     State('input-m', 'value'),
     State('input-q', 'value'),
     State('input-v', 'value'),
     State('input-w', 'value'),
     State('input-cell', 'value')]
)

def calculate_and_plot_path(n_clicks, n, m, q, v, w,d):

    T_s=d/(v+w)
    T_p=d/math.sqrt(v**2-w**2)
    if n<=100 and m<=100:
      if q<=n and q<=m:
        if n_clicks > 0:

            LB,TP=calculate_LB(n,m,q,T_s,T_p)

            # Replace the placeholder variables below with the actual inputs
            finalAnswer, totalStepsAns = generateAllDronesPath(n, m, q)

            operation_time=totalObjectValue(finalAnswer,T_s,T_p)

            # Return the lower bound calculated by the function calculate_LB
            plot_and_message = [
                html.Div("Accomplished!", style={'margin-left': '10px','fontSize': 20}),
                html.Div("LB is : {} (s)".format(LB), style={'marginTop': '30px','margin-left': '10px','fontSize': 20}),
                html.Hr(),
                html.Div("Operation time is : {} (s)".format(operation_time), style={'marginTop': '10px','margin-left': '10px','fontSize': 20}),
                html.Hr(),
                html.Div("The final answer is : {}".format(finalAnswer), style={'marginTop': '10px','margin-left': '10px','fontSize': 20}),
                html.Hr(),
                #html.Img(src='data:image/png;base64,{}'.format(plotPath2(finalAnswer,n,m)))
            ]
            n_clicks=0
            return plot_and_message


      else:
        plot_and_message = [
                html.Div("Error!", style={'margin-left': '10px','margin-left': '10px','fontSize': 20}),
                html.Div("q cannot be more than n and m", style={'marginTop': '30px','margin-left': '10px','fontSize': 20})
            ]
        n_clicks=0
        return plot_and_message
    else:
      plot_and_message = [
                html.Div("Failed!", style={'margin-left': '10px','margin-left': '10px','fontSize': 20}),
                html.Div("n and m should be less than 100 for demo version", style={'marginTop': '30px','margin-left': '10px','fontSize': 20})
            ]
      n_clicks=0
      return "",plot_and_message


@app.callback(
    Output('output03', 'children'),
    [Input('plot-btn', 'n_clicks')],
    [State('input-n', 'value'),
     State('input-m', 'value'),
     State('input-q', 'value')
    ]
)

def generating_plot(n_clicks, n, m,q):
    if n<=30 and m<=30:
      if q<=n and q<=m:
        if n_clicks > 0:

            finalAnswer, totalStepsAns = generateAllDronesPath(n, m, q)
            # Return the lower bound calculated by the function calculate_LB
            plot_and_message = [

                html.Img(src='data:image/png;base64,{}'.format(plotPath2(finalAnswer,n,m)))
            ]
            n_clicks=0
            return plot_and_message


      else:
        plot_and_message = [
                html.Div("Error!", style={'margin-left': '10px','margin-left': '10px','fontSize': 20}),
                html.Div("q cannot be more than n and m", style={'marginTop': '30px','margin-left': '10px','fontSize': 20})
            ]
        n_clicks=0
        return plot_and_message
    else:
      plot_and_message = [
                html.Div("Failed!", style={'margin-left': '10px','margin-left': '10px','fontSize': 20}),
                html.Div("Plot can be generated when n and m are less than 30 (use python file in github for large plots)", style={'marginTop': '30px','margin-left': '10px','fontSize': 20})
            ]
      n_clicks=0
      return plot_and_message





if __name__ == '__main__':
    app.run_server(debug=True)

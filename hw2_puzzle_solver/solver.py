#######
#kullanicidan giris alma
#######
import heapq
n = int(input("Enter size of tile puzzle (integer greater that 0): "))  

state = [] 
print("\n Enter start state row by row (numbers delimited by white space): \n")

for i in range(0,n):
    s = list(map(int,input("Enter Start state: row %a : " % str(i+1)).strip().split()))[:n**2]  #strip virgul ekler, split() bosluk siler
    for k in s:
        state.append(k)
print("\n Enter goal state row by row (numbers delimited by white space): \n")
goal_state = []
for i in range(0,n):
    s = list(map(int,input("Enter Goal state: row %a : " % str(i+1)).strip().split()))[:n**2]  
    for k in s:
        goal_state.append(k)

def legal_moves(row,col):
            
    legal_action = ['Down', 'Left', 'Up','Right'] 
    
    if row == 0:  # ust satirda yukariya izin verilmez
        legal_action.remove('Up')
    elif row == n-1:  # alt satirda asagiya izin verilmez
        legal_action.remove('Down')
    if col == 0:
        legal_action.remove('Left')
    elif col == n-1:  
        legal_action.remove('Right')
    return legal_action 

#sonraki durumlar geri gelir
def generate_child(node,g):
    
    children = []      
        
    x = node.index(0) 
           
    i = int(x / n)  #row 
    j = int(x % n)   #column
    
    legal_actions=legal_moves(i,j)

    for action in legal_actions:

        new_state = node.copy()   
            
        if action == 'Down':
            new_state[x], new_state[x+n] = new_state[x+n], new_state[x]
            
        elif action == 'Left':
            new_state[x], new_state[x-1] = new_state[x-1], new_state[x]
#Listedeki her hareket icin 0'in indeksi degistirilir
        elif action == 'Up':
            new_state[x], new_state[x-n] = new_state[x-n], new_state[x]
            
        elif action == 'Right':
            new_state[x], new_state[x+1] = new_state[x+1], new_state[x]
            
            
        children.append([f(g,manhattan(new_state)),new_state,action])
        
    return children

def manhattan(mstate):
    manhattan = 0 
    for i in mstate:

        if i != 0: 
            t = mstate.index(i) 
            s = goal_state.index(i) 
            
            drow = abs(int(t/n)-int(s/n))
            dcol = abs(int(t%n)-int(s%n)) 

            manhattan += drow + dcol

    return manhattan 

def f(g,h): 
    f = g+h 
    return f


def astar(initialState,goalState):
    
    openList = [] 
    closedList = [] 

    openList.append([0,initialState,''])
    
    g = 1 #g mevcut durumun baslangica olan uzakligidir
    while openList:

        heapq.heapify(openList) 
                                
        node = heapq.heappop(openList) 
        closedList.append(node)

        if node[1] == goalState:
            
            return find_path(closedList) 
            break 

        ch = generate_child(node[1],g) 

        for child in ch: 

            if child not in openList: 
                openList.append(child)
        g += 1 

    return 
            
def find_path(lst): 


    steps = []
    for i in range(1,len(lst)): #baslangic durumu haric
        steps.append(lst[i][2]) #tum adimlari bir listeye koyma
    return steps
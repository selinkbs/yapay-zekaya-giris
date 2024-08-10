from solver import astar, state, goal_state, n
#ice aktarma
 
steps = astar(state,goal_state) #tum adimlari iceren liste

if not steps and state!= goal_state:  #hic adim yoksa cozum yoktur
    print('\n No Solution Could Be Found')
    
elif not steps and state == goal_state: #baslangic ve son durumlar ayni olursa
    print('\n There is Nothing to Solve')

else:
    print('\n','----Initial State----','\n')

    k = 1 #count
    
    while k<n+1:  #NxN olarak yazdirmak icin dongu
         print('',*state[n*(k-1):n*k]) 
         k += 1
    print('\n','----------------')
    print('\n','Solution')
    print('\n','----------------')

    for a in steps:
        
        t = list(state).index(0) #0in konumunu bulma

        print('\n',a,'\n')  #adimlari yazdirma
                
        if a == 'Up':
            state[t], state[t-n] = state[t-n], state[t]
        elif a == 'Down':
            state[t], state[t+n] = state[t+n], state[t]
        elif a == 'Left':
            state[t], state[t-1] = state[t-1], state[t]
        elif a == 'Right':
            state[t], state[t+1] = state[t+1], state[t]

        l=1  #count
        while l<n+1: #durumu matris olarak yazdirma
              
            print('',*state[n*(l-1):n*l])
            l += 1
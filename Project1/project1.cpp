#include<stdio.h>
#include<vector>
#include<iostream>
#include<fstream>
#include<algorithm>
#include<string.h>


struct node { 
    int state[9];
    int id=0;
    int parentid=0;
    int c2c=0;
    int voidpos=0;
};

class puzzle {
//    public :

    std::vector<node> Nodes;
    node node_init, node_goal;
    int* goal_state;
    std::vector<int> nodePath;

    public :

    puzzle(){
        std::cout<<1<<"\n";
        int start[9] = {0, 4, 7, 1, 2, 8, 3, 5, 6};   //column wise
        int end[9] = {1, 4, 7, 2, 5, 8, 3, 6, 0}; 
        for(int i=0; i<9; i++){
            node_init.state[i]=start[i];
        }
        node_init.id = 0;
        node_init.parentid = 0;
        node_init.c2c = 0;
        node_init.voidpos = 1;
        Nodes.push_back(node_init);
        for(int i=0; i<9; i++){
            node_goal.state[i]=end[i];
        }
        node_goal.voidpos = 9;
    }

    puzzle(int* start){
        int end[9] = {1, 4, 7, 2, 5, 8, 3, 6, 0}; 
        for(int i=0; i<9; i++){
            node_init.state[i]=start[i];
            //std::cout<<node_init.state[i];
            if(start[i] == 0){
                node_init.voidpos = i+1;
            }
        }
        node_init.id = 0;
        node_init.parentid = 0;
        node_init.c2c = 0;
        Nodes.push_back(node_init);
        for(int i=0; i<9; i++){
            node_goal.state[i]=end[i];
            if(end[i] == 0){
                node_goal.voidpos = i+1;
            }
        }
        std::cout<<Nodes.size()<<"\n";
    }

    puzzle(int* start, int* end){
        for(int i=0; i<9; i++){
            node_init.state[i]=start[i];
            if(start[i] == 0){
                node_init.voidpos = i+1;
            }
        }
        node_init.id = 0;
        node_init.parentid = 0;
        node_init.c2c = 0;
        Nodes.push_back(node_init);
        for(int i=0; i<9; i++){
            node_goal.state[i]=end[i];
            if(end[i] == 0){
                node_goal.voidpos = i+1;
            }
        }
    }

    node moveUp(int currentid){
        node next;
        int c = Nodes[currentid].voidpos;
        for(int i=0; i<9; i++){
            next.state[i]=Nodes[currentid].state[i];
        }
        next.state[c-1] = next.state[c-2];
        next.state[c-2] = 0;
        next.id = Nodes.size();
        next.parentid = currentid;
        next.voidpos = c-1;
        next.c2c = Nodes[currentid].c2c + 1;
        return next;
    }
    node moveDown(int currentid){
        node next;
        int c = Nodes[currentid].voidpos;
        for(int i=0; i<9; i++){
            next.state[i]=Nodes[currentid].state[i];
        }
        next.state[c-1] = next.state[c];
        next.state[c] = 0;
        next.id = Nodes.size();
        next.parentid = currentid;
        next.voidpos = c+1;
        next.c2c = Nodes[currentid].c2c + 1;
        return next;
    }
    node moveLeft(int currentid){
        node next;
        int c = Nodes[currentid].voidpos;
        for(int i=0; i<9; i++){
            next.state[i]=Nodes[currentid].state[i];
        }
        next.state[c-1] = next.state[(c-4)];
        next.state[c-4] = 0;
        next.id = Nodes.size();
        next.parentid = currentid;
        next.voidpos = c-3;
        next.c2c = Nodes[currentid].c2c + 1;
        return next;
    }
    node moveRight(int currentid){
        int c = Nodes[currentid].voidpos;
        node next;
        for(int i=0; i<9; i++){
            next.state[i]=Nodes[currentid].state[i];
        }
        next.state[c-1] = next.state[(c+2)];
        next.state[c+2] = 0;
        next.id = Nodes.size();
        next.parentid = currentid;
        next.voidpos = c+3;
        next.c2c = Nodes[currentid].c2c + 1;
        return next;
    }
    void bsearch(){
        int t=0;
        std::vector<int> parentids;
        //parentids.push_back(0);
        //std::cout<<999;
        std::vector<int> childids;
        //std::cout<<999<<parentids.size();

        parentids.push_back(Nodes[0].id);
        //std::cout<<parentids[0];
        bool flag = true;
        while(flag){
            for(int i=0; i<parentids.size(); i++){
                int m,n;
                int currentid = parentids[i];
                int c = Nodes[parentids[i]].voidpos;
                int p = Nodes[Nodes[parentids[i]].parentid].voidpos;
                //printNode(currentid);
                if(c%3==0){
                    m = 3;
                    n = c/3;
                }
                else{
                    m = c%3;
                    n = (c/3) + 1;
                }
                //std::cout<<"m: "<<m<<" n: "<<n;
                //flag=false;
                //break;
                if(c==node_goal.voidpos){
                    if(checkForGoal(currentid)){
                        node_goal.id = currentid;
                        node_goal.c2c = Nodes[currentid].c2c;
                        node_goal.parentid = Nodes[currentid].parentid;
                        flag = false;
                        break;
                    }
                }
                if((m-1>0) && (c-1!=p)){                           //UP
                    node next = moveUp(currentid);
                    childids.push_back(next.id);
                    Nodes.push_back(next);
                }
                if((m+1<4) && (c+1!=p)){
                    node next = moveDown(currentid);
                    childids.push_back(next.id);
                    Nodes.push_back(next);
                }
                if((n-1>0) && (c-3!=p)){
                    node next = moveLeft(currentid);
                    childids.push_back(next.id);
                    Nodes.push_back(next);
                }
                if((n+1<4) && (c+3!=p)){
                    node next = moveRight(currentid);
                    childids.push_back(next.id);
                    Nodes.push_back(next);
                }
            }
            parentids = childids;
            // for(int i =0; i<parentids.size(); i++){
            //     std::cout<<"id : "<<parentids[i]<<" ;voidpos : "<<Nodes[parentids[i]].voidpos<<",";
            // }
            // std::cout<<" : iter no. : "<<t<<"\n";
            // t++;
            childids.clear();
            if(Nodes.size()>400000){
                //std::cout<<Nodes.size();
                printf("Goal can't be reached. Quitting program.");
                flag = false;
            }
        }
    }

    bool checkForGoal(int currentid){
        for(int i=0; i<9; i++){
            if(Nodes[currentid].state[i] == node_goal.state[i]){
                continue;
            }
            else{
                return false;
            }
            node_goal.id = Nodes[currentid].id;
            node_goal.c2c = Nodes[currentid].c2c;
            node_goal.parentid = Nodes[currentid].parentid;
        }
        return true;
    }

    void nextNode(int parentid,int currentid){
        int c = Nodes[currentid].voidpos;
        int p = Nodes[parentid].voidpos;
        if(c==9){
            if(checkForGoal(currentid)){
                return;
            }
        }
        if((((c-1)%3)!=0) && (c-1!=p)){
            node next = moveUp(currentid);
            Nodes.push_back(next);
            nextNode(currentid, next.id);
        }
        if((((c+1)%3)!=1) && (c+1!=p)){
            node next = moveDown(currentid);
            Nodes.push_back(next);
            nextNode(currentid, next.id);
        }
        if(((c/3)-1!=0) && (c-3!=p)){
            node next = moveLeft(currentid);
            Nodes.push_back(next);
            nextNode(currentid, next.id);
        }
        if(((c/3)+1!=4) && (c+3!=p)){
            node next = moveRight(currentid);
            Nodes.push_back(next);
            nextNode(currentid, next.id);
        }

    }

    void printNode(int nodeid){
        printf("Node State : ");
        for(int i=0; i<9; i++){
            printf(" %d ",Nodes[nodeid].state[i]);
        }
        printf("; Node ID : %d ; Node Parent : %d ; Cost2Come : %d \n",Nodes[nodeid].id,Nodes[nodeid].parentid,Nodes[nodeid].c2c);
    }

    void printPath(){    
        int currentid = node_goal.id;
        while(true){
            nodePath.push_back(currentid);
            if(currentid == 0){break;}
            currentid = Nodes[currentid].parentid;
        }
        std::reverse(nodePath.begin(), nodePath.end());
        for(int i=0; i< nodePath.size();i++){
            printNode(nodePath[i]);
        }
    }

    void generatetxt(){
        std::ofstream outfile ("nodePath.txt");
        for(int i=0; i< nodePath.size();i++){
            for(int j=0; j<9; j++){
                outfile << " "<<Nodes[nodePath[i]].state[j];
            }
            outfile << "\n";
        }
    }

    /*void checkNode(int currentid){
        for(int i=0; i<Nodes.size; i++){
            break;
        }
    }*/
};
int main(int argc, char* argv[])
{
    std::cout<<argc;
    bool err = false;
    if(argc == 1 || argc == 2){
        puzzle puz;
        //std::cout<<1;
        puz.bsearch();
        puz.printPath();
        puz.generatetxt();
    }
    else if(argc == 3){
        int len1 = strlen(argv[2]);
        int start[9];
        if(len1!=9){
            printf("Too many or too few elements in start state vector. Execute command again%d",len1);
        }
        else{
            char* st = argv[2];
            for(int i=0;i<9;i++){
                start[i] = st[i] - 48;
            }
            for(int i=0; i<9;i++){
                for(int j=i+1;j<9;j++){
                    if(start[i]==start[j]){
                        err = true;
                        break;
                    }
                }
            }
            if(err){
                printf("Invalid State");
            }
            else{
                puzzle puz(start);
                //std::cout<<1;
                puz.bsearch();
                puz.printPath();
                puz.generatetxt();
            }
        }  
    }
    else if(argc == 4){
        int len1 = strlen(argv[2]);
        int len2 = strlen(argv[3]);
        int start[9], end[9];
        if(len1!=9){
            printf("Too many or too few elements in start state vector. Execute command again%d",len1);
        }
        else if(len2!=9){
            printf("Too many or too few elements in goal state vector. Execute command again");
        }
        else{
            char* st = argv[2];
            char* en = argv[3];
            for(int i=0;i<9;i++){
                start[i] = st[i] - 48;
                end[i] = en[i] - 48;
            }
            for(int i=0; i<9;i++){
                for(int j=i+1;j<9;j++){
                    if((start[i]==start[j]) || (end[i] == end[j])){
                        err = true;
                        break;
                    }
                }
            }
            if(err){
                printf("Invalid State");
            }
            else{
                puzzle puz(start,end);
                //std::cout<<1;
                puz.bsearch();
                puz.printPath();
                puz.generatetxt();
            }
        }   
    }
    return 0;
}
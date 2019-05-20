#include<stdio.h>
#include<vector>
#include<iostream>
#include<fstream>
#include<algorithm>
#include<string.h>
#include<set>
#include<cmath>


struct node { 
    int pos;
    int id=0;
    int parentid=0;
    int c2c=0;
    int voidpos=0;
    uint64_t nodeid = 0;
};

class puzzle {

    std::vector<node> Nodes;
    std::set<uint64_t> Nodeset;
    node node_init, node_goal;
    int* goal_state;
    std::vector<int> nodePath;

    public :

    puzzle(){
        int start[9] = {0, 4, 7, 1, 2, 8, 3, 5, 6};   //column wise
        int end[9] = {1, 4, 7, 2, 5, 8, 3, 6, 0}; 
        for(int i=0; i<9; i++){
            node_init.state[i]=start[i];
            node_init.nodeid = node_init.nodeid + start[i]*pow(10,8-i);
        }
        node_init.id = 0;
        node_init.parentid = 0;
        node_init.c2c = 0;
        node_init.voidpos = 1;
        Nodes.push_back(node_init);
        for(int i=0; i<9; i++){
            node_goal.state[i]=end[i];
            node_goal.nodeid = node_goal.nodeid + end[i]*pow(10,8-i);
        }
        node_goal.voidpos = 9;
        Nodeset.insert(Nodes[0].nodeid);
    }

    puzzle(int* start){
        int end[9] = {1, 4, 7, 2, 5, 8, 3, 6, 0}; 
        for(int i=0; i<9; i++){
            node_init.state[i]=start[i];
            node_init.nodeid = node_init.nodeid + start[i]*pow(10,8-i);
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
            node_goal.nodeid = node_goal.nodeid + end[i]*pow(10,8-i);
            if(end[i] == 0){
                node_goal.voidpos = i+1;
            }
        }
        Nodeset.insert(Nodes[0].nodeid);
    }

    puzzle(int* start, int* end){
        for(int i=0; i<9; i++){
            node_init.state[i]=start[i];
            node_init.nodeid = node_init.nodeid + start[i]*pow(10,8-i);
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
            node_goal.nodeid = node_goal.nodeid + end[i]*pow(10,8-i);
            if(end[i] == 0){
                node_goal.voidpos = i+1;
            }
        }
        Nodeset.insert(Nodes[0].nodeid);
    }


    bool checkSolvability(){
        int invcount = 0;
        int arr[9];
        int j;
        for(int j = 0;j<3;j++){
            for(int i=0; i<3; i++){
                arr[i + j*3] = Nodes[0].state[j + i*3];
            }
        }
        for(int i=0;i<9-1;i++){
            for(int j=i+1;j<9;j++){
                if(arr[j] && arr[i] && arr[i]>arr[j]){
                invcount++;
                }
            }
        }
        return (invcount%2 == 0);
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
        for(int i=0;i<9;i++){
            next.nodeid = next.nodeid + next.state[i]*pow(10,8-i);
        }
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
        for(int i=0;i<9;i++){
            next.nodeid = next.nodeid + next.state[i]*pow(10,8-i);
        }
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
        for(int i=0;i<9;i++){
            next.nodeid = next.nodeid + next.state[i]*pow(10,8-i);
        }
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
        for(int i=0;i<9;i++){
            next.nodeid = next.nodeid + next.state[i]*pow(10,8-i);
        }
        return next;
    }

    bool bsearch(){

        printf("\nSearching for goal state...\n\n");
        bool ret;
        bool flag = true;
        std::vector<int> parentids;
        std::vector<int> childids;
        parentids.push_back(Nodes[0].id);
        if(Nodes[0].nodeid == node_goal.nodeid){
            node_goal.id = Nodes[0].id;
            node_goal.c2c = Nodes[0].c2c;
            node_goal.parentid = Nodes[0].parentid;
            ret = true;
            flag = false;
            printf("\nGOAL STATE FOUND. Total Nodes searched = %d\n",Nodes.size());
        }
        while(flag){
            for(int i=0; i<parentids.size(); i++){
                int m,n;
                int currentid = parentids[i];
                int c = Nodes[parentids[i]].voidpos;
                int p = Nodes[Nodes[parentids[i]].parentid].voidpos;
                if(c%3==0){
                    m = 3;
                    n = c/3;
                }
                else{
                    m = c%3;
                    n = (c/3) + 1;
                }

                if((m-1>0) && (c-1!=p)){                           //UP
                    node next = moveUp(currentid);
                    if(Nodeset.count(next.nodeid) == 0){
                        if(next.nodeid == node_goal.nodeid){
                            node_goal.id = next.id;
                            node_goal.c2c = next.c2c;
                            node_goal.parentid = next.parentid;
                            ret = true;
                            flag = false;
                            Nodes.push_back(next);
                            printf("\n GOAL STATE FOUND. Total Nodes searched = %d\n",Nodes.size());
                            break;
                        }
                        childids.push_back(next.id);
                        Nodes.push_back(next);
                        Nodeset.insert(next.nodeid);
                    }
                }
                if((m+1<4) && (c+1!=p)){                           //DOWN
                    node next = moveDown(currentid);
                    if(Nodeset.count(next.nodeid) == 0){
                        if(next.nodeid == node_goal.nodeid){
                            node_goal.id = next.id;
                            node_goal.c2c = next.c2c;
                            node_goal.parentid = next.parentid;
                            ret = true;
                            flag = false;
                            Nodes.push_back(next);
                            printf("\n GOAL STATE FOUND. Total Nodes searched = %d\n",Nodes.size());
                            break;
                        }
                        childids.push_back(next.id);
                        Nodes.push_back(next);
                        Nodeset.insert(next.nodeid);
                    }
                }
                if((n-1>0) && (c-3!=p)){                           //LEFT
                    node next = moveLeft(currentid);
                    if(Nodeset.count(next.nodeid) == 0){
                        if(next.nodeid == node_goal.nodeid){
                            node_goal.id = next.id;
                            node_goal.c2c = next.c2c;
                            node_goal.parentid = next.parentid;
                            ret = true;
                            flag = false;
                            Nodes.push_back(next);
                            printf("\n GOAL STATE FOUND. Total Nodes searched = %d\n",Nodes.size());
                            break;
                        }
                        childids.push_back(next.id);
                        Nodes.push_back(next);
                        Nodeset.insert(next.nodeid);
                    }
                }
                if((n+1<4) && (c+3!=p)){                           //RIGHT
                    node next = moveRight(currentid);
                    if(Nodeset.count(next.nodeid) == 0){
                        if(next.nodeid == node_goal.nodeid){
                            node_goal.id = next.id;
                            node_goal.c2c = next.c2c;
                            node_goal.parentid = next.parentid;
                            ret = true;
                            flag = false;
                            Nodes.push_back(next);
                            printf("\n GOAL STATE FOUND. Total Nodes searched = %d\n",Nodes.size());
                            break;
                        }
                        childids.push_back(next.id);
                        Nodes.push_back(next);
                        Nodeset.insert(next.nodeid);
                    }   
                }
            }
            parentids = childids;
            if(Nodes.size()>400000 || parentids.empty()){
                printf("Goal can't be reached. Total nodes searched = %d.\nQuitting program.\n",Nodes.size());
                ret = false;
                flag = false;
            }
            parentids = childids;
            childids.clear();
        }
        return ret;
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
        printf("\n\nThe path from start node is as follow .....\n\n");
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
        std::ofstream nodePathfile ("nodePath.txt");
        std::ofstream Nodesfile ("Nodes.txt");
        std::ofstream NodesInfofile ("NodeInfo.txt");
        for(int i=0; i< nodePath.size();i++){
            for(int j=0; j<9; j++){
                nodePathfile << "\t"<<Nodes[nodePath[i]].state[j];
            }
            nodePathfile << "\n";
        }
        for(int i=0; i< Nodes.size();i++){
            for(int j=0; j<9; j++){
                Nodesfile << "\t"<<Nodes[i].state[j];
            }
            Nodesfile << "\n";
            NodesInfofile << "\t"<<Nodes[i].id<<"\t"<<Nodes[i].parentid<<"\t"<<Nodes[i].c2c<<"\n";
        }
    }
};
int main(int argc, char* argv[])
{
    bool err = false;
    if(argc == 1 || argc == 2){
        puzzle puz;
        if(puz.bsearch()){
            puz.printPath();
        }
        puz.generatetxt();
    }
    else if(argc == 3){
        int len1 = strlen(argv[2]);
        int start[9];
        if(len1!=9){
            printf("Too many or too few elements in start state vector. Execute command again.\n",len1);
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
                printf("Invalid State\n");
            }
            else{
                puzzle puz(start);
                if(!puz.checkSolvability()){
                    printf("Given state not solvable. Quitting.\n");
                }
                else if(puz.bsearch()){
                    puz.printPath();
                }
                puz.generatetxt();
            }
        }  
    }
    else if(argc == 4){
        int len1 = strlen(argv[2]);
        int len2 = strlen(argv[3]);
        int start[9], end[9];
        if(len1!=9){
            printf("Too many or too few elements in start state vector. Execute command again.\n",len1);
        }
        else if(len2!=9){
            printf("Too many or too few elements in goal state vector. Execute command again.\n");
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
                if(puz.bsearch()){
                    puz.printPath();
                }
                puz.generatetxt();
            }
        }   
    }
    return 0;
}
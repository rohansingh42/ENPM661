// C++ code to demonstrate copy of vector 
// by iterative method. 
#include<iostream> 
#include<vector> 
#include<set>
//using namespace std; 

struct a{
    int b=1;
    int c=0;
    bool operator<(const a &other) const { return b<other.b;}
    bool operator==(const a &other) const { return b==other.b;}
}; 
int main() 
{ 
    // std::set<a>::iterator it;
    // Initializing vector with values 
    std::vector<int> vect1{1, 2, 3, 4}; 
    std::set<int> testset;
    a te;
    // testset.insert(te);
    //int vect1[2] = {1,2};
    // Declaring new vector 
    std::vector<int> vect2{7, 8, 9}; 
    vect1.push_back(10);
    std::vector<int> w;
    std::vector<int> s;
    // Using assignment operator to copy one 
    // vector to other 
    //vect2 = vect1; 
  
    std::cout << "Old vector elements are : "; 
    for (int i=0; i<vect1.size(); i++) 
        std::cout << vect1[i] << " "; 
    std::cout << std::endl; 
  
    std::cout << "New vector elements are : "; 
    for (int i=0; i<vect2.size(); i++) 
        std::cout << vect2[i] << " "; 
    std::cout<< std::endl; 
  
    // Changing value of vector to show that a new 
    // copy is created. 
    vect1[0] = 2; 
  
    std::cout << "The first element of old vector is :"; 
    std::cout << vect1[0] << std::endl; 
    std::cout << "The first element of new vector is :"; 
    std::cout << vect2[0] <<std::endl;

    a t,r;
    t.b = 0;
    r.b = 0;

    int y;
    int z;
    int p[2] = {9,8};
    int q[2] = {9,8};
    y = 2;
    z = 2;
    testset.insert(y);
    auto it = testset.find(z);
    std::cout<<testset.count(z);
    if(it == testset.end()){std::cout<<"hdhdh";}

    // y=p; 
    // z=y;
    // std::cout<<z[1];
    // int h[2]={0,1};
    // int g[2];
    // // g=h;
    // std::cout<<g[1];

    // int arr[3][3] = {{1,2,3},{4,5,6},{7,8,9}};
    // int *temp;
    // temp = (int*)arr;
    // for(int i=0;i<9;i++){std::cout<<temp[i];}
  
    return 0; 
}
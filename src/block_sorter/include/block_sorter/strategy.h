#include "block_sorter/includeFiles.h"

using namespace std;

inline void read(int &x){
    x=0;char ch=getchar();bool f=0;
    while (ch>'9'||ch<'0') {if (ch=='-') f=1;ch=getchar();}
    while (ch>='0'&&ch<='9') x=(x<<1)+(x<<3)+(ch^48),ch=getchar();
    if (f) x=-x;
    return;
}
int f[14][10];
struct node{
    int dx,dy;
};

node brick[7][4][4];
int suck[7][4];
int ID=0;
int n[7];
bool flag=0;
bool check(int id,int wa,int x,int y){
    for (int i=0;i<4;i++){
    if (x+brick[id][wa][i].dx>=14 || x+brick[id][wa][i].dx<0 ||
        y+brick[id][wa][i].dy>=10 || y+brick[id][wa][i].dy<0 ||
        f[x+brick[id][wa][i].dx][y+brick[id][wa][i].dy]!=-1) return 0;}
    return 1;
}
void la(int id,int wa,int x,int y,int t){
    for (int i=0;i<4;i++)
    f[x+brick[id][wa][i].dx][y+brick[id][wa][i].dy]=t;
    return;
}

void dfs(int x,int y){
    if (flag) return;
    // 所有方块已放完即成功退出，不需要填满整个网格
    { bool all_placed = true;
      for (int i = 0; i < 7; i++) { if (cnt[i] > 0) { all_placed = false; break; } }
      if (all_placed) { flag = 1; return; } }
    if (x>=14) return;
    if (y==10){
        dfs(x+1,0);
        return;
    }
    if (f[x][y]!=-1){
        dfs(x,y+1);
        return;
    }
    for (int id=0;id<7;++id){
        if (cnt[id]>0){
            for (int i=0;i<n[id];++i){
                if (check(id,i,x,y)){
                    cnt[id]--;
                    la(id,i,x,y,ID++);
                    order.push_back(orderType{x+brick[id][i][suck[id][i]].dx,y+brick[id][i][suck[id][i]].dy,id,i});
                    if (y==9) dfs(x+1,0);
                    else dfs(x,y+1);
                    if (flag) return;
                    --ID;
                    order.pop_back();
                    la(id,i,x,y,-1);
                    cnt[id]++;
                }
            }
        }
    }
    return;
}

void strategy(){
    flag=0;
    ID=0;
    memset(f,-1,sizeof f);
    string data_dir = DATA_DIR;
    string t_path = data_dir + "/data/t.txt";
    freopen(t_path.c_str(), "r", stdin);
    int t;
    for (int i=0;i<7;i++){
        read(n[i]);
        for (int j=0;j<n[i];j++){
            for (int k=0;k<4;k++){
                read(brick[i][j][k].dx);
                read(brick[i][j][k].dy);
                read(t);
                if (t==1) suck[i][j]=k;
            }
        }
    }
    for (int i=0;i<14;i++) {
        for (int j=0;j<10;j++) {
            read(f[i][j]);
            if (f[i][j]==1) ID=1;
            --f[i][j];
        }
    }
    fclose(stdin);
    dfs(0,0);
    return;
}

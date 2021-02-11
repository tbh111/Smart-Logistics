///////////////////////////////////////////////////////////
//			A*�㷨  ����Ѱ·�㷨
//			�㷨��һ�־�̬·����������·����Ч���㷨
//				1����ʽ��ʾΪ�� f(n)=g(n)+h(n),
//				2�� ��������·������
//						���ĳ�����ڵķ����Ѿ��� open list �У���������·���Ƿ���ţ�
//						Ҳ����˵���ɵ�ǰ���� ( ����ѡ�еķ��� ) �����Ǹ������Ƿ���и�С�� G ֵ��
//						���û�У������κβ�����
/////////////////////////////////////////////////////////

#include<iostream>
#include<fstream>
#include<math.h>

using namespace std;

const int DISTANCE = 10;
const int direction[8][2] = {{-1,-1},{-1,0},{-1,1},{0,-1},{0,1},{1,-1},{1,0},{1,1}};// ����

enum{LENGTH=40,WIDE=40};//���������С
enum{VIABLE, WALL, INOPEN, INCLOSE, STARTPOINT, DESTINATION};
struct Node
{
	//char perperty;// ���ԣ� ��ǽ��������������
	int    flag; //��־λ 0 Ϊ���ߣ� 1 Ϊǽ��  2 ��openlist  3 �� closelist�� 4 Ϊ��� 5 Ϊ�յ�
	unsigned int location_x;
	unsigned int location_y; 
	unsigned int value_h;
	unsigned int value_g;
	unsigned int value_f;
	Node* parent;
	Node();
};
Node::Node()
{
	flag = 0;
	value_h = 0;
	value_g = 0;
	value_f =  0;
	parent = NULL;
}
///////////////////////////////////////////////////////////////
// ���� openlist
//////////////////////////////////////////////////////////////
struct OpenList
{
	Node *opennode;
	OpenList* next;
	OpenList(){next = NULL;};
};


void AddNode2Open(OpenList* openlist, Node* node)
{
	if(openlist == NULL)
	{
		cout<<"no data in openlist!"<<endl;
		return;
	}
	if(node->flag != STARTPOINT)
	{
		node->flag = INOPEN;
	}
	OpenList* temp =  new OpenList;
	temp->next = NULL;
	temp->opennode = node;

//	if(openlist->next==NULL)
//	{openlist->next = temp;return;}

	while(openlist->next != NULL)
	{
		if(node->value_f < openlist->next->opennode->value_f)
		{
			OpenList* tempadd = openlist->next;
			temp->next = tempadd;
			openlist->next = temp;
			break;
		}
		else
			openlist = openlist->next;
	}
	openlist->next = temp;

}

/////////////////////////////////////////////////////////////
//   ���� closelist
////////////////////////////////////////////////////////////
struct CloseList
{
	Node *closenode;
	CloseList* next;
	CloseList(){ next = NULL;};
};
// openlist �˴�����Ϊָ�������
void AddNode2Close(CloseList* close, OpenList* &open)
{
	if(open == NULL)
	{
		cout<<"no data in openlist!"<<endl;
		return;
	}
	if(open->opennode->flag != STARTPOINT)
		open->opennode->flag = INCLOSE;

	if(close->closenode == NULL)
	{	
		close->closenode = open->opennode;
		OpenList* tempopen = open;
		open = open->next;
		//open->opennode=NULL;
	//	open->next=NULL;
		delete tempopen;
		return;
	}
	while(close->next != NULL)
		close = close->next;

	CloseList* temp = new CloseList;
	temp->closenode = open->opennode;
	temp->next = NULL;
	close->next = temp;

	OpenList* tempopen = open;
	open = open->next;
	delete tempopen;
}

////////////////////////////////////////////////////////
//   ������
///////////////////////////////////////////////////////
class AStartFindPath
{
public:
	Node m_node[LENGTH][WIDE];
	AStartFindPath();

	virtual ~AStartFindPath(){};

	void InitNodeMap( char aa[][WIDE], OpenList *open);
	void FindDestinnation(OpenList* open,CloseList* close, char aa[][WIDE]);
	OpenList* FindMinInOpen(OpenList* open);
	bool Insert2OpenList(OpenList* , int x, int y);
	bool IsInOpenList(OpenList*, int x, int y);
	bool IsInCloseList(OpenList*, int x, int y);
	void IsChangeParent(OpenList*, int x, int y);
	bool IsAviable(OpenList* , int x, int y);
	unsigned int DistanceManhattan(int d_x, int d_y, int x, int y);
private:
	unsigned int steps;
	int startpoint_x;
	int startpoint_y;
	int endpoint_x;
	int endpoint_y;
	
};

AStartFindPath::AStartFindPath()
{
	steps=0;
	startpoint_x = -1;
	startpoint_y = -1;
	endpoint_y = -1; 
	endpoint_x = -1;
}
void AStartFindPath::FindDestinnation(OpenList* open,CloseList* close, char aa[][WIDE])
{
	Insert2OpenList(open,startpoint_x,startpoint_y);// ���
	AddNode2Close(close,open);// ���ŵ� close��
	//OpenList* temp=FindMinInOpen(open);
	while(!Insert2OpenList(open, open->opennode->location_x, open->opennode->location_y))
	{
		AddNode2Close(close,open);
		if(open == NULL)
		{
			cout<<"unable to locate exit"<<endl;
			return;
		}
	}
	      Node *tempnode = &m_node[endpoint_x][endpoint_y];
		  while(tempnode->parent->flag!=STARTPOINT)
	{
		tempnode=tempnode->parent;
		aa[tempnode->location_x][tempnode->location_y]='@';
	}
		  m_node;
}
// ��openlist���ҵ���С�� fֵ  �ڵ�
OpenList* AStartFindPath:: FindMinInOpen(OpenList* open)
{
	return open;
}
//////////////////////////////////////////////////////////////////////////
//  ���ٽ��Ľڵ���� openlist��
//				0      1      2  
//				3      S      4
//				5      6      7
/////////////////////////////////////////////////////////////////////////////
bool AStartFindPath::Insert2OpenList(OpenList* open,int center_x, int center_y)
{
	for(int i=0; i<8 ; i++)
	{
		int new_x=center_x + direction[i][0];
		int new_y=center_y+ direction[i][1];

		if(new_x>=0 && new_y>=0 && new_x<LENGTH &&
			new_y<WIDE &&
			IsAviable(open, new_x, new_y))// 0
		{
			if(	m_node[new_x][new_y].flag == DESTINATION)
			{
				m_node[new_x][new_y].parent = &m_node[center_x][center_y];
				return true;
			}
			m_node[new_x][new_y].flag =INOPEN;
			m_node[new_x][new_y].parent = &m_node[center_x][center_y];
			m_node[new_x][new_y].value_h = 
				DistanceManhattan(endpoint_x, endpoint_y, new_x,new_y);//�����پ���

			if(0==i || 2==i||5==i||7==i)
				m_node[new_x][new_y].value_g = m_node[center_x][center_y].value_g+14;
			else
				m_node[new_x][new_y].value_g = m_node[center_x][center_y].value_g+10;

			m_node[new_x][new_y].value_f = m_node[new_x][new_y].value_g+m_node[new_x][new_y].value_h;

			AddNode2Open(open, &m_node[new_x][new_y]);// ���뵽 openlist��
		}
	}
	IsChangeParent(open, center_x,  center_y);
	//if(counts>1000)
	//	return true;
	//else
	return false;
}
// �Ƿ��и��õ�·��
void AStartFindPath::IsChangeParent(OpenList* open,int center_x, int center_y)
{
	for(int i=0; i<8 ; i++)
	{
		int new_x=center_x + direction[i][0];
		int new_y=center_y + direction[i][1];
		if(new_x>=0 && new_y>=0 && new_x<LENGTH &&
			new_y<WIDE &&
			IsInOpenList(open, new_x, new_y))// 0
		{

			if(0==i|| 2==i|| 5==i|| 7==i)
			{
				if(m_node[new_x][new_y].value_g >  m_node[center_x][center_y].value_g+14)
				{
					m_node[new_x][new_y].parent = &m_node[center_x][center_y];
					m_node[new_x][new_y].value_g =   m_node[center_x][center_y].value_g+14;
				}
			}
			else
			{
				if(m_node[new_x][new_y].value_g >   m_node[center_x][center_y].value_g+10)
				{
					m_node[new_x][new_y].parent = &m_node[center_x][center_y];
					m_node[new_x][new_y].value_g =   m_node[center_x][center_y].value_g+10;
				}
			}
		}
	}
}

bool AStartFindPath::IsAviable(OpenList* open, int x, int y)
{
	if(IsInOpenList( open, x, y))
		return false;
	if(IsInCloseList( open, x, y))
		return false;
	if(m_node[x][y].flag == WALL )
		return false;
	else 
		return true;
}
bool AStartFindPath::IsInOpenList(OpenList* openlist, int x,int y)
{
	if(m_node[x][y].flag == INOPEN)
		return true;
	else 
		return false;
}

bool AStartFindPath::IsInCloseList(OpenList* openlist, int x,int y)
{
	if(m_node[x][y].flag == INCLOSE|| m_node[x][y].flag==STARTPOINT)
		return true;
	else 
		return false;
}
//��ʾ��ͼ
void DisplayMap(char aa[][WIDE] )
{
	for(int i=0; i< LENGTH ;i++)
	{
		for(int j=0; j<WIDE; j++)
			cout<<aa[i][j];
		cout<<endl;
	}
}
unsigned int AStartFindPath::DistanceManhattan(int d_x, int d_y, int x, int y)
{
	unsigned int temp = (abs(d_x - x) + abs(d_y-y))*DISTANCE;
	return temp;
}
//��ʼ�� node
void AStartFindPath::InitNodeMap( char aa[][WIDE], OpenList * openlist)
{
	ifstream fin;
	fin.open("starmap.txt");
	if(!fin.is_open())
	{
		cout<<"Failed to open file!"<<endl;
		return ;
	}
	for(int i=0; i< LENGTH; i++)
	{
		for(int j=0; j< WIDE; j++)
		{	
			fin>>aa[i][j];
			m_node[i][j].location_x = i;
			m_node[i][j].location_y = j;
			m_node[i][j].parent = NULL;		
			switch(aa[i][j])
			{
			case '.':
				m_node[i][j].flag = VIABLE;		
				break;
			case 'x':
				m_node[i][j].flag = WALL;
				break;
			case 's':
				m_node[i][j].flag = STARTPOINT;	
				openlist->next = NULL;
				openlist->opennode = &m_node[i][j];//  �����ŵ� OPenList��		
				startpoint_x = i;
				startpoint_y = j;
				break;
			case 'd':
				m_node[i][j].flag = DESTINATION;
				endpoint_x = i;
				endpoint_y = j;
				break;
			}
		}
		fin.get();
	}
	//cout<<aa[startpoint_x][startpoint_y]<<endl;
	fin.close();
}

int main()
{
	AStartFindPath findpath;
	OpenList* openlist = new OpenList;
	CloseList* closelist = new CloseList;
	//closelist=NULL;
	const OpenList* HEADOPEN = openlist;
	const CloseList* HEADCLOSE = closelist;
	closelist->closenode = NULL;

	char aa[LENGTH][WIDE] = {0};

	findpath.InitNodeMap( aa,openlist);
	/*
	findpath.m_node[0][1].value_f=100;
	findpath.m_node[0][2].value_f=400;
	findpath.m_node[0][3].value_f=200;
	findpath.m_node[0][4].value_f=500;
	findpath.m_node[0][5].value_f=10;
	AddNode2Open(openlist, &findpath.m_node[0][1]);
	AddNode2Open(openlist, &findpath.m_node[0][2]);
	AddNode2Open(openlist, &findpath.m_node[0][3]);
	AddNode2Open(openlist, &findpath.m_node[0][4]);
	AddNode2Open(openlist, &findpath.m_node[0][5]);
	AddNode2Close(closelist,openlist);
AddNode2Close(closelist,openlist);
AddNode2Close(closelist,openlist);
AddNode2Close(closelist,openlist);
AddNode2Close(closelist,openlist);
AddNode2Close(closelist,openlist);
AddNode2Close(closelist,openlist);
AddNode2Close(closelist,openlist);
AddNode2Close(closelist,openlist);*/

	findpath.FindDestinnation(openlist,closelist, aa);

	for(int i=0; i<LENGTH ;i++)
	{
		for(int j=0; j<WIDE; j++)
		{
			cout<<aa[i][j];
		}
		cout<<endl;
	}
//	cout<<aa[endpoint/50][endpoint%50]<<endl;
	//DisplayMap(aa);
	
}
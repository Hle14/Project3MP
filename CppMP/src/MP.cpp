#include "MP.hpp"
#include "PseudoRandom.hpp"
#include "MyTimer.hpp"
#include <cstring>

MotionPlanner::MotionPlanner(Simulator * const simulator)
{
    m_simulator = simulator;   

    Vertex *vinit = new Vertex();

    vinit->m_parent   = -1;   
    vinit->m_nchildren= 0;    
    vinit->m_state[0] = m_simulator->GetRobotCenterX();
    vinit->m_state[1] = m_simulator->GetRobotCenterY();

    AddVertex(vinit);
    m_vidAtGoal = -1;
    m_totalSolveTime = 0;
}

MotionPlanner::~MotionPlanner(void)
{
    //do not delete m_simulator  

    const int n = m_vertices.size();
    for(int i = 0; i < n; ++i)
	delete m_vertices[i];
}


void MotionPlanner::ExtendTree(const int vid,const double sto[])
{
	//check if step distance small (i.e. ||vid - sto|| < distOneStep)
	double dx = m_vertices[vid]->m_state[0] - sto[0];
	double dy = m_vertices[vid]->m_state[1] - sto[1];
	double dist = sqrt(pow(dx,2) + pow(dy,2));
	
	double rr = Simulator::GetRobotRadius();
	if(dist > Simulator::m_distOneStep)
	{
		return;
	}
	
	////check for collision of points along line w/ obstacles
	double px = sto[0];
	double py = sto[1];
	
	for(int j=0; j<dist/m_simulator->m_distOneStep; j++)//number of points to check is line segment length/step-size
	{
		//loop checks sto against every obstacle for collision
		const int n = m_simulator->GetNrObstacles();
    
		for(int i = 0; i < n; ++i)//check one point against every obstacle for collision
		{
			const double x = m_simulator->GetObstacleCenterX(i);
			const double y = m_simulator->GetObstacleCenterY(i);
			const double r = m_simulator->GetObstacleRadius(i);
			const double d = sqrt(pow((vx - x),2) + pow((vy - y),2));
	
			if(d < rr + r)
			{
				return; //vertex would put robot in collision with obstacle
			}
		}

		j++;

		//get next point along edge to test for collision
		px += dx * m_simulator->m_distOneStep * (double)j;
		py += dy * m_simulator->m_distOneStep * (double)j;
	}

	/////successful completion of loops = edge not in collision --> add vertex to tree
	//first create the vertex
	Vertex* v = new Vertex();

	v->m_parent   = vid;   
	v->m_nchildren= 0;    
	v->m_state[0] = sto[0];
	v->m_state[1] = sto[1];

	AddVertex(v);
}

void MotionPlanner::ExtendRandom(void)
{
    Clock clk;
    StartTime(&clk);

//your code
    
    m_totalSolveTime += ElapsedTime(&clk);
}

void MotionPlanner::ExtendRRT(void)
{
    Clock clk;
    StartTime(&clk);
 
//your code
    
    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::ExtendEST(void)
{
    Clock clk;
    StartTime(&clk);

//your code    
    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::ExtendMyApproach(void)
{
    Clock clk;
    StartTime(&clk);
 
//your code
    
    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::AddVertex(Vertex * const v)
{
    if(v->m_type == Vertex::TYPE_GOAL)
	m_vidAtGoal = m_vertices.size();
    m_vertices.push_back(v); 
    if(v->m_parent >= 0)
	(++m_vertices[v->m_parent]->m_nchildren);
}

void MotionPlanner::GetPathFromInitToGoal(std::vector<int> *path) const
{
    std::vector<int> rpath;
    
    rpath.clear();
    
    int i = m_vidAtGoal;
    do
    {
	rpath.push_back(i);
	i = m_vertices[i]->m_parent;	
    } 
    while(i >= 0);
    
    path->clear();
    for(int i = rpath.size() - 1; i >= 0; --i)
	path->push_back(rpath[i]);
}

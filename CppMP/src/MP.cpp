#include "MP.hpp"
#include "PseudoRandom.hpp"
#include "MyTimer.hpp"
#include <cstring>
#include <iostream>
#include <unistd.h>

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
	double distx = sto[0] - m_vertices[vid]->m_state[0];
	double disty = sto[1] - m_vertices[vid]->m_state[1];
	double dist = sqrt(pow(distx,2) + pow(disty,2));
	
	double px = m_vertices[vid]->m_state[0];
	double py = m_vertices[vid]->m_state[1];
	double distOneStep = m_simulator->GetDistOneStep();

	double dx = distx / dist * distOneStep;
	double dy = disty / dist * distOneStep;
	
	//loop checks sto against every obstacle for collision
	const int n = m_simulator->GetNrObstacles();

	//for(int j=0; j < dist/distOneStep; j++)//number of points to check is line segment length/step-size
	//{
		//get next point along edge to test for collision
		px += dx;
		py += dy;

		double rr = m_simulator->GetRobotRadius();
		for(int i = 0; i < n; ++i)//check one point against every obstacle for collision
		{
			double x = m_simulator->GetObstacleCenterX(i);
			double y = m_simulator->GetObstacleCenterY(i);
			double r = m_simulator->GetObstacleRadius(i);
			double d = sqrt(pow((px - x),2) + pow((py - y),2));
	
			if((rr + r) > d)
			{
				return; //vertex would put robot in collision with obstacle
			}
		}

		Vertex* v = new Vertex();

		//v->m_parent   = vid;   
		v->m_parent = vid;
		v->m_nchildren= 0;    
		v->m_state[0] = px;
		v->m_state[1] = py;

		AddVertex(v);		
	//}
}

void MotionPlanner::ExtendRandom(void)
{
    Clock clk;
    StartTime(&clk);

//MY CODE
	
	//first uniformly select a random node/vertex in current tree to attach new configuration to
	int vid = (int)PseudoRandomUniformReal(0,m_vertices.size()-1);

	//next select a configuration to attach to selected node/vertex in tree
	double sto[2];
	if (PseudoRandomUniformReal(0,1) <= 0.1) //10% of the time, try to attach a path from a tree branch to q.goal
	{
		sto[0] = m_simulator->GetGoalCenterX();
		sto[1] = m_simulator->GetGoalCenterY();
	} else //90% of the time, sample a random configuration to add to tree
	{
		m_simulator->SampleState(sto);
	}

	//attempt to add configuration sto to the tree
	ExtendTree(vid,sto);
//MY CODE
    
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

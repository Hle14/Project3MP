#include "MP.hpp"
#include "PseudoRandom.hpp"
#include "MyTimer.hpp"
#include <cstring>
#include <iostream>
#include <vector>
#define MYCHOICE 1;
/* mychoice value determines which definition of ExtendMyApproach is compiled

0	selection of vid is deterministic by shortest distance from goal
1	selection of vid is probabilistic w/ weight = 1/(1 + distance from goal)

*/

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

	double rr = m_simulator->GetRobotRadius();

	for(int j=0; j < dist/distOneStep; j++)//number of points to check is line segment length/step-size
	{
		//get next point along edge to test for collision
		px += dx;
		py += dy;

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

		if(j==0)
		{
			v->m_parent = vid;
		} else
		{
			v->m_parent = m_vertices.size() - 1;
		}
		v->m_nchildren= 0;    
		v->m_state[0] = px;
		v->m_state[1] = py;

		AddVertex(v);

		//check dist to goal from v
		double xToGoal = m_simulator->GetGoalCenterX() - px;
		double yToGoal = m_simulator->GetGoalCenterY() - py;
		double vToGoal = sqrt(pow(xToGoal,2) + pow(yToGoal,2));
		double rr = m_simulator->GetRobotRadius();
		//if(vToGoal <= 0.5) //TRY TO USE GOAL CENTER/RADIUS AND ROBOT CENTER/RADIUS HERE INSTEAD
		if(vToGoal <= rr)
		{
			m_vidAtGoal = v->m_parent;
			break;
		}
	}
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
 
//MY CODE

	//first select a configuration to attach to selected node/vertex in tree
	double sto[2];
	if (PseudoRandomUniformReal(0,1) <= 0.1) //10% of the time, try to attach a path from a tree branch to q.goal
	{
		sto[0] = m_simulator->GetGoalCenterX();
		sto[1] = m_simulator->GetGoalCenterY();
	} else //90% of the time, sample a random configuration to add to tree
	{
		m_simulator->SampleState(sto);
	}

	//next determine which vertex in tree is closest to new configuration
	int vid = 0;
	double min_dist = 10000000; //default arbitrarily large min_dist

	for(int i=0; i<=m_vertices.size() - 1; i++)
	{
		//calculate distance to new configuration for each vertex
		double distx = sto[0] - m_vertices[i]->m_state[0];
		double disty = sto[1] - m_vertices[i]->m_state[1];
		double dist = sqrt(pow(distx,2) + pow(disty,2));

		if(dist < min_dist) //keep track of the vertex nearest to new config
		{
			vid = i;
			min_dist = dist;
		}
	}

	//attempt to add configuration sto to the tree
	ExtendTree(vid,sto);
//MY CODE
    
    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::ExtendEST(void)
{
    Clock clk;
    StartTime(&clk);

//MY CODE 

	//first select a configuration to attach to selected node/vertex in tree
	double sto[2];
	if (PseudoRandomUniformReal(0,1) <= 0.1) //10% of the time, try to attach a path from a tree branch to q.goal
	{
		sto[0] = m_simulator->GetGoalCenterX();
		sto[1] = m_simulator->GetGoalCenterY();
	} else //90% of the time, sample a random configuration to add to tree
	{
		m_simulator->SampleState(sto);
	}

	////next determine which vertex in tree should be added based on no. of children

	//find sum of m_nchildren for all vertices in tree
	double total_weight = 0;
	
	std::vector<double> partial_weight; //each index holds the sum weight of all vertices w/ vid < index
	for(int i=0; i<=m_vertices.size()-1; i++)
	{
		total_weight += 1.0/(1.0 + m_vertices[i]->m_nchildren); //weight of each vertex inversely proportional to its degree
		partial_weight.push_back(total_weight);
	}

	double random_weight = PseudoRandomUniformReal(0,total_weight);
	int vid = 0;
	for(int i=0; i<=partial_weight.size()-1; i++)
	{
		if(random_weight <= partial_weight[i])
		{
			vid = i;
			break;
		}
	}
	//attempt to add configuration sto to the tree
	ExtendTree(vid,sto);
//MY CODE */
    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::ExtendMyApproach(void)
{
    Clock clk;
    StartTime(&clk);
 
//MY CODE
	//first select a configuration to attach to selected node/vertex in tree
	double sto[2];
	if (PseudoRandomUniformReal(0,1) <= 0.1) //10% of the time, try to attach a path from a tree branch to q.goal
	{
		sto[0] = m_simulator->GetGoalCenterX();
		sto[1] = m_simulator->GetGoalCenterY();
	} else //90% of the time, sample a random configuration to add to tree
	{
		m_simulator->SampleState(sto);
	}

	////next select the node/vertex in tree to attach sto to

	//calculate distances from each vertex to goal
	double gx = m_simulator->GetGoalCenterX();
	double gy = m_simulator->GetGoalCenterY();

	int vid = 0;

#if !MYCHOICE //deterministic by distance
	double min_dist = 1000000; //arbitrarily large starting minimum distance
	for(int i=0; i<=m_vertices.size()-1; i++)
	{
		
		double distx = gx - m_vertices[i]->m_state[0];
		double disty = gy - m_vertices[i]->m_state[1];
		double dist = sqrt(pow(distx,2) + pow(disty,2));

		if(dist < min_dist)
		{
			
			vid = i;
			min_dist = dist;
		}

	}
#elif MYCHOICE//probabilistic by distance

	double total_dist = 0;
	std::vector<double> dist;

	//calculate the sums 0 to vid for each vertex
	for(int i=0; i<=m_vertices.size()-1; i++)
	{	
		double distx = gx - m_vertices[i]->m_state[0];
		double disty = gy - m_vertices[i]->m_state[1];
		
		//total_dist += 1.0/(1.0 + sqrt(pow(distx,2) + pow(disty,2))); //w = 1/(1 + dist)
		total_dist += 1.0/pow((1.0 + sqrt(pow(distx,2) + pow(disty,2))),2); // w = 1/(1 + dist)^2 stronger effect
		dist.push_back(total_dist);
		
		loopcount++;
	}

	//obtain a random weight
	double rand_weight = PseudoRandomUniformReal(0,total_dist);

	//determine which vertex corresponds to the weight (i.e. a sum of vertices w/ i=0:vid)
	for(int i=0; i<dist.size(); i++)
	{
		if(rand_weight <= dist[i])
		{
			vid = i;
			break;
		}
	}
#endif

	ExtendTree(vid,sto);
	
//MY CODE*/
    
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

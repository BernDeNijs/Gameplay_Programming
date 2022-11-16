#include "stdafx.h"
#include "ENavGraph.h"
#include "framework\EliteAI\EliteGraphs\EliteGraphAlgorithms\EAStar.h"

using namespace Elite;

Elite::NavGraph::NavGraph(const Polygon& contourMesh, float playerRadius = 1.0f) :
	Graph2D(false),
	m_pNavMeshPolygon(nullptr)
{
	//Create the navigation mesh (polygon of navigatable area= Contour - Static Shapes)
	m_pNavMeshPolygon = new Polygon(contourMesh); // Create copy on heap

	//Get all shapes from all static rigidbodies with NavigationCollider flag
	auto vShapes = PHYSICSWORLD->GetAllStaticShapesInWorld(PhysicsFlags::NavigationCollider);

	//Store all children
	for (auto shape : vShapes)
	{
		shape.ExpandShape(playerRadius);
		m_pNavMeshPolygon->AddChild(shape);
	}

	//Triangulate
	m_pNavMeshPolygon->Triangulate();

	//Create the actual graph (nodes & connections) from the navigation mesh
	CreateNavigationGraph();
}

Elite::NavGraph::~NavGraph()
{
	delete m_pNavMeshPolygon; 
	m_pNavMeshPolygon = nullptr;
}

int Elite::NavGraph::GetNodeIdxFromLineIdx(int lineIdx) const
{
	auto nodeIt = std::find_if(m_Nodes.begin(), m_Nodes.end(), [lineIdx](const NavGraphNode* n) { return n->GetLineIndex() == lineIdx; });
	if (nodeIt != m_Nodes.end())
	{
		return (*nodeIt)->GetIndex();
	}

	return invalid_node_index;
}

Elite::Polygon* Elite::NavGraph::GetNavMeshPolygon() const
{
	return m_pNavMeshPolygon;
}

void Elite::NavGraph::CreateNavigationGraph()
{
	//1. Go over all the edges of the navigationmesh and create nodes

	//A:
	//loop over all lines
	for (auto& lines : m_pNavMeshPolygon->GetLines())
	{
		//check if line is connected to another triangle
		int lineIdx = lines->index;
		if (m_pNavMeshPolygon->GetTrianglesFromLineIndex(lineIdx).size() >= 2)
		{
			//create node on graph & give it idx
			AddNode(new NavGraphNode(this->GetNextFreeNodeIndex(),
				lineIdx,
				(lines->p1+lines->p2)/2.f
				));
		}
	}
	//2. Create connections now that every node is created
	//B:
	for (auto& triangles : m_pNavMeshPolygon->GetTriangles())
	{
		std::vector<int> validNodes{};
		for (auto& triIdx :triangles->metaData.IndexLines)
		{
			//check if valid node exists
			if (GetNodeIdxFromLineIdx(triIdx) != invalid_node_index)
			{
				//push node idx not tri idx
				validNodes.push_back(GetNodeIdxFromLineIdx(triIdx));
			}
		}
		//if 2 valid nodes connect them
		if (validNodes.size() == 2)
		{
			
		}
		//if 3 valid nodes connect them
		else if (validNodes.size() == 3)
		{
			AddConnection(new GraphConnection2D(validNodes[0], validNodes[1]));
			AddConnection(new GraphConnection2D(validNodes[1], validNodes[2]));
			AddConnection(new GraphConnection2D(validNodes[2], validNodes[0]));
		}
		else
		{
			std::cout << "too many nodes , ENavGraph.cpp , B \n";
		}
	}

	//3. Set the connections cost to the actual distance
	//C:

	SetConnectionCostsToDistance();

}


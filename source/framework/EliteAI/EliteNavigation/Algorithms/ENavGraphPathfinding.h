#pragma once
#include <vector>
#include <iostream>
#include "framework/EliteMath/EMath.h"
#include "framework\EliteAI\EliteGraphs\ENavGraph.h"
#include "framework\EliteAI\EliteGraphs\EliteGraphAlgorithms\EAStar.h"

namespace Elite
{
	class NavMeshPathfinding
	{
	public:
		static std::vector<Vector2> FindPath(Vector2 startPos, Vector2 endPos, NavGraph* pNavGraph, std::vector<Vector2>& debugNodePositions, std::vector<Portal>& debugPortals)
		{

			//Create the path to return
			std::vector<Vector2> finalPath{};

			//Get the start and endTriangle
			const Triangle* startTriangle{ pNavGraph->GetNavMeshPolygon()->GetTriangleFromPosition(startPos) };
			const Triangle* endTriangle{ pNavGraph->GetNavMeshPolygon()->GetTriangleFromPosition(endPos) };
			//We have valid start/end triangles and they are not the same
			
			if (startTriangle == nullptr)
			{
				return finalPath;
			}
			if (endTriangle == nullptr)
			{
				return finalPath;
			}
			if (startTriangle == endTriangle)
			{
				finalPath.push_back(endPos);
				return finalPath;
			}
	

			//=> Start looking for a path
			//Copy the graph
			auto graphCopy{ pNavGraph->Clone()};
			
			//Create extra node for the Start Node (Agent's position

			const auto& startNode{ new NavGraphNode(graphCopy->GetNextFreeNodeIndex(),-1,startPos) };
			graphCopy->AddNode(startNode);


			for (auto& lineIdx : startTriangle->metaData.IndexLines)
			{
				auto p1 = pNavGraph->GetNavMeshPolygon()->GetLines()[lineIdx]->p1;
				auto p2 = pNavGraph->GetNavMeshPolygon()->GetLines()[lineIdx]->p2;
				auto tempNodePos = (p1 + p2) / 2.f;
				const auto& tempNode{ graphCopy->GetNodeAtWorldPos(tempNodePos) };
				if (tempNode == nullptr)
				{
					continue;
				}
				if (tempNode == startNode)
				{
					continue;
				}

				auto newConnection{ new GraphConnection2D(startNode->GetIndex(), tempNode->GetIndex(),Distance(startPos,tempNodePos)) };
				if (graphCopy->IsUniqueConnection(newConnection->GetFrom(), newConnection->GetTo()))
				{
					graphCopy->AddConnection(newConnection);
				}
				else
				{
					delete newConnection;
				}
			}

			//Create extra node for the endNode
			const auto& endNode{new NavGraphNode(graphCopy->GetNextFreeNodeIndex(),-1,endPos) };
			graphCopy->AddNode(endNode);

			for (auto& lineIdx : endTriangle->metaData.IndexLines)
			{
				auto p1 = pNavGraph->GetNavMeshPolygon()->GetLines()[lineIdx]->p1;
				auto p2 = pNavGraph->GetNavMeshPolygon()->GetLines()[lineIdx]->p2;
				auto tempNodePos = (p1 + p2) / 2.f;
				const auto& tempNode{ graphCopy->GetNodeAtWorldPos(tempNodePos) };
				if (tempNode == nullptr)
				{
					continue;
				}
				if (tempNode == endNode)
				{
					continue;
				}
				auto newConnection{ new GraphConnection2D(endNode->GetIndex(), tempNode->GetIndex(), Distance(endPos, tempNodePos)) };
				if (graphCopy->IsUniqueConnection(newConnection->GetFrom(),newConnection->GetTo()))
				{
					graphCopy->AddConnection(newConnection);
				}
				else
				{
					delete newConnection;
				}
				

			}
			//Run A star on new graph
			auto aStar = AStar<NavGraphNode, GraphConnection2D>(graphCopy.get(), HeuristicFunctions::Chebyshev);
			const auto tempPath{ aStar.FindPath(startNode,endNode) };
			//
			//for (const auto& nodes : tempPath)
			//{
			//	finalPath.push_back(nodes->GetPosition());
			//}
			////OPTIONAL BUT ADVICED: Debug Visualisation
			//debugNodePositions = finalPath;
			////Run optimiser on new graph, MAKE SURE the A star path is working properly before starting this section and uncommenting this!!!
			////m_Portals = SSFA::FindPortals(nodes, m_pNavGraph->GetNavMeshPolygon());
			////finalPath = SSFA::OptimizePortals(m_Portals);

			return finalPath;
		}
	};
}

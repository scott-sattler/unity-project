using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AStarGrid : MonoBehaviour
{
    public bool debugging = true;
    public GameObject debugging_ObjectStart;
    public GameObject debugging_ObjectFinish;
    public static bool debugging_RunAStar = true;
    public bool debugging_Continious = false; // TODO: debugging_Continious needs optimization.
    public bool debugging_Postprocessor = false;

    Vector2Int gridLocationPrevious;
    Vector2Int gridLocationFinish;
	
	// Update is called once per frame
	void Update () {

        if (debugging_RunAStar && debugging)
        {
            List<Vector2Int> aStarList = new List<Vector2Int>();

            Vector2Int gridLocationStart = Utilities.SpaceToGrid(debugging_ObjectStart.transform.position);
            if (debugging) Debug.Log("running A* from: " + "[ " + gridLocationStart[0] + " , " + gridLocationStart[1] + " ].");

            gridLocationFinish = Utilities.SpaceToGrid(debugging_ObjectFinish.transform.position);
            if (debugging) Debug.Log("running A* to: " + "[ " + gridLocationFinish[0] + " , " + gridLocationFinish[1] + " ].");

            // Reset grid on move, if debuggingContinious.
            if (gridLocationPrevious != null && debugging_Continious && !EqualGridLocations(gridLocationPrevious, gridLocationFinish))
            {
                foreach (CombinedData tile in GlobalVariables.objectInformationArray)
                    tile.ObjectHighlight.SetActive(false);

                gridLocationPrevious = gridLocationFinish;
            }

            if (debugging_RunAStar || (gridLocationPrevious != null && !EqualGridLocations(gridLocationPrevious, gridLocationFinish)))
            {
                aStarList = AStarSearch(gridLocationStart, gridLocationFinish);

                foreach (Vector2Int tile in aStarList)
                {
                    if (debugging) GlobalVariables.objectInformationArray[tile[0], tile[1]].ObjectHighlight.SetActive(true);
                }

                // accepts int[] if (debugging) Debug.Log("aStarList: " + Utilities.PrintArray(null, aStarList, true));
            }

            if (!debugging_Continious)
                debugging_RunAStar = false;

            gridLocationPrevious = gridLocationFinish;
        }

        if (Input.GetKeyDown(KeyCode.C))
        {
            foreach (CombinedData tile in GlobalVariables.objectInformationArray)
                tile.ObjectHighlight.SetActive(false);
        }
	}

    // Second implementation - copying structure of Wikipedia pseudocode.
    // Made readability and performance improvements.
    public List<Vector2Int> AStarSearch(Vector2Int startNodeLocation, Vector2Int goalNodeLocation)
    {
        int count = 0;

        List<NodeData> exploredNodes = new List<NodeData>();    // closed set.
        List<NodeData> evaluatedNodes = new List<NodeData>();   // open set.
        List<NodeData> discoveredNodes = new List<NodeData>();
        List<Vector2Int> bestPathFound = new List<Vector2Int>();
        NodeData currentNode = new NodeData();

        // Manual evaluation for seeding while loop.
        NodeData startNode = new NodeData
        {
            Location = startNodeLocation,
            Gn = 0,
            Hn = Heuristic(startNodeLocation, goalNodeLocation),
            Fn = Heuristic(startNodeLocation, goalNodeLocation), // TODO: How to set both equal without using another var.
            CameFrom = new List<Vector2Int>(),
        };
        currentNode = startNode;
        evaluatedNodes.Add(currentNode);

        while (evaluatedNodes.Count > 0)
        {
            // Find the node with the lowest f(x), in the open (evaluated) set.
            int bestNodeIndex = 0;
            float bestFn = Mathf.Infinity;
            for (int i = 0; i < evaluatedNodes.Count; i++)
            {
                if (evaluatedNodes[i].Fn < bestFn)
                {
                    bestNodeIndex = i;
                    bestFn = evaluatedNodes[i].Fn;
                }
            }

            // Move to the lowest f(x) node.
            currentNode = evaluatedNodes[bestNodeIndex];

            // Construct best path (excluding current node).
            bestPathFound = currentNode.CameFrom;

            // Test for solution.
            if (EqualGridLocations(currentNode.Location, goalNodeLocation))
            {
                if (debugging) Debug.Log("Solution found in [" + count + "] while loops.");
                bestPathFound.Add(currentNode.Location);
                return bestPathFound;
            }

            // Discover nodes from the current node.
            discoveredNodes = DiscoveredNodes(currentNode.Location);

            // Find f(x) and construct discovered nodes.
            for (int i = 0; i < discoveredNodes.Count; i++)
            {
                // Exclude closed and already discovered nodes.
                if (NodeExists(discoveredNodes[i], exploredNodes) || NodeExists(discoveredNodes[i], evaluatedNodes))
                    continue;

                // f(x) = g(x) + h(x)
                discoveredNodes[i].Gn = currentNode.Gn + 1;
                discoveredNodes[i].Hn = Heuristic(discoveredNodes[i].Location, goalNodeLocation);
                discoveredNodes[i].Fn = discoveredNodes[i].Gn + discoveredNodes[i].Hn;

                // CameFrom information.
                discoveredNodes[i].CameFrom = currentNode.CameFrom; // Deep copy built in.
                discoveredNodes[i].AddCameFrom(currentNode.Location);

                // Add to evaluated set.
                evaluatedNodes.Add(discoveredNodes[i]);
            }
            // Remove all moved and duplicate nodes (all nodes).
            discoveredNodes.Clear();

            // Move current node from evaluated (open) to explored (closed) nodes.
            exploredNodes.Add(evaluatedNodes[bestNodeIndex]);
            evaluatedNodes.RemoveAt(bestNodeIndex);

            count++;
            // PrintArray only accepts int[] if (debugging) Debug.Log("Best partial path found: " + Utilities.PrintArray(null, bestPathFound, true) + Utilities.PrintArray(currentNode.Location, null, true));

            // This can be used to show every explored space. (Needs to use a different visual representation.)
            //GlobalVariables.objectInformationArray[currentNode.Location[0], currentNode.Location[1]].ObjectHighlight.SetActive(true);
        }

        // If no path is found, start position is returned.
        if (debugging) Debug.Log("No path could be found!");
        //return new List<Vector2Int> { startNodeLocation };

        // If no path is found, null is returned.
        return null;
    }

    public List<Vector2Int> PostProcessed_CornerSmoothing(List<Vector2Int> AStarPath)
    {
        List<Vector2Int> postProcessedPath = AStarPath;
        int currentIndex = 0; // Start index of 3 node group. // TODO: Rename.
        //Vector2Int currentSubCornerStart;
        Vector2Int secondNodeDirection = Vector2Int.zero;
        Vector2Int thirdNodeDirection = Vector2Int.zero;
        Vector2Int insideCornerNode = Vector2Int.zero;

        // All corners are 3 node groups. ("Guard Clause".)
        if (AStarPath.Count < 2)
            return AStarPath;

        while (currentIndex < AStarPath.Count - 2)  // -1 from 0th index; -1 from 3 node group.
        {
            // Find direction between node1 and node2. (Second node minus first.)
            secondNodeDirection[0] = AStarPath[currentIndex + 1][0] - AStarPath[currentIndex][0];
            secondNodeDirection[1] = AStarPath[currentIndex + 1][1] - AStarPath[currentIndex][1];

            // Find direction between node2 and node3. (Third node minus second.)
            thirdNodeDirection[0] = AStarPath[currentIndex + 2][0] - AStarPath[currentIndex + 1][0];
            thirdNodeDirection[1] = AStarPath[currentIndex + 2][1] - AStarPath[currentIndex + 1][1];

            // Test for change in direction. (Does the difference get expectedly larger?)
            bool cond1 = Math.Abs(secondNodeDirection[0] + thirdNodeDirection[0]) == 1;
            bool cond2 = Math.Abs(secondNodeDirection[1] + thirdNodeDirection[1]) == 1;
            if (cond1 && cond2) // Corner found.
            {
                // Find node inside corner (From first node, go the direction from the second to third nodes.
                insideCornerNode[0] = AStarPath[currentIndex][0] + thirdNodeDirection[0];
                insideCornerNode[1] = AStarPath[currentIndex][1] + thirdNodeDirection[1];
                // TODO: PrintArray int[] if (debugging_postprocessor) Debug.Log("Corner found at: " + Utilities.PrintArray(insideCornerNode, null, true));

                // Test if the space inside the corner is occupied.
                if (GlobalVariables.tileInformationArray[insideCornerNode[0], insideCornerNode[1]].Traversable)
                {
                    // Create a line from node1 to node3. (Remove node2 from path.)
                    postProcessedPath.RemoveAt(currentIndex + 1);
                    currentIndex += 1;
                }
                else
                    currentIndex += 2;
            }
            // If no change in direction is detected, move to node3.
            else
                currentIndex += 2;
        }

        // Return post processed path.
        return postProcessedPath;
    }

    // Straight line distance between current and goal nodes.
    float Heuristic(Vector2Int currentNodeLocation, Vector2Int goalNodeLocation)
    {
        float heuristic = Mathf.Infinity;

        Vector2 currentNodeWorld = Utilities.GridToSpace(currentNodeLocation);
        Vector2 goalNodeWorld = Utilities.GridToSpace(goalNodeLocation);

        heuristic = Vector2.Distance(currentNodeWorld, goalNodeWorld);

        return heuristic;
    }

    // Discovers up-to 8 nodes adjacent to the current node.
    List<NodeData> DiscoveredNodes(Vector2Int currenNode)
    {
        List<NodeData> discoveredNodes = new List<NodeData>();

        // Construct list of possible nodes (reusing code before NodeData).
        List<Vector2Int> discoveredNodesInt = new List<Vector2Int>
        {
            // Up, right, down, left.
            new Vector2Int ( currenNode[0] + 0, currenNode[1] + 1 ),
            new Vector2Int ( currenNode[0] + 1, currenNode[1] + 0 ),
            new Vector2Int ( currenNode[0] - 1, currenNode[1] + 0 ),
            new Vector2Int ( currenNode[0] + 0, currenNode[1] - 1 )
            // NE, SE, SW, NW.
            //new int[2] { currenNode[0] + 1, currenNode[1] + 1 },
            //new int[2] { currenNode[0] + 1, currenNode[1] - 1 },
            //new int[2] { currenNode[0] - 1, currenNode[1] - 1 },
            //new int[2] { currenNode[0] - 1, currenNode[1] + 1 }
        };

        // Get bounds.
        var length0 = GlobalVariables.objectInformationArray.GetLength(0);
        var length1 = GlobalVariables.objectInformationArray.GetLength(1);

        // Inspect 0th element for bounds.
        for (int i = discoveredNodesInt.Count - 1; i >= 0; i--)
        {
            if (discoveredNodesInt[i][0] < 0 || discoveredNodesInt[i][0] > length0 - 1)
                discoveredNodesInt.RemoveAt(i);
        }

        // Inspect 1st element for bounds.
        for (int i = discoveredNodesInt.Count - 1; i >= 0; i--)
        {
            if (discoveredNodesInt[i][1] < 0 || discoveredNodesInt[i][1] > length1 - 1)
                discoveredNodesInt.RemoveAt(i);
        }

        // Inspect spaces for occupation.
        for (int i = discoveredNodesInt.Count - 1; i >= 0; i--)
        {
            if (!GlobalVariables.tileInformationArray[discoveredNodesInt[i][0], discoveredNodesInt[i][1]].Traversable)
                discoveredNodesInt.RemoveAt(i);
        }

        // PrintArray only accepts int[] if (debugging) Debug.Log(discoveredNodesInt.Count + " nodes discovered. " + Utilities.PrintArray(null, discoveredNodesInt, true));

        // Construct NodeData.
        foreach (Vector2Int discoveredNode in discoveredNodesInt)
        {
            NodeData newNodeData = new NodeData
            {
                Location = discoveredNode
            };
            discoveredNodes.Add(newNodeData);
        }

        return discoveredNodes;
    }

    // Check for discovered nodes in the evaluated list.
    bool NodeExists(NodeData discoveredNode, List<NodeData> evaluatedNodes)
    {
        foreach (NodeData evaluatedNode in evaluatedNodes)
        {
            bool condition = EqualGridLocations(discoveredNode.Location, evaluatedNode.Location);
            if (condition)
                return true;
        }
        return false;
    }

    // Also occurs in Utilities.
    //bool EqualGridLocations(int[] locationOne, int[] locationTwo)
    //{
    //    if (locationOne[0] == locationTwo[0] && locationOne[1] == locationTwo[1])
    //    {
    //        return true;
    //    }
    //    return false;
    //}
    // OVERLOAD.
    bool EqualGridLocations(Vector2Int locationOne, Vector2Int locationTwo)
    {
        if (locationOne.x == locationTwo.x && locationOne.y == locationTwo.y)
        {
            return true;
        }
        return false;
    }

}


/// <summary> int[] grid location; 3 floats f(x), g(x), h(x); List<int[]> grid path. </summary>
public class NodeData
{
    public Vector2Int Location { get; set; } = new Vector2Int();

    public float Fn { get; set; } = Mathf.Infinity;

    public float Gn { get; set; } = Mathf.Infinity;

    public float Hn { get; set; } = Mathf.Infinity;

    public List<Vector2Int> cameFrom = new List<Vector2Int>();

    public List<Vector2Int> CameFrom
    {
        get { return cameFrom; }
        set
        {
            List<Vector2Int> setCameFrom = new List<Vector2Int>();

            foreach (Vector2Int listElement in value)
                setCameFrom.Add(listElement);

            cameFrom = setCameFrom;
        }
    }

    public void AddCameFrom(Vector2Int listElement)
    {
        cameFrom.Add(listElement);
    }

    public List<Vector2Int> DeepCopy_CameFrom()
    {
        List<Vector2Int> copy = new List<Vector2Int>();

        foreach (Vector2Int listElement in cameFrom)
            copy.Add(listElement);

        return copy;
    }
}
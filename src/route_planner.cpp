#include "route_planner.h"
#include <algorithm>
using std::sort;
using std::vector;
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    /* save closeset nodes to the user coordinates as start node and end node */
    start_node = &(m_Model.FindClosestNode (start_x ,start_y) ); 
    end_node = &(m_Model.FindClosestNode (end_x ,end_y) ); 
}

 /* method to Calculate heustrical value */ 
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
        return node->distance(*end_node) ;
}


/* method to expand the current node by adding all unvisited neighbors to the open list*/
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
        current_node->FindNeighbors() ; 
        /*iterate over all neighbors */
        for (auto& neighbor_node : current_node->neighbors)
        {
            /* set the current node as parent for neighbors */
            neighbor_node->parent = current_node ; 
            /* calculate g value for the neighbors  */
            neighbor_node->g_value =  current_node->distance(*neighbor_node) + current_node->g_value;
            /* calculate h value for the neighbors */
            neighbor_node->h_value = CalculateHValue( neighbor_node) ; 
            /*add neighbor node to the open nodes list */ 
            open_list.push_back(neighbor_node) ;
            /* mark the node as visited */
            neighbor_node->visited =  true ;
        }
        
}

/* function to compare between two nodes and return true if the first node has greater f value*/ 
bool Compare(RouteModel::Node * a, RouteModel::Node * b) {
  return  ((a->g_value + a->h_value) > ( b->g_value + b->h_value)) ; 
}

/* method to sort the open list and return the next node. */
RouteModel::Node *RoutePlanner::NextNode() {
    RouteModel::Node *temp ; 
    /* Sort the open_list according to the sum of the h value and g value.  */
    sort(RoutePlanner::open_list.begin(), RoutePlanner::open_list.end() , Compare ); 
    temp = open_list.back() ; 
    /* remove the node from open_list */
    open_list.pop_back();
    return temp ;
}


/* method to return the final path found from your A* search.*/
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

   RouteModel::Node * crnt_tmp ;
   crnt_tmp = current_node;
   while ( crnt_tmp->parent != nullptr)
   {  
        distance = distance + crnt_tmp->distance(*crnt_tmp->parent) ; 
        path_found.emplace_back(*(crnt_tmp));  
        crnt_tmp = crnt_tmp->parent ; 
   }
    path_found.push_back(*(start_node)); 

    std::reverse(path_found.begin(), path_found.end());
    
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

/* the A* Search algorithm */
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    
    start_node->g_value = 0.0;
    start_node->visited = true;
    AddNeighbors(start_node) ; 
    while(!open_list.empty())
    { 
        current_node = NextNode() ; 
       if (current_node == end_node)
       {
             m_Model.path = ConstructFinalPath(current_node);
             break ;
       }  
       else
       {
           /* code */
            AddNeighbors(current_node) ; 
       }

    }   
}
#include "route_planner.h"
#include <algorithm>
#include <vector>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Used m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
 RouteModel::Node *n1=&m_Model.FindClosestNode(start_x, start_y);
 RouteModel::Node *n2=&m_Model.FindClosestNode(end_x, end_y);
    // Stored the nodes in the RoutePlanner's start_node and end_node attributes.
 start_node=n1;
 end_node=n2; 
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  	return (*node).distance((*end_node));
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
float old_g = 0.0;
  
current_node->FindNeighbors();

 for (auto neighbor : current_node->neighbors){
   	 if (!(*neighbor).visited)
     {
		(*neighbor).parent=current_node;
   		old_g=(*current_node).g_value;
		(*neighbor).h_value=CalculateHValue(neighbor);
		(*neighbor).g_value=old_g + (*neighbor).distance(*current_node);
		(*neighbor).visited=true;
		open_list.push_back(neighbor);
          }
  } 
}

bool Compare(const RouteModel::Node* a, const RouteModel::Node* b){
	return (a->h_value + a->g_value) > (b->h_value + b->g_value);
}

RouteModel::Node *RoutePlanner::NextNode() {
sort(open_list.begin(), open_list.end(), Compare);  
auto Lowest_f = open_list.back();
open_list.pop_back();
return Lowest_f;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Created path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
	RouteModel::Node current_node_val=*current_node;
   
	while( current_node_val.parent!=nullptr)
    {
    	path_found.push_back(current_node_val);
      	RouteModel::Node *parent_node=current_node_val.parent;
		distance += (*parent_node).distance(current_node_val);
		current_node_val = (*parent_node);
		 }

  	path_found.push_back(current_node_val);
  	std::reverse(path_found.begin(),path_found.end());
  
    distance *= m_Model.MetricScale(); 
    return path_found;
}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
	start_node->visited = true;
	start_node->h_value = CalculateHValue(start_node);

	current_node=start_node;
	open_list.push_back(current_node);
  	
  	RouteModel::Node *Next_node;
  
	while(Next_node!=end_node)
    {
    Next_node=NextNode();
  	AddNeighbors(Next_node);
	}
	m_Model.path=ConstructFinalPath(Next_node);
}

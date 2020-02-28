#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    this->start_node = &model.FindClosestNode(start_x, start_y);
    this->end_node = &model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	return node->distance(*this->end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	current_node->FindNeighbors();
	for(auto neighbor : current_node->neighbors){
		neighbor->parent = current_node;
		neighbor->h_value = this->CalculateHValue(neighbor);
		neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
		neighbor->visited = true;
		this->open_list.push_back(neighbor);
	}

}

bool CompareNodes(const RouteModel::Node* a, const RouteModel::Node* b) {
	return (a->h_value + a->g_value) > ( b->h_value + b->g_value);
}

RouteModel::Node *RoutePlanner::NextNode() {
	std::sort(this->open_list.begin(), this->open_list.end(), CompareNodes);
	auto node = this->open_list.back();
	this->open_list.pop_back();
	return node;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node != nullptr){
        path_found.insert(path_found.begin(), *current_node);
        if (current_node->parent) // start node does not have parent
            distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }   

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    this->distance = distance;
    return path_found;

}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = this->start_node;
    current_node->h_value = this->CalculateHValue(this->end_node);
    current_node->g_value = 0;
    current_node->visited = true;
    this->open_list.push_back(current_node);

    while (this->open_list.size() > 0) {
        current_node = this->NextNode();
        if (current_node->x == this->end_node->x && current_node->y == this->end_node->y) {
            this->open_list.push_back(current_node);
            break;
	}
        this->AddNeighbors(current_node);
    }
    m_Model.path = this->ConstructFinalPath(current_node);
}

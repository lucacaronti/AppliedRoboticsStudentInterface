#include "dijkstra.hpp"
#include <queue>
#include <vector>
#include <iostream>
#include <algorithm>

#define INF_D 2e63

namespace cv{
    /* Overload operator < for cv::Point in order to use map container */
    bool operator<(const Point2d& l, const Point2d& r){
        return (l.x < r.x) || (l.x == r.x && l.y < r.y);
    }
}

Dijkstra::Dijkstra(){}
Dijkstra::~Dijkstra(){}
/*!
  * @brief Print all nodes connections
  */
void Dijkstra::print() const{
    std::cout<<"Numer of nodes: "<<nodes.size()<<std::endl;
    for(auto itm = nodes.begin(); itm != nodes.end(); itm++){
        std::cout << "node(" << itm->first.x << "," << itm->first.y << ")" << std::endl;
        for(auto itl = itm->second.begin(); itl != itm->second.end(); itl++){
            std::cout << "\t -> (" << itl->second.x << "," << itl->second.y << ") w=" << itl->first << std::endl;
        }
    }
}

/*!
  * @brief Add a edge in the graph
  * @param[in]  cv::Point2d start   start point of the edge
  * @param[in]  cv::Point2d end     end point of the edge
  * @param[in]  double weight       the weight of the edge
  * @return[bool] false if the edge already exists, true otherwise
  */
bool Dijkstra::addEdge(const cv::Point2d start, const cv::Point2d end, const double weight){
    auto start_node = nodes.find(start);
    auto end_node = nodes.find(end);

    // Add the first connection (start (to)-> end)
    if(start_node != nodes.end()){
        //search if connection already exists
        std::list<distNode_t>::iterator itl;
        bool exists = false;
        for(itl = start_node->second.begin(); itl != start_node->second.end(); itl++){
            if(itl->second == end){
                exists = true;
                break;
            }
        }

        if(!exists)
            // Doesn't exist, add it
            start_node->second.emplace_back(weight, end);
        else
            return false; // Already exists -> return falase
    }else{
        std::list<distNode_t> tmp_list;
        tmp_list.emplace_back(weight, end);
        nodes.emplace(start,tmp_list);
    }

    // Add the opposite connection (end (to)-> starts)
    if(end_node != nodes.end()){
        end_node->second.emplace_back(weight, start);
    }else{
        std::list<distNode_t> tmp_list;
        tmp_list.emplace_back(weight, start);
        nodes.emplace(end, tmp_list);
    }
    return true;
}

/*!
  * @brief Finds the shortest path with Dijkstra algorithm
  * @param[in]  cv::Point2d start   start point 
  * @param[in]  cv::Point2d end     end point
  * @param[in/out]  std::vector<cv::Point2d>& best_path resulting best path
  * @return[bool] false if no path is found, true otherwise
  */
bool Dijkstra::shortesPath(const cv::Point2d start, const cv::Point2d end, std::vector<cv::Point2d>& best_path) const
{
    // Create a prority queue
    std::priority_queue<distNode_t, std::vector<distNode_t>, std::greater<distNode_t> > pq;

    // Create a map of best parent node
    std::map<cv::Point2d, cv::Point2d> best_paretn;

    // Create a map to store distance of points (from src)
    std::map<cv::Point2d, double> dist;

    // Init all the nodes dist as infinity, and set as best_parent themselves
    for(auto itm = nodes.begin(); itm != nodes.end(); itm++){
        dist.insert(std::make_pair(itm->first, INF_D));
        best_paretn.insert(std::make_pair(itm->first, itm->first));
    }
    // Set distance of start (from start) as 0
    dist[start] = 0;

    // Push in the priority queue the fisrt point to analyze (start)
    pq.push(std::make_pair(0, start));

    cv::Point2d current_node(start);

    // Until there are point to analyze:
    while (!pq.empty()){

        current_node = pq.top().second; // analyze the closer point

        pq.pop(); // pop the current_node
        
        // Iterate all connections
        for(auto it = nodes.at(current_node).begin(); it != nodes.at(current_node).end(); it++){
            double weight = it->first;
            cv::Point2d adj_node = it->second;

            /* Check if there is a shorter path until this node */
            if(dist[adj_node] > dist[current_node] + weight){
                dist[adj_node] = dist[current_node] + weight;
                best_paretn[adj_node].x = current_node.x;
                best_paretn[adj_node].y = current_node.y;
                pq.emplace(std::make_pair(dist[adj_node], adj_node));
            }
        }
    }
    if(best_paretn[end] == end){
        return false; // Return false is found no connections
    }
    std::vector<cv::Point2d> tmp_best_path;
    /* construct a path from end to start */
    cv::Point2d parent = end;
    while(parent != start){
        tmp_best_path.emplace_back(parent);
        parent = best_paretn[parent];
    }
    tmp_best_path.emplace_back(start);

    /* Flip the path, and check if the last point in best_path is equal to the start point of new path */
    for(auto it_bp = tmp_best_path.rbegin(); it_bp != tmp_best_path.rend(); it_bp++){
        if(it_bp == tmp_best_path.rbegin()){
            if(best_path.size() != 0){
                if(*it_bp != best_path[best_path.size() - 1]){
                    best_path.emplace_back(*it_bp);
                }
            }else{
                best_path.emplace_back(*it_bp);
            }
        }else{
            best_path.emplace_back(*it_bp);
        }
            
    }
    return true;
}
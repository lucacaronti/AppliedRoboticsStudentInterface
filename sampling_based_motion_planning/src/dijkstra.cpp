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

Dijkstra::Dijkstra(){
}

void Dijkstra::print() const{
    std::cout<<nodes.size()<<std::endl;
    for(auto itm = nodes.begin(); itm != nodes.end(); itm++){
        std::cout << "node(" << itm->first.x << "," << itm->first.y << ")" << std::endl;
        for(auto itl = itm->second.begin(); itl != itm->second.end(); itl++){
            std::cout << "\t -> (" << itl->second.x << "," << itl->second.y << ") w=" << itl->first << std::endl;
        }
    }
}

bool Dijkstra::addEdge(cv::Point2d start, cv::Point2d end, double weight){
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
        nodes.emplace(std::make_pair(start,tmp_list));
    }

    // Add the opposite connection (end (to)-> starts)
    if(end_node != nodes.end()){
        end_node->second.emplace_back(weight, start);
    }else{
        std::list<distNode_t> tmp_list;
        tmp_list.emplace_back(weight, start);
        nodes.emplace(std::make_pair(end, tmp_list));
    }
    return true;
}

bool Dijkstra::shortesPath(cv::Point2d start, cv::Point2d end, std::vector<cv::Point2d>& best_path)
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
        
        for(auto it = nodes[current_node].begin(); it != nodes[current_node].end(); it++){
            double weight = it->first;
            cv::Point2d adj_node = it->second;

            if(dist[adj_node] > dist[current_node] + weight){
                dist[adj_node] = dist[current_node] + weight;
                best_paretn[adj_node].x = current_node.x;
                best_paretn[adj_node].y = current_node.y;
                pq.push(std::make_pair(dist[adj_node], adj_node));
            }
        }
    }
    if(best_paretn[end] == end){
        return false;
    }
    cv::Point2d parent = end;
    while(parent != start){
        best_path.emplace_back(parent);
        parent = best_paretn[parent];
    }
    best_path.emplace_back(start);
    return true;
}
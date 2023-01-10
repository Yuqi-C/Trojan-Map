#include "trojanmap.h"

//-----------------------------------------------------
// TODO: Student should implement the following:
//-----------------------------------------------------
/**
 * GetLat: Get the latitude of a Node given its id. If id does not exist, return -1.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : latitude
 */
double TrojanMap::GetLat(const std::string& id) {
    if(data.count(id)){
      return data[id].lat;
    }
    return -1;
}

/**
 * GetLon: Get the longitude of a Node given its id. If id does not exist, return -1.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(const std::string& id) { 
  if(data.count(id)){
      return data[id].lon;
    }
  return -1;
}

/**
 * GetName: Get the name of a Node given its id. If id does not exist, return "NULL".
 * 
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 */
std::string TrojanMap::GetName(const std::string& id) {
  if(data.count(id)){
      return data[id].name;
    }
  return "NULL";
}

/**
 * GetNeighborIDs: Get the neighbor ids of a Node. If id does not exist, return an empty vector.
 * 
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(const std::string& id) {
  if(data.count(id)){
      return data[id].neighbors;
    }
  return {};
}

/**
 * GetID: Given a location name, return the id. 
 * If the node does not exist, return an empty string. 
 *
 * @param  {std::string} name          : location name
 * @return {int}  : id
 */
std::string TrojanMap::GetID(const std::string& name) {
  std::string res = "";
  for(auto it = data.begin(); it != data.end(); it++){
    if(name == it->second.name){
      return it->first;
    }
  }
  return res;
}

/**
 * GetPosition: Given a location name, return the position. If id does not exist, return (-1, -1).
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name) {
  std::pair<double, double> results(-1, -1);
  for(auto it = data.begin(); it != data.end(); it++){
    if(name == it->second.name){
      return std::make_pair(it->second.lat, it->second.lon);
    }
  }
  return results;
}


/**
 * CalculateEditDistance: Calculate edit distance between two location names
 * 
 */

int TrojanMap::CalculateEditDistance(std::string a, std::string b){
    int a_len = a.length();
    int b_len = b.length();
    std::vector<std::vector<int>> d(b_len + 1, std::vector<int>(a_len + 1, 0));

    for(int i = 0; i <= a_len; i++){
      for(int j = 0; j <= b_len; j++){
        if(i == 0){
          d[j][i] = j;
          continue;
        }
        if(j == 0){
          d[j][i] = i;
          continue;
        }
          
        if( tolower(a[i-1]) == tolower(b[j-1]) ){
          d[j][i] = std::min( std::min(d[j-1][i] + 1, d[j][i-1] + 1), d[j-1][i-1]);
        }
        else{
          d[j][i] = std::min( std::min(d[j-1][i] + 1, d[j][i-1] + 1), d[j-1][i-1] + 1);            
        }  
      }
    }
    return d[b_len][a_len];
}

/**
 * FindClosestName: Given a location name, return the name with smallest edit distance.
 *
 * @param  {std::string} name          : location name
 * @return {std::string} tmp           : similar name
 */
std::string TrojanMap::FindClosestName(std::string name) {
  std::string tmp = "";
  int min = INT8_MAX;
  for(auto it = data.begin(); it != data.end(); it++){
    if(CalculateEditDistance(name, it->second.name) < min){
      min = CalculateEditDistance(name, it->second.name);
      tmp.replace(0, tmp.length(), it->second.name);
    }
  }
  return tmp;
}


/**
 * Autocomplete: Given a parital name return all the possible locations with
 * partial name as the prefix. The function should be case-insensitive.
 *
 * @param  {std::string} name          : partial name
 * @return {std::vector<std::string>}  : a vector of full names
 */
std::vector<std::string> TrojanMap::Autocomplete(std::string name){
  std::vector<std::string> results;
  for(auto it = data.begin(); it != data.end(); it++){
    if(it->second.name.length() < name.size()){
      continue;
    }
    std::string sub_ = it->second.name.substr(0, name.length());
    for(unsigned long int i = 0; i < sub_.length(); i++){
        sub_[i] = tolower(sub_[i]);
        name[i] = tolower(name[i]);
    }
    if(sub_ == name){
      results.push_back(it->second.name);
    }
  }
  return results;
}

/**
 * CalculateDistance: Get the distance between 2 nodes. 
 * 
 * @param  {std::string} a  : a_id
 * @param  {std::string} b  : b_id
 * @return {double}  : distance in mile
 */
double TrojanMap::CalculateDistance(const std::string &a_id, const std::string &b_id) {
  // Do not change this function
  Node a = data[a_id];
  Node b = data[b_id];
  double dlon = (b.lon - a.lon) * M_PI / 180.0;
  double dlat = (b.lat - a.lat) * M_PI / 180.0;
  double p = pow(sin(dlat / 2),2.0) + cos(a.lat * M_PI / 180.0) * cos(b.lat * M_PI / 180.0) * pow(sin(dlon / 2),2.0);
  double c = 2 * asin(std::min(1.0,sqrt(p)));
  return c * 3961;
}

/**
 * CalculatePathLength: Calculates the total path length for the locations inside the vector.
 * 
 * @param  {std::vector<std::string>} path : path
 * @return {double}                        : path length
 */
double TrojanMap::CalculatePathLength(const std::vector<std::string> &path) {
  // Do not change this function
  double sum = 0;
  for (int i = 0;i < int(path.size())-1; i++) {
    sum += CalculateDistance(path[i], path[i+1]);
  }
  return sum;
}

/**
 * CalculateShortestPath_Dijkstra: Given 2 locations, return the shortest path which is a
 * list of id. Hint: Use priority queue.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Dijkstra(
    std::string location1_name, std::string location2_name) {
  std::string src_id = GetID(location1_name);
  std::string dst_id = GetID(location2_name);  
  
  if(location1_name == location2_name){
    return {src_id};
  }

  typedef std::pair<std::string, double> node_dist;
  auto comp = []( node_dist a, node_dist b){return a.second > b.second;};
  typedef std::priority_queue<node_dist, std::vector<node_dist>, decltype(comp)> min_priority_queue;
  
  min_priority_queue min_pq(comp);
  std::unordered_set<std::string> visited;
  std::unordered_map<std::string, double> dist;
  std::unordered_map<std::string, std::string> pre_ID;
  std::vector<std::string> path;

  min_pq.push(node_dist(src_id,0));
  dist[src_id] = 0;
  while(!min_pq.empty()){
    node_dist temp = min_pq.top();
    min_pq.pop();
    visited.insert(temp.first);
    if(temp.first == dst_id){
      break;
    }

    for(auto& adj : data[temp.first].neighbors){
      if(visited.count(adj)){
        continue;
      }
      double dist_between = CalculateDistance(adj, temp.first);
      if( dist.count(adj) == 0 || dist[adj] > dist_between + dist[temp.first] ){
        dist[adj] = dist_between + dist[temp.first];
        pre_ID[adj] = temp.first;
        min_pq.push(node_dist(adj, dist[adj]));
      }  
    }
  }

  if(pre_ID.count(dst_id) == 0){
    return {};
  }

  std::string id_ptr = dst_id;
  while(id_ptr != src_id){
    path.push_back(id_ptr);
    id_ptr = pre_ID[id_ptr];
  }
  path.push_back(id_ptr);
  std::reverse(path.begin(), path.end());
  return path;
}

/**
 * CalculateShortestPath_Bellman_Ford: Given 2 locations, return the shortest path which is a
 * list of id. Hint: Do the early termination when there is no change on distance.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Bellman_Ford(
    std::string location1_name, std::string location2_name){
  std::vector<std::string> path;
  std::string src_id = GetID(location1_name);
  std::string dst_id = GetID(location2_name);
  if(src_id == dst_id){
    return {};
  }

  std::unordered_map<std::string, double> dist;
  std::unordered_map<std::string, std::string> pre_id;
  
  dist[src_id] = 0;
  bool stop = false;
  long unsigned int count = 0;
  while( !stop && count < data.size()-1){
    count++;
    stop = true;
    for(auto& vertex:dist){
      for(std::string& adj: data[vertex.first].neighbors){
        double dist_between = CalculateDistance(adj, vertex.first);
        if(dist.count(adj) == 0 || dist[adj] > dist[vertex.first] + dist_between){
          dist[adj] = dist[vertex.first] + dist_between;
          stop = false;
          pre_id[adj] = vertex.first; 
        }
      }
    }
  }

  if(pre_id.count(dst_id) == 0){
    return {};
  }

  std::string id_ptr = dst_id;
  while(id_ptr != src_id){
    path.push_back(id_ptr);
    id_ptr = pre_id[id_ptr];
  }
  path.push_back(id_ptr);
  std::reverse(path.begin(), path.end());
  return path;
}

/**
 * Travelling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path
 */
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Brute_force(
                                    std::vector<std::string> location_ids) {
  std::vector<int> cur_path = {0};
  double min_cost = INT_MAX;
  std::vector<std::vector<double>> weights(location_ids.size(), std::vector<double>(location_ids.size()));
  std::vector<std::vector<int>> all_path;
  std::vector<std::vector<std::string>> all_path_id;

  for(long unsigned int i = 0; i < location_ids.size(); i++){
    for(long unsigned j = i + 1; j < location_ids.size(); j++){
      weights[i][j] = CalculateDistance(location_ids[i], location_ids[j]);
      weights[j][i] = weights[i][j];
    }
  }

  TravellingTrojan_Brute_force_aux(0, 0, weights, all_path, cur_path, min_cost);
  
  //all path ID-representation
  for(long unsigned int i = 0; i < all_path.size(); i++){
    std::vector<std::string> v;
    for(int entry : all_path[i]){
      v.push_back(location_ids[entry]);
    }
    all_path_id.push_back(v);
  }
  
  std::pair<double, std::vector<std::vector<std::string>>> records(min_cost, all_path_id);
  return records;
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Backtracking(
                                    std::vector<std::string> location_ids) {
  std::vector<int> cur_path = {0};
  double min_cost = INT_MAX;
  std::vector<std::vector<double>> weights(location_ids.size(), std::vector<double>(location_ids.size()));
  std::vector<std::vector<int>> all_path;
  std::vector<std::vector<std::string>> all_path_id;

  for(long unsigned int i = 0; i < location_ids.size(); i++){
    for(long unsigned j = i + 1; j < location_ids.size(); j++){
      weights[i][j] = CalculateDistance(location_ids[i], location_ids[j]);
      weights[j][i] = weights[i][j];
    }
  }

  TravellingTrojan_Backtracking_aux(0, 0, weights, all_path, cur_path, min_cost);
  
  //all path ID-representation
  for(long unsigned int i = 0; i < all_path.size(); i++){
    std::vector<std::string> v;
    for(int entry : all_path[i]){
      v.push_back(location_ids[entry]);
    }
    all_path_id.push_back(v);
  }
  
  std::pair<double, std::vector<std::vector<std::string>>> records(min_cost, all_path_id);
  return records;
}


std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_2opt(
      std::vector<std::string> location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> records;
  std::vector<std::vector<std::string>> all_path;
  std::vector<std::string> cur_path; 
  double min_dist;

  if(location_ids.empty()){
    return records;
  }

  for(auto n : location_ids) cur_path.push_back(n);
  cur_path.push_back(location_ids[0]);


  if(location_ids.size() < 4){
    records.first = CalculatePathLength(cur_path);
    records.second.push_back(cur_path);
    return records;
  } 

  //at least 4 edges
  all_path.push_back(cur_path);
  min_dist = CalculatePathLength(cur_path);
  bool stop = false;
  while(!stop){
    stop = true;
    for(unsigned long int i = 1; i < location_ids.size() - 1; i++){
      for(unsigned long int j = i + 1; j < location_ids.size(); j++){
        std::reverse(cur_path.begin() + i, cur_path.begin() + j);
        double cur_dist = CalculatePathLength(cur_path);
        if(min_dist > cur_dist){
          min_dist = cur_dist;
          all_path.push_back(cur_path);
          stop = false;
        }else{
          std::reverse(cur_path.begin() + i, cur_path.begin() + j);
        }
      }
    }
  }

  records.first = min_dist;
  records.second = all_path;
  return records;
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_3opt(
      std::vector<std::string> location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> records;
  std::vector<std::vector<std::string>> all_path;
  std::vector<std::string> cur_path; 
  double min_dist;

  if(location_ids.empty()){
    return records;
  }

  for(auto n : location_ids) cur_path.push_back(n);
  cur_path.push_back(location_ids[0]);

  if(location_ids.size() < 5){
    records.first = CalculatePathLength(cur_path);
    records.second.push_back(cur_path);
    return records;
  } 

  //at least 5 nodes
  all_path.push_back(cur_path);
  min_dist = CalculatePathLength(cur_path);
  bool stop = false;
  while(!stop){
    stop = true;
    for(int m = 1; m < int(cur_path.size()) - 3; m++){
      for(int n = m + 1; n < int(cur_path.size()) - 2; n++){
        for(int p = n + 1; p < int(cur_path.size()) - 1; p++){
          std::vector<std::string> swap(cur_path);
          std::string node_m = swap[m];
          std::string node_n = swap[n];
          std::string node_p = swap[p];
          swap[n]=node_m;
          swap[p]=node_n;
          swap[m]=node_p;
          double temp = CalculatePathLength(swap);

          if(temp < min_dist){
            min_dist = temp;
            cur_path = swap;
            all_path.push_back(swap);
            stop = false;
            continue;
          }
          
          swap = cur_path;
          node_m = swap[m];
          node_n = swap[n];
          node_p = swap[p];
          swap[m] = node_n;
          swap[n] = node_p;
          swap[p] = node_m;
          temp = CalculatePathLength(swap);
          
          if(temp < min_dist){
            min_dist = temp;
            cur_path = swap;
            all_path.push_back(swap);
            stop = false;
          } 
        }
      }
    }       
  }

  records.first = min_dist;
  records.second = all_path;
  return records;
}        
/**
 * Given CSV filename, it read and parse locations data from CSV file,
 * and return locations vector for topological sort problem.
 *
 * @param  {std::string} locations_filename     : locations_filename
 * @return {std::vector<std::string>}           : locations 
 */
std::vector<std::string> TrojanMap::ReadLocationsFromCSVFile(std::string locations_filename){
  std::vector<std::string> location_names_from_csv;

  std::fstream fin;
  fin.open(locations_filename, std::ios::in);
  std::string line;

  getline(fin, line);
  while(getline(fin,line)){
    if(line != ""){
      location_names_from_csv.push_back(line);
    }
  }

  fin.close();
  return location_names_from_csv;
}

/**
 * Given CSV filenames, it read and parse dependencise data from CSV file,
 * and return dependencies vector for topological sort problem.
 *
 * @param  {std::string} dependencies_filename     : dependencies_filename
 * @return {std::vector<std::vector<std::string>>} : dependencies
 */
std::vector<std::vector<std::string>> TrojanMap::ReadDependenciesFromCSVFile(std::string dependencies_filename){
  std::vector<std::vector<std::string>> dependencies_from_csv;
  
  std::fstream fin;
  fin.open(dependencies_filename, std::ios::in);
  std::string line;

  getline(fin, line);
  while(getline(fin,line)){
    auto pos = line.find(',');
    if(line != ""){
      std::string name_1 = line.substr(0, pos);
      std::string name_2 = line.substr(pos+1);
      dependencies_from_csv.push_back({name_1, name_2});
    }
  }

  fin.close();
  return dependencies_from_csv;
}

/**
 * DeliveringTrojan: Given a vector of location names, it should return a sorting of nodes
 * that satisfies the given dependencies. If there is no way to do it, return a empty vector.
 *
 * @param  {std::vector<std::string>} locations                     : locations
 * @param  {std::vector<std::vector<std::string>>} dependencies     : prerequisites
 * @return {std::vector<std::string>} results                       : results
 */
std::vector<std::string> TrojanMap::DeliveringTrojan(std::vector<std::string> &locations,
                                                     std::vector<std::vector<std::string>> &dependencies){
  std::vector<std::string> result;
  std::unordered_map<std::string, std::vector<std::string>> neighbor;

  for(auto id : locations){
    neighbor[id] = {};
  }

  for(auto& dep : dependencies){
    neighbor[dep[0]].push_back(dep[1]);
  }

  FindTopology(result, locations, neighbor);

  if(result.size() != neighbor.size()) return {};
  std::reverse(result.begin(), result.end());
  return result;
}

/**
 * inSquare: Give a id retunr whether it is in square or not.
 *
 * @param  {std::string} id            : location id
 * @param  {std::vector<double>} square: four vertexes of the square area
 * @return {bool}                      : in square or not
 */
bool TrojanMap::inSquare(std::string id, std::vector<double> &square) {
  double lat = GetLat(id);
  double lon = GetLon(id);
  if( lon > square[0] && lon < square[1] && lat < square[2] && lat > square[3]){
    return true;
  }else{
    return false;
  }  
}

/**
 * GetSubgraph: Give four vertexes of the square area, return a list of location ids in the squares
 *
 * @param  {std::vector<double>} square         : four vertexes of the square area
 * @return {std::vector<std::string>} subgraph  : list of location ids in the square
 */
std::vector<std::string> TrojanMap::GetSubgraph(std::vector<double> &square) {
  // include all the nodes in subgraph
  std::vector<std::string> subgraph;
  for(auto entry : data){
    if(inSquare(entry.first, square)){
      subgraph.push_back(entry.first);
    }
  }
  return subgraph;
}

/**
 * Cycle Detection: Given four points of the square-shape subgraph, return true if there
 * is a cycle path inside the square, false otherwise.
 * 
 * @param {std::vector<std::string>} subgraph: list of location ids in the square
 * @param {std::vector<double>} square: four vertexes of the square area
 * @return {bool}: whether there is a cycle or not
 */
bool TrojanMap::CycleDetection(std::vector<std::string> &subgraph, std::vector<double> &square){
  std::unordered_map<std::string, int> visited;
  for(auto& id : subgraph){
    visited[id] = 0;
  }

  for(auto& id : subgraph){
    if(!visited[id])
      if(DFS(id, visited, "null"))
        return true;
  }
  return false;
}

/**
 * FindNearby: Given a class name C, a location name L and a number r, 
 * find all locations in class C on the map near L with the range of r and return a vector of string ids
 * 
 * @param {std::string} className: the name of the class
 * @param {std::string} locationName: the name of the location
 * @param {int} r: search radius
 * @param {int} k: search numbers
 * @return {std::vector<std::string>}: location name that meets the requirements
 */
std::vector<std::string> TrojanMap::FindNearby(std::string attributesName, std::string name, double r, int k){
  std::vector<std::string> res;
  std::string id = GetID(name);
  if( data[id].attributes.empty()){
    return {};
  }

  typedef std::pair<std::string, double> node_dist;
  auto comp = []( node_dist a, node_dist b){return a.second > b.second;};
  typedef std::priority_queue<node_dist, std::vector<node_dist>, decltype(comp)> min_priority_queue;
  min_priority_queue min_pq(comp);

  for(auto& entry : data){
    if(entry.first == id) continue;
    if(entry.second.attributes.empty()) continue;
    double dist = CalculateDistance(id, entry.first);
    if(dist < r && entry.second.attributes.count(attributesName)) min_pq.push(node_dist(entry.first, dist));
  }

  while(!min_pq.empty()){
    if(int(res.size()) >= k) break;
    std::string temp = min_pq.top().first;
    res.push_back(temp);
    min_pq.pop();
  }

  return res;
}

/**
 * CreateGraphFromCSVFile: Read the map data from the csv file
 * 
 */
void TrojanMap::CreateGraphFromCSVFile() {
  // Do not change this function
  std::fstream fin;
  fin.open("src/lib/data.csv", std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);

    Node n;
    int count = 0;
    while (getline(s, word, ',')) {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '{'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '}'), word.end());
      if (count == 0)
        n.id = word;
      else if (count == 1)
        n.lat = stod(word);
      else if (count == 2)
        n.lon = stod(word);
      else if (count == 3)
        n.name = word;
      else {
        word.erase(std::remove(word.begin(), word.end(), ' '), word.end());
        if (isalpha(word[0]))
          n.attributes.insert(word);
        if (isdigit(word[0]))
          n.neighbors.push_back(word);
      }
      count++;
    }
    data[n.id] = n;
  }
  fin.close();
}

//-----------------------------------------------------
//user-defined method
//-----------------------------------------------------

/**
 * TSP Brute Force auxiliary
 */
void TrojanMap::TravellingTrojan_Brute_force_aux(int cur_node, double cur_cost, std::vector<std::vector<double>>& weights, 
                                                std::vector<std::vector<int>>& all_path, std::vector<int>& cur_path, double& min_cost){
  if(cur_path.size() == weights.size()){
    double final_cost = cur_cost + weights[cur_node][0];
    if(final_cost < min_cost){
      cur_path.push_back(0);
      min_cost = final_cost;
      all_path.push_back(cur_path);
      cur_path.pop_back();
    }
    return;
  }

  for(long unsigned int i = 0; i < weights.size(); i++){
    if(std::find(cur_path.begin(), cur_path.end(), i) != cur_path.end()){
      continue;
    }
    cur_path.push_back(i);
    TravellingTrojan_Brute_force_aux(i, cur_cost + weights[cur_node][i], weights, all_path, cur_path, min_cost);
    cur_path.pop_back();
  }
}

/**
 * TSP Backtracking auxiliary
 */
void TrojanMap::TravellingTrojan_Backtracking_aux(int cur_node, double cur_cost, std::vector<std::vector<double>>& weights, 
                                                  std::vector<std::vector<int>>& all_path,std::vector<int>& cur_path, double& min_cost){
  if(cur_path.size() == weights.size()){
    double final_cost = cur_cost + weights[cur_node][0];
    if(final_cost < min_cost){
      cur_path.push_back(0);
      min_cost = final_cost;
      all_path.push_back(cur_path);
      cur_path.pop_back();
    }
    return;
  }

  //Early Backtracking
  if(cur_cost >= min_cost){
    return;
  }

  for(long unsigned int i = 0; i < weights.size(); i++){
    if(std::find(cur_path.begin(), cur_path.end(), i) != cur_path.end()){
      continue;
    }
    cur_path.push_back(i);
    TravellingTrojan_Brute_force_aux(i, cur_cost + weights[cur_node][i], weights, all_path, cur_path, min_cost);
    cur_path.pop_back();
  }
}

/**
 * Find Topology auxiliary
 */
void TrojanMap::FindTopology(std::vector<std::string>& result, std::vector<std::string>& locations,
                    std::unordered_map<std::string, std::vector<std::string>>& neighbor){
  
  if(locations.size() == 1){
    result.push_back(locations[0]);
    return;
  }

  for(unsigned long int i = 0; i < locations.size(); i++){
    std::string id = locations[i];
    if(neighbor[id].empty()){
      result.push_back(id);
      locations.erase(locations.begin() + i);
      for(auto entry : locations){
        auto itr = std::find(neighbor[entry].begin(), neighbor[entry].end(), id);
        if(itr != neighbor[entry].end()) neighbor[entry].erase(itr);
      }  
      FindTopology(result, locations, neighbor);
      break;
    }    
  }
}

/**
 * Implementation of Cycle Detection using DFS
 */
bool TrojanMap::DFS(std::string cur_id, std::unordered_map<std::string, int>& visited, std::string parent){
  visited[cur_id] = 1;
  for(auto adj : GetNeighborIDs(cur_id)){
    if(visited.count(adj) == 0){
      continue;
    }
    if(!visited[adj]){
      if(DFS(adj, visited, cur_id)){
        return true;
      }
    }else if(adj != parent){
      return true;
    }
  }
  return false;
}